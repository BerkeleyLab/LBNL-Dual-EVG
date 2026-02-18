// Provide sequencer start signals for injector
module injectorSequenceControl #(
    parameter SYSCLK_RATE               = -1,
    parameter ALIGNMENT_SYNC_COUNT      =  2,
    parameter [ALIGNMENT_SYNC_COUNT*32-1:0]
        TX_CLK_PER_ALIGNMENT             = {-32'd1, -32'd1}
    ) (
    input              sysClk,
    input       [31:0] sysGPIO_OUT,

    input              sysCsrStrobe,
    output wire [31:0] sysStatus,

    input              sysCsrAlignStrobe,
    output wire [31:0] sysAlignStatus,

    input              powerline_a,

    input                               evgTxClk,
    input    [ALIGNMENT_SYNC_COUNT-1:0] evgHeartbeat,
    output                              evgHeartbeatAlign,
    output                              evgHeartbeatCore,
    output reg                          evgSequenceStart = 0);

localparam ALIGNMENT_SYNC_COUNT_MAX_WIDTH = 3;

if (ALIGNMENT_SYNC_COUNT > (1 << ALIGNMENT_SYNC_COUNT_MAX_WIDTH)) begin
    ALIGNMENT_SYNC_COUNT_is_bigger_than_8 err();
end

localparam ALIGNMENT_SYNC_COUNT_WIDTH = (ALIGNMENT_SYNC_COUNT <= 1)?
    1 : $clog2(ALIGNMENT_SYNC_COUNT);

///////////////////////////////////////////////////////////////////////////////
// System clock domain

// Scale sysClk to milliseconds
localparam SYSCLK_PER_MILLISECOND = SYSCLK_RATE / 1000;
localparam SYSCLK_DIVIDER_RELOAD = SYSCLK_PER_MILLISECOND - 2;
localparam SYSCLK_DIVIDER_WIDTH = $clog2(SYSCLK_DIVIDER_RELOAD+1)+1;
reg [SYSCLK_DIVIDER_WIDTH-1:0] sysClkDivider = SYSCLK_DIVIDER_RELOAD;
wire sysClkDividerDone = sysClkDivider[SYSCLK_DIVIDER_WIDTH-1];

// Injection cycle trigger
localparam CYCLE_COUNTER_RELOAD_WIDTH = 16;
localparam CYCLE_COUNTER_WIDTH = CYCLE_COUNTER_RELOAD_WIDTH + 1;
reg [CYCLE_COUNTER_WIDTH-1:0] sysCycleCounter = 0;
wire sysCycleCounterDone = sysCycleCounter[CYCLE_COUNTER_WIDTH-1];
reg [CYCLE_COUNTER_RELOAD_WIDTH-1:0] sysCycleCounterReload = 1400 - 2;
reg sysCycleEnabled = 0;
reg sysInjectorStartToggle = 0;

always @(posedge sysClk) begin
    if (sysClkDividerDone) begin
        sysClkDivider <= SYSCLK_DIVIDER_RELOAD;
    end
    else begin
        sysClkDivider <= sysClkDivider - 1;
    end

    if (sysCsrStrobe) begin
        if (sysGPIO_OUT[31]) begin
            sysCycleCounterReload <= sysGPIO_OUT[CYCLE_COUNTER_RELOAD_WIDTH-1:0];
        end
        else begin
            if (sysGPIO_OUT[1]) begin
                sysCycleEnabled <= 0;
            end
            else if (sysGPIO_OUT[0]) begin
                sysCycleEnabled <= 1;
            end
            else if (sysGPIO_OUT[7] && !sysCycleEnabled) begin
                sysInjectorStartToggle <= !sysInjectorStartToggle;
            end
        end
    end

    if (sysCycleEnabled) begin
        if (sysClkDividerDone) begin
            if (sysCycleCounterDone) begin
                sysCycleCounter <= {1'b0, sysCycleCounterReload};
                sysInjectorStartToggle <= !sysInjectorStartToggle;
            end
            else begin
                sysCycleCounter <= sysCycleCounter - 1;
            end
        end
    end
    else begin
        sysCycleCounter <= 0;
    end
end

reg [ALIGNMENT_SYNC_COUNT_MAX_WIDTH-1:0 ] sysAlignCounterSel = 0;
reg [ALIGNMENT_SYNC_COUNT_MAX_WIDTH-1:0 ] sysEvgHeartbeatSel = 0;
always @(posedge sysClk) begin
    if (sysCsrAlignStrobe) begin
        if (sysGPIO_OUT[31]) begin
            sysAlignCounterSel <= sysGPIO_OUT[ALIGNMENT_SYNC_COUNT_MAX_WIDTH-1:0];
        end
        else if (sysGPIO_OUT[30]) begin
            sysEvgHeartbeatSel <= sysGPIO_OUT[ALIGNMENT_SYNC_COUNT_MAX_WIDTH-1:0];
        end
    end
end

// Power line trigger
wire sysPowerline, sysPowerlineTimeout;
powerlineTrigger #(.CLK_RATE(SYSCLK_RATE))
  powerlineTrigger (
    .clk(sysClk),
    .powerline_a(powerline_a),
    .trigger(sysPowerline),
    .powerlineTimeout(sysPowerlineTimeout));


///////////////////////////////////////////////////////////////////////////////
// Event generator clock domain


// Forward to TX clk

wire [ALIGNMENT_SYNC_COUNT_MAX_WIDTH-1:0] alignCounterSel;
wire [ALIGNMENT_SYNC_COUNT_MAX_WIDTH-1:0] evgHeartbeatSel;

forwardData #(
    .DATA_WIDTH(ALIGNMENT_SYNC_COUNT_MAX_WIDTH+
                ALIGNMENT_SYNC_COUNT_MAX_WIDTH))
  forwardData (
    .inClk(sysClk),
    .inData({sysAlignCounterSel,
            sysEvgHeartbeatSel}),
    .outClk(evgTxClk),
    .outData({alignCounterSel,
              evgHeartbeatSel}));

wire [ALIGNMENT_SYNC_COUNT-1:0] alignmentCounterDone;
wire [ALIGNMENT_SYNC_COUNT-1:0] alignmentCounterSynced;
wire alignmentCounterSyncedAll = &alignmentCounterSynced;

genvar i;
generate
for (i = 0; i < ALIGNMENT_SYNC_COUNT; i = i + 1) begin

localparam TX_CLK_PER_ALIGNMENT_LOCAL = TX_CLK_PER_ALIGNMENT[i*32+:32];

alignmentGenerator #(
    .CLK_PER_ALIGNMENT(TX_CLK_PER_ALIGNMENT_LOCAL))
  alignmentGenerator (
    .clk(evgTxClk),

    .heartbeatStrobe(evgHeartbeat[i]),
    .alignmentCounterDone(alignmentCounterDone[i]),
    .alignmentCounterSynced(alignmentCounterSynced[i]));

end
endgenerate

// Detect cycle start requests
(*ASYNC_REG="true"*) reg injectorStartToggle_m = 0;
reg injectorStartToggle = 0, injectorStartToggle_d = 0;

// Detect power line coincidence
(*ASYNC_REG="true"*) reg powerline_m = 0;
(*ASYNC_REG="true"*) reg powerlineTimeout_m = 0;
reg powerline = 0, powerline_d = 0, powerlineTimeout = 0;

// Synchronization state machine
localparam ST_IDLE             = 2'd0,
           ST_AWAIT_POWER_LINE = 2'd1,
           ST_AWAIT_ALIGNMENT  = 2'd2,
           ST_TRIGGER          = 2'd3;
reg [1:0] injectorStartState = ST_IDLE;
// Latch only the part that will be used
reg [ALIGNMENT_SYNC_COUNT_WIDTH-1:0] alignCounterSelLatch = 0;

always @(posedge evgTxClk) begin
    injectorStartToggle_m <= sysInjectorStartToggle;
    injectorStartToggle   <= injectorStartToggle_m;
    injectorStartToggle_d <= injectorStartToggle;
    powerline_m <= sysPowerline;
    powerline   <= powerline_m;
    powerline_d <= powerline;
    powerlineTimeout_m <= sysPowerlineTimeout;
    powerlineTimeout   <= powerlineTimeout_m;

    case (injectorStartState)
    ST_IDLE: begin
        evgSequenceStart <= 0;
        if (injectorStartToggle != injectorStartToggle_d) begin
            injectorStartState <= ST_AWAIT_POWER_LINE;
            alignCounterSelLatch <= alignCounterSel[ALIGNMENT_SYNC_COUNT_WIDTH-1:0];
        end
    end
    ST_AWAIT_POWER_LINE: begin
        if (powerlineTimeout || (powerline && !powerline_d)) begin
            injectorStartState <= ST_AWAIT_ALIGNMENT;
        end
    end
    ST_AWAIT_ALIGNMENT: begin
        if (alignmentCounterDone[alignCounterSelLatch]) begin
            injectorStartState <= ST_TRIGGER;
        end
    end
    ST_TRIGGER: begin
        evgSequenceStart <= 1;
        injectorStartState <= ST_IDLE;
    end
    default: ;
    endcase
end

// Alignement heartbeat
wire [ALIGNMENT_SYNC_COUNT_WIDTH-1:0] evgHeartbeatAlignSel =
    alignCounterSel[ALIGNMENT_SYNC_COUNT_WIDTH-1:0];
assign evgHeartbeatAlign = evgHeartbeat[evgHeartbeatAlignSel];

// EVG heartbeat
wire [ALIGNMENT_SYNC_COUNT_WIDTH-1:0] evgHeartbeatCoreSel =
    evgHeartbeatSel[ALIGNMENT_SYNC_COUNT_WIDTH-1:0];
assign evgHeartbeatCore = evgHeartbeat[evgHeartbeatCoreSel];

// Don't bother with CDC for alignmentCounterSynced. This is a very slow
// signal. Sampled ~ @1s.
assign sysStatus = { !sysPowerlineTimeout, alignmentCounterSyncedAll,
                     {24-2-CYCLE_COUNTER_RELOAD_WIDTH{1'b0}},
                     sysCycleCounterReload,
                     {8-1{1'b0}}, sysCycleEnabled };

// in TX CLK domain, but these signals change only ~@1s
assign sysAlignStatus = {{12{1'b0}},
                        {4-ALIGNMENT_SYNC_COUNT_MAX_WIDTH{1'b0}}, evgHeartbeatSel,
                        {4-ALIGNMENT_SYNC_COUNT_MAX_WIDTH{1'b0}}, alignCounterSel,
                        {4-ALIGNMENT_SYNC_COUNT_WIDTH{1'b0}}, alignCounterSelLatch,
                        {8-ALIGNMENT_SYNC_COUNT{1'b0}}, alignmentCounterSynced };

endmodule
