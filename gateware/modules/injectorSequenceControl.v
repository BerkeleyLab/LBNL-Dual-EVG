// Provide sequencer start signals for injector
module injectorSequenceControl #(
    parameter SYSCLK_RATE               = -1,
    parameter ALIGNMENT_SYNC_COUNT      =  2,
    parameter [ALIGNMENT_SYNC_COUNT*32-1:0]
        TX_CLK_PER_ALIGNMENT             = {-32'd1, -32'd1}
    ) (
    input              sysClk,
    input              sysCsrStrobe,
    input       [31:0] sysGPIO_OUT,
    output wire [31:0] sysStatus,

    input              powerline_a,

    input                               evgTxClk,
    input    [ALIGNMENT_SYNC_COUNT-1:0] evgHeartbeat,
    output reg                          evgSequenceStart = 0);

localparam ALIGNMENT_SYNC_COUNT_MAX_WIDTH = 4;

if (ALIGNMENT_SYNC_COUNT > (1 << ALIGNMENT_SYNC_COUNT_MAX_WIDTH)) begin
    ALIGNMENT_SYNC_COUNT_is_bigger_than_4 err();
end

localparam ALIGNMENT_SYNC_COUNT_WIDTH = $clog2(ALIGNMENT_SYNC_COUNT);

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
reg [ALIGNMENT_SYNC_COUNT_MAX_WIDTH-1:0 ] sysAlignCounterSel = 0;
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
        else if (sysGPIO_OUT[30]) begin
            sysAlignCounterSel <= sysGPIO_OUT[ALIGNMENT_SYNC_COUNT_MAX_WIDTH-1:0];
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


// Forward alignCounterSel to TX clk

wire [ALIGNMENT_SYNC_COUNT_MAX_WIDTH-1:0] alignCounterSel;

forwardData #(
    .DATA_WIDTH(ALIGNMENT_SYNC_COUNT_MAX_WIDTH))
  forwardData (
    .inClk(sysClk),
    .inData(sysAlignCounterSel),
    .outClk(evgTxClk),
    .outData(alignCounterSel));

wire [ALIGNMENT_SYNC_COUNT-1:0] alignmentCounterDone;
wire [ALIGNMENT_SYNC_COUNT-1:0] alignmentCounterSynced;

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

// Don't bother with CDC for alignmentCounterSynced. This is a very slow
// signal. Sampled approx. once a sec.
assign sysStatus = { !sysPowerlineTimeout, alignmentCounterSynced,
                     {24-1-ALIGNMENT_SYNC_COUNT-CYCLE_COUNTER_RELOAD_WIDTH{1'b0}},
                     sysCycleCounterReload,
                     sysAlignCounterSel, {4-1{1'b0}}, sysCycleEnabled };

endmodule
