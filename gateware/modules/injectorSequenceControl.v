// Provide sequencer start signals for injector
module injectorSequenceControl #(
    parameter SYSCLK_RATE = -1,
    parameter TXCLK_PER_BR_AR_ALIGNMENT = -1
    ) (
    input              sysClk,
    input              sysCsrStrobe,
    input       [31:0] sysGPIO_OUT,
    output wire [31:0] sysStatus,

    input              powerline_a,

    input      evgTxClk,
    input      evgHeartbeat,
    output reg evgSequenceStart = 0);

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
            sysCycleCounterReload<=sysGPIO_OUT[CYCLE_COUNTER_RELOAD_WIDTH-1:0];
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

// Produce booster/accumulator bucket alignment coincidence marker
localparam ALIGNMENT_COUNTER_RELOAD = TXCLK_PER_BR_AR_ALIGNMENT - 2;
localparam ALIGNMENT_COUNTER_WIDTH = $clog2(ALIGNMENT_COUNTER_RELOAD+1)+1;
reg [ALIGNMENT_COUNTER_WIDTH-1:0] alignmentCounter;
wire alignmentCounterDone = alignmentCounter[ALIGNMENT_COUNTER_WIDTH-1];
reg alignmentCounterSynced = 0;

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

always @(posedge evgTxClk) begin
    injectorStartToggle_m <= sysInjectorStartToggle;
    injectorStartToggle   <= injectorStartToggle_m;
    injectorStartToggle_d <= injectorStartToggle;
    powerline_m <= sysPowerline;
    powerline   <= powerline_m;
    powerline_d <= powerline;
    powerlineTimeout_m <= sysPowerlineTimeout;
    powerlineTimeout   <= powerlineTimeout_m;

    if (evgHeartbeat) begin
        alignmentCounterSynced <= alignmentCounterDone;
        alignmentCounter <= ALIGNMENT_COUNTER_RELOAD;
    end
    else if (alignmentCounterDone) begin
        alignmentCounter <= ALIGNMENT_COUNTER_RELOAD;
    end
    else begin
        alignmentCounter <= alignmentCounter - 1;
    end

    case (injectorStartState)
    ST_IDLE: begin
        evgSequenceStart <= 0;
        if (injectorStartToggle != injectorStartToggle_d) begin
            injectorStartState <= ST_AWAIT_POWER_LINE;
        end
    end
    ST_AWAIT_POWER_LINE: begin
        if (powerlineTimeout || (powerline && !powerline_d)) begin
            injectorStartState <= ST_AWAIT_ALIGNMENT;
        end
    end
    ST_AWAIT_ALIGNMENT: begin
        if (alignmentCounterDone) begin
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

assign sysStatus = { !sysPowerlineTimeout, alignmentCounterSynced,
                     {24-2-CYCLE_COUNTER_RELOAD_WIDTH{1'b0}},
                     sysCycleCounterReload,
                     {8-1{1'b0}}, sysCycleEnabled };

endmodule
