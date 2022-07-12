// Provide sequencer start signals for accumulator/storage swap
module swapoutSequenceControl #(
    parameter CLOCK_PER_ARSR_COINCIDENCE = -1,
    parameter DEBUG                      = "false"
    ) (
    input              sysClk,
    input              sysCsrStrobe,
    input       [31:0] sysGPIO_OUT,
    output wire [31:0] sysStatus,

    input                           evgTxClk,
    (*mark_debug=DEBUG*) input      evgHeartbeatRequest,
    (*mark_debug=DEBUG*) output reg evgSequenceStart = 0);

localparam OFFSET_WIDTH = 16;

///////////////////////////////////////////////////////////////////////////////
// System clock domain

reg [OFFSET_WIDTH-1:0] sysOffset = 0;
reg sysSwapoutStartToggle = 0, sysSwapoutStartToggle_d = 0;
always @(posedge sysClk) begin
    sysSwapoutStartToggle_d <= sysSwapoutStartToggle;
    if (sysCsrStrobe) begin
        sysOffset <= sysGPIO_OUT[OFFSET_WIDTH-1:0];
        sysSwapoutStartToggle <= !sysSwapoutStartToggle;
    end
end

///////////////////////////////////////////////////////////////////////////////
// Event generator clock domain

// Detect cycle start requests
(*ASYNC_REG="true"*) reg swapoutStartToggle_m = 0;
reg swapoutStartToggle = 0, swapoutStartToggle_d = 0;
reg evgHeartbeatRequest_d = 1;
reg evgHeartbeatFault = 1;

// Detect AR/SR coincidence
localparam COINC_COUNTER_RELOAD = CLOCK_PER_ARSR_COINCIDENCE - 2;
localparam COINC_COUNTER_WIDTH = $clog2(COINC_COUNTER_RELOAD+1) + 1;
(*mark_debug=DEBUG*)
reg [COINC_COUNTER_WIDTH-1:0] coincCounter = COINC_COUNTER_RELOAD;
wire coincidenceDetect = coincCounter[COINC_COUNTER_WIDTH-1];
always @(posedge evgTxClk) begin
    evgHeartbeatRequest_d <= evgHeartbeatRequest;
    if (evgHeartbeatRequest && !evgHeartbeatRequest_d) begin
        coincCounter <= COINC_COUNTER_RELOAD;
        if (coincidenceDetect) begin
            evgHeartbeatFault <= 0;
        end
        else begin
            evgHeartbeatFault <= 1;
        end
    end
    else if (coincidenceDetect) begin
        coincCounter <= COINC_COUNTER_RELOAD;
    end
    else begin
        coincCounter <= coincCounter - 1;
    end
end

// Synchronization state machine
// FIXME: This code is my best guess at what's needed for starting a swapout sequence.
localparam ST_IDLE              = 2'd0,
           ST_AWAIT_COINCIDENCE = 2'd1,
           ST_OFFSETTING        = 2'd2,
           ST_TRIGGER           = 2'd3;
(*mark_debug=DEBUG*) reg [1:0] swapoutStartState = ST_IDLE;
(*mark_debug=DEBUG*) reg [OFFSET_WIDTH:0] offsetCounter = 0;
wire offsetCounterDone = offsetCounter[OFFSET_WIDTH];

always @(posedge evgTxClk) begin
    swapoutStartToggle_m <= sysSwapoutStartToggle_d;
    swapoutStartToggle   <= swapoutStartToggle_m;
    swapoutStartToggle_d <= swapoutStartToggle;

    case (swapoutStartState)
    ST_IDLE: begin
        evgSequenceStart <= 0;
        if (swapoutStartToggle != swapoutStartToggle_d) begin
            swapoutStartState <= ST_AWAIT_COINCIDENCE;
        end
    end
    ST_AWAIT_COINCIDENCE: begin
        if (coincidenceDetect) begin
            offsetCounter <= {1'b0, sysOffset};
            swapoutStartState <= ST_OFFSETTING;
        end
    end
    ST_OFFSETTING: begin
        offsetCounter <= offsetCounter - 1;
        if (offsetCounterDone) begin
            swapoutStartState <= ST_TRIGGER;
        end
    end
    ST_TRIGGER: begin
        evgSequenceStart <= 1;
        swapoutStartState <= ST_IDLE;
    end
    default: ;
    endcase
end

assign sysStatus = {evgHeartbeatFault, {32-1-OFFSET_WIDTH{1'b0}}, sysOffset };

endmodule
