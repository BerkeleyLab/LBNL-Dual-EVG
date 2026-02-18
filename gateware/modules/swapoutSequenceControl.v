// Provide sequencer start signals for accumulator/storage swap
module swapoutSequenceControl #(
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

    input                               evgTxClk,
    input    [ALIGNMENT_SYNC_COUNT-1:0] evgHeartbeat,
    output                              evgHeartbeatAlign,
    output                              evgHeartbeatCore,
    output reg                          evgSequenceStart = 0);

localparam OFFSET_WIDTH = 16;
localparam ALIGNMENT_SYNC_COUNT_MAX_WIDTH = 3;

if (ALIGNMENT_SYNC_COUNT > (1 << ALIGNMENT_SYNC_COUNT_MAX_WIDTH)) begin
    ALIGNMENT_SYNC_COUNT_is_bigger_than_8 err();
end

localparam ALIGNMENT_SYNC_COUNT_WIDTH = (ALIGNMENT_SYNC_COUNT <= 1)?
    1 : $clog2(ALIGNMENT_SYNC_COUNT);

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

// Synchronization state machine
// FIXME: This code is my best guess at what's needed for starting a swapout sequence.
localparam ST_IDLE              = 2'd0,
           ST_AWAIT_COINCIDENCE = 2'd1,
           ST_OFFSETTING        = 2'd2,
           ST_TRIGGER           = 2'd3;
reg [1:0] swapoutStartState = ST_IDLE;
reg [OFFSET_WIDTH:0] offsetCounter = 0;
wire offsetCounterDone = offsetCounter[OFFSET_WIDTH];
// Latch only the part that will be used
reg [ALIGNMENT_SYNC_COUNT_WIDTH-1:0] alignCounterSelLatch = 0;

// Detect cycle start requests
(*ASYNC_REG="true"*) reg swapoutStartToggle_m = 0;
reg swapoutStartToggle = 0, swapoutStartToggle_d = 0;

always @(posedge evgTxClk) begin
    swapoutStartToggle_m <= sysSwapoutStartToggle_d;
    swapoutStartToggle   <= swapoutStartToggle_m;
    swapoutStartToggle_d <= swapoutStartToggle;

    case (swapoutStartState)
    ST_IDLE: begin
        evgSequenceStart <= 0;
        if (swapoutStartToggle != swapoutStartToggle_d) begin
            swapoutStartState <= ST_AWAIT_COINCIDENCE;
            alignCounterSelLatch <= alignCounterSel[ALIGNMENT_SYNC_COUNT_WIDTH-1:0];
        end
    end
    ST_AWAIT_COINCIDENCE: begin
        if (alignmentCounterDone[alignCounterSelLatch]) begin
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

// Alignement heartbeat
wire [ALIGNMENT_SYNC_COUNT_WIDTH-1:0] evgHeartbeatAlignSel =
    alignCounterSel[ALIGNMENT_SYNC_COUNT_WIDTH-1:0];
assign evgHeartbeatAlign = evgHeartbeat[evgHeartbeatAlignSel];

// EVG heartbeat
wire [ALIGNMENT_SYNC_COUNT_WIDTH-1:0] evgHeartbeatCoreSel =
    evgHeartbeatSel[ALIGNMENT_SYNC_COUNT_WIDTH-1:0];
assign evgHeartbeatCore = evgHeartbeat[evgHeartbeatCoreSel];

assign sysStatus = {!alignmentCounterSyncedAll, {32-1-OFFSET_WIDTH{1'b0}}, sysOffset };

// in TX CLK domain, but these signals change only ~@1s
assign sysAlignStatus = {{12{1'b0}},
                        {4-ALIGNMENT_SYNC_COUNT_MAX_WIDTH{1'b0}}, evgHeartbeatSel,
                        {4-ALIGNMENT_SYNC_COUNT_MAX_WIDTH{1'b0}}, alignCounterSel,
                        {4-ALIGNMENT_SYNC_COUNT_WIDTH{1'b0}}, alignCounterSelLatch,
                        {8-ALIGNMENT_SYNC_COUNT{1'b0}}, alignmentCounterSynced };

endmodule
