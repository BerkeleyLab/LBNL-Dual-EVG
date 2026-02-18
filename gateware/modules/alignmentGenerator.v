// Generate alignment strobes
module alignmentGenerator #(
    parameter CLK_PER_ALIGNMENT = -1
    ) (
    input       clk,

    input       heartbeatStrobe,
    output      alignmentCounterDone,
    output reg  alignmentCounterSynced = 0);

// Produce alignment marker
localparam ALIGNMENT_COUNTER_RELOAD = CLK_PER_ALIGNMENT - 2;
localparam ALIGNMENT_COUNTER_WIDTH = $clog2(ALIGNMENT_COUNTER_RELOAD+1)+1;
reg [ALIGNMENT_COUNTER_WIDTH-1:0] alignmentCounter;

always @(posedge clk) begin
    if (heartbeatStrobe) begin
        alignmentCounterSynced <= alignmentCounterDone;
        alignmentCounter <= ALIGNMENT_COUNTER_RELOAD;
    end
    else if (alignmentCounterDone) begin
        alignmentCounter <= ALIGNMENT_COUNTER_RELOAD;
    end
    else begin
        alignmentCounter <= alignmentCounter - 1;
    end
end

assign alignmentCounterDone = alignmentCounter[ALIGNMENT_COUNTER_WIDTH-1];

endmodule
