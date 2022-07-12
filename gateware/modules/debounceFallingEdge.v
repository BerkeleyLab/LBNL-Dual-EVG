// Debounce a falling-edge signal

module debounceFallingEdge #(
    parameter HIGH_COUNTER_WIDTH = 6
    ) (
    input  clk,
    input  inputActiveLow,
    output debouncedActiveHigh);

localparam COUNTER_WIDTH = HIGH_COUNTER_WIDTH + 1;

reg [COUNTER_WIDTH-1:0] counter = 0;
assign debouncedActiveHigh = counter[COUNTER_WIDTH-1];

always @(posedge clk) begin
    if (inputActiveLow == 0) begin
        counter <= ~0;
    end
    else if (debouncedActiveHigh) begin
        counter <= counter - 1;
    end
end
endmodule
