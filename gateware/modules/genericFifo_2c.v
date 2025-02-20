// Generic FIFO wrapper for bedrock/dsp/fifo.v
// with the option of non FWFT operation

module genericFifo_2c #(
    parameter aw = 3,
    parameter dw = 8,
    parameter fwft = 1
) (
    input wr_clk,
    input [dw - 1: 0] din,
    input we,
    output full,
    // -1: empty, 0: single element, 2**aw - 1: full
    output [aw:0] wr_count,

    input rd_clk,
    output [dw - 1: 0] dout,
    input re,
    output empty,
    // -1: empty, 0: single element, 2**aw - 1: full
    output [aw:0] rd_count
);

wire [dw - 1: 0] dout_;
fifo_2c #(
    .aw(aw),
    .dw(dw)
) fifo_2c (
    .wr_clk(wr_clk),
    .we(we),
    .din(din),
    .wr_count(wr_count),
    .full(full),

    .rd_clk(rd_clk),
    .re(re),
    .dout(dout_),
    .rd_count(rd_count),
    .empty(empty)
);

generate

if (fwft) begin
    assign dout = dout_;
end else begin

    reg [dw - 1: 0] dout_r;
    always @(posedge rd_clk) begin
        if (re)
            dout_r <= dout_;
    end

    assign dout = dout_r;
end
endgenerate

endmodule
