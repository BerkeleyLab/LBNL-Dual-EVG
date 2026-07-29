// Stream event content to PMOD
// application specific module

module streamToPmod #(
    parameter TIMEOUT_CYCLES = 62500000,
    parameter [3:0] MSG_HEADER = 4'b1010
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [7:0]  dataIn,
    input  wire        aligned,
    input  wire        charIsK,
    output reg  [7:0]  dataOut
);

localparam TIMEOUT_WIDTH = $clog2(TIMEOUT_CYCLES+1);
reg [TIMEOUT_WIDTH-1:0] timeoutCnt = TIMEOUT_CYCLES;
wire timeout = timeoutCnt == 0;
wire header_matched = (dataIn[7:4] == MSG_HEADER);

always @(posedge clk) begin
    if (rst) begin
        timeoutCnt <= TIMEOUT_CYCLES;
        dataOut    <= 8'h00;
    end else begin
        if (aligned & ~charIsK & header_matched) begin
            timeoutCnt <= TIMEOUT_CYCLES;
        end else begin
            if (!timeout) begin
                timeoutCnt <= timeoutCnt - 1;
            end
        end

        if(timeout || !aligned) begin
            dataOut <= 0;
        end else if(!charIsK & header_matched) begin
            dataOut <= dataIn;
        end
    end
end

endmodule
