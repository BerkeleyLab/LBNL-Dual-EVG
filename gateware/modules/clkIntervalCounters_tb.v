`timescale 1ns / 1ns
module clkIntervalCounters_tb;

reg clk;

integer cc;
integer errors = 0;
initial begin
    if ($test$plusargs("vcd")) begin
        $dumpfile("clkIntervalCounters.vcd");
        $dumpvars(5,clkIntervalCounters_tb);
    end

    clk = 0;
    for (cc = 0; cc < 50000; cc = cc+1) begin
        #10; clk = 1;
        #11; clk = 0;
    end

    if (errors == 0)
        $display("PASS");
end

wire [31:0] microsecondsSinceBoot;
wire [31:0] secondsSinceBoot;
wire PPS;

clkIntervalCounters #(
    .CLK_RATE(100),
    .SIMULATION("true"))
  clkIntervalCounters(
    .clk(clk),
    .microsecondsSinceBoot(microsecondsSinceBoot),
    .secondsSinceBoot(secondsSinceBoot),
    .PPS(PPS)
);

endmodule
