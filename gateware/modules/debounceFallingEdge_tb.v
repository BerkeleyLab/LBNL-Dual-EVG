`timescale 1ns / 1ns
module debounceFallingEdge_tb;

reg clk;

integer cc;
integer errors = 0;
initial begin
    if ($test$plusargs("vcd")) begin
        $dumpfile("debounceFallingEdge.vcd");
        $dumpvars(5,debounceFallingEdge_tb);
    end

    clk = 0;
    for (cc = 0; cc < 5000; cc = cc+1) begin
        #10; clk = 1;
        #11; clk = 0;
    end

    if (errors == 0)
        $display("PASS");
end

reg inputActiveLow = 1;
wire debouncedActiveHigh;
localparam HIGH_COUNTER_WIDTH = 6;
debounceFallingEdge #(
    .HIGH_COUNTER_WIDTH(HIGH_COUNTER_WIDTH))
  debounceFallingEdge(
    .clk(clk),
    .inputActiveLow(inputActiveLow),
    .debouncedActiveHigh(debouncedActiveHigh));

integer counterInput = 0;
integer counterDebouncedActiveHigh = 0;
integer rightAnswer = 0;
reg fault;
localparam IDLE             = 2'd0,
           CNT_EDGES        = 2'd1,
           CMP_RESULTS      = 2'd2;
reg [1:0] testState = IDLE;
always @(posedge clk) begin
    inputActiveLow <= !(cc == 100 || cc == 200 + (cc % 10));

    case(testState)
    IDLE: begin
        if(!inputActiveLow)
            testState <= CNT_EDGES;
            counterInput <= 1;
            counterDebouncedActiveHigh <= 0;
    end

    CNT_EDGES: begin
        if (!inputActiveLow) begin
            counterInput <= counterInput + 1;
        end

        if (debouncedActiveHigh) begin
            counterDebouncedActiveHigh <= counterDebouncedActiveHigh + 1;
        end else begin
            testState <= CMP_RESULTS;
        end
    end

    // expected result is the width of input + 2<<HIGH_COUNTER_WIDTH
    CMP_RESULTS: begin
        rightAnswer = counterInput + (1<<HIGH_COUNTER_WIDTH)-1;
        fault = rightAnswer != counterDebouncedActiveHigh;
        $display("%x  %x  %s", rightAnswer, counterDebouncedActiveHigh, fault?"FAULT":"    .");
        if (fault)
            errors = errors+1;

        testState <= IDLE;
    end
    endcase
end

endmodule
