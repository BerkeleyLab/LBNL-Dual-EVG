// Keep track of time using system clock

module clkIntervalCounters #(
    parameter CLK_RATE = 100000000
    ) (
    input             clk,
    output reg [31:0] microsecondsSinceBoot,
    output reg [31:0] secondsSinceBoot,
    output reg        PPS);

localparam USEC_DIVIDER_WIDTH = $clog2((CLK_RATE/1000000) - 1);
reg [USEC_DIVIDER_WIDTH:0] usecDivider = (CLK_RATE/1000000) - 2;
wire usecTick = usecDivider[USEC_DIVIDER_WIDTH];

localparam SEC_DIVIDER_WIDTH = $clog2(1000000 - 1);
reg [SEC_DIVIDER_WIDTH:0] secDivider = 1000000 - 2;
wire secTick = secDivider[SEC_DIVIDER_WIDTH];

always @(posedge clk) begin
    if (usecTick) begin
        usecDivider <= (CLK_RATE/1000000) - 2;
        microsecondsSinceBoot <= microsecondsSinceBoot + 1;
        if (secTick) begin
            secDivider <= 1000000 - 2;
            secondsSinceBoot <= secondsSinceBoot + 1;
            PPS <= 1;
        end
        else begin
            secDivider <= secDivider - 1;
            PPS <= 0;
        end
    end
    else begin
        usecDivider <= usecDivider - 1;
    end
end

endmodule
