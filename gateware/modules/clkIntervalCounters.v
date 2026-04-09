// Keep track of time using system clock

module clkIntervalCounters #(
    parameter CLK_RATE = 100000000,
    parameter WITH_POWERLINE_GEN = "FALSE"
    ) (
    input             clk,
    output reg [31:0] microsecondsSinceBoot,
    output reg [31:0] secondsSinceBoot,
    output wire       usecTick,
    output wire       secTick,
    output reg        PPS,
    output reg        powerline);

localparam USEC_DIVIDER_WIDTH = $clog2((CLK_RATE/1000000) - 1);
reg [USEC_DIVIDER_WIDTH:0] usecDivider = (CLK_RATE/1000000) - 2;
assign usecTick = usecDivider[USEC_DIVIDER_WIDTH];

localparam SEC_DIVIDER_WIDTH = $clog2(1000000 - 1);
reg [SEC_DIVIDER_WIDTH:0] secDivider = 1000000 - 2;
assign secTick = secDivider[SEC_DIVIDER_WIDTH];

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

generate
if (WITH_POWERLINE_GEN != "TRUE" && WITH_POWERLINE_GEN != "FALSE") begin
    WITH_POWERLINE_GEN_only_TRUE_or_FALSE_SUPPORTED err();
end
endgenerate

generate
if (WITH_POWERLINE_GEN == "TRUE") begin

// 60Hz is 1/60*1e6 ~= 16667us
localparam POWERLINE_DIVIDER_WIDTH = $clog2(16667 - 1);
reg [POWERLINE_DIVIDER_WIDTH:0] powerlineDivider = 16667 - 2;
wire powerlineTick = powerlineDivider[POWERLINE_DIVIDER_WIDTH];

always @(posedge clk) begin
    if(usecTick) begin
        if (powerlineTick) begin
            powerlineDivider <= 16667 - 2;
            powerline <= 1;
        end
        else begin
            powerlineDivider <= powerlineDivider - 1;
            powerline <= 0;
        end
    end
end

end
endgenerate

endmodule
