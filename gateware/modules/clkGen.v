// Monitor EVR and generate related clocks
// Nets with names beginning with evr are in the EVR clock domain.

module clkGen #(
    parameter SYSCLK_FREQUENCY     = -1,
    parameter DEFAULT_RATE_COUNT   = -1,
    parameter DEBUG                = "false",
    parameter COUNTER_WIDTH        = -1) (
    input              sysClk,
    input              csrStrobe,
    input       [31:0] GPIO_OUT,
    output wire [31:0] csr,

    input                           clk,
    input                           en,
    (*mark_debug=DEBUG*) input      heartbeatStrobe,
    (*mark_debug=DEBUG*) input      pulsePerSecondStrobe,
    (*mark_debug=DEBUG*) output reg clkGenSynced = 0,
    (*mark_debug=DEBUG*) output reg clkGen = 0,
    (*mark_debug=DEBUG*) output reg clkGenStrobe = 0,
    (*mark_debug=DEBUG*) output
          [COUNTER_WIDTH-1:0]  clkGenCounter);

localparam COUNTER_WIDTH_MAX    = 24;
localparam COUNTER_HALF_WIDTH   = COUNTER_WIDTH - 1;

generate
if ($clog2(DEFAULT_RATE_COUNT+1) > COUNTER_WIDTH) begin
    DEFAULT_RATE_COUNT_bigger_than_COUNTER_WIDTH();
end
endgenerate

generate
if (COUNTER_WIDTH > COUNTER_WIDTH_MAX) begin
    COUNTER_WIDTH_bigger_than_COUNTER_WIDTH_MAX();
end
endgenerate

//////////////////////////////////////////////////////////////////////////////
// SYS CLK domain
//////////////////////////////////////////////////////////////////////////////

reg [COUNTER_WIDTH-1:0] sysClkDivisor = DEFAULT_RATE_COUNT;
(*mark_debug=DEBUG*)reg [COUNTER_HALF_WIDTH-1:0] reloadLo, reloadHi;
always @(posedge sysClk) begin
    if (csrStrobe) begin
        sysClkDivisor <= GPIO_OUT[8+:COUNTER_WIDTH];
    end
    reloadLo <= ((sysClkDivisor + 1) >> 1) - 1;
    reloadHi <= (sysClkDivisor >> 1) - 1;
end

wire heartBeatValid, pulsePerSecondValid;
assign csr = {{COUNTER_WIDTH_MAX-COUNTER_WIDTH{1'b0}}, sysClkDivisor,
              {8-3{1'b0}}, pulsePerSecondValid, heartBeatValid, clkGenSynced};

//////////////////////////////////////////////////////////////////////////////
// CLK domain
//////////////////////////////////////////////////////////////////////////////

(*mark_debug=DEBUG*)reg [COUNTER_WIDTH-1:0] fullCounter = 0;
(*mark_debug=DEBUG*)reg [COUNTER_HALF_WIDTH-1:0] counter = 0;
always @(posedge clk) begin
    if (heartbeatStrobe) begin
        clkGen <= 1;
        clkGenStrobe <= 0;
        counter <= reloadHi;
        fullCounter <= 0;
        clkGenSynced <= (!clkGen && (counter == 0));
    end
    else if (en) begin
        if (counter == 0) begin
            clkGen <= !clkGen;
            clkGenStrobe <= 0;

            if (clkGen) begin
                counter <= reloadLo;
                // Weird, but counter counts only half
                // of the whole divisor, so the up counter
                // needs to increment here too
                fullCounter <= fullCounter + 1;
            end
            else begin
                counter <= reloadHi;
                fullCounter <= 0;
            end
        end
        else begin
            clkGenStrobe <= 0;
            counter <= counter - 1;
            fullCounter <= fullCounter + 1;

            // assert strobe one clock before to match
            // other counter behaviors
            if (!clkGen && counter == 1) begin
                clkGenStrobe <= 1;
            end
        end
    end
    else begin
        clkGenStrobe <= 0;
    end
end

assign clkGenCounter = fullCounter;

pulseWatchdog #(
    .SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
    .PULSE_FREQUENCY(1),
    .DEBUG(DEBUG))
  hbWatchdog (
    .clk(clk),
    .pulseIn(heartbeatStrobe),
    .sysClk(sysClk),
    .isValid(heartBeatValid));

pulseWatchdog #(
    .SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
    .PULSE_FREQUENCY(1),
    .DEBUG(DEBUG))
  ppsWatchdog (
    .clk(clk),
    .pulseIn(pulsePerSecondStrobe),
    .sysClk(sysClk),
    .isValid(pulsePerSecondValid));

endmodule
