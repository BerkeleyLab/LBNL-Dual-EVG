// Monitor EVR and generate related clocks
// Nets with names beginning with evr are in the EVR clock domain.

module clkGen #(
    parameter SYSCLK_FREQUENCY      = -1,
    parameter DEFAULT_RATE_COUNT    = -1,
    parameter DEBUG                 = "false",
    // Don't change these
    parameter DIVISOR_WIDTH        = 24,
    parameter COUNTER_WIDTH        = DIVISOR_WIDTH - 1,
    parameter FULL_COUNTER_WIDTH   = DIVISOR_WIDTH) (
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
          [FULL_COUNTER_WIDTH-1:0]  clkGenCounter);

generate
if ($clog2(DEFAULT_RATE_COUNT+1) > DIVISOR_WIDTH) begin
    DEFAULT_RATE_COUNT_bigger_than_DIVISOR_WIDTH();
end
endgenerate

//////////////////////////////////////////////////////////////////////////////
// SYS CLK domain
//////////////////////////////////////////////////////////////////////////////

reg [DIVISOR_WIDTH-1:0] sysClkDivisor = DEFAULT_RATE_COUNT;
(*mark_debug=DEBUG*)reg [COUNTER_WIDTH-1:0] reloadLo, reloadHi;
always @(posedge sysClk) begin
    if (csrStrobe) begin
        sysClkDivisor <= GPIO_OUT[8+:DIVISOR_WIDTH];
    end
    reloadLo <= ((sysClkDivisor + 1) >> 1) - 1;
    reloadHi <= (sysClkDivisor >> 1) - 1;
end

wire heartBeatValid, pulsePerSecondValid;
assign csr = {sysClkDivisor,
              {8-3{1'b0}}, pulsePerSecondValid, heartBeatValid, clkGenSynced};

//////////////////////////////////////////////////////////////////////////////
// CLK domain
//////////////////////////////////////////////////////////////////////////////

(*mark_debug=DEBUG*)reg [FULL_COUNTER_WIDTH-1:0] fullCounter = 0;
(*mark_debug=DEBUG*)reg [COUNTER_WIDTH-1:0] counter = 0;
always @(posedge clk) begin
    if (en) begin
        if (heartbeatStrobe) begin
            clkGen <= 1;
            clkGenStrobe <= 1;
            counter <= reloadHi;
            fullCounter <= 0;
            clkGenSynced <= (!clkGen && (counter == 0));
        end
        else begin
            if (counter == 0) begin
                clkGen <= !clkGen;

                if (clkGen) begin
                    clkGenStrobe <= 0;
                    counter <= reloadLo;
                end
                else begin
                    clkGenStrobe <= 1;
                    counter <= reloadHi;
                    fullCounter <= 0;
                end
            end
            else begin
                clkGenStrobe <= 0;
                counter <= counter - 1;
                fullCounter <= fullCounter + 1;
            end
        end
    end
    else begin
        if (heartbeatStrobe) begin
            clkGenSynced <= 0;
        end
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
