// Monitor EVR and generate related clocks
// Nets with names beginning with evr are in the EVR clock domain.

module clkGen #(
    parameter SYSCLK_FREQUENCY      = -1,
    parameter DEFAULT_RATE_COUNT    = -1,
    parameter DEBUG                 = "false"
    ) (
    input              sysClk,
    input              csrStrobe,
    input       [31:0] GPIO_OUT,
    output wire [31:0] csr,

    input                           clk,
    (*mark_debug=DEBUG*) input      heartbeatMarker,
    (*mark_debug=DEBUG*) input      pulsePerSecondMarker,
    (*mark_debug=DEBUG*) output reg clkGenSynced = 0,
    (*mark_debug=DEBUG*) output reg clkGen = 0,
    (*mark_debug=DEBUG*) output reg clkGenStrobe = 0);

localparam DIVISOR_WIDTH = 24;
localparam COUNTER_WIDTH = DIVISOR_WIDTH - 1;

generate
if ($clog2(DEFAULT_RATE_COUNT+1) > DIVISOR_WIDTH) begin
    DEFAULT_RATE_COUNT_bigger_than_DIVISOR_WIDTH();
end
endgenerate

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

(*mark_debug=DEBUG*)reg [COUNTER_WIDTH-1:0] counter = 0;
reg heartbeatMarker_d;
always @(posedge clk) begin
    heartbeatMarker_d <= heartbeatMarker;
    if (heartbeatMarker && !heartbeatMarker_d) begin
        clkGen <= 1;
        clkGenStrobe <= 1;
        counter <= reloadHi;
        clkGenSynced <= (!clkGen && (counter == 0));
    end
    else if (counter == 0) begin
        clkGen <= !clkGen;
        if (clkGen) begin
            clkGenStrobe <= 0;
            counter <= reloadLo;
        end
        else begin
            clkGenStrobe <= 1;
            counter <= reloadHi;
        end
    end else begin
        clkGenStrobe <= 0;
        counter <= counter - 1;
    end
end

markerWatchdog #(.SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
                      .DEBUG(DEBUG))
  hbWatchdog (.sysClk(sysClk),
              .markerIn(heartbeatMarker),
              .isValid(heartBeatValid));

markerWatchdog #(.SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
                      .DEBUG(DEBUG))
  ppsWatchdog (.sysClk(sysClk),
               .markerIn(pulsePerSecondMarker),
               .isValid(pulsePerSecondValid));
endmodule
