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

    input                           evrClk,
    (*mark_debug=DEBUG*) input      evrHeartbeatMarker,
    (*mark_debug=DEBUG*) input      evrPulsePerSecondMarker,
    (*mark_debug=DEBUG*) output reg evrClkGenSynced = 0,
    (*mark_debug=DEBUG*) output reg evrClkGen = 0,
    (*mark_debug=DEBUG*) output reg evrClkGenStrobe = 0);

localparam DIVISOR_WIDTH = 24;
localparam COUNTER_WIDTH = DIVISOR_WIDTH - 1;

generate
if ($clog2(DEFAULT_RATE_COUNT+1) > DIVISOR_WIDTH) begin
    DEFAULT_EVR_RATE_COUNT_bigger_than_DIVISOR_WIDTH();
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
              {8-3{1'b0}}, pulsePerSecondValid, heartBeatValid, evrClkGenSynced};

(*mark_debug=DEBUG*)reg [COUNTER_WIDTH-1:0] evrCounter = 0;
reg evrHeartbeatMarker_d;
always @(posedge evrClk) begin
    evrHeartbeatMarker_d <= evrHeartbeatMarker;
    if (evrHeartbeatMarker && !evrHeartbeatMarker_d) begin
        evrClkGen <= 1;
        evrClkGenStrobe <= 1;
        evrCounter <= reloadHi;
        evrClkGenSynced <= (!evrClkGen && (evrCounter == 0));
    end
    else if (evrCounter == 0) begin
        evrClkGen <= !evrClkGen;
        if (evrClkGen) begin
            evrClkGenStrobe <= 0;
            evrCounter <= reloadLo;
        end
        else begin
            evrClkGenStrobe <= 1;
            evrCounter <= reloadHi;
        end
    end else begin
        evrClkGenStrobe <= 0;
        evrCounter <= evrCounter - 1;
    end
end

markerWatchdog #(.SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
                      .DEBUG(DEBUG))
  hbWatchdog (.sysClk(sysClk),
              .evrMarker(evrHeartbeatMarker),
              .isValid(heartBeatValid));

markerWatchdog #(.SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
                      .DEBUG(DEBUG))
  ppsWatchdog (.sysClk(sysClk),
               .evrMarker(evrPulsePerSecondMarker),
               .isValid(pulsePerSecondValid));
endmodule
