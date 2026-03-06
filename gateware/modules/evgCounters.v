// Collection of counters for EVG

module evgCounters #(
    parameter SYSCLK_FREQUENCY      = -1,
    parameter DEBUG                 = "false",
    parameter NUM_COUNTERS          =  2,
    parameter [NUM_COUNTERS*32-1:0]
        DEFAULT_RATE_COUNTS = {-32'd1, -32'd1}) (
    input              sysClk,
    input       [31:0] GPIO_OUT,

    input          [NUM_COUNTERS-1:0] csrStrobes,
    output wire [NUM_COUNTERS*32-1:0] csrs,

    input                           clk,
    input        [NUM_COUNTERS-1:0] ens,
    (*mark_debug=DEBUG*) input      heartbeatStrobe,
    (*mark_debug=DEBUG*) input      pulsePerSecondStrobe,

    (*mark_debug=DEBUG*) output [NUM_COUNTERS-1:0]      clkGenSynceds,
    (*mark_debug=DEBUG*) output [NUM_COUNTERS-1:0]      clkGens,
    (*mark_debug=DEBUG*) output [NUM_COUNTERS-1:0]      clkGenStrobes,
    (*mark_debug=DEBUG*) output [NUM_COUNTERS*24-1:0]   clkGenCounters);

genvar i;
generate
for (i = 0; i < NUM_COUNTERS; i = i + 1) begin

localparam DEFAULT_RATE_COUNT_LOCAL = DEFAULT_RATE_COUNTS[i*32+:32];

clkGen #(
    .SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
    .DEFAULT_RATE_COUNT(DEFAULT_RATE_COUNT_LOCAL),
    .DEBUG(DEBUG))
  clkGen (
    .sysClk(sysClk),
    .csrStrobe(csrStrobes[i]),
    .GPIO_OUT(GPIO_OUT),
    .csr(csrs[i*32+:32]),

    .clk(clk),
    .en(ens[i]),
    .heartbeatStrobe(heartbeatStrobe),
    .pulsePerSecondStrobe(pulsePerSecondStrobe),
    .clkGenSynced(clkGenSynceds[i]),
    .clkGen(clkGens[i]),
    .clkGenStrobe(clkGenStrobes[i]),

    .clkGenCounter(clkGenCounters[i*24+:24])
    );

end
endgenerate

endmodule
