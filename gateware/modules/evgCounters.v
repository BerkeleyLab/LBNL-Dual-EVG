// Collection of counters for EVG

module evgCounters #(
    parameter SYSCLK_FREQUENCY      = -1,
    parameter DEBUG                 = "false",
    parameter NUM_COUNTERS          =  2,
    parameter [NUM_COUNTERS*32-1:0]
        COUNTER_WIDTHS = {-32'd1, -32'd1},
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
    (*mark_debug=DEBUG*) output [NUM_COUNTERS*32-1:0]   clkGenCounters);

genvar i;
generate
for (i = 0; i < NUM_COUNTERS; i = i + 1) begin

localparam DEFAULT_RATE_COUNT_LOCAL = DEFAULT_RATE_COUNTS[i*32+:32];
localparam COUNTER_WIDTH_LOCAL = COUNTER_WIDTHS[i*32+:32];

clkGen #(
    .SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
    .DEFAULT_RATE_COUNT(DEFAULT_RATE_COUNT_LOCAL),
    .COUNTER_WIDTH(COUNTER_WIDTH_LOCAL),
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

    .clkGenCounter(clkGenCounters[i*32+:COUNTER_WIDTH_LOCAL])
    );

assign clkGenCounters[(i+1)*32-1-:32-COUNTER_WIDTH_LOCAL] = 0;

end
endgenerate

endmodule
