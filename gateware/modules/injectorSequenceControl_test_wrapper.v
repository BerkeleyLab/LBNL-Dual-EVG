module injectorSequenceControl_test_wrapper #(
    parameter SYSCLK_RATE                   = 100000000,
    // Number of BR/AR alignment periods per BR/AR coincidence
    parameter BR_AR_ALIGN_PER_BR_AR_COINC   = 125,
    // Number of RF coincidences per BR/AR coincidence
    parameter RF_COINC_PER_BR_AR_COINC      = 304,

    // Free running counters must divide evenly into heartbeat
    parameter CLK_PER_BR_ORBIT_CLOCK_DIV4   = 125,
    parameter CLK_PER_TICKS_COINCIDENCE     = 167,
    parameter CLK_PER_BR_AR_ALIGNMENT       = 167 * 304,
    parameter CLK_PER_BR_AR_COINCIDENCE     = 167 * 304 * 125,

    parameter ALT_CLK_PER_BR_AR_ALIGNMENT   = 125 * 82,

    parameter ALIGNMENT_SYNC_COUNT          = 2,

    parameter CLK_PER_HEARTBEAT             = 167 * 304 * 125,
    parameter ALT_CLK_PER_HEARTBEAT         = 125 * 82 * 1216,

    // DON'T CHANGE THESE
    parameter RF_COINC_IDX_WIDTH            = $clog2(RF_COINC_PER_BR_AR_COINC + 1),
    parameter RF_ALIGN_IDX_WIDTH            = $clog2(BR_AR_ALIGN_PER_BR_AR_COINC + 1),
    parameter CLK_PER_TICKS_COINC_WIDTH     = $clog2(CLK_PER_TICKS_COINCIDENCE + 1),
    parameter CLK_PER_BR_AR_COINC_WIDTH     = $clog2(CLK_PER_BR_AR_COINCIDENCE + 1),
    parameter CLK_PER_BR_ORBIT_DIV4_WIDTH   = $clog2(CLK_PER_BR_ORBIT_CLOCK_DIV4 + 1),
    parameter CLK_PER_BR_AR_ALIGN_WIDTH     = $clog2(CLK_PER_BR_AR_ALIGNMENT + 1)
) (
    input              sysClk,
    input       [31:0] sysGPIO_OUT,

    input              sysCsrStrobe,
    output wire [31:0] sysStatus,

    input              sysCsrAlignStrobe,
    output wire [31:0] sysAlignStatus,

    input              sysCsrTargetStrobe,
    output wire [31:0] sysTargetStatus,
    output wire [31:0] sysTargetStatus2,

    input                                   evgTxClk,

    output wire    [RF_COINC_IDX_WIDTH-1:0] evgRFCoincCountMon,
    output wire    [RF_ALIGN_IDX_WIDTH-1:0] evgRFAlignCountMon,
    output wire  [ALIGNMENT_SYNC_COUNT-1:0] evgAlignCounterDone,

    output wire        evgHeartbeatAlign,
    output wire        evgHeartbeatCore,
    output wire        evgSequenceStart
);

///////////////////////////////////////////////////////////////////////////////
// Heartbeat Generation via heartbeatGenerator

wire evgHeartbeatRequest;
wire [ALIGNMENT_SYNC_COUNT-1:0] evgHeartbeat;

heartbeatGenerator #(
    .TX_CLK_PER_HEARTBEAT(CLK_PER_HEARTBEAT)
) heartbeatGen (
    .sampCoincidenceMarker(1'b0),
    .sysRealignToggleIn(1'b0),
    .txClk(evgTxClk),
    .txCoincidenceMarker(),
    .txHeartbeatStrobe(evgHeartbeatRequest)
);

assign evgHeartbeat = {ALIGNMENT_SYNC_COUNT{evgHeartbeatRequest}};

///////////////////////////////////////////////////////////////////////////////
// PPS and 60Hz generation

wire sysPPSmarker;
wire sysPowerline;

clkIntervalCounters #(
    .CLK_RATE(SYSCLK_RATE),
    .WITH_POWERLINE_GEN("TRUE"))
  clkIntervalCounters (
    .clk(sysClk),
    .microsecondsSinceBoot(),
    .secondsSinceBoot(),
    .PPS(sysPPSmarker),
    .powerline(sysPowerline));

wire sysPowerlineTrigger;

powerlineTrigger #(
    .CLK_RATE(SYSCLK_RATE))
  powerlineTrigger (
    .clk(sysClk),
    .powerline_a(sysPowerline),
    .trigger(sysPowerlineTrigger));

///////////////////////////////////////////////////////////////////////////////
// evgCounters

localparam NUM_EVG_COUNTERS = 6;
wire [NUM_EVG_COUNTERS*32-1:0] clkGenCounters;
wire [NUM_EVG_COUNTERS-1:0]    clkGenStrobes;
wire [NUM_EVG_COUNTERS-1:0]    clkGenSynceds;

// Extract the counters we need to feed to injectorSequenceControl
wire [31:0] rfCoincPerArBrAlignCounter   = clkGenCounters[191:160]; // Index 5
wire [31:0] brArAlignPerBrArCoincCounter = clkGenCounters[159:128]; // Index 4

// Extract the specific strobes needed to drive the cascaded 'ens' enables
wire rfF1CoincStrobe = clkGenStrobes[3];
wire brArAlignStrobe = clkGenStrobes[0];

// Cascaded enables: Index 5 and 4 are enabled by the lower-level counter strobes
wire [NUM_EVG_COUNTERS-1:0] evgCountersEn = {rfF1CoincStrobe, brArAlignStrobe, 4'b1111};

evgCounters #(
    .SYSCLK_FREQUENCY(SYSCLK_RATE),
    .DEBUG("false"),
    .NUM_COUNTERS(NUM_EVG_COUNTERS),
    .COUNTER_WIDTHS({
        RF_COINC_IDX_WIDTH[31:0],
        RF_ALIGN_IDX_WIDTH[31:0],
        CLK_PER_TICKS_COINC_WIDTH[31:0],
        CLK_PER_BR_AR_COINC_WIDTH[31:0],
        CLK_PER_BR_ORBIT_DIV4_WIDTH[31:0],
        CLK_PER_BR_AR_ALIGN_WIDTH[31:0]
    }),
    .DEFAULT_RATE_COUNTS({
        RF_COINC_PER_BR_AR_COINC[31:0],
        BR_AR_ALIGN_PER_BR_AR_COINC[31:0],
        CLK_PER_TICKS_COINCIDENCE[31:0],
        CLK_PER_BR_AR_COINCIDENCE[31:0],
        CLK_PER_BR_ORBIT_CLOCK_DIV4[31:0],
        CLK_PER_BR_AR_ALIGNMENT[31:0]
    })
) evgCountersInst (
    .sysClk(sysClk),
    .GPIO_OUT(32'b0),
    .csrStrobes({NUM_EVG_COUNTERS{1'b0}}),
    .csrs(),
    .clk(evgTxClk),
    .ens(evgCountersEn),
    .heartbeatStrobe(evgHeartbeatAlign),
    .pulsePerSecondStrobe(1'b0),
    .clkGenSynceds(clkGenSynceds),
    .clkGens(),
    .clkGenStrobes(clkGenStrobes),
    .clkGenCounters(clkGenCounters)
);

///////////////////////////////////////////////////////////////////////////////
// DUT

injectorSequenceControl #(
    .SYSCLK_RATE(SYSCLK_RATE),
    .ALIGNMENT_SYNC_COUNT(ALIGNMENT_SYNC_COUNT),
    .RF_COINC_IDX_WIDTH(RF_COINC_IDX_WIDTH),
    .RF_ALIGN_IDX_WIDTH(RF_ALIGN_IDX_WIDTH),
    .RF_COINC_TERM_WIDTH(RF_ALIGN_IDX_WIDTH),
    .TX_CLK_PER_ALIGNMENT({
        ALT_CLK_PER_BR_AR_ALIGNMENT[31:0],
        CLK_PER_BR_AR_ALIGNMENT[31:0]
    })
) injectorSequenceControl (
    .sysClk(sysClk),
    .sysGPIO_OUT(sysGPIO_OUT),
    .sysCsrStrobe(sysCsrStrobe),
    .sysCsrAlignStrobe(sysCsrAlignStrobe),
    .sysCsrTargetStrobe(sysCsrTargetStrobe),

    .sysStatus(sysStatus),
    .sysAlignStatus(sysAlignStatus),
    .sysTargetStatus(sysTargetStatus),
    .sysTargetStatus2(sysTargetStatus2),
    .powerline_a(sysPowerlineTrigger),
    .evgTxClk(evgTxClk),

    .evgRFCoincCount(rfCoincPerArBrAlignCounter[RF_COINC_IDX_WIDTH-1:0]),
    .evgRFAlignCount(brArAlignPerBrArCoincCounter[RF_ALIGN_IDX_WIDTH-1:0]),

    .evgRFCoincCountMon(evgRFCoincCountMon),
    .evgRFAlignCountMon(evgRFAlignCountMon),

    .evgAlignCounterDone(evgAlignCounterDone),
    .evgHeartbeat(evgHeartbeat),
    .evgHeartbeatAlign(evgHeartbeatAlign),
    .evgHeartbeatCore(evgHeartbeatCore),
    .evgSequenceStart(evgSequenceStart)
);

endmodule
