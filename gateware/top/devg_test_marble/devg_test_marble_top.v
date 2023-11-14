// Top level module for ALSU Event Generator

module devg_test_marble_top #(
    // Include file is machine generated from C header
    `include "gpioIDX.vh"
    parameter ILA_CHIPSCOPE_DBG       = "FALSE",
    parameter FALLBACK_TO_SYS_PPS     = "FALSE",
    parameter SYSCLK_FREQUENCY        = 100000000,
    parameter TXCLK_NOMINAL_FREQUENCY = 125000000
    ) (
    input  DDR_REF_CLK_P, DDR_REF_CLK_N,
    output VCXO_EN,
    output PHY_RSTN,

    output wire BOOT_CS_B,
    output wire BOOT_MOSI,
    input       BOOT_MISO,

    input            RGMII_RX_CLK,
    input            RGMII_RX_CTRL,
    input      [3:0] RGMII_RXD,
    output wire      RGMII_TX_CLK,
    output wire      RGMII_TX_CTRL,
    output wire[3:0] RGMII_TXD,

    input  MGT_CLK_0_P, MGT_CLK_0_N,
    input  MGT_CLK_1_P, MGT_CLK_1_N,
    output QSFP1_TX_1_P, QSFP1_TX_1_N,
    input  QSFP1_RX_1_P, QSFP1_RX_1_N,
    output QSFP1_TX_2_P, QSFP1_TX_2_N,
    input  QSFP1_RX_2_P, QSFP1_RX_2_N,
    output QSFP1_TX_3_P, QSFP1_TX_3_N,
    input  QSFP1_RX_3_P, QSFP1_RX_3_N,
    output QSFP1_TX_4_P, QSFP1_TX_4_N,
    input  QSFP1_RX_4_P, QSFP1_RX_4_N,

    input  MGT_CLK_2_P, MGT_CLK_2_N,
    input  MGT_CLK_3_P, MGT_CLK_3_N,
    output QSFP2_TX_1_P, QSFP2_TX_1_N,
    input  QSFP2_RX_1_P, QSFP2_RX_1_N,
    output QSFP2_TX_2_P, QSFP2_TX_2_N,
    input  QSFP2_RX_2_P, QSFP2_RX_2_N,
    output QSFP2_TX_3_P, QSFP2_TX_3_N,
    input  QSFP2_RX_3_P, QSFP2_RX_3_N,
    output QSFP2_TX_4_P, QSFP2_TX_4_N,
    input  QSFP2_RX_4_P, QSFP2_RX_4_N,

    input  EXT0_CLK_P, EXT0_CLK_N,
    input  EXT1_CLK_P, EXT1_CLK_N,

    input  FPGA_SCLK,
    input  FPGA_CSB,
    input  FPGA_MOSI,
    output FPGA_MISO,

    // BNC board
    inout   PMOD1_0,
    inout   PMOD1_1,
    inout   PMOD1_2,
    inout   PMOD1_3,
    output  PMOD1_4,
    output  PMOD1_5,
    output  PMOD1_6,
    output  PMOD1_7,

    // LED board
    output PMOD2_0,
    output PMOD2_1,
    output PMOD2_2,
    output PMOD2_3,
    output PMOD2_4,
    output PMOD2_5,

    // Buttons
    output PMOD2_6,
    output PMOD2_7,

    output TWI_SCL,
    inout  TWI_SDA,

    // The RxD and TxD directions are with respect
    // to the USB/UART chip, not the FPGA!
    output FPGA_RxD,
    input  FPGA_TxD);

localparam TOD_SECONDS_WIDTH     = 32;
localparam DISTRIBUTED_BUS_WIDTH = 8;
localparam GPIO_WIDTH            = 32;
localparam DRP_DATA_WIDTH        = 16;

// Sanity check
if ((CFG_EVG1_CLK_PER_HEARTBEAT % CFG_EVG1_CLK_PER_BR_AR_ALIGNMENT)!=0)
    CFG_EVG1_CLK_PER_BR_AR_ALIGNMENT_BAD();

///////////////////////////////////////////////////////////////////////////////
assign VCXO_EN = 0;
assign PHY_RSTN = 1;

///////////////////////////////////////////////////////////////////////////////
// Clocks
wire sysClk, refClk125, refClk125d90, clkLatencySampler;
wire ethernetRxClk, ethernetTxClk;
wire evg1RefClk, evg2RefClk;
wire evg1TxClk, evg2TxClk;

///////////////////////////////////////////////////////////////////////////////
// Resets
wire sysReset_n;
wire DUMMY1_sysReset_n = sysReset_n;
wire DUMMY2_sysReset_n = sysReset_n;

//////////////////////////////////////////////////////////////////////////////
// General-purpose I/O block
wire                    [31:0] GPIO_IN[0:GPIO_IDX_COUNT-1];
wire                    [31:0] GPIO_OUT;
wire      [GPIO_IDX_COUNT-1:0] GPIO_STROBES;
wire [(GPIO_IDX_COUNT*32)-1:0] GPIO_IN_FLATTENED;
genvar i;
generate
for (i = 0 ; i < GPIO_IDX_COUNT ; i = i + 1) begin : gpio_flatten
    assign GPIO_IN_FLATTENED[ (i*32)+31 : (i*32)+0 ] = GPIO_IN[i];
end
endgenerate

`include "firmwareBuildDate.v"
assign GPIO_IN[GPIO_IDX_FIRMWARE_BUILD_DATE] = FIRMWARE_BUILD_DATE;
`include "gitHash.vh"
assign GPIO_IN[GPIO_IDX_GITHASH] = GIT_REV_32BIT;

//////////////////////////////////////////////////////////////////////////////
// Front panel controls
(*ASYNC_REG="TRUE"*) reg Reset_RecoveryModeSwitch_m, DisplayModeSwitch_m;
reg Reset_RecoveryModeSwitch, DisplayModeSwitch;
always @(posedge sysClk) begin
    Reset_RecoveryModeSwitch_m <= !PMOD2_6;
    DisplayModeSwitch_m        <= !PMOD2_7;
    Reset_RecoveryModeSwitch   <= Reset_RecoveryModeSwitch_m;
    DisplayModeSwitch          <= DisplayModeSwitch_m;
end
assign GPIO_IN[GPIO_IDX_USER_GPIO_CSR] = {
        Reset_RecoveryModeSwitch, DisplayModeSwitch,
        28'b0, gpsPPSvalid, bncPPSvalid };

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Boot Flash
wire spiFlashClk;
`ifndef SIMULATE
STARTUPE2 aspiClkPin(.USRCCLKO(spiFlashClk), .USRCCLKTS(1'b0));
`endif // `ifndef SIMULATE
// Trivial bit-banging connection to bootstrap flash memory
spiFlashBitBang #(.DEBUG("false"))
  spiFlash_i (
    .sysClk(sysClk),
    .sysGPIO_OUT(GPIO_OUT),
    .sysCSRstrobe(GPIO_STROBES[GPIO_IDX_QSPI_FLASH_CSR]),
    .sysStatus(GPIO_IN[GPIO_IDX_QSPI_FLASH_CSR]),
    .spiFlashClk(spiFlashClk),
    .spiFlashMOSI(BOOT_MOSI),
    .spiFlashCS_B(BOOT_CS_B),
    .spiFlashMISO(BOOT_MISO));

///////////////////////////////////////////////////////////////////////////////
// Microcontroller I/O

mmcMailbox #(.DEBUG("false"))
  mmcMailbox (
    .clk(sysClk),
    .GPIO_OUT(GPIO_OUT),
    .GPIO_STROBE(GPIO_STROBES[GPIO_IDX_MMC_MAILBOX]),
    .csr(GPIO_IN[GPIO_IDX_MMC_MAILBOX]),
    .SCLK(FPGA_SCLK),
    .CSB(FPGA_CSB),
    .MOSI(FPGA_MOSI),
    .MISO(FPGA_MISO));

///////////////////////////////////////////////////////////////////////////////
// Coincidence detection
wire EXT0_CLK, EXT1_CLK;
IBUFDS_GTE2 f1IBUF (
    .I(EXT0_CLK_P),
    .IB(EXT0_CLK_N),
    .CEB(1'b0),
    .O(EXT0_CLK));
IBUFDS_GTE2 f2IBUF (
    .I(EXT1_CLK_P),
    .IB(EXT1_CLK_N),
    .CEB(1'b0),
    .O(EXT1_CLK));

BUFG f1BUFG (.I(EXT0_CLK), .O(evg1RefClk));
BUFG f2BUFG (.I(EXT1_CLK), .O(evg2RefClk));

wire evg1HeartbeatRequest, evg2HeartbeatRequest;

coincidenceRecorder #(
    .CHANNEL_COUNT(2),
    .CYCLES_PER_ACQUISITION(1023),
    .SAMPLE_CLKS_PER_COINCIDENCE(CFG_EVG2_CLK_PER_RF_COINCIDENCE),
    .INPUT_CYCLES_PER_COINCIDENCE(CFG_EVG1_CLK_PER_RF_COINCIDENCE),
    .TX_CLK_PER_HEARTBEAT(CFG_EVG1_CLK_PER_HEARTBEAT))
  coincidenceRecorder1 (
    .sysClk(sysClk),
    .sysCsrStrobe(GPIO_STROBES[GPIO_IDX_EVG_1_COINC_CSR]),
    .sysGPIO_OUT(GPIO_OUT),
    .sysCsr(GPIO_IN[GPIO_IDX_EVG_1_COINC_CSR]),
    .samplingClk(evg2RefClk),
    .value_a({evg1RefClk, evg1TxClk}),
    .txClk(evg1TxClk),
    .txHeartbeatStrobe(evg1HeartbeatRequest));

coincidenceRecorder #(
    .CHANNEL_COUNT(2),
    .CYCLES_PER_ACQUISITION(1023),
    .SAMPLE_CLKS_PER_COINCIDENCE(CFG_EVG1_CLK_PER_RF_COINCIDENCE),
    .INPUT_CYCLES_PER_COINCIDENCE(CFG_EVG2_CLK_PER_RF_COINCIDENCE),
    .TX_CLK_PER_HEARTBEAT(CFG_EVG2_CLK_PER_HEARTBEAT))
  coincidenceRecorder2 (
    .sysClk(sysClk),
    .sysCsrStrobe(GPIO_STROBES[GPIO_IDX_EVG_2_COINC_CSR]),
    .sysGPIO_OUT(GPIO_OUT),
    .sysCsr(GPIO_IN[GPIO_IDX_EVG_2_COINC_CSR]),
    .samplingClk(evg1RefClk),
    .value_a({evg2RefClk, evg2TxClk}),
    .txClk(evg2TxClk),
    .txHeartbeatStrobe(evg2HeartbeatRequest));

//////////////////////////////////////////////////////////////////////////////
// Debounce timing markers
wire bncPowerline_a;
wire powerlineMarker;
debounceFallingEdge debouncePowerline (
    .clk(sysClk),
    .inputActiveLow(bncPowerline_a),
    .debouncedActiveHigh(powerlineMarker));

//////////////////////////////////////////////////////////////////////////////
// I2C
// Three channel version a holdover from Marble Mini layout, but keep
// the extra two channels as dummies until the IIC command table has
// been updated.
wire [2:0] sda_drive, sda_sense;
wire [3:0] iic_proc_o;
wire DUMMY2_SFP_SCL, DUMMY1_SFP_SCL;
wire scl0;
i2cHandler #(.CLK_RATE(SYSCLK_FREQUENCY),
             .CHANNEL_COUNT(3),
             .DEBUG("false"))
  i2cHandler (
    .clk(sysClk),
    .csrStrobe(GPIO_STROBES[GPIO_IDX_I2C_CHUNK_CSR]),
    .GPIO_OUT(GPIO_OUT),
    .status(GPIO_IN[GPIO_IDX_I2C_CHUNK_CSR]),
    .scl({DUMMY2_SFP_SCL, DUMMY1_SFP_SCL, scl0}),
    .sda_drive(sda_drive),
    .sda_sense(sda_sense));
IOBUF sdaIO0 (.I(1'b0),
              .IO(TWI_SDA),
              .O(sda_sense[0]),
              .T(iic_proc_o[2] ? iic_proc_o[1] : sda_drive[0]));
assign sda_sense[2:1] = 0;
assign TWI_SCL = iic_proc_o[2] ? iic_proc_o[0] : scl0;
wire [3:0] iic_proc_i = { sda_sense[0], iic_proc_o[2:0] };

//////////////////////////////////////////////////////////////////////////////
// Timekeeping
wire sysPPSmarker;
clkIntervalCounters #(.CLK_RATE(SYSCLK_FREQUENCY))
  clkIntervalCounters (
    .clk(sysClk),
    .microsecondsSinceBoot(GPIO_IN[GPIO_IDX_MICROSECONDS_SINCE_BOOT]),
    .secondsSinceBoot(GPIO_IN[GPIO_IDX_SECONDS_SINCE_BOOT]),
    .PPS(sysPPSmarker));

//////////////////////////////////////////////////////////////////////////////
// Validate PPS signal sources
wire bncPPS_a;
wire bncPPSvalid;
ppsCheck #(.CLK_RATE(SYSCLK_FREQUENCY)) bncPPScheck (
    .clk(sysClk),
    .pps_a(bncPPS_a),
    .ppsValid(bncPPSvalid));

wire gpsPPS_a = 1'b0;
wire gpsPPSvalid;
ppsCheck #(.CLK_RATE(SYSCLK_FREQUENCY)) gpsPPScheck (
    .clk(sysClk),
    .pps_a(gpsPPS_a),
    .ppsValid(gpsPPSvalid));

generate
if (FALLBACK_TO_SYS_PPS != "TRUE" && FALLBACK_TO_SYS_PPS != "FALSE") begin
    FALLBACK_TO_SYS_PPS_only_TRUE_or_FALSE_SUPPORTED();
end
endgenerate

wire bestPPS_a;
generate
if (FALLBACK_TO_SYS_PPS == "TRUE") begin

assign bestPPS_a = bncPPSvalid ? bncPPS_a : (gpsPPSvalid ? gpsPPS_a : sysPPSmarker);

end
endgenerate

generate
if (FALLBACK_TO_SYS_PPS == "FALSE") begin

assign bestPPS_a = bncPPSvalid ? bncPPS_a : gpsPPS_a;

end
endgenerate

//////////////////////////////////////////////////////////////////////////////
// NTP server support
wire ppsToggle, ppsMarker, ppsMarkerValid;
wire [31:0] posixSeconds, ntpStatusReg;
ntpClock #(.CLK_RATE(SYSCLK_FREQUENCY),
           .DEBUG("false"))
  ntpClock (
    .clk(sysClk),
    .writeStrobe(GPIO_STROBES[GPIO_IDX_NTP_SERVER_SECONDS]),
    .writeData(GPIO_OUT),
    .pps_a(bestPPS_a),
    .ppsToggle(ppsToggle),
    .ppsMarker(ppsMarker),
    .seconds(GPIO_IN[GPIO_IDX_NTP_SERVER_SECONDS]),
    .fraction(GPIO_IN[GPIO_IDX_NTP_SERVER_FRACTION]),
    .posixSeconds(posixSeconds),
    .status(ntpStatusReg));

assign GPIO_IN[GPIO_IDX_NTP_SERVER_STATUS] = ntpStatusReg;
assign ppsMarkerValid = ntpStatusReg[0];

/////////////////////////////////////////////////////////////////////////////
// First generator (injector)
wire injectorSequenceStart;
wire [15:0] evg1TxData;
wire  [1:0] evg1TxCharIsK;
injectorSequenceControl #(
    .SYSCLK_RATE(SYSCLK_FREQUENCY),
    .TXCLK_PER_BR_AR_ALIGNMENT(CFG_EVG1_CLK_PER_BR_AR_ALIGNMENT))
  injectorSequenceControl (
    .sysClk(sysClk),
    .sysCsrStrobe(GPIO_STROBES[GPIO_IDX_INJECTION_CYCLE_CSR]),
    .sysGPIO_OUT(GPIO_OUT),
    .sysStatus(GPIO_IN[GPIO_IDX_INJECTION_CYCLE_CSR]),
    .powerline_a(powerlineMarker),
    .evgTxClk(evg1TxClk),
    .evgHeartbeat(evg1HeartbeatRequest),
    .evgSequenceStart(injectorSequenceStart));

wire [3:0] qsfp1RxP = {QSFP1_RX_4_P, QSFP1_RX_3_P, QSFP1_RX_2_P, QSFP1_RX_1_P};
wire [3:0] qsfp1RxN = {QSFP1_RX_4_N, QSFP1_RX_3_N, QSFP1_RX_2_N, QSFP1_RX_1_N};
wire [3:0] qsfp1TxP;
wire [3:0] qsfp1TxN;
assign {QSFP1_TX_4_P, QSFP1_TX_3_P, QSFP1_TX_2_P, QSFP1_TX_1_P} = qsfp1TxP;
assign {QSFP1_TX_4_N, QSFP1_TX_3_N, QSFP1_TX_2_N, QSFP1_TX_1_N} = qsfp1TxN;
wire [3:0] evg1RxClksOut;
wire [3:0] evg1TxClksOut;
wire [3:0] evg1RxClksIn;
wire [3:0] evg1TxClksIn;
wire [3:0] evg1RxClks;
wire [3:0] evg2RxClks;

wire evg1RefClkUnbuf;
IBUFDS_GTE2 evg1RefBuf (.I(MGT_CLK_0_P), .IB(MGT_CLK_0_N), .O(evg1RefClkUnbuf));

generate
for (i = 0 ; i < 4 ; i = i + 1) begin : evg1_mgt_fanout
wire gt0_qplloutclk_i, gt0_qplloutrefclk_i;
localparam integer rOff = i * GPIO_IDX_PER_MGTWRAPPER;
mgtWrapper #(.EVG(1),
             .MGT_ID(i),
             .SAMPLING_CLOCK_RATE(500000000),
             .DEBUG("false"),
             .DRP_DEBUG("false"))
  evg1mgt (
    .sysClk(sysClk),
    .GPIO_OUT(GPIO_OUT),
    .drpStrobe(GPIO_STROBES[GPIO_IDX_EVG_1_0_DRP_CSR+rOff]),
    .drpStatus(GPIO_IN[GPIO_IDX_EVG_1_0_DRP_CSR+rOff]),
    .latency(GPIO_IN[GPIO_IDX_EVG_1_0_LATENCY+rOff]),
    .evgTxClkIn(evg1TxClksIn[i]),
    .evgTxClkOut(evg1TxClksOut[i]),
    .evgTxData(evg1TxData),
    .evgTxCharIsK(evg1TxCharIsK),
    .refClk(evg1RefClkUnbuf),
    .gt0_qplloutclk_i(gt0_qplloutclk_i),
    .gt0_qplloutrefclk_i(gt0_qplloutrefclk_i),
    .samplingClk(clkLatencySampler),
    .tx_p(qsfp1TxP[i]),
    .tx_n(qsfp1TxN[i]),
    .evgRxClkIn(evg1RxClksIn[i]),
    .evgRxClkOut(evg1RxClksOut[i]),
    .rx_p(qsfp1RxP[i]),
    .rx_n(qsfp1RxN[i]));

// each MGT has its own rxClk which is disciplined
// by the CDR circuitry
BUFG
  evg1RxBuf (
    .I(evg1RxClksOut[i]),
    .O(evg1RxClks[i]));

assign evg1RxClksIn[i] = evg1RxClks[i];
end
endgenerate

wire evg1GtTxReset = GPIO_IN[GPIO_IDX_EVG_1_0_DRP_CSR][30];
wire evg1GtRxReset = GPIO_IN[GPIO_IDX_EVG_1_0_DRP_CSR][29];
wire evg1CpllReset = GPIO_IN[GPIO_IDX_EVG_1_0_DRP_CSR][28];
wire evg1GtRxIsAligned = GPIO_IN[GPIO_IDX_EVG_1_0_DRP_CSR][27];
wire evg1GtTxFSMResetDone = GPIO_IN[GPIO_IDX_EVG_1_0_DRP_CSR][26];
wire evg1GtRxFSMResetDone = GPIO_IN[GPIO_IDX_EVG_1_0_DRP_CSR][25];
wire evg1TxResetDone = GPIO_IN[GPIO_IDX_EVG_1_0_DRP_CSR][24];
wire evg1RxResetDone = GPIO_IN[GPIO_IDX_EVG_1_0_DRP_CSR][23];
wire evg1CpllLock = GPIO_IN[GPIO_IDX_EVG_1_0_DRP_CSR][22];

//////////////////////////////////////////////////////////////////////////////
// Buffer EVG1 Tx clocks

// MGT 0 shares txClk with all others
BUFG evg1TxBuf (.I(evg1TxClksOut[0]), .O(evg1TxClk));

assign evg1TxClksIn = {4{evg1TxClk}};

wire [CFG_HARDWARE_TRIGGER_COUNT-1:0] evg1HwTrigger;
wire [CFG_EVIO_DIAG_IN_COUNT-1:0] evg1DiagnosticIn;
evg #(
    .SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
    .TXCLK_NOMINAL_FREQUENCY(TXCLK_NOMINAL_FREQUENCY),
    .TOD_SECONDS_WIDTH(TOD_SECONDS_WIDTH),
    .DISTRIBUTED_BUS_WIDTH(DISTRIBUTED_BUS_WIDTH),
    .GPIO_WIDTH(GPIO_WIDTH),
    .SEQUENCE_RAM_CAPACITY(CFG_SEQUENCE_RAM_CAPACITY),
    .HARDWARE_TRIGGER_COUNT(CFG_HARDWARE_TRIGGER_COUNT),
    .DEBUG("true"))
  evg1 (
    .sysClk(sysClk),
    .sysGPIO_OUT(GPIO_OUT),
    .sysSequencerCSRstrobe(GPIO_STROBES[GPIO_IDX_EVG_1_SEQ_CSR]),
    .sysHardwareTriggerCSRstrobe(GPIO_STROBES[GPIO_IDX_EVG_1_HW_CSR]),
    .sysSoftwareTriggerCSRstrobe(GPIO_STROBES[GPIO_IDX_EVG_1_SW_CSR]),
    .sysSequencerStatus(GPIO_IN[GPIO_IDX_EVG_1_SEQ_CSR]),
    .sysSequenceReadback(GPIO_IN[GPIO_IDX_EVG_1_SEQ_RBK]),
    .sysHardwareTriggerStatus(GPIO_IN[GPIO_IDX_EVG_1_HW_CSR]),
    .sysSoftwareTriggerStatus(GPIO_IN[GPIO_IDX_EVG_1_SW_CSR]),
    .sysPPStoggle(ppsToggle),
    .sysSeconds(posixSeconds),
    .hwTriggers_a(evg1HwTrigger),
    .diagnosticIn_a(evg1DiagnosticIn),
    .evgTxClk(evg1TxClk),
    .evgTxData(evg1TxData),
    .evgTxCharIsK(evg1TxCharIsK),
    .evgHeartbeatRequest(evg1HeartbeatRequest),
    .evgSequenceStart(injectorSequenceStart));

evLogger #(.DEBUG("false"))
  evg1Logger (
    .sysClk(sysClk),
    .GPIO_OUT(GPIO_OUT),
    .csrStrobe(GPIO_STROBES[GPIO_IDX_EVG_1_LOG_CSR]),
    .status(GPIO_IN[GPIO_IDX_EVG_1_LOG_CSR]),
    .evgTxClk(evg1TxClk),
    .evgTxData(evg1TxData),
    .evgTxCharIsK(evg1TxCharIsK));

/////////////////////////////////////////////////////////////////////////////
// Second generator (accumulator and storage rings)
wire swapoutSequenceStart;
wire [15:0] evg2TxData;
wire  [1:0] evg2TxCharIsK;
swapoutSequenceControl
    #(.CLOCK_PER_ARSR_COINCIDENCE(CFG_EVG2_CLOCK_PER_ARSR_COINCIDENCE),
      .DEBUG("false"))
  swapoutSequenceControl (
    .sysClk(sysClk),
    .sysCsrStrobe(GPIO_STROBES[GPIO_IDX_SWAPOUT_CYCLE_CSR]),
    .sysGPIO_OUT(GPIO_OUT),
    .sysStatus(GPIO_IN[GPIO_IDX_SWAPOUT_CYCLE_CSR]),
    .evgTxClk(evg2TxClk),
    .evgHeartbeatRequest(evg2HeartbeatRequest),
    .evgSequenceStart(swapoutSequenceStart));

wire [3:0] qsfp2RxP = {QSFP2_RX_4_P, QSFP2_RX_3_P, QSFP2_RX_2_P, QSFP2_RX_1_P};
wire [3:0] qsfp2RxN = {QSFP2_RX_4_N, QSFP2_RX_3_N, QSFP2_RX_2_N, QSFP2_RX_1_N};
wire [3:0] qsfp2TxP;
wire [3:0] qsfp2TxN;
assign {QSFP2_TX_4_P, QSFP2_TX_3_P, QSFP2_TX_2_P, QSFP2_TX_1_P} = qsfp2TxP;
assign {QSFP2_TX_4_N, QSFP2_TX_3_N, QSFP2_TX_2_N, QSFP2_TX_1_N} = qsfp2TxN;
wire [3:0] evg2RxClksOut;
wire [3:0] evg2TxClksOut;
wire [3:0] evg2RxClksIn;
wire [3:0] evg2TxClksIn;

wire evg2RefClkUnbuf;
IBUFDS_GTE2 evg2RefBuf (.I(MGT_CLK_3_P), .IB(MGT_CLK_3_N), .O(evg2RefClkUnbuf));

generate
for (i = 0 ; i < 4 ; i = i + 1) begin : evg2_mgt_fanout
wire gt0_qplloutclk_i, gt0_qplloutrefclk_i;
localparam integer rOff = i * GPIO_IDX_PER_MGTWRAPPER;
mgtWrapper #(.EVG(2),
             .MGT_ID(i),
             .SAMPLING_CLOCK_RATE(500000000),
             .DEBUG("false"),
             .DRP_DEBUG("false"),
             .FORCE_GTE_COMMON("true"))
  evg2mgt (
    .sysClk(sysClk),
    .GPIO_OUT(GPIO_OUT),
    .drpStrobe(GPIO_STROBES[GPIO_IDX_EVG_2_0_DRP_CSR+rOff]),
    .drpStatus(GPIO_IN[GPIO_IDX_EVG_2_0_DRP_CSR+rOff]),
    .latency(GPIO_IN[GPIO_IDX_EVG_2_0_LATENCY+rOff]),
    .evgTxClkIn(evg2TxClksIn[i]),
    .evgTxClkOut(evg2TxClksOut[i]),
    .evgTxData(evg2TxData),
    .evgTxCharIsK(evg2TxCharIsK),
    .refClk(evg2RefClkUnbuf),
    .gt0_qplloutclk_i(gt0_qplloutclk_i),
    .gt0_qplloutrefclk_i(gt0_qplloutrefclk_i),
    .samplingClk(clkLatencySampler),
    .tx_p(qsfp2TxP[i]),
    .tx_n(qsfp2TxN[i]),
    .evgRxClkIn(evg2RxClksIn[i]),
    .evgRxClkOut(evg2RxClksOut[i]),
    .rx_p(qsfp2RxP[i]),
    .rx_n(qsfp2RxN[i]));

// each MGT has its own rxClk which is disciplined
// by the CDR circuitry
BUFG
  evg2RxBuf (
    .I(evg2RxClksOut[i]),
    .O(evg2RxClks[i]));

assign evg2RxClksIn[i] = evg2RxClks[i];

end
endgenerate

wire evg2GtTxReset = GPIO_IN[GPIO_IDX_EVG_2_0_DRP_CSR][30];
wire evg2GtRxReset = GPIO_IN[GPIO_IDX_EVG_2_0_DRP_CSR][29];
wire evg2CpllReset = GPIO_IN[GPIO_IDX_EVG_2_0_DRP_CSR][28];
wire evg2GtRxIsAligned = GPIO_IN[GPIO_IDX_EVG_2_0_DRP_CSR][27];
wire evg2GtTxFSMResetDone = GPIO_IN[GPIO_IDX_EVG_2_0_DRP_CSR][26];
wire evg2GtRxFSMResetDone = GPIO_IN[GPIO_IDX_EVG_2_0_DRP_CSR][25];
wire evg2TxResetDone = GPIO_IN[GPIO_IDX_EVG_2_0_DRP_CSR][24];
wire evg2RxResetDone = GPIO_IN[GPIO_IDX_EVG_2_0_DRP_CSR][23];
wire evg2CpllLock = GPIO_IN[GPIO_IDX_EVG_2_0_DRP_CSR][22];

//////////////////////////////////////////////////////////////////////////////
// Buffer EVG2 clocks

// MGT 0 shares txClk with all others
BUFG evg2TxBuf (.I(evg2TxClksOut[0]), .O(evg2TxClk));

assign evg2TxClksIn = {4{evg2TxClk}};

wire [CFG_HARDWARE_TRIGGER_COUNT-1:0] evg2HwTrigger;
wire [CFG_EVIO_DIAG_IN_COUNT-1:0] evg2DiagnosticIn;
evg #(
    .SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
    .TXCLK_NOMINAL_FREQUENCY(TXCLK_NOMINAL_FREQUENCY),
    .TOD_SECONDS_WIDTH(TOD_SECONDS_WIDTH),
    .DISTRIBUTED_BUS_WIDTH(DISTRIBUTED_BUS_WIDTH),
    .GPIO_WIDTH(GPIO_WIDTH),
    .SEQUENCE_RAM_CAPACITY(CFG_SEQUENCE_RAM_CAPACITY),
    .HARDWARE_TRIGGER_COUNT(CFG_HARDWARE_TRIGGER_COUNT),
    .DEBUG("false"))
  evg2 (
    .sysClk(sysClk),
    .sysGPIO_OUT(GPIO_OUT),
    .sysSequencerCSRstrobe(GPIO_STROBES[GPIO_IDX_EVG_2_SEQ_CSR]),
    .sysHardwareTriggerCSRstrobe(GPIO_STROBES[GPIO_IDX_EVG_2_HW_CSR]),
    .sysSoftwareTriggerCSRstrobe(GPIO_STROBES[GPIO_IDX_EVG_2_SW_CSR]),
    .sysSequencerStatus(GPIO_IN[GPIO_IDX_EVG_2_SEQ_CSR]),
    .sysSequenceReadback(GPIO_IN[GPIO_IDX_EVG_2_SEQ_RBK]),
    .sysHardwareTriggerStatus(GPIO_IN[GPIO_IDX_EVG_2_HW_CSR]),
    .sysSoftwareTriggerStatus(GPIO_IN[GPIO_IDX_EVG_2_SW_CSR]),
    .sysPPStoggle(ppsToggle),
    .sysSeconds(posixSeconds),
    .hwTriggers_a(evg2HwTrigger),
    .diagnosticIn_a(evg2DiagnosticIn),
    .evgTxClk(evg2TxClk),
    .evgTxData(evg2TxData),
    .evgTxCharIsK(evg2TxCharIsK),
    .evgHeartbeatRequest(evg2HeartbeatRequest),
    .evgSequenceStart(swapoutSequenceStart));

evLogger #(.DEBUG("false"))
  evg2Logger (
    .sysClk(sysClk),
    .GPIO_OUT(GPIO_OUT),
    .csrStrobe(GPIO_STROBES[GPIO_IDX_EVG_2_LOG_CSR]),
    .status(GPIO_IN[GPIO_IDX_EVG_2_LOG_CSR]),
    .evgTxClk(evg2TxClk),
    .evgTxData(evg2TxData),
    .evgTxCharIsK(evg2TxCharIsK));

/////////////////////////////////////////////////////////////////////////////
// BNC board

localparam BNC_NUM_CHANNELS = 4;

// Buffer control
wire [BNC_NUM_CHANNELS-1:0] bncDataIn;
wire [BNC_NUM_CHANNELS-1:0] bncDataOut;
wire [BNC_NUM_CHANNELS-1:0] bncT;

// FPGA logic
wire [BNC_NUM_CHANNELS-1:0] bncDirToBuff;    // dir = low (FPGA is output), dir = high (FPGA is input)
wire [BNC_NUM_CHANNELS-1:0] bncDataToBuff;   // data generated by FPGA, dir = low
wire [BNC_NUM_CHANNELS-1:0] bncDataFromBuff; // data generated externally, dir = high

iobufGeneric iobufGenericBNCCh1 (
    .bufferIn(bncDataIn[0]),
    .bufferOut(bncDataOut[0]),
    // 3-state enable input, high=input, low=output
    .bufferT(bncT[0]),
    .bufferInOut(PMOD1_0));

assign PMOD1_4 = !bncT[0];

iobufGeneric iobufGenericBNCCh2 (
    .bufferIn(bncDataIn[1]),
    .bufferOut(bncDataOut[1]),
    // 3-state enable input, high=input, low=output
    .bufferT(bncT[1]),
    .bufferInOut(PMOD1_1));

assign PMOD1_5 = !bncT[1];

iobufGeneric iobufGenericBNCCh3 (
    .bufferIn(bncDataIn[2]),
    .bufferOut(bncDataOut[2]),
    // 3-state enable input, high=input, low=output
    .bufferT(bncT[2]),
    .bufferInOut(PMOD1_2));

assign PMOD1_6 = !bncT[2];

iobufGeneric iobufGenericBNCCh4 (
    .bufferIn(bncDataIn[3]),
    .bufferOut(bncDataOut[3]),
    // 3-state enable input, high=input, low=output
    .bufferT(bncT[3]),
    .bufferInOut(PMOD1_3));

assign PMOD1_7 = !bncT[3];

generate
for (i = 0 ; i < BNC_NUM_CHANNELS ; i = i + 1) begin : bnc_interface

assign bncT[i] = (!bncDirToBuff[i])? bncDataToBuff[i] : bncDirToBuff[i];
assign bncDataIn[i] = !bncDirToBuff[i];
assign bncDataFromBuff[i] = (bncDirToBuff[i])? bncDataOut[i] : 1'b0;

end
endgenerate

// fixed direction assignments
assign bncDirToBuff[0] = 1'b1; // input
assign bncDirToBuff[1] = 1'b1; // input
assign bncDirToBuff[2] = 1'b0; // output
assign bncDirToBuff[3] = 1'b0; // output

// PPS

assign bncPPS_a = bncDataFromBuff[0];
assign evg1HwTrigger = {CFG_HARDWARE_TRIGGER_COUNT{1'b0}};
assign evg1DiagnosticIn = {CFG_EVIO_DIAG_IN_COUNT{1'b0}};

// 60Hz
assign bncPowerline_a = bncDataFromBuff[1];
assign evg2HwTrigger = {CFG_HARDWARE_TRIGGER_COUNT{1'b0}};
assign evg2DiagnosticIn = {CFG_EVIO_DIAG_IN_COUNT{1'b0}};

// EVG 1
assign bncDataToBuff[2] = evg1DiagnosticOut;

// EVG 2
assign bncDataToBuff[3] = evg2DiagnosticOut;

// Unused as channels 0 and 1 are inputs
assign bncDataToBuff[0] = 1'b0;
assign bncDataToBuff[1] = 1'b0;

/////////////////////////////////////////////////////////////////////////////
// LEDs
wire evg1HeartbeatStretch;

pulseStretcher #(
    .CLK_FREQUENCY(TXCLK_NOMINAL_FREQUENCY),
    .STRETCH_MS(100),
    .RETRIGGERABLE("true"))
  evg1HeartbeatPulseStretcher (
    .clk(evg1TxClk),
    .rst_a(!evg1TxResetDone),
    .pulse_a(evg1HeartbeatRequest),
    .pulseStretch(evg1HeartbeatStretch)
);

wire evg2HeartbeatStretch;

pulseStretcher #(
    .CLK_FREQUENCY(TXCLK_NOMINAL_FREQUENCY),
    .STRETCH_MS(100),
    .RETRIGGERABLE("true"))
  evg2HeartbeatPulseStretcher (
    .clk(evg2TxClk),
    .rst_a(!evg2TxResetDone),
    .pulse_a(evg2HeartbeatRequest),
    .pulseStretch(evg2HeartbeatStretch)
);

wire ppsStretch;

pulseStretcher #(
    .CLK_FREQUENCY(SYSCLK_FREQUENCY),
    .STRETCH_MS(100),
    .RETRIGGERABLE("false"))
  ppsStretcher (
    .clk(sysClk),
    .rst_a(!sysReset_n),
    .pulse_a(!bncPPS_a),
    .pulseStretch(ppsStretch)
);

wire powerlineStretch;

pulseStretcher #(
    .CLK_FREQUENCY(SYSCLK_FREQUENCY),
    .STRETCH_MS(100),
    .RETRIGGERABLE("false"))
  powerlineStretcher (
    .clk(sysClk),
    .rst_a(!sysReset_n),
    .pulse_a(bncPowerline_a),
    .pulseStretch(powerlineStretch)
);

// LEDs
assign PMOD2_0 = evg1HeartbeatStretch;
assign PMOD2_1 = evg2HeartbeatStretch;
assign PMOD2_2 = ppsStretch;
assign PMOD2_3 = powerlineStretch;

// Unused
assign PMOD2_4 = 1'b0;
assign PMOD2_5 = 1'b0;

// Buttons
assign PMOD2_6 = 1'b1;
assign PMOD2_7 = 1'b1;

/////////////////////////////////////////////////////////////////////////////
// Measure clock rates
localparam FREQ_COUNTERS_NUM = 15;
localparam FREQ_SEL_WIDTH = $clog2(FREQ_COUNTERS_NUM+1);
reg   [FREQ_SEL_WIDTH-1:0] frequencyMonitorSelect;
wire [29:0] measuredFrequency;
always @(posedge sysClk) begin
    if (GPIO_STROBES[GPIO_IDX_FREQ_MONITOR_CSR]) begin
        frequencyMonitorSelect <= GPIO_OUT[FREQ_SEL_WIDTH-1:0];
    end
end
assign GPIO_IN[GPIO_IDX_FREQ_MONITOR_CSR] = { 2'b0, measuredFrequency };
freq_multi_count #(
        .NF(FREQ_COUNTERS_NUM),  // number of frequency counters in a block
        .NG(1),  // number of frequency counter blocks
        .gw(4),  // Gray counter width
        .cw(1),  // macro-cycle counter width
        .rw($clog2(SYSCLK_FREQUENCY*4/3)), // reference counter width
        .uw(30)) // unknown counter width
  frequencyCounters (
    .unk_clk({evg2RxClks[1], evg2RxClks[2], evg2RxClks[3],
              evg1RxClks[1], evg1RxClks[2], evg1RxClks[3],
              ethernetRxClk, ethernetTxClk,
              evg2RxClks[0], evg2TxClk, evg2RefClk,
              evg1RxClks[0], evg1TxClk, evg1RefClk,
              sysClk}),
    .refclk(sysClk),
    .refMarker(ppsMarkerValid ? ppsMarker : sysPPSmarker),
    .source_state(),
    .addr(frequencyMonitorSelect),
    .frequency(measuredFrequency));

/////////////////////////////////////////////////////////////////////////////
// Measure fan speeds
wire DUMMY_FMC2_FAN2_TACH = 1'b0;
wire DUMMY_FMC2_FAN1_TACH = 1'b0;
wire DUMMY_FMC1_FAN2_TACH = 1'b0;
wire DUMMY_FMC1_FAN1_TACH = 1'b0;
fanTach #(.CLK_FREQUENCY(SYSCLK_FREQUENCY),
          .FAN_COUNT(CFG_FAN_COUNT))
  fanTachs (
    .clk(sysClk),
    .csrStrobe(GPIO_STROBES[GPIO_IDX_FAN_TACHOMETERS]),
    .GPIO_OUT(GPIO_OUT),
    .value(GPIO_IN[GPIO_IDX_FAN_TACHOMETERS]),
    .tachs_a({DUMMY_FMC2_FAN2_TACH, DUMMY_FMC2_FAN1_TACH,
                DUMMY_FMC1_FAN2_TACH, DUMMY_FMC1_FAN1_TACH}));

//////////////////////////////////////////////////////////////////////////////
// Diagnostic I/O

localparam OUTPUT_SELECT_WIDTH = 2;

wire [CFG_EVIO_DIAG_OUT_COUNT-1:0] diagnostic1ProgrammableOutputs;
wire [OUTPUT_SELECT_WIDTH-1:0] diagnostic1Select;
wire evg1AuxSwitch_n = 1'b1;
diagnosticIO #(.INPUT_WIDTH(CFG_EVIO_DIAG_IN_COUNT),
               .OUTPUT_WIDTH(CFG_EVIO_DIAG_OUT_COUNT),
               .OUTPUT_SELECT_WIDTH(OUTPUT_SELECT_WIDTH))
  fmc1IO (
    .sysClk(sysClk),
    .csrStrobe(GPIO_STROBES[GPIO_IDX_FMC1_DIAGNOSTIC]),
    .GPIO_OUT(GPIO_OUT),
    .status(GPIO_IN[GPIO_IDX_FMC1_DIAGNOSTIC]),
    .auxSwitch_n(evg1AuxSwitch_n),
    .diagnosticIn(evg1DiagnosticIn),
    .diagnosticOut(diagnostic1ProgrammableOutputs),
    .diagnosticOutputSelect(diagnostic1Select));
wire evg1DiagnosticOut =
     (diagnostic1Select == 2'h1) ? evg1RefClk :
     (diagnostic1Select == 2'h2) ? evg1TxClk :
     (diagnostic1Select == 2'h3) ? evg1HeartbeatRequest :
                                     diagnostic1ProgrammableOutputs;

wire [CFG_EVIO_DIAG_OUT_COUNT-1:0] diagnostic2ProgrammableOutputs;
wire [OUTPUT_SELECT_WIDTH-1:0] diagnostic2Select;
wire evg2AuxSwitch_n = 1'b1;
diagnosticIO #(.INPUT_WIDTH(CFG_EVIO_DIAG_IN_COUNT),
               .OUTPUT_WIDTH(CFG_EVIO_DIAG_OUT_COUNT),
               .OUTPUT_SELECT_WIDTH(OUTPUT_SELECT_WIDTH))
  fmc2IO (
    .sysClk(sysClk),
    .csrStrobe(GPIO_STROBES[GPIO_IDX_FMC2_DIAGNOSTIC]),
    .GPIO_OUT(GPIO_OUT),
    .status(GPIO_IN[GPIO_IDX_FMC2_DIAGNOSTIC]),
    .auxSwitch_n(evg2AuxSwitch_n),
    .diagnosticIn(evg2DiagnosticIn),
    .diagnosticOut(diagnostic2ProgrammableOutputs),
    .diagnosticOutputSelect(diagnostic2Select));
wire evg2DiagnosticOut =
     (diagnostic2Select == 2'h1) ? evg2RefClk :
     (diagnostic2Select == 2'h2) ? evg2TxClk :
     (diagnostic2Select == 2'h3) ? evg2HeartbeatRequest :
                                     diagnostic2ProgrammableOutputs;

///////////////////////////////////////////////////////////////////////////////
// Ethernet
badger badger (
    .sysClk(sysClk),
    .sysGPIO_OUT(GPIO_OUT),
    .sysConfigStrobe(GPIO_STROBES[GPIO_IDX_NET_CONFIG_CSR]),
    .sysTxStrobe(GPIO_STROBES[GPIO_IDX_NET_TX_CSR]),
    .sysRxStrobe(GPIO_STROBES[GPIO_IDX_NET_RX_CSR]),
    .sysTxStatus(GPIO_IN[GPIO_IDX_NET_TX_CSR]),
    .sysRxStatus(GPIO_IN[GPIO_IDX_NET_RX_CSR]),
    .sysRxDataStrobe(GPIO_STROBES[GPIO_IDX_NET_RX_DATA]),
    .sysRxData(GPIO_IN[GPIO_IDX_NET_RX_DATA]),
    .refClk125(refClk125),
    .refClk125d90(refClk125d90),
    .rx_clk(ethernetRxClk),
    .tx_clk(ethernetTxClk),
    .RGMII_RX_CLK(RGMII_RX_CLK),
    .RGMII_RX_CTRL(RGMII_RX_CTRL),
    .RGMII_RXD(RGMII_RXD),
    .RGMII_TX_CLK(RGMII_TX_CLK),
    .RGMII_TX_CTRL(RGMII_TX_CTRL),
    .RGMII_TXD(RGMII_TXD));

///////////////////////////////////////////////////////////////////////////////
// IIC to FMC components (crosspoint, Firefly)
// Note that Firefly devices use SCL stretching to limit the data rate so
// the conventional approach of making SCL unidirectional can not be used.
(*MARK_DEBUG="false"*) wire evio_iic_scl_i, evio_iic_scl_o, evio_iic_scl_t;
(*MARK_DEBUG="false"*) wire evio_iic_sda_i, evio_iic_sda_o, evio_iic_sda_t;
(*MARK_DEBUG="false"*) wire fmc1_scl_i = 0, fmc1_sda_i = 0, fmc2_scl_i = 0, fmc2_sda_i = 0;
(*MARK_DEBUG="false"*) wire [CFG_EVIO_FIREFLY_COUNT/2:0] evio_iic_gpo;

assign evio_iic_scl_i = evio_iic_gpo[0] ? fmc2_scl_i : fmc1_scl_i;
assign evio_iic_sda_i = evio_iic_gpo[0] ? fmc2_sda_i : fmc1_sda_i;

wire DUMMY1_fireflySelect_n = ~evio_iic_gpo[CFG_EVIO_FIREFLY_COUNT/2:1];
wire DUMMY2_fireflySelect_n = ~evio_iic_gpo[CFG_EVIO_FIREFLY_COUNT/2:1];

wire [CFG_EVIO_FIREFLY_COUNT-1:0] DUMMY1_fireflyPresent_n = ~0;
assign GPIO_IN[GPIO_IDX_FMC1_FIREFLY] = {{32-CFG_EVIO_FIREFLY_COUNT{1'b0}},
                                         DUMMY1_fireflyPresent_n };
wire [CFG_EVIO_FIREFLY_COUNT-1:0] DUMMY2_fireflyPresent_n = ~0;
assign GPIO_IN[GPIO_IDX_FMC2_FIREFLY] = {{32-CFG_EVIO_FIREFLY_COUNT{1'b0}},
                                         DUMMY2_fireflyPresent_n };

// Make this a black box for simulation
`ifndef SIMULATE
///////////////////////////////////////////////////////////////////////////////
// Block design
bd bd_i (
    .clkSrc125_clk_p(DDR_REF_CLK_P),
    .clkSrc125_clk_n(DDR_REF_CLK_N),
    .ext_reset_in(1'b1),
    .aux_reset_in(1'b1),

    .sysClk(sysClk),
    .sysReset_n(sysReset_n),
    .refClk125(refClk125),
    .refClk125d90(refClk125d90),
    .clkLatencySampler(clkLatencySampler),

    .GPIO_IN(GPIO_IN_FLATTENED),
    .GPIO_OUT(GPIO_OUT),
    .GPIO_STROBES(GPIO_STROBES),

    .iic_proc_gpio_tri_i(iic_proc_i),
    .iic_proc_gpio_tri_o(iic_proc_o),

    .evio_iic_scl_i(evio_iic_scl_i),
    .evio_iic_scl_o(evio_iic_scl_o),
    .evio_iic_scl_t(evio_iic_scl_t),
    .evio_iic_sda_i(evio_iic_sda_i),
    .evio_iic_sda_o(evio_iic_sda_o),
    .evio_iic_sda_t(evio_iic_sda_t),
    .evio_iic_gpo(evio_iic_gpo),

    // Connection from optional GPS receiver
    .gps_uart_ctsn(1'b0),
    .gps_uart_dcdn(1'b0),
    .gps_uart_dsrn(1'b0),
    .gps_uart_ri(1'b0),
    .gps_uart_rxd(1'b0),

    // Yes, these assignments look reversed.  See comment on port declarations.
    .console_rxd(FPGA_TxD),
    .console_txd(FPGA_RxD));
`endif // `ifndef SIMULATE

generate
if (ILA_CHIPSCOPE_DBG != "TRUE" && ILA_CHIPSCOPE_DBG != "FALSE") begin
    ILA_CHIPSCOPE_DBG_only_TRUE_or_FALSE_SUPPORTED();
end
endgenerate

generate
if (ILA_CHIPSCOPE_DBG == "TRUE") begin

wire [255:0] probe;
`ifndef SIMULATE
ila_td256_s4096_cap ila_td256_s4096_cap_inst (
    .clk(sysClk),
    .probe0(probe)
);
`endif

assign probe[0] = bncPPS_a;
assign probe[1] = bncPPSvalid;
assign probe[2] = bestPPS_a;
assign probe[3] = ppsToggle;
assign probe[4] = ppsMarker;
assign probe[5] = ppsMarkerValid;

assign probe[31:6] = 0;

assign probe[32] = evg1GtTxReset;
assign probe[33] = evg1GtRxReset;
assign probe[34] = evg1CpllReset;
assign probe[35] = evg1GtRxIsAligned;
assign probe[36] = evg1GtTxFSMResetDone;
assign probe[37] = evg1GtRxFSMResetDone;
assign probe[38] = evg1TxResetDone;
assign probe[39] = evg1RxResetDone;
assign probe[40] = evg1CpllLock;

assign probe[63:41] = 0;

assign probe[64] = evg2GtTxReset;
assign probe[65] = evg2GtRxReset;
assign probe[66] = evg2CpllReset;
assign probe[67] = evg2GtRxIsAligned;
assign probe[68] = evg2GtTxFSMResetDone;
assign probe[69] = evg2GtRxFSMResetDone;
assign probe[70] = evg2TxResetDone;
assign probe[71] = evg2RxResetDone;
assign probe[72] = evg2CpllLock;

assign probe[255:73] = 0;

end // end if
endgenerate

endmodule
