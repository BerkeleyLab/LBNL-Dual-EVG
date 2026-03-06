// Top level module for ALSU Event Generator

module devg_marble_top #(
    // Include file is machine generated from C header
    `include "gpioIDX.vh"
    parameter ILA_CHIPSCOPE_DBG       = "FALSE",
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
    output MGT_TX_1_P, MGT_TX_1_N,
    input  MGT_RX_1_P, MGT_RX_1_N,
    output MGT_TX_2_P, MGT_TX_2_N,
    input  MGT_RX_2_P, MGT_RX_2_N,

    // Currently unsused, but kept here for compatibility
    // and possible future use
    input  FMC1_CLK0_M2C_P, FMC1_CLK0_M2C_N,
    input  FMC2_CLK0_M2C_P, FMC2_CLK0_M2C_N,

    input        [CFG_HARDWARE_TRIGGER_COUNT-1:0] FMC1_hwTrigger,
    input                                         FMC1_auxInput,
    input        [CFG_EVIO_DIAG_IN_COUNT-1:0] FMC1_diagnosticIn,
    output wire [CFG_EVIO_DIAG_OUT_COUNT-1:0] FMC1_diagnosticOut,
    input              [CFG_EVIO_FIREFLY_COUNT-1:0] FMC1_fireflyPresent_n,
    output           [CFG_EVIO_FIREFLY_COUNT/2-1:0] FMC1_fireflySelect_n,
    output                                        FMC1_sysReset_n,
    input                                         FMC1_FAN1_TACH,
    input                                         FMC1_FAN2_TACH,
    inout                                         FMC1_EVIO_SCL,
    inout                                         FMC1_EVIO_SDA,

    input        [CFG_HARDWARE_TRIGGER_COUNT-1:0] FMC2_hwTrigger,
    input                                         FMC2_auxInput,
    input        [CFG_EVIO_DIAG_IN_COUNT-1:0] FMC2_diagnosticIn,
    output wire [CFG_EVIO_DIAG_OUT_COUNT-1:0] FMC2_diagnosticOut,
    input              [CFG_EVIO_FIREFLY_COUNT-1:0] FMC2_fireflyPresent_n,
    output           [CFG_EVIO_FIREFLY_COUNT/2-1:0] FMC2_fireflySelect_n,
    output                                        FMC2_sysReset_n,
    input                                         FMC2_FAN1_TACH,
    input                                         FMC2_FAN2_TACH,
    inout                                         FMC2_EVIO_SCL,
    inout                                         FMC2_EVIO_SDA,

    input  FPGA_SCLK,
    input  FPGA_CSB,
    input  FPGA_MOSI,
    output FPGA_MISO,

    // Optional GPS receiver
    input  PMOD1_0, // 3D-Fix (unused)
    input  PMOD1_1, // RxData (unused)
    input  PMOD1_2, // TxData
    input  PMOD1_3, // PPS
    input  PMOD1_4,
    input  PMOD1_5,
    input  PMOD1_6,
    input  PMOD1_7,

    // Display
    output PMOD2_0,
    inout  PMOD2_1,
    output PMOD2_2,
    output PMOD2_3,
    output PMOD2_4,
    output PMOD2_5,
    input  PMOD2_6,
    input  PMOD2_7,

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
wire evg1RxClk, evg2RxClk;

///////////////////////////////////////////////////////////////////////////////
// Resets
wire sysReset_n;
assign FMC1_sysReset_n = sysReset_n;
assign FMC2_sysReset_n = sysReset_n;

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
        28'b0, gpsPPSvalid, fmcPPSvalid };

/////////////////////////////////////////////////////////////////////////////
// Display
wire DISPLAY_SPI_SDA_O, DISPLAY_SPI_SDA_T, DISPLAY_SPI_SDA_I;
IOBUF DISPLAY_MOSI_Buf(.IO(PMOD2_1),
                       .I(DISPLAY_SPI_SDA_O),
                       .T(DISPLAY_SPI_SDA_T),
                       .O(DISPLAY_SPI_SDA_I));
st7789v #(.CLK_RATE(SYSCLK_FREQUENCY),
          .COMMAND_QUEUE_ADDRESS_WIDTH(11),
          .DEBUG("false"))
  st7789v (.clk(sysClk),
           .csrStrobe(GPIO_STROBES[GPIO_IDX_DISPLAY_CSR]),
           .dataStrobe(GPIO_STROBES[GPIO_IDX_DISPLAY_DATA]),
           .gpioOut(GPIO_OUT),
           .status(GPIO_IN[GPIO_IDX_DISPLAY_CSR]),
           .readData(GPIO_IN[GPIO_IDX_DISPLAY_DATA]),
           .DISPLAY_BACKLIGHT_ENABLE(PMOD2_2),
           .DISPLAY_RESET_N(PMOD2_4),
           .DISPLAY_CMD_N(PMOD2_5),
           .DISPLAY_CLK(PMOD2_3),
           .DISPLAY_CS_N(PMOD2_0),
           .DISPLAY_SDA_O(DISPLAY_SPI_SDA_O),
           .DISPLAY_SDA_T(DISPLAY_SPI_SDA_T),
           .DISPLAY_SDA_I(DISPLAY_SPI_SDA_I));

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
wire [CFG_EVG1_HEARTBEAT_COUNT-1:0] evg1HeartbeatRequest;
wire [CFG_EVG2_HEARTBEAT_COUNT-1:0] evg2HeartbeatRequest;
wire sysRealignToggle;
wire sampEvg1CoincidenceMarker, sampEvg2CoincidenceMarker;
wire evg1CoincidenceMarker, evg2CoincidenceMarker;

coincidenceRecorder #(
    .CHANNEL_COUNT(2),
    .CYCLES_PER_ACQUISITION(1023),
    .SAMPLE_CLKS_PER_COINCIDENCE(CFG_EVG2_CLK_PER_RF_COINCIDENCE),
    .INPUT_CYCLES_PER_COINCIDENCE(CFG_EVG1_CLK_PER_RF_COINCIDENCE),
    .HEARTBEAT_GEN_COUNT(CFG_EVG1_HEARTBEAT_COUNT),
    .TX_CLK_PER_HEARTBEAT({
        CFG_EVG1_ALT_CLK_PER_HEARTBEAT,
        CFG_EVG1_CLK_PER_HEARTBEAT}))
  coincidenceRecorder1 (
    .sysClk(sysClk),
    .sysCsrStrobe(GPIO_STROBES[GPIO_IDX_EVG_1_COINC_CSR]),
    .sysGPIO_OUT(GPIO_OUT),
    .sysCsr(GPIO_IN[GPIO_IDX_EVG_1_COINC_CSR]),
    .sysRealignToggle(sysRealignToggle),
    .sysRealignToggleIn(sysRealignToggle),
    .samplingClk(evg2RefClk),
    .refClk({evg1TxClk, evg1RefClk}),
    .coincidenceMarker(sampEvg1CoincidenceMarker),
    .txClk(evg1TxClk),
    .txCoincidenceMarker(evg1CoincidenceMarker),
    .txHeartbeatStrobe(evg1HeartbeatRequest));

coincidenceRecorder #(
    .CHANNEL_COUNT(2),
    .CYCLES_PER_ACQUISITION(1023),
    .SAMPLE_CLKS_PER_COINCIDENCE(CFG_EVG1_CLK_PER_RF_COINCIDENCE),
    .INPUT_CYCLES_PER_COINCIDENCE(CFG_EVG2_CLK_PER_RF_COINCIDENCE),
    .HEARTBEAT_GEN_COUNT(CFG_EVG2_HEARTBEAT_COUNT),
    .TX_CLK_PER_HEARTBEAT(CFG_EVG2_CLK_PER_HEARTBEAT))
  coincidenceRecorder2 (
    .sysClk(sysClk),
    .sysCsrStrobe(GPIO_STROBES[GPIO_IDX_EVG_2_COINC_CSR]),
    .sysGPIO_OUT(GPIO_OUT),
    .sysRealignToggleIn(sysRealignToggle),
    .sysCsr(GPIO_IN[GPIO_IDX_EVG_2_COINC_CSR]),
    .samplingClk(evg1RefClk),
    .refClk({evg2TxClk, evg2RefClk}),
    .coincidenceMarker(sampEvg2CoincidenceMarker),
    .txClk(evg2TxClk),
    .txCoincidenceMarker(evg2CoincidenceMarker),
    .txHeartbeatStrobe(evg2HeartbeatRequest));

//////////////////////////////////////////////////////////////////////////////
// Debounce timing markers
wire powerlineMarker;
debounceFallingEdge debouncePowerline (
    .clk(sysClk),
    .inputActiveLow(FMC2_auxInput),
    .debouncedActiveHigh(powerlineMarker));

//////////////////////////////////////////////////////////////////////////////
// I2C
// Three channel version a holdover from Marble Mini layout, but keep
// the extra two channels as dummies until the IIC command table has
// been updated.
wire [2:0] sda_drive, sda_sense;
wire [3:0] iic_proc_o;
wire FMC2_SFP_SCL, FMC1_SFP_SCL;
wire scl0;
i2cHandler #(.CLK_RATE(SYSCLK_FREQUENCY),
             .CHANNEL_COUNT(3),
             .DEBUG("false"))
  i2cHandler (
    .clk(sysClk),
    .csrStrobe(GPIO_STROBES[GPIO_IDX_I2C_CHUNK_CSR]),
    .GPIO_OUT(GPIO_OUT),
    .status(GPIO_IN[GPIO_IDX_I2C_CHUNK_CSR]),
    .scl({FMC2_SFP_SCL, FMC1_SFP_SCL, scl0}),
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
wire fmcPPS_a = !FMC1_auxInput;
wire fmcPPSvalid;
ppsCheck #(.CLK_RATE(SYSCLK_FREQUENCY)) fmcPPScheck (
    .clk(sysClk),
    .pps_a(fmcPPS_a),
    .ppsValid(fmcPPSvalid));

wire gpsPPS_a = PMOD1_3;
wire gpsPPSvalid;
ppsCheck #(.CLK_RATE(SYSCLK_FREQUENCY)) gpsPPScheck (
    .clk(sysClk),
    .pps_a(gpsPPS_a),
    .ppsValid(gpsPPSvalid));

wire bestPPS_a = fmcPPSvalid ? fmcPPS_a : gpsPPS_a;

//////////////////////////////////////////////////////////////////////////////
// NTP server support for F1 domain
wire [31:0] sysNtpSeconds_f1, sysNtpFraction_f1, sysPosixSeconds_f1, sysPosixSecondsNext_f1, sysNtpStatusReg_f1;
wire [31:0] evgNtpSeconds_f1, evgNtpFraction_f1, evgPosixSeconds_f1, evgPosixSecondsNext_f1, evgNtpStatusReg_f1;
wire evgPpsToggle_f1, evgPpsMarker_f1;
wire sysPpsToggle_f1, sysPpsMarker_f1;
wire evgPpsStrobe_f1;
ntpClock #(.CLK_RATE(TXCLK_NOMINAL_FREQUENCY),
           .DEBUG("false"))
  ntpClock_f1 (
    .sysClk(sysClk),
    .writeStrobe(GPIO_STROBES[GPIO_IDX_NTP_SERVER_SECONDS]),
    .writeData(GPIO_OUT),
    .sysPpsToggle(sysPpsToggle_f1),
    .sysPpsMarker(sysPpsMarker_f1),
    .sysSeconds(sysNtpSeconds_f1),
    .sysFraction(sysNtpFraction_f1),
    .sysPosixSeconds(sysPosixSeconds_f1),
    .sysPosixSecondsNext(sysPosixSecondsNext_f1),
    .sysStatus(sysNtpStatusReg_f1),

    .clk(evg1TxClk),
    .pps_a(bestPPS_a),
    .ppsToggle(evgPpsToggle_f1),
    .ppsStrobe(evgPpsStrobe_f1),
    .ppsMarker(evgPpsMarker_f1),
    .seconds(evgNtpSeconds_f1),
    .fraction(evgNtpFraction_f1),
    .posixSeconds(evgPosixSeconds_f1),
    .posixSecondsNext(evgPosixSecondsNext_f1),
    .status(evgNtpStatusReg_f1));

assign GPIO_IN[GPIO_IDX_NTP_SERVER_SECONDS] = sysNtpSeconds_f1;
assign GPIO_IN[GPIO_IDX_NTP_SERVER_FRACTION] = sysNtpFraction_f1;
assign GPIO_IN[GPIO_IDX_NTP_SERVER_STATUS] = sysNtpStatusReg_f1;
wire ppsMarkerValid = sysNtpStatusReg_f1[0];
wire ppsMarker = sysPpsMarker_f1;

/////////////////////////////////////////////////////////////////////////////
// First generator (injector)
wire injectorSequenceStart;
wire evg1HeartbeatAlign, evg1HeartbeatCore;
wire [15:0] evg1TxData;
wire  [1:0] evg1TxCharIsK;
injectorSequenceControl #(
    .SYSCLK_RATE(SYSCLK_FREQUENCY),
    .ALIGNMENT_SYNC_COUNT(CFG_EVG1_HEARTBEAT_COUNT),
    .TX_CLK_PER_ALIGNMENT({
        CFG_EVG1_ALT_CLK_PER_BR_AR_ALIGNMENT,
        CFG_EVG1_CLK_PER_BR_AR_ALIGNMENT}))
  injectorSequenceControl (
    .sysClk(sysClk),
    .sysGPIO_OUT(GPIO_OUT),
    .sysCsrStrobe(GPIO_STROBES[GPIO_IDX_INJECTION_CYCLE_CSR]),
    .sysStatus(GPIO_IN[GPIO_IDX_INJECTION_CYCLE_CSR]),
    .sysCsrAlignStrobe(GPIO_STROBES[GPIO_IDX_INJECTION_ALIGN_CSR]),
    .sysAlignStatus(GPIO_IN[GPIO_IDX_INJECTION_ALIGN_CSR]),
    .powerline_a(powerlineMarker),
    .evgTxClk(evg1TxClk),
    .evgHeartbeat(evg1HeartbeatRequest),
    .evgHeartbeatAlign(evg1HeartbeatAlign),
    .evgHeartbeatCore(evg1HeartbeatCore),
    .evgSequenceStart(injectorSequenceStart));

wire evg1RxClkOut;
wire evg1TxClkOut;
wire evg1RxClkIn;
wire evg1TxClkIn;
wire evg1RefClkUnbuf;
IBUFDS_GTE2 evg1RefBuf (.I(MGT_CLK_0_P), .IB(MGT_CLK_0_N), .O(evg1RefClkUnbuf));
BUFG f1BUFG (.I(evg1RefClkUnbuf), .O(evg1RefClk));

wire gt0_qplloutclk_i, gt0_qplloutrefclk_i;
mgtWrapper #(.EVG(1),
             .SAMPLING_CLOCK_RATE(500000000),
             .DEBUG("false"),
             .DRP_DEBUG("false"))
  evg1mgt (
    .sysClk(sysClk),
    .GPIO_OUT(GPIO_OUT),
    .drpStrobe(GPIO_STROBES[GPIO_IDX_EVG_1_0_DRP_CSR]),
    .drpStatus(GPIO_IN[GPIO_IDX_EVG_1_0_DRP_CSR]),
    .latency(GPIO_IN[GPIO_IDX_EVG_1_0_LATENCY]),
    .evgTxClkIn(evg1TxClkIn),
    .evgTxClkOut(evg1TxClkOut),
    .evgTxData(evg1TxData),
    .evgTxCharIsK(evg1TxCharIsK),
    .refClk(evg1RefClkUnbuf),
    .samplingClk(clkLatencySampler),
    .gt0_qplloutclk_i(gt0_qplloutclk_i),
    .gt0_qplloutrefclk_i(gt0_qplloutrefclk_i),
    .tx_p(MGT_TX_1_P),
    .tx_n(MGT_TX_1_N),
    .evgRxClkIn(evg1RxClkIn),
    .evgRxClkOut(evg1RxClkOut),
    .rx_p(MGT_RX_1_P),
    .rx_n(MGT_RX_1_N));

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
// Buffer EVG1 clocks
BUFG evg1RxBuf (.I(evg1RxClkOut), .O(evg1RxClk));
BUFG evg1TxBuf (.I(evg1TxClkOut), .O(evg1TxClk));

assign evg1RxClkIn = evg1RxClk;
assign evg1TxClkIn = evg1TxClk;

wire [CFG_HARDWARE_TRIGGER_COUNT-1:0] FMC1_hwTrigger_remap =
    {FMC1_hwTrigger[3], FMC1_hwTrigger[2], FMC1_hwTrigger[1], FMC1_hwTrigger[0],
     1'b0, 1'b0, FMC1_hwTrigger[5], FMC1_hwTrigger[4]};

evg #(
    .SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
    .TXCLK_NOMINAL_FREQUENCY(TXCLK_NOMINAL_FREQUENCY),
    .TOD_SECONDS_WIDTH(TOD_SECONDS_WIDTH),
    .DISTRIBUTED_BUS_WIDTH(DISTRIBUTED_BUS_WIDTH),
    .GPIO_WIDTH(GPIO_WIDTH),
    .SEQUENCE_RAM_CAPACITY(CFG_SEQUENCE_RAM_CAPACITY),
    .HARDWARE_TRIGGER_COUNT(CFG_HARDWARE_TRIGGER_COUNT),
    .DEBUG("false"))
  evg1 (
    .sysClk(sysClk),
    .sysGPIO_OUT(GPIO_OUT),
    .sysSequencerCSRstrobe(GPIO_STROBES[GPIO_IDX_EVG_1_SEQ_CSR]),
    .sysHardwareTriggerCSRstrobe(GPIO_STROBES[GPIO_IDX_EVG_1_HW_CSR]),
    .sysSoftwareTriggerCSRstrobe(GPIO_STROBES[GPIO_IDX_EVG_1_SW_CSR]),
    .sysSequencerStatus(GPIO_IN[GPIO_IDX_EVG_1_SEQ_CSR]),
    .sysSequencerStatusNtpSeconds(GPIO_IN[GPIO_IDX_EVG_1_SEQ_SECONDS_CSR]),
    .sysSequencerStatusNtpFraction(GPIO_IN[GPIO_IDX_EVG_1_SEQ_FRACTION_CSR]),
    .sysSequenceReadback(GPIO_IN[GPIO_IDX_EVG_1_SEQ_RBK]),
    .sysHardwareTriggerStatus(GPIO_IN[GPIO_IDX_EVG_1_HW_CSR]),
    .sysSoftwareTriggerStatus(GPIO_IN[GPIO_IDX_EVG_1_SW_CSR]),
    .sysSequencerStatusFIFOCSRstrobe(GPIO_STROBES[GPIO_IDX_EVG_1_SEQ_STATUS_FIFO_CSR]),
    .sysSequencerStatusFifo(GPIO_IN[GPIO_IDX_EVG_1_SEQ_STATUS_FIFO_CSR]),
    .hwTriggers_a(FMC1_hwTrigger_remap),
    .evgTxClk(evg1TxClk),
    .evgTxData(evg1TxData),
    .evgTxCharIsK(evg1TxCharIsK),
    .evgHeartbeatRequest(evg1HeartbeatCore),
    .evgSequenceStart(injectorSequenceStart),
    .evgPPStoggle(evgPpsToggle_f1),
    .evgSeconds(evgPosixSeconds_f1),
    .evgSecondsNext(evgPosixSecondsNext_f1),
    .evgNtpSeconds(evgNtpSeconds_f1),
    .evgNtpFraction(evgNtpFraction_f1));

evLogger #(.DEBUG("false"))
  evg1LoggerDisplay (
    .sysClk(sysClk),
    .GPIO_OUT(GPIO_OUT),
    .csrStrobe(GPIO_STROBES[GPIO_IDX_EVG_1_DISP_LOG_CSR]),
    .status(GPIO_IN[GPIO_IDX_EVG_1_DISP_LOG_CSR]),
    .evgTxClk(evg1TxClk),
    .evgTxData(evg1TxData),
    .evgTxCharIsK(evg1TxCharIsK));

evFIFO evg1FIFOtlog (
  .sysClk(sysClk),
  .sysCsrStrobe(GPIO_STROBES[GPIO_IDX_EVG_1_TLOG_CSR]),
  .sysGpioOut(GPIO_OUT),
  .sysCsr(GPIO_IN[GPIO_IDX_EVG_1_TLOG_CSR]),
  .sysDataTicks(GPIO_IN[GPIO_IDX_EVG_1_TLOG_TICKS]),
  .evClk(evg1TxClk),
  .evChar(evg1TxData[7:0]),
  .evCharIsK(evg1TxCharIsK[0]));

//////////////////////////////////////////////////////////////////////////////
// NTP server support for F2 domain
wire [31:0] sysNtpSeconds_f2, sysNtpFraction_f2, sysPosixSeconds_f2, sysPosixSecondsNext_f2, sysNtpStatusReg_f2;
wire [31:0] evgNtpSeconds_f2, evgNtpFraction_f2, evgPosixSeconds_f2, evgPosixSecondsNext_f2, evgNtpStatusReg_f2;
wire evgPpsToggle_f2, evgPpsMarker_f2;
wire sysPpsToggle_f2, sysPpsMarker_f2;
wire evgPpsStrobe_f2;
ntpClock #(.CLK_RATE(TXCLK_NOMINAL_FREQUENCY),
           .DEBUG("false"))
  ntpClock_f2 (
    .sysClk(sysClk),
    .writeStrobe(GPIO_STROBES[GPIO_IDX_NTP_SERVER_F2_SECONDS]),
    .writeData(GPIO_OUT),
    .sysPpsToggle(sysPpsToggle_f2),
    .sysPpsMarker(sysPpsMarker_f2),
    .sysSeconds(sysNtpSeconds_f2),
    .sysFraction(sysNtpFraction_f2),
    .sysPosixSeconds(sysPosixSeconds_f2),
    .sysPosixSecondsNext(sysPosixSecondsNext_f2),
    .sysStatus(sysNtpStatusReg_f2),

    .clk(evg2TxClk),
    .pps_a(bestPPS_a),
    .ppsToggle(evgPpsToggle_f2),
    .ppsStrobe(evgPpsStrobe_f2),
    .ppsMarker(evgPpsMarker_f2),
    .seconds(evgNtpSeconds_f2),
    .fraction(evgNtpFraction_f2),
    .posixSeconds(evgPosixSeconds_f2),
    .posixSecondsNext(evgPosixSecondsNext_f2),
    .status(evgNtpStatusReg_f2));

assign GPIO_IN[GPIO_IDX_NTP_SERVER_F2_SECONDS] = sysNtpSeconds_f2;
assign GPIO_IN[GPIO_IDX_NTP_SERVER_F2_FRACTION] = sysNtpFraction_f2;
assign GPIO_IN[GPIO_IDX_NTP_SERVER_F2_STATUS] = sysNtpStatusReg_f2;

/////////////////////////////////////////////////////////////////////////////
// Second generator (accumulator and storage rings)
wire swapoutSequenceStart;
wire evg2HeartbeatAlign, evg2HeartbeatCore;
wire [15:0] evg2TxData;
wire  [1:0] evg2TxCharIsK;
swapoutSequenceControl #(
    .ALIGNMENT_SYNC_COUNT(CFG_EVG2_HEARTBEAT_COUNT),
    .TX_CLK_PER_ALIGNMENT(CFG_EVG2_CLOCK_PER_AR_SR_COINCIDENCE))
  swapoutSequenceControl (
    .sysClk(sysClk),
    .sysGPIO_OUT(GPIO_OUT),
    .sysCsrStrobe(GPIO_STROBES[GPIO_IDX_SWAPOUT_CYCLE_CSR]),
    .sysStatus(GPIO_IN[GPIO_IDX_SWAPOUT_CYCLE_CSR]),
    .sysCsrAlignStrobe(GPIO_STROBES[GPIO_IDX_SWAPOUT_ALIGN_CSR]),
    .sysAlignStatus(GPIO_IN[GPIO_IDX_SWAPOUT_ALIGN_CSR]),
    .evgTxClk(evg2TxClk),
    .evgHeartbeat(evg2HeartbeatRequest),
    .evgHeartbeatAlign(evg2HeartbeatAlign),
    .evgHeartbeatCore(evg2HeartbeatCore),
    .evgSequenceStart(swapoutSequenceStart));

wire evg2RxClkOut;
wire evg2TxClkOut;
wire evg2RxClkIn;
wire evg2TxClkIn;
wire evg2RefClkUnbuf;
IBUFDS_GTE2 evg2RefBuf (.I(MGT_CLK_1_P), .IB(MGT_CLK_1_N), .O(evg2RefClkUnbuf));
BUFG f2BUFG (.I(evg2RefClkUnbuf), .O(evg2RefClk));

mgtWrapper #(.EVG(2),
             .SAMPLING_CLOCK_RATE(500000000),
             .DEBUG("false"),
             .DRP_DEBUG("false"))
  evg2mgt (
    .sysClk(sysClk),
    .GPIO_OUT(GPIO_OUT),
    .drpStrobe(GPIO_STROBES[GPIO_IDX_EVG_2_0_DRP_CSR]),
    .drpStatus(GPIO_IN[GPIO_IDX_EVG_2_0_DRP_CSR]),
    .latency(GPIO_IN[GPIO_IDX_EVG_2_0_LATENCY]),
    .evgTxClkIn(evg2TxClkIn),
    .evgTxClkOut(evg2TxClkOut),
    .evgTxData(evg2TxData),
    .evgTxCharIsK(evg2TxCharIsK),
    .refClk(evg2RefClkUnbuf),
    .samplingClk(clkLatencySampler),
    .gt0_qplloutclk_i(gt0_qplloutclk_i),
    .gt0_qplloutrefclk_i(gt0_qplloutrefclk_i),
    .tx_p(MGT_TX_2_P),
    .tx_n(MGT_TX_2_N),
    .evgRxClkIn(evg2RxClkIn),
    .evgRxClkOut(evg2RxClkOut),
    .rx_p(MGT_RX_2_P),
    .rx_n(MGT_RX_2_N));

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
BUFG evg2RxBuf (.I(evg2RxClkOut), .O(evg2RxClk));
BUFG evg2TxBuf (.I(evg2TxClkOut), .O(evg2TxClk));

assign evg2RxClkIn = evg2RxClk;
assign evg2TxClkIn = evg2TxClk;

wire [CFG_HARDWARE_TRIGGER_COUNT-1:0] FMC2_hwTrigger_remap =
    {FMC2_hwTrigger[3], FMC2_hwTrigger[2], FMC2_hwTrigger[1], FMC2_hwTrigger[0],
     1'b0, 1'b0, FMC2_hwTrigger[5], FMC2_hwTrigger[4]};

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
    .sysSequencerStatusNtpSeconds(GPIO_IN[GPIO_IDX_EVG_2_SEQ_SECONDS_CSR]),
    .sysSequencerStatusNtpFraction(GPIO_IN[GPIO_IDX_EVG_2_SEQ_FRACTION_CSR]),
    .sysSequenceReadback(GPIO_IN[GPIO_IDX_EVG_2_SEQ_RBK]),
    .sysHardwareTriggerStatus(GPIO_IN[GPIO_IDX_EVG_2_HW_CSR]),
    .sysSoftwareTriggerStatus(GPIO_IN[GPIO_IDX_EVG_2_SW_CSR]),
    .sysSequencerStatusFIFOCSRstrobe(GPIO_STROBES[GPIO_IDX_EVG_2_SEQ_STATUS_FIFO_CSR]),
    .sysSequencerStatusFifo(GPIO_IN[GPIO_IDX_EVG_2_SEQ_STATUS_FIFO_CSR]),
    .hwTriggers_a(FMC2_hwTrigger_remap),
    .evgTxClk(evg2TxClk),
    .evgTxData(evg2TxData),
    .evgTxCharIsK(evg2TxCharIsK),
    .evgHeartbeatRequest(evg2HeartbeatCore),
    .evgSequenceStart(swapoutSequenceStart),
    .evgPPStoggle(evgPpsToggle_f2),
    .evgSeconds(evgPosixSeconds_f2),
    .evgSecondsNext(evgPosixSecondsNext_f2),
    .evgNtpSeconds(evgNtpSeconds_f2),
    .evgNtpFraction(evgNtpFraction_f2));

evLogger #(.DEBUG("false"))
  evg2LoggerDisplay (
    .sysClk(sysClk),
    .GPIO_OUT(GPIO_OUT),
    .csrStrobe(GPIO_STROBES[GPIO_IDX_EVG_2_DISP_LOG_CSR]),
    .status(GPIO_IN[GPIO_IDX_EVG_2_DISP_LOG_CSR]),
    .evgTxClk(evg2TxClk),
    .evgTxData(evg2TxData),
    .evgTxCharIsK(evg2TxCharIsK));

evFIFO evg2FIFOtlog (
  .sysClk(sysClk),
  .sysCsrStrobe(GPIO_STROBES[GPIO_IDX_EVG_2_TLOG_CSR]),
  .sysGpioOut(GPIO_OUT),
  .sysCsr(GPIO_IN[GPIO_IDX_EVG_2_TLOG_CSR]),
  .sysDataTicks(GPIO_IN[GPIO_IDX_EVG_2_TLOG_TICKS]),
  .evClk(evg2TxClk),
  .evChar(evg2TxData[7:0]),
  .evCharIsK(evg2TxCharIsK[0]));

/////////////////////////////////////////////////////////////////////////////
// Measure clock rates
localparam FREQ_COUNTERS_NUM = 9;
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
    .unk_clk({ethernetRxClk, ethernetTxClk,
              evg2RxClk, evg2TxClk, evg2RefClk,
              evg1RxClk, evg1TxClk, evg1RefClk,
              sysClk}),
    .refclk(sysClk),
    .refMarker(ppsMarkerValid ? ppsMarker : sysPPSmarker),
    .source_state(),
    .addr(frequencyMonitorSelect),
    .frequency(measuredFrequency));

/////////////////////////////////////////////////////////////////////////////
// Measure fan speeds
fanTach #(.CLK_FREQUENCY(SYSCLK_FREQUENCY),
          .FAN_COUNT(CFG_FAN_COUNT))
  fanTachs (
    .clk(sysClk),
    .csrStrobe(GPIO_STROBES[GPIO_IDX_FAN_TACHOMETERS]),
    .GPIO_OUT(GPIO_OUT),
    .value(GPIO_IN[GPIO_IDX_FAN_TACHOMETERS]),
    .tachs_a({FMC2_FAN2_TACH, FMC2_FAN1_TACH,
                FMC1_FAN2_TACH, FMC1_FAN1_TACH}));

//////////////////////////////////////////////////////////////////////////////
// EVG 1 Rates generation

localparam NUM_EVG1_COUNTERS = 6;

wire RFf1CoincClockSynced, BRARCoincClockSynced, BROrbitClockDiv4ClockSynced;
wire BRARAlignClockSynced, BRARCoincPerRFCoincClockSynced, BRARAlignPerBRARCoincClockSynced;

wire RFf1CoincClock, BRARCoincClock, BROrbitClockDiv4Clock;
wire BRARAlignClock, BRARCoincPerRFCoincClock, BRARAlignPerBRARCoincClock;

wire RFf1CoincStrobe, BRARCoincStrobe, BROrbitStrobeDiv4Strobe;
wire BRARAlignStrobe, BRARCoincPerRFCoincStrobe, BRARAlignPerBRARCoincStrobe;

wire [23:0] RFf1CoincCounter, BRARCoincCounter, BROrbitCounterDiv4Counter;
wire [23:0] BRARAlignCounter, BRARCoincPerRFCoincCounter, BRARAlignPerBRARCoincCounter;

wire [NUM_EVG1_COUNTERS-1:0] evg1CountersEn = {RFf1CoincStrobe, BRARAlignStrobe,
                                                {4{1'b1}}};

evgCounters #(
    .SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
    .DEBUG("false"),
    .NUM_COUNTERS(NUM_EVG1_COUNTERS),
    .DEFAULT_RATE_COUNTS({
        CFG_EVG1_BR_AR_COINC_PER_RF_COINC,
        CFG_EVG1_BR_AR_ALIGN_PER_BR_AR_COINC,
        CFG_EVG1_CLK_PER_TICKS_COINCIDENCE,
        CFG_EVG1_CLK_PER_BR_AR_COINCIDENCE,
        CFG_EVG1_CLK_PER_BR_ORBIT_CLOCK_DIV4,
        CFG_EVG1_CLK_PER_BR_AR_ALIGNMENT}))
  evg1Counters (
    .sysClk(sysClk),
    .GPIO_OUT(GPIO_OUT),

    .csrStrobes(0),
    .csrs({
        GPIO_IN[GPIO_IDX_EVG_1_CLK_GEN_6_CSR],
        GPIO_IN[GPIO_IDX_EVG_1_CLK_GEN_5_CSR],
        GPIO_IN[GPIO_IDX_EVG_1_CLK_GEN_4_CSR],
        GPIO_IN[GPIO_IDX_EVG_1_CLK_GEN_3_CSR],
        GPIO_IN[GPIO_IDX_EVG_1_CLK_GEN_2_CSR],
        GPIO_IN[GPIO_IDX_EVG_1_CLK_GEN_1_CSR]}),

    .clk(evg1TxClk),
    .ens(evg1CountersEn),
    .heartbeatStrobe(evg1HeartbeatAlign),
    .pulsePerSecondStrobe(evgPpsStrobe_f1),

    .clkGenSynceds({
        BRARCoincPerRFCoincClockSynced,
        BRARAlignPerBRARCoincClockSynced,
        RFf1CoincClockSynced,
        BRARCoincClockSynced,
        BROrbitClockDiv4ClockSynced,
        BRARAlignClockSynced}),
    .clkGens({
        BRARCoincPerRFCoincClock,
        BRARAlignPerBRARCoincClock,
        RFf1CoincClock,
        BRARCoincClock,
        BROrbitClockDiv4Clock,
        BRARAlignClock}),
    .clkGenStrobes({
        BRARCoincPerRFCoincStrobe,
        BRARAlignPerBRARCoincStrobe,
        RFf1CoincStrobe,
        BRARCoincStrobe,
        BROrbitStrobeDiv4Strobe,
        BRARAlignStrobe}),
    .clkGenCounters({
        BRARCoincPerRFCoincCounter,
        BRARAlignPerBRARCoincCounter,
        RFf1CoincCounter,
        BRARCoincCounter,
        BROrbitCounterDiv4Counter,
        BRARAlignCounter})
    );

//////////////////////////////////////////////////////////////////////////////
// EVG 2 Rates generation

localparam NUM_EVG2_COUNTERS = 3;

wire ARSRCoincClock;
wire SROrbitClock;
wire AROrbitClock;
wire ARSRCoincClockSynced;
wire SROrbitClockSynced;
wire AROrbitClockSynced;

evgCounters #(
    .SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
    .DEBUG("false"),
    .NUM_COUNTERS(NUM_EVG2_COUNTERS),
    .DEFAULT_RATE_COUNTS({
        CFG_EVG2_CLOCK_PER_AR_SR_COINCIDENCE,
        CFG_EVG2_CLOCK_PER_SR_ORBIT_CLOCK,
        CFG_EVG2_CLOCK_PER_AR_ORBIT_CLOCK}))
  evg2Counters (
    .sysClk(sysClk),
    .GPIO_OUT(GPIO_OUT),

    .csrStrobes(0),
    .csrs({
        GPIO_IN[GPIO_IDX_EVG_2_CLK_GEN_3_CSR],
        GPIO_IN[GPIO_IDX_EVG_2_CLK_GEN_2_CSR],
        GPIO_IN[GPIO_IDX_EVG_2_CLK_GEN_1_CSR]}),

    .clk(evg2TxClk),
    .ens({NUM_EVG2_COUNTERS{1'b1}}),
    .heartbeatStrobe(evg2HeartbeatAlign),
    .pulsePerSecondStrobe(evgPpsStrobe_f2),

    .clkGenSynceds({
        ARSRCoincClockSynced,
        SROrbitClockSynced,
        AROrbitClockSynced}),
    .clkGens({
        ARSRCoincClock,
        SROrbitClock,
        AROrbitClock}),
    .clkGenStrobes());

//////////////////////////////////////////////////////////////////////////////
// Diagnostic I/O
localparam DIAG1_SELECT_WIDTH = 3;
wire [CFG_EVIO_DIAG_OUT_COUNT-1:0] diagnostic1ProgrammableOutputs;
wire [DIAG1_SELECT_WIDTH-1:0] diagnostic1Select;
wire FMC1_auxSwitch_n, FMC2_auxSwitch_n;
diagnosticIO #(.INPUT_WIDTH(CFG_EVIO_DIAG_IN_COUNT),
               .OUTPUT_WIDTH(CFG_EVIO_DIAG_OUT_COUNT),
               .OUTPUT_SELECT_WIDTH(DIAG1_SELECT_WIDTH))
  fmc1IO (
    .sysClk(sysClk),
    .csrStrobe(GPIO_STROBES[GPIO_IDX_FMC1_DIAGNOSTIC]),
    .GPIO_OUT(GPIO_OUT),
    .status(GPIO_IN[GPIO_IDX_FMC1_DIAGNOSTIC]),
    .auxSwitch_n(FMC1_auxSwitch_n),
    .diagnosticIn(FMC1_diagnosticIn),
    .diagnosticOut(diagnostic1ProgrammableOutputs),
    .diagnosticOutputSelect(diagnostic1Select));
assign FMC1_diagnosticOut =
     (diagnostic1Select == 3'h1) ? { evg1RefClk, evg1TxClk } :
     (diagnostic1Select == 3'h2) ? { evg1HeartbeatCore, evg1TxClk } :
     (diagnostic1Select == 3'h3) ? { BRARAlignClock, evg1CoincidenceMarker } :
     (diagnostic1Select == 3'h4) ? { evg1HeartbeatAlign, BROrbitClockDiv4Clock} :
     (diagnostic1Select == 3'h5) ? { evg1HeartbeatAlign, BRARAlignClock} :
     (diagnostic1Select == 3'h6) ? { evg1HeartbeatAlign, BRARCoincClock} :
     (diagnostic1Select == 3'h7) ? { BRARAlignClock, BRARCoincClock} :
                                     diagnostic1ProgrammableOutputs;

localparam DIAG2_SELECT_WIDTH = 3;
wire [CFG_EVIO_DIAG_OUT_COUNT-1:0] diagnostic2ProgrammableOutputs;
wire [DIAG2_SELECT_WIDTH-1:0] diagnostic2Select;
diagnosticIO #(.INPUT_WIDTH(CFG_EVIO_DIAG_IN_COUNT),
               .OUTPUT_WIDTH(CFG_EVIO_DIAG_OUT_COUNT),
               .OUTPUT_SELECT_WIDTH(DIAG2_SELECT_WIDTH))
  fmc2IO (
    .sysClk(sysClk),
    .csrStrobe(GPIO_STROBES[GPIO_IDX_FMC2_DIAGNOSTIC]),
    .GPIO_OUT(GPIO_OUT),
    .status(GPIO_IN[GPIO_IDX_FMC2_DIAGNOSTIC]),
    .auxSwitch_n(FMC1_auxSwitch_n),
    .diagnosticIn(FMC2_diagnosticIn),
    .diagnosticOut(diagnostic2ProgrammableOutputs),
    .diagnosticOutputSelect(diagnostic2Select));
assign FMC2_diagnosticOut =
     (diagnostic2Select == 3'h1) ? { evg2RefClk, evg2TxClk } :
     (diagnostic2Select == 3'h2) ? { evg2HeartbeatCore, evg2TxClk } :
     (diagnostic2Select == 3'h3) ? { AROrbitClock, evg2CoincidenceMarker } :
     (diagnostic2Select == 3'h4) ? { evg2HeartbeatAlign, AROrbitClock } :
     (diagnostic2Select == 3'h5) ? { evg2HeartbeatAlign, SROrbitClock } :
     (diagnostic2Select == 3'h6) ? { evg2HeartbeatAlign, ARSRCoincClock } :
     (diagnostic2Select == 3'h7) ? { AROrbitClock, ARSRCoincClock } :
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
(*MARK_DEBUG="false"*) wire fmc1_scl_i, fmc1_sda_i, fmc2_scl_i, fmc2_sda_i;
(*MARK_DEBUG="false"*) wire [CFG_EVIO_FIREFLY_COUNT/2:0] evio_iic_gpo;

IOBUF FMC1_EVIO_SCL_IOBUF (.I(evio_iic_scl_o),
                          .IO(FMC1_EVIO_SCL),
                          .O(fmc1_scl_i),
                          .T(evio_iic_scl_t | evio_iic_gpo[0]));
IOBUF FMC1_EVIO_SDA_IOBUF (.I(evio_iic_sda_o),
                          .IO(FMC1_EVIO_SDA),
                          .O(fmc1_sda_i),
                          .T(evio_iic_sda_t | evio_iic_gpo[0]));
IOBUF FMC2_EVIO_SCL_IOBUF (.I(evio_iic_scl_o),
                          .IO(FMC2_EVIO_SCL),
                          .O(fmc2_scl_i),
                          .T(evio_iic_scl_t | !evio_iic_gpo[0]));
IOBUF FMC2_EVIO_SDA_IOBUF (.I(evio_iic_sda_o),
                          .IO(FMC2_EVIO_SDA),
                          .O(fmc2_sda_i),
                          .T(evio_iic_sda_t | !evio_iic_gpo[0]));

assign evio_iic_scl_i = evio_iic_gpo[0] ? fmc2_scl_i : fmc1_scl_i;
assign evio_iic_sda_i = evio_iic_gpo[0] ? fmc2_sda_i : fmc1_sda_i;

assign FMC1_fireflySelect_n = ~evio_iic_gpo[CFG_EVIO_FIREFLY_COUNT/2:1];
assign FMC2_fireflySelect_n = ~evio_iic_gpo[CFG_EVIO_FIREFLY_COUNT/2:1];

assign GPIO_IN[GPIO_IDX_FMC1_FIREFLY] = {{32-CFG_EVIO_FIREFLY_COUNT{1'b0}},
                                         FMC1_fireflyPresent_n };
assign GPIO_IN[GPIO_IDX_FMC2_FIREFLY] = {{32-CFG_EVIO_FIREFLY_COUNT{1'b0}},
                                         FMC2_fireflyPresent_n };

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
    .gps_uart_rxd(PMOD1_2),

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
    .clk(evg1TxClk),
    .probe0(probe)
);
`endif

assign probe[0] = gpsPPS_a;
assign probe[1] = gpsPPSvalid;
assign probe[2] = bestPPS_a;
assign probe[3] = sysPpsToggle_f1;
assign probe[4] = sysPpsMarker_f1;
assign probe[5] = sysPpsToggle_f2;
assign probe[6] = sysPpsMarker_f2;

assign probe[7]  = BRARCoincPerRFCoincStrobe;
assign probe[8]  = BRARAlignPerBRARCoincStrobe;
assign probe[9] = RFf1CoincStrobe;
assign probe[10] = BRARCoincStrobe;
assign probe[11] = BROrbitStrobeDiv4Strobe;
assign probe[12] = BRARAlignStrobe;

assign probe[13] = BRARCoincPerRFCoincClockSynced;
assign probe[14] = BRARAlignPerBRARCoincClockSynced;
assign probe[15] = RFf1CoincClockSynced;
assign probe[16] = BRARCoincClockSynced;
assign probe[17] = BROrbitClockDiv4ClockSynced;
assign probe[18] = BRARAlignClockSynced;

assign probe[31:19] = 0;

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

assign probe[151:128] = BRARCoincPerRFCoincCounter;
assign probe[175:152] = BRARAlignPerBRARCoincCounter;
assign probe[199:176] = RFf1CoincCounter;
assign probe[223:200] = BRARCoincCounter;
assign probe[247:224] = BRARAlignCounter;

assign probe[255:248] = 0;

end // end if
endgenerate

endmodule
