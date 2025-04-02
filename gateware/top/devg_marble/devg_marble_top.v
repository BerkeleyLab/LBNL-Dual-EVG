// Top level module for ALSU Event Generator

module devg_marble_top #(
    // Include file is machine generated from C header
    `include "gpioIDX.vh"
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
wire evg1HeartbeatRequest, evg2HeartbeatRequest;
wire sysRealignToggle;
wire evg1CoincidenceMarker, evg2CoincidenceMarker;


localparam EVG1_SAMPLE_COUNTER_WIDTH = $clog2(CFG_EVG2_CLK_PER_RF_COINCIDENCE);
wire [EVG1_SAMPLE_COUNTER_WIDTH-1:0] evg1SampleCounterDbg;

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
    .sysRealignToggle(sysRealignToggle),
    .sysRealignToggleIn(sysRealignToggle),
    .samplingClk(evg2RefClk),
    .sampleCounterDbg(evg1SampleCounterDbg),
    .refClk({evg1TxClk, evg1RefClk}),
    .coincidenceMarker(evg1CoincidenceMarker),
    .txClk(evg1TxClk),
    .txHeartbeatStrobe(evg1HeartbeatRequest));

localparam EVG2_SAMPLE_COUNTER_WIDTH = $clog2(CFG_EVG1_CLK_PER_RF_COINCIDENCE);
wire [EVG2_SAMPLE_COUNTER_WIDTH-1:0] evg2SampleCounterDbg;

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
    .sysRealignToggleIn(sysRealignToggle),
    .sysCsr(GPIO_IN[GPIO_IDX_EVG_2_COINC_CSR]),
    .samplingClk(evg1RefClk),
    .sampleCounterDbg(evg2SampleCounterDbg),
    .refClk({evg2TxClk, evg2RefClk}),
    .coincidenceMarker(evg2CoincidenceMarker),
    .txClk(evg2TxClk),
    .txHeartbeatStrobe(evg2HeartbeatRequest));

wire [255:0] probe;
`ifndef SIMULATE
ila_td256_s4096_cap ila_td256_s4096_cap_inst (
    .clk(evg2RefClk),
    .probe0(probe)
);
`endif

assign probe[0+:EVG1_SAMPLE_COUNTER_WIDTH] = evg1SampleCounterDbg;
assign probe[32+:EVG2_SAMPLE_COUNTER_WIDTH] = evg2SampleCounterDbg;

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

//////////////////////////////////////////////////////////////////////////////
// Buffer EVG1 clocks
BUFG evg1RxBuf (.I(evg1RxClkOut), .O(evg1RxClk));
BUFG evg1TxBuf (.I(evg1TxClkOut), .O(evg1TxClk));

assign evg1RxClkIn = evg1RxClk;
assign evg1TxClkIn = evg1TxClk;

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
    .sysSequencerStatusNtpSeconds(GPIO_IN[GPIO_IDX_EVG_1_SEQ_SECONDS_CSR]),
    .sysSequencerStatusNtpFraction(GPIO_IN[GPIO_IDX_EVG_1_SEQ_FRACTION_CSR]),
    .sysSequenceReadback(GPIO_IN[GPIO_IDX_EVG_1_SEQ_RBK]),
    .sysHardwareTriggerStatus(GPIO_IN[GPIO_IDX_EVG_1_HW_CSR]),
    .sysSoftwareTriggerStatus(GPIO_IN[GPIO_IDX_EVG_1_SW_CSR]),
    .sysSequencerStatusFIFOCSRstrobe(GPIO_STROBES[GPIO_IDX_EVG_1_SEQ_STATUS_FIFO_CSR]),
    .sysSequencerStatusFifo(GPIO_IN[GPIO_IDX_EVG_1_SEQ_STATUS_FIFO_CSR]),
    .hwTriggers_a(FMC1_hwTrigger),
    .evgTxClk(evg1TxClk),
    .evgTxData(evg1TxData),
    .evgTxCharIsK(evg1TxCharIsK),
    .evgHeartbeatRequest(evg1HeartbeatRequest),
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

//////////////////////////////////////////////////////////////////////////////
// Buffer EVG2 clocks
BUFG evg2RxBuf (.I(evg2RxClkOut), .O(evg2RxClk));
BUFG evg2TxBuf (.I(evg2TxClkOut), .O(evg2TxClk));

assign evg2RxClkIn = evg2RxClk;
assign evg2TxClkIn = evg2TxClk;

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
    .hwTriggers_a(FMC2_hwTrigger),
    .evgTxClk(evg2TxClk),
    .evgTxData(evg2TxData),
    .evgTxCharIsK(evg2TxCharIsK),
    .evgHeartbeatRequest(evg2HeartbeatRequest),
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
wire [31:0] BRARAlignClockStatus;
wire BRARAlignClockSynced;
wire BRARAlignClock;
clkGen #(.SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
          .DEFAULT_RATE_COUNT(CFG_EVG1_CLK_PER_BR_AR_ALIGNMENT),
          .DEBUG("false"))
  evgBRARAlignClock(.sysClk(sysClk),
          .csrStrobe(1'b0),
          .GPIO_OUT(GPIO_OUT),
          .csr(BRARAlignClockStatus),

          .evrClk(evg1TxClk),
          .evrHeartbeatMarker(evg1HeartbeatRequest),
          .evrPulsePerSecondMarker(evgPpsMarker_f1),

          .evrClkGenSynced(BRARAlignClockSynced),
          .evrClkGen(BRARAlignClock),
          .evrClkGenStrobe());

wire [31:0] BROrbitClockDiv4ClockStatus;
wire BROrbitClockDiv4ClockSynced;
wire BROrbitClockDiv4Clock;
clkGen #(.SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
          .DEFAULT_RATE_COUNT(CFG_EVG1_CLK_PER_BR_ORBIT_CLOCK_DIV4),
          .DEBUG("false"))
  evgBROrbitClockDiv4Clock(.sysClk(sysClk),
          .csrStrobe(1'b0),
          .GPIO_OUT(GPIO_OUT),
          .csr(BROrbitClockDiv4ClockStatus),

          .evrClk(evg1TxClk),
          .evrHeartbeatMarker(evg1HeartbeatRequest),
          .evrPulsePerSecondMarker(evgPpsMarker_f1),

          .evrClkGenSynced(BROrbitClockDiv4ClockSynced),
          .evrClkGen(BROrbitClockDiv4Clock),
          .evrClkGenStrobe());

wire [31:0] BRARCoincClockStatus;
wire BRARCoincClockSynced;
wire BRARCoincClock;
clkGen #(.SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
          .DEFAULT_RATE_COUNT(CFG_EVG1_CLK_PER_BR_AR_COINCIDENCE),
          .DEBUG("false"))
  evgBRARCoincClock(.sysClk(sysClk),
          .csrStrobe(1'b0),
          .GPIO_OUT(GPIO_OUT),
          .csr(BRARCoincClockStatus),

          .evrClk(evg1TxClk),
          .evrHeartbeatMarker(evg1HeartbeatRequest),
          .evrPulsePerSecondMarker(evgPpsMarker_f1),

          .evrClkGenSynced(BRARCoincClockSynced),
          .evrClkGen(BRARCoincClock),
          .evrClkGenStrobe());

//////////////////////////////////////////////////////////////////////////////
// EVG 2 Rates generation
wire [31:0] AROrbitClockStatus;
wire AROrbitClockSynced;
wire AROrbitClock;
clkGen #(.SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
          .DEFAULT_RATE_COUNT(CFG_EVG2_CLOCK_PER_AR_ORBIT_CLOCK),
          .DEBUG("false"))
  evgAROrbitClock(.sysClk(sysClk),
          .csrStrobe(1'b0),
          .GPIO_OUT(GPIO_OUT),
          .csr(AROrbitClockStatus),

          .evrClk(evg2TxClk),
          .evrHeartbeatMarker(evg2HeartbeatRequest),
          .evrPulsePerSecondMarker(evgPpsMarker_f2),

          .evrClkGenSynced(AROrbitClockSynced),
          .evrClkGen(AROrbitClock),
          .evrClkGenStrobe());

wire [31:0] SROrbitClockStatus;
wire SROrbitClockSynced;
wire SROrbitClock;
clkGen #(.SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
          .DEFAULT_RATE_COUNT(CFG_EVG2_CLOCK_PER_SR_ORBIT_CLOCK),
          .DEBUG("false"))
  evgSROrbitClock(.sysClk(sysClk),
          .csrStrobe(1'b0),
          .GPIO_OUT(GPIO_OUT),
          .csr(SROrbitClockStatus),

          .evrClk(evg2TxClk),
          .evrHeartbeatMarker(evg2HeartbeatRequest),
          .evrPulsePerSecondMarker(evgPpsMarker_f2),

          .evrClkGenSynced(SROrbitClockSynced),
          .evrClkGen(SROrbitClock),
          .evrClkGenStrobe());

wire [31:0] ARSRCoincClockStatus;
wire ARSRCoincClockSynced;
wire ARSRCoincClock;
clkGen #(.SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
          .DEFAULT_RATE_COUNT(CFG_EVG2_CLOCK_PER_ARSR_COINCIDENCE),
          .DEBUG("false"))
  evgARSRCoincClock (.sysClk(sysClk),
          .csrStrobe(1'b0),
          .GPIO_OUT(GPIO_OUT),
          .csr(ARSRCoincClockStatus),

          .evrClk(evg2TxClk),
          .evrHeartbeatMarker(evg2HeartbeatRequest),
          .evrPulsePerSecondMarker(evgPpsMarker_f2),

          .evrClkGenSynced(ARSRCoincClockSynced),
          .evrClkGen(ARSRCoincClock),
          .evrClkGenStrobe());

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
     (diagnostic1Select == 3'h2) ? { evg1HeartbeatRequest, evg1TxClk } :
     (diagnostic1Select == 3'h3) ? { BRARAlignClock, evg1CoincidenceMarker } :
     (diagnostic1Select == 3'h4) ? { evg1HeartbeatRequest, BROrbitClockDiv4Clock} :
     (diagnostic1Select == 3'h5) ? { evg1HeartbeatRequest, BRARAlignClock} :
     (diagnostic1Select == 3'h6) ? { evg1HeartbeatRequest, BRARCoincClock} :
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
     (diagnostic2Select == 3'h2) ? { evg2HeartbeatRequest, evg2TxClk } :
     (diagnostic2Select == 3'h3) ? { AROrbitClock, evg2CoincidenceMarker } :
     (diagnostic2Select == 3'h4) ? { evg2HeartbeatRequest, AROrbitClock } :
     (diagnostic2Select == 3'h5) ? { evg2HeartbeatRequest, SROrbitClock } :
     (diagnostic2Select == 3'h6) ? { evg2HeartbeatRequest, ARSRCoincClock } :
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

endmodule
