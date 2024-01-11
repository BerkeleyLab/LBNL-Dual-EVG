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
wire FMC1_CLK0_M2C, FMC2_CLK0_M2C;
IBUFDS f1IBUF (.I(FMC1_CLK0_M2C_P), .IB(FMC1_CLK0_M2C_N),
               .O(FMC1_CLK0_M2C));
IBUFDS f2IBUF (.I(FMC2_CLK0_M2C_P), .IB(FMC2_CLK0_M2C_N),
               .O(FMC2_CLK0_M2C));

BUFG f1BUFG (.I(FMC1_CLK0_M2C), .O(evg1RefClk));
BUFG f2BUFG (.I(FMC2_CLK0_M2C), .O(evg2RefClk));

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

wire evg1RxClkOut;
wire evg1TxClkOut;
wire evg1RxClkIn;
wire evg1TxClkIn;
wire evg1RefClkUnbuf;
IBUFDS_GTE2 evg1RefBuf (.I(MGT_CLK_0_P), .IB(MGT_CLK_0_N), .O(evg1RefClkUnbuf));

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
    .sysSequenceReadback(GPIO_IN[GPIO_IDX_EVG_1_SEQ_RBK]),
    .sysHardwareTriggerStatus(GPIO_IN[GPIO_IDX_EVG_1_HW_CSR]),
    .sysSoftwareTriggerStatus(GPIO_IN[GPIO_IDX_EVG_1_SW_CSR]),
    .sysPPStoggle(ppsToggle),
    .sysSeconds(posixSeconds),
    .hwTriggers_a(FMC1_hwTrigger),
    .diagnosticIn_a(FMC1_diagnosticIn),
    .evgTxClk(evg1TxClk),
    .evgTxData(evg1TxData),
    .evgTxCharIsK(evg1TxCharIsK),
    .evgHeartbeatRequest(evg1HeartbeatRequest),
    .evgSequenceStart(injectorSequenceStart));

evLogger #(.DEBUG("false"))
  evg1Logger (
    .sysClk(sysClk),
    .GPIO_OUT(GPIO_OUT),
    .csrStrobe(GPIO_STROBES[GPIO_IDX_EVG_1_DISP_LOG_CSR]),
    .status(GPIO_IN[GPIO_IDX_EVG_1_DISP_LOG_CSR]),
    .csrStrobeAux(GPIO_STROBES[GPIO_IDX_EVENT_LOG_1_CSR]),
    .statusAux(GPIO_IN[GPIO_IDX_EVENT_LOG_1_CSR]),
    .sysDataTicks(GPIO_IN[GPIO_IDX_EVENT_LOG_1_TICKS]),
    .debug_led(debug_led),
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

wire evg2RxClkOut;
wire evg2TxClkOut;
wire evg2RxClkIn;
wire evg2TxClkIn;
wire evg2RefClkUnbuf;
IBUFDS_GTE2 evg2RefBuf (.I(MGT_CLK_1_P), .IB(MGT_CLK_1_N), .O(evg2RefClkUnbuf));

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
    .sysSequenceReadback(GPIO_IN[GPIO_IDX_EVG_2_SEQ_RBK]),
    .sysHardwareTriggerStatus(GPIO_IN[GPIO_IDX_EVG_2_HW_CSR]),
    .sysSoftwareTriggerStatus(GPIO_IN[GPIO_IDX_EVG_2_SW_CSR]),
    .sysPPStoggle(ppsToggle),
    .sysSeconds(posixSeconds),
    .hwTriggers_a(FMC2_hwTrigger),
    .diagnosticIn_a(FMC2_diagnosticIn),
    .evgTxClk(evg2TxClk),
    .evgTxData(evg2TxData),
    .evgTxCharIsK(evg2TxCharIsK),
    .evgHeartbeatRequest(evg2HeartbeatRequest),
    .evgSequenceStart(swapoutSequenceStart));

evLogger #(.DEBUG("false"))
  evg2Logger (
    .sysClk(sysClk),
    .GPIO_OUT(GPIO_OUT),
    .csrStrobe(GPIO_STROBES[GPIO_IDX_EVG_2_DISP_LOG_CSR]),
    .status(GPIO_IN[GPIO_IDX_EVG_2_DISP_LOG_CSR]),
    .csrStrobeLogger2(GPIO_STROBES[GPIO_IDX_EVG_2_TLOG_CSR]),
    .statusLogger2(GPIO_IN[GPIO_IDX_EVG_2_TLOG_CSR]),
    .sysDataTicks(GPIO_IN[GPIO_IDX_EVG_2_TLOG_TICKS]),
    .evgTxClk(evg2TxClk),
    .evgTxData(evg2TxData),
    .evgTxCharIsK(evg2TxCharIsK));

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
// Diagnostic I/O
wire [CFG_EVIO_DIAG_OUT_COUNT-1:0] diagnostic1ProgrammableOutputs;
wire                             [1:0] diagnostic1Select;
wire FMC1_auxSwitch_n, FMC2_auxSwitch_n;
diagnosticIO #(.INPUT_WIDTH(CFG_EVIO_DIAG_IN_COUNT),
               .OUTPUT_WIDTH(CFG_EVIO_DIAG_OUT_COUNT),
               .OUTPUT_SELECT_WIDTH(2))
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
     (diagnostic1Select == 2'h1) ? { evg1RefClk, evg1TxClk } :
     (diagnostic1Select == 2'h2) ? { evg1HeartbeatRequest, evg1TxClk } :
     (diagnostic1Select == 2'h3) ? { evg2HeartbeatRequest, evg1TxClk } :
                                     diagnostic1ProgrammableOutputs;

wire [CFG_EVIO_DIAG_OUT_COUNT-1:0] diagnostic2ProgrammableOutputs;
wire                             [1:0] diagnostic2Select;
diagnosticIO #(.INPUT_WIDTH(CFG_EVIO_DIAG_IN_COUNT),
               .OUTPUT_WIDTH(CFG_EVIO_DIAG_OUT_COUNT),
               .OUTPUT_SELECT_WIDTH(2))
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
     (diagnostic2Select == 2'h1) ? { evg2RefClk, evg2TxClk } :
     (diagnostic2Select == 2'h2) ? { evg2HeartbeatRequest, evg2TxClk } :
     (diagnostic2Select == 2'h3) ? { evg1HeartbeatRequest, evg2TxClk } :
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
