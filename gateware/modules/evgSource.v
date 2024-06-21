// Aggregate sub-blocks into single event generator data source

module evgSource #(
    parameter SYSCLK_FREQUENCY             = 100000000,
    parameter TXCLK_NOMINAL_FREQUENCY      = 125000000,
    parameter TOD_SECONDS_WIDTH            = 32,
    parameter GPIO_WIDTH                   = 32,
    parameter SEQUENCE_RAM_CAPACITY        = -1,
    parameter HARDWARE_TRIGGER_COUNT       = 4,
    parameter DISTRIBUTED_BUS_WIDTH        = 8,
    parameter DEBUG                        = "false"
    ) (
    input                  sysClk,
    input [GPIO_WIDTH-1:0] sysGPIO_OUT,

    input                  sysSequencerCSRstrobe,
    input                  sysHardwareTriggerCSRstrobe,
    input                  sysSoftwareTriggerCSRstrobe,
    input                  sysPPStoggle,
    input [GPIO_WIDTH-1:0] sysSeconds,
    input [GPIO_WIDTH-1:0] sysSecondsNext,
    input [GPIO_WIDTH-1:0] sysNtpSeconds,
    input [GPIO_WIDTH-1:0] sysNtpFraction,

    output wire [GPIO_WIDTH-1:0] sysSequencerStatus,
    output wire [GPIO_WIDTH-1:0] sysSequencerStatusNtpSeconds,
    output wire [GPIO_WIDTH-1:0] sysSequencerStatusNtpFraction,
    output wire [GPIO_WIDTH-1:0] sysSequenceReadback,

    output wire [GPIO_WIDTH-1:0] sysHardwareTriggerStatus,
    output wire [GPIO_WIDTH-1:0] sysSoftwareTriggerStatus,

    input  [HARDWARE_TRIGGER_COUNT-1:0] hwTriggers_a,
    input                               evgHeartbeatRequest,
    input                               evgSequenceStart,

    // Distributed bus
    input [DISTRIBUTED_BUS_WIDTH-1:0] evgDistributedBus,

    // Transmitter connections
    input              evgTxClk,
    output wire [15:0] evgTxData,
    output wire  [1:0] evgTxCharIsK);

localparam EVENTCODE_WIDTH       = 8;

wire [EVENTCODE_WIDTH-1:0] evgSequenceEventTDATA;
wire                       evgSequenceEventTVALID;
wire [EVENTCODE_WIDTH-1:0] evgHardwareEventTDATA;
wire                       evgHardwareEventTVALID;
wire                       evgHardwareEventTREADY;
wire [EVENTCODE_WIDTH-1:0] evgSoftwareEventTDATA;
wire                       evgSoftwareEventTVALID;
wire                       evgSoftwareEventTREADY;

evgCore #(.SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
          .TXCLK_NOMINAL_FREQUENCY(TXCLK_NOMINAL_FREQUENCY),
          .TOD_SECONDS_WIDTH(TOD_SECONDS_WIDTH))
  evgCore (
    .sysPPStoggle(sysPPStoggle),
    .sysSeconds(sysSeconds),
    .sysSecondsNext(sysSecondsNext),
    .evgHeartbeatRequest(evgHeartbeatRequest),
    .evgTxClk(evgTxClk),
    .evgTxData(evgTxData),
    .evgTxCharIsK(evgTxCharIsK),
    .evgDistributedBus(evgDistributedBus),
    .evgSequenceEventTDATA(evgSequenceEventTDATA),
    .evgSequenceEventTVALID(evgSequenceEventTVALID),
    .evgHardwareEventTDATA(evgHardwareEventTDATA),
    .evgHardwareEventTVALID(evgHardwareEventTVALID),
    .evgHardwareEventTREADY(evgHardwareEventTREADY),
    .evgSoftwareEventTDATA(evgSoftwareEventTDATA),
    .evgSoftwareEventTVALID(evgSoftwareEventTVALID),
    .evgSoftwareEventTREADY(evgSoftwareEventTREADY));

evgSequencer #(.SEQUENCE_RAM_CAPACITY(SEQUENCE_RAM_CAPACITY),
               .DEBUG(DEBUG))
   evgSequencer (
    .sysClk(sysClk),
    .sysCSRstrobe(sysSequencerCSRstrobe),
    .sysGPIO_OUT(sysGPIO_OUT),
    .sysNtpSeconds(sysNtpSeconds),
    .sysNtpFraction(sysNtpFraction),
    .status(sysSequencerStatus),
    .statusNtpSeconds(sysSequencerStatusNtpSeconds),
    .statusNtpFraction(sysSequencerStatusNtpFraction),
    .sysSequenceReadback(sysSequenceReadback),
    .evgTxClk(evgTxClk),
    .evgSequenceStart(evgSequenceStart),
    .evgSequenceEventTDATA(evgSequenceEventTDATA),
    .evgSequenceEventTVALID(evgSequenceEventTVALID));

evgHardwareTrigger #(.HARDWARE_TRIGGER_COUNT(HARDWARE_TRIGGER_COUNT))
  evgHardwareTrigger (
    .sysClk(sysClk),
    .sysCSRstrobe(sysHardwareTriggerCSRstrobe),
    .sysGPIO_OUT(sysGPIO_OUT),
    .status(sysHardwareTriggerStatus),
    .evgTxClk(evgTxClk),
    .hwTriggers_a(hwTriggers_a),
    .evgHardwareEventTDATA(evgHardwareEventTDATA),
    .evgHardwareEventTVALID(evgHardwareEventTVALID),
    .evgHardwareEventTREADY(evgHardwareEventTREADY));

evgSoftwareTrigger evgSoftwareTrigger (
    .sysClk(sysClk),
    .sysCSRstrobe(sysSoftwareTriggerCSRstrobe),
    .sysGPIO_OUT(sysGPIO_OUT),
    .status(sysSoftwareTriggerStatus),
    .evgTxClk(evgTxClk),
    .evgSoftwareEventTDATA(evgSoftwareEventTDATA),
    .evgSoftwareEventTVALID(evgSoftwareEventTVALID),
    .evgSoftwareEventTREADY(evgSoftwareEventTREADY));

endmodule
