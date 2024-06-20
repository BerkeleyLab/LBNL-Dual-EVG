// The top level wrapper for a single event generator module.
// Used to include transceiver wrapper, but no longer now that
// transceiver wrappers differ.

module evg #(
    parameter SYSCLK_FREQUENCY             = 100000000,
    parameter TXCLK_NOMINAL_FREQUENCY      = 125000000,
    parameter TOD_SECONDS_WIDTH            = 32,
    parameter DISTRIBUTED_BUS_WIDTH        = 8,
    parameter GPIO_WIDTH                   = 32,
    parameter SEQUENCE_RAM_CAPACITY        = -1,
    parameter HARDWARE_TRIGGER_COUNT       = 4,
    parameter DEBUG                        = "false"
    ) (
    input                  sysClk,
    input [GPIO_WIDTH-1:0] sysGPIO_OUT,

    input                  sysSequencerCSRstrobe,
    input                  sysHardwareTriggerCSRstrobe,
    input                  sysSoftwareTriggerCSRstrobe,
    input                  sysPPStoggle,
    input           [31:0] sysSeconds,
    input           [31:0] sysSecondsNext,

    output wire [GPIO_WIDTH-1:0] sysSequencerStatus,
    output wire [GPIO_WIDTH-1:0] sysSequenceReadback,
    output wire [GPIO_WIDTH-1:0] sysHardwareTriggerStatus,
    output wire [GPIO_WIDTH-1:0] sysSoftwareTriggerStatus,

    input [HARDWARE_TRIGGER_COUNT-1:0] hwTriggers_a,

    (*mark_debug=DEBUG*) input              evgTxClk,
    (*mark_debug=DEBUG*) output wire [15:0] evgTxData,
    (*mark_debug=DEBUG*) output wire  [1:0] evgTxCharIsK,
    (*mark_debug=DEBUG*) input              evgHeartbeatRequest,
    (*mark_debug=DEBUG*) input              evgSequenceStart);

//
// Place 1 clock cycle heartbeat on LSB of distributed
// bus as required by user timing receivers.  Assume that
// hearbeat interval is no more than four seconds or so.
//
reg evgHeartbeat = 0;
always @(posedge evgTxClk) begin
    if (evgHeartbeatRequest) begin
        evgHeartbeat <= 1;
    end
    else begin
        evgHeartbeat <= 0;
    end
end

//
// Place 1 clock cycle ~100 kHz on bit 1 of distributed
// bus for use as round-trip latency measurement 'ping'.
//
localparam PING_COUNTER_RELOAD = ((TXCLK_NOMINAL_FREQUENCY / 100000) / 2) - 2;
localparam PING_COUNTER_WIDTH = $clog2(PING_COUNTER_RELOAD+1) + 1;
reg [PING_COUNTER_WIDTH-1:0] evgPingCounter = PING_COUNTER_RELOAD;
wire evgPingCounterDone = evgPingCounter[PING_COUNTER_WIDTH-1];
reg evgPing = 0;
always @(posedge evgTxClk) begin
    if (evgPingCounterDone) begin
        evgPing <= 1;
        evgPingCounter <= PING_COUNTER_RELOAD;
    end
    else begin
        evgPing <= 0;
        evgPingCounter <= evgPingCounter - 1;
    end
end

wire [DISTRIBUTED_BUS_WIDTH-1:0] dBus = {{DISTRIBUTED_BUS_WIDTH-2{1'b0}},
                                        evgPing,
                                        evgHeartbeat};

evgSource #(
    .SYSCLK_FREQUENCY(SYSCLK_FREQUENCY),
    .TXCLK_NOMINAL_FREQUENCY(TXCLK_NOMINAL_FREQUENCY),
    .TOD_SECONDS_WIDTH(TOD_SECONDS_WIDTH),
    .GPIO_WIDTH(GPIO_WIDTH),
    .SEQUENCE_RAM_CAPACITY(SEQUENCE_RAM_CAPACITY),
    .HARDWARE_TRIGGER_COUNT(HARDWARE_TRIGGER_COUNT),
    .DEBUG(DEBUG))
  evgs (
    .sysClk(sysClk),
    .sysGPIO_OUT(sysGPIO_OUT),
    .sysSequencerCSRstrobe(sysSequencerCSRstrobe),
    .sysHardwareTriggerCSRstrobe(sysHardwareTriggerCSRstrobe),
    .sysSoftwareTriggerCSRstrobe(sysSoftwareTriggerCSRstrobe),
    .sysSequencerStatus(sysSequencerStatus),
    .sysSequenceReadback(sysSequenceReadback),
    .sysHardwareTriggerStatus(sysHardwareTriggerStatus),
    .sysSoftwareTriggerStatus(sysSoftwareTriggerStatus),
    .sysPPStoggle(sysPPStoggle),
    .sysSeconds(sysSeconds),
    .sysSecondsNext(sysSecondsNext),
    .hwTriggers_a(hwTriggers_a),
    .evgHeartbeatRequest(evgHeartbeatRequest),
    .evgSequenceStart(evgSequenceStart),
    .evgDistributedBus(dBus),
    .evgTxClk(evgTxClk),
    .evgTxData(evgTxData),
    .evgTxCharIsK(evgTxCharIsK));

endmodule
