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

    output wire [GPIO_WIDTH-1:0] sysSequencerStatus,
    output wire [GPIO_WIDTH-1:0] sysSequenceReadback,
    output wire [GPIO_WIDTH-1:0] sysHardwareTriggerStatus,
    output wire [GPIO_WIDTH-1:0] sysSoftwareTriggerStatus,

    input [HARDWARE_TRIGGER_COUNT-1:0] hwTriggers_a,
    input                              diagnosticIn_a,

    (*mark_debug=DEBUG*) input              evgTxClk,
    (*mark_debug=DEBUG*) output wire [15:0] evgTxData,
    (*mark_debug=DEBUG*) output wire  [1:0] evgTxCharIsK,
    (*mark_debug=DEBUG*) input              evgHeartbeatRequest,
    (*mark_debug=DEBUG*) input              evgSequenceStart);

//
// Place 50% duty cycle heartbeat on LSB of distributed
// bus as required by user timing receivers.  Assume that
// hearbeat interval is no more than four seconds or so.
//
localparam HB_EXTENDER_WIDTH = $clog2(TXCLK_NOMINAL_FREQUENCY) + 2;
reg [HB_EXTENDER_WIDTH-1:0] evgHbInterval = 0, evgHbExtender = 0;
wire evgHbExtenderMSB = evgHbExtender[HB_EXTENDER_WIDTH-1];
always @(posedge evgTxClk) begin
    if (evgHeartbeatRequest) begin
        evgHbInterval <= 0;
        evgHbExtender <= {1'b1, evgHbInterval[HB_EXTENDER_WIDTH-1:1]};
    end
    else begin
        evgHbInterval <= evgHbInterval + 1;
        if (evgHbExtenderMSB) begin
            evgHbExtender <= evgHbExtender - 1;
        end
    end
end

//
// Place ~100 kHz, 50% duty cycle on bit 1 of distributed
// bus for use as round-trip latency measurement 'ping'.
//
localparam PING_COUNTER_RELOAD = ((TXCLK_NOMINAL_FREQUENCY / 100000) / 2) - 2;
localparam PING_COUNTER_WIDTH = $clog2(PING_COUNTER_RELOAD+1) + 1;
reg [PING_COUNTER_WIDTH-1:0] evgPingCounter = PING_COUNTER_RELOAD;
wire evgPingCounterDone = evgPingCounter[PING_COUNTER_WIDTH-1];
reg evgPing = 0;
always @(posedge evgTxClk) begin
    if (evgPingCounterDone) begin
        evgPingCounter <= PING_COUNTER_RELOAD;
        evgPing <= !evgPing;
    end
    else begin
        evgPingCounter <= evgPingCounter - 1;
    end
end

//
// Place diagnostic input on bit 2 of distributed bus.
//
(*ASYN_REG="true"*) reg evgDiag_m;
reg evgDiag;
always @(posedge evgTxClk) begin
    evgDiag_m <= diagnosticIn_a;
    evgDiag   <= evgDiag_m;
end

wire [DISTRIBUTED_BUS_WIDTH-1:0] dBus = {{DISTRIBUTED_BUS_WIDTH-3{1'b0}},
                                        evgDiag,
                                        evgPing,
                                        evgHbExtenderMSB};

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
    .hwTriggers_a(hwTriggers_a),
    .evgHeartbeatRequest(evgHeartbeatRequest),
    .evgSequenceStart(evgSequenceStart),
    .evgDistributedBus(dBus),
    .evgTxClk(evgTxClk),
    .evgTxData(evgTxData),
    .evgTxCharIsK(evgTxCharIsK));

endmodule
