// Generate transmitter stream
//
// Nets with names beginning with sys are in transmitter clock domain.
// All other nets are in transmitter clock domain.

module evgCore #(
    parameter SYSCLK_FREQUENCY        = 100000000,
    parameter TXCLK_NOMINAL_FREQUENCY = 125000000,
    parameter TOD_SECONDS_WIDTH       = 32   // Y2038 issues?
    ) (
    // Synchronization with external environment
    input        sysPPStoggle,
    input [31:0] sysSeconds,
    input        evgHeartbeatRequest,

    // Transmitter connections
    input              evgTxClk,
    output wire [15:0] evgTxData,
    output wire  [1:0] evgTxCharIsK,

    // Distributed bus
    input [7:0] evgDistributedBus,

    // Event requests
    input   [7:0] evgSequenceEventTDATA,
    input         evgSequenceEventTVALID,
    input   [7:0] evgHardwareEventTDATA,
    input         evgHardwareEventTVALID,
    output wire   evgHardwareEventTREADY,
    input   [7:0] evgSoftwareEventTDATA,
    input         evgSoftwareEventTVALID,
    output wire   evgSoftwareEventTREADY);

localparam EVCODE_TOD_SHIFT_ZERO = 8'h70;
localparam EVCODE_TOD_SHIFT_ONE  = 8'h71;
localparam EVCODE_HEARTBEAT      = 8'h7A;
localparam EVCODE_TOD_MARKER     = 8'h7D;
localparam EVCODE_IDLE           = 8'h00;
localparam EVCODE_K28_5          = 8'hBC;

// PPS
(*ASYNC_REG="true"*) reg evgPPStoggle_m = 0;
reg evgPPStoggle = 0, evgPPStoggle_d = 0;

// Time of day updates
// Start transmission about 785 ms after PPS marker
localparam TOD_DELAY_875_MILLISECONDS = ((TXCLK_NOMINAL_FREQUENCY / 8) * 7) - 1;
// About 1 us between bits
localparam TOD_DELAY_COUNTER_RELOAD = (TXCLK_NOMINAL_FREQUENCY/1000000) - 1;
localparam TOD_COUNTER_WIDTH = $clog2(TOD_DELAY_875_MILLISECONDS+1) + 1;
reg [TOD_COUNTER_WIDTH-1:0] todDelay = 0;
wire todDelayDone = todDelay[TOD_COUNTER_WIDTH-1];
reg todStart = 0, evgPPSrequest = 0;
localparam TOD_BIT_COUNTER_WIDTH = $clog2(TOD_SECONDS_WIDTH)+1;
reg [TOD_BIT_COUNTER_WIDTH-1:0] todBitCounter;
wire todBitCounterDone = todBitCounter[TOD_BIT_COUNTER_WIDTH-1];
reg todRequest = 0;
reg [TOD_SECONDS_WIDTH-1:0] todShiftReg;

// Comma insertion
localparam COMMA_INHIBIT_COUNTER_WIDTH = 3;
localparam COMMA_INHIBIT_COUNTER_RELOAD = 4 - 2; // Comma every 4th slot
reg [COMMA_INHIBIT_COUNTER_WIDTH-1:0] commaInhibitCounter = 0;
wire commaInhibitCounterDone=commaInhibitCounter[COMMA_INHIBIT_COUNTER_WIDTH-1];

// Transmitter stream
// Override distributed bus bit 3 with PPS toggle
reg [7:0] evgTxCode    = 0;
reg       evgTxCodeIsK = 0;
assign evgTxData = { evgDistributedBus[7:4],evgPPStoggle,evgDistributedBus[2:0],
                     evgTxCode };
assign evgTxCharIsK = { 1'b0, evgTxCodeIsK };

// Enable hardware-triggered event if no higher priority event is pending.
assign evgHardwareEventTREADY = !evgSequenceEventTVALID
                             && !evgHeartbeatRequest
                             && !evgPPSrequest;

// Enable software-triggered event if no higher priority event is pending.
assign evgSoftwareEventTREADY = !evgSequenceEventTVALID
                             && !evgHeartbeatRequest
                             && !evgPPSrequest
                             && !evgHardwareEventTVALID;

always @(posedge evgTxClk) begin
    // Request pulse-per-second (time of day update) transmission at PPS marker.
    // Start time of day transmission about 875 ms after PPS marker.
    evgPPStoggle_m <= sysPPStoggle;
    evgPPStoggle   <= evgPPStoggle_m;
    evgPPStoggle_d <= evgPPStoggle;
    if (evgPPStoggle != evgPPStoggle_d) begin
        if (!evgPPSrequest) begin
            evgPPSrequest <= 1;
        end
        todDelay <= TOD_DELAY_875_MILLISECONDS;
        todBitCounter <= TOD_SECONDS_WIDTH - 1;
        todStart <= 1;
    end
    else if (todDelayDone) begin
        if (!todRequest && !todBitCounterDone) begin
            todBitCounter <= todBitCounter - 1;
            if (todStart) begin
                todStart <= 0;
                // Crosses clock domains, but won't be changing at this time.
                todShiftReg <= sysSeconds;
            end
            else begin
                todShiftReg <= {todShiftReg[TOD_SECONDS_WIDTH-2:0], 1'bx};
            end
            todDelay <= TOD_DELAY_COUNTER_RELOAD;
            todRequest <= 1;
        end
    end
    else begin
        todDelay <= todDelay - 1;
    end

    // Limit comma rate
    if (!commaInhibitCounterDone) begin
        commaInhibitCounter <= commaInhibitCounter - 1;
    end

    //
    // Send events in priority order
    //
    if (evgSequenceEventTVALID) begin
        evgTxCode <= evgSequenceEventTDATA;
        evgTxCodeIsK <= 0;
    end
    else if (evgHeartbeatRequest) begin
        evgTxCode <= EVCODE_HEARTBEAT;
        evgTxCodeIsK <= 0;
    end
    else if (evgPPSrequest) begin
        evgTxCode <= EVCODE_TOD_MARKER;
        evgTxCodeIsK <= 0;
        evgPPSrequest <= 0;
    end
    else if (evgHardwareEventTVALID) begin
        evgTxCode <= evgHardwareEventTDATA;
        evgTxCodeIsK <= 0;
    end
    else if (evgSoftwareEventTVALID) begin
        evgTxCode <= evgSoftwareEventTDATA;
        evgTxCodeIsK <= 0;
    end
    else if (todRequest) begin
        evgTxCode <= todShiftReg[TOD_SECONDS_WIDTH-1] ? EVCODE_TOD_SHIFT_ONE :
                                                        EVCODE_TOD_SHIFT_ZERO;
        evgTxCodeIsK <= 0;
        todRequest <= 0;
    end
    else if (commaInhibitCounterDone) begin
        evgTxCode <= EVCODE_K28_5;
        evgTxCodeIsK <= 1;
        commaInhibitCounter <= COMMA_INHIBIT_COUNTER_RELOAD;
    end
    else begin
        evgTxCode <= EVCODE_IDLE;
        evgTxCodeIsK <= 0;
    end
end

endmodule
