// Generate hardware-iniated event requests

module evgHardwareTrigger #(
    parameter HARDWARE_TRIGGER_COUNT = -1,
    parameter EVENTCODE_WIDTH = 8
    ) (
    // Processor block connections
    input              sysClk,
    input              sysCSRstrobe,
    input       [31:0] sysGPIO_OUT,
    output wire [31:0] status,

    // Synchronization
    input                              evgTxClk,
    input [HARDWARE_TRIGGER_COUNT-1:0] hwTriggers_a,

    // Event requests
    output reg [EVENTCODE_WIDTH-1:0] evgHardwareEventTDATA,
    output reg                       evgHardwareEventTVALID = 0,
    input                            evgHardwareEventTREADY);

localparam ADDRESS_WIDTH = $clog2(HARDWARE_TRIGGER_COUNT);

localparam EVCODE_IDLE = {EVENTCODE_WIDTH{1'b0}};

reg [(HARDWARE_TRIGGER_COUNT*EVENTCODE_WIDTH)-1:0] eventCodes;
reg [HARDWARE_TRIGGER_COUNT-1:0] eventToggles = 0, eventMatches = 0;

reg sysEventCodeToggle = 0;
(*ASYNC_REG="true"*) reg eventCodeToggle_m = 0;
reg eventCodeToggle = 0, eventCodeToggle_d = 0;

// Per-input debouncing and falling edge detection
genvar chan;
generate
for (chan = 0 ; chan < HARDWARE_TRIGGER_COUNT ; chan = chan + 1) begin : hw
    (*ASYNC_REG="true"*) reg in_m;
    reg in_s, in_d;
    reg [7:0] debounceCounter = 0;
    wire debounceDone = debounceCounter[6];
    wire [EVENTCODE_WIDTH-1:0] myCode;
    assign myCode = eventCodes[chan*EVENTCODE_WIDTH+:EVENTCODE_WIDTH];
    always @(posedge evgTxClk) begin
        in_m <= hwTriggers_a[chan];
        in_s <= in_m;
        if (!in_s) begin
            if (debounceDone && (myCode != EVCODE_IDLE)) begin
                eventToggles[chan] <= !eventToggles[chan];
            end
            debounceCounter <= 0;
        end
        else if (!debounceDone) begin
            debounceCounter <= debounceCounter + 1;
        end
    end
end
endgenerate
wire [HARDWARE_TRIGGER_COUNT-1:0] pendingChannels = eventToggles ^ eventMatches;

// Priority encoder
// Not parameterized, but simple
if (HARDWARE_TRIGGER_COUNT != 8) begin
    NeedToChangePriorityEncoderFirmware();
end
wire [ADDRESS_WIDTH-1:0] priorityBitnum = pendingChannels[7] ? 7 :
                                          pendingChannels[6] ? 6 :
                                          pendingChannels[5] ? 5 :
                                          pendingChannels[4] ? 4 :
                                          pendingChannels[3] ? 3 :
                                          pendingChannels[2] ? 2 :
                                          pendingChannels[1] ? 1 : 0;
wire [HARDWARE_TRIGGER_COUNT-1:0] priorityChannelBit = 1 << priorityBitnum;
wire [ADDRESS_WIDTH-1:0] priorityIndex=priorityBitnum;
wire [EVENTCODE_WIDTH-1:0] priorityChannelEvent =
                    eventCodes[priorityIndex*EVENTCODE_WIDTH+:EVENTCODE_WIDTH];

// Serialize event requests
always @(posedge evgTxClk) begin
    eventCodeToggle_m <= sysEventCodeToggle;
    eventCodeToggle   <= eventCodeToggle_m;
    eventCodeToggle_d <= eventCodeToggle;
    if (eventCodeToggle != eventCodeToggle_d) begin
        eventCodes <= sysEventCodes;
    end

    if (evgHardwareEventTVALID) begin
        if (evgHardwareEventTREADY) begin
            evgHardwareEventTVALID <= 0;
        end
    end
    else begin
        if (pendingChannels != 0) begin
            eventMatches <= eventMatches ^ priorityChannelBit;
            evgHardwareEventTVALID <= 1;
            evgHardwareEventTDATA <= priorityChannelEvent;
        end
    end
end

///////////////////////////////////////////////////////////////////////////////
// System clock domain
reg [(HARDWARE_TRIGGER_COUNT * EVENTCODE_WIDTH)-1:0] sysEventCodes;
wire                     sysWr = sysGPIO_OUT[31];
wire [EVENTCODE_WIDTH-1:0] sysEvnt = sysGPIO_OUT[0+:EVENTCODE_WIDTH];
wire [ADDRESS_WIDTH-1:0] sysAddr = sysGPIO_OUT[EVENTCODE_WIDTH+:ADDRESS_WIDTH];
reg [EVENTCODE_WIDTH-1:0] sysRbkEvent;

reg [ADDRESS_WIDTH-1:0] sysAddress;
always @(posedge sysClk) begin
    sysRbkEvent <= sysEventCodes[sysAddress*EVENTCODE_WIDTH+:EVENTCODE_WIDTH];
    if (sysCSRstrobe) begin
        sysAddress <= sysAddr;
        if (sysWr) begin
           sysEventCodes[sysAddr*EVENTCODE_WIDTH+:EVENTCODE_WIDTH] <= sysEvnt;
           sysEventCodeToggle <= !sysEventCodeToggle;
        end
    end
end

assign status = { {16-HARDWARE_TRIGGER_COUNT{1'b0}}, hwTriggers_a,
                  {16-ADDRESS_WIDTH-EVENTCODE_WIDTH{1'b0}},
                  sysAddress, sysRbkEvent };

endmodule
