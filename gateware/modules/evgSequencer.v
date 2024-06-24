// Generate event sequences
//
// Nets with names beginning with sys are in transmitter clock domain.
// All other nets are in transmitter clock domain.

module evgSequencer # (
    parameter SEQUENCE_RAM_CAPACITY = 2048,
    parameter EVENTCODE_WIDTH       = 8,
    parameter DEBUG                 = "false"
    ) (
    // Processor block connections
    // Some readback values are not in the system clock domain.  The
    // software is aware of this and reads until the values are stable.
    input              sysClk,
    input              sysCSRstrobe,
    input       [31:0] sysGPIO_OUT,
    input       [31:0] sysNtpSeconds,
    input       [31:0] sysNtpFraction,
    output reg  [31:0] status,
    output reg  [31:0] statusNtpSeconds,
    output reg  [31:0] statusNtpFraction,
    output reg  [31:0] sysSequenceReadback,

    // Synchronization
    input      evgTxClk,
    input      evgSequenceStart,

    // Event requests
    output reg [EVENTCODE_WIDTH-1:0] evgSequenceEventTDATA,
    output reg                       evgSequenceEventTVALID);

localparam END_OF_TABLE_EVENT_CODE  = 8'h7F;

localparam SEQ_CSR_CMD_SET_ADDRESS = 2'h1;
localparam SEQ_CSR_CMD_LATCH_GAP   = 2'h2;
localparam SEQ_CSR_CMD_WRITE_ENTRY = 2'h3;
wire [1:0] csrCmdCode = sysGPIO_OUT[31:30];

localparam SEQUENCE_ADDRESS_WIDTH = $clog2(SEQUENCE_RAM_CAPACITY);
localparam SEQUENCE_RAM_DATA_WIDTH = SEQUENCE_GAP_WIDTH + EVENTCODE_WIDTH;

localparam SEQUENCE_GAP_WIDTH = 28;
localparam GAP_COUNTER_WIDTH = SEQUENCE_GAP_WIDTH + 1;
reg [GAP_COUNTER_WIDTH-1:0] gapCounter;
wire gapCounterDone = gapCounter[GAP_COUNTER_WIDTH-1];

localparam READ_COUNTER_WIDTH = SEQUENCE_ADDRESS_WIDTH + 1;
reg [READ_COUNTER_WIDTH-1:0] readCounter;
wire readCounterOverflow = readCounter[READ_COUNTER_WIDTH-1];

// Simple dual-port RAM
localparam DPRAM_ADDRESS_WIDTH = SEQUENCE_ADDRESS_WIDTH + 1;
reg [SEQUENCE_RAM_DATA_WIDTH-1:0] sequenceRAM [0:(2*SEQUENCE_RAM_CAPACITY)-1];
reg [SEQUENCE_RAM_DATA_WIDTH-1:0] sysSequenceRAMrbk, sequenceRAMQ;
reg seqSelect = 0;
wire [DPRAM_ADDRESS_WIDTH-1:0] readAddress = { seqSelect,
                           readCounter[0+:SEQUENCE_ADDRESS_WIDTH] +
                           {{SEQUENCE_ADDRESS_WIDTH-1{1'b0}}, gapCounterDone} };
always @(posedge evgTxClk) begin
    sequenceRAMQ <= sequenceRAM[readAddress];
end
wire [SEQUENCE_GAP_WIDTH-1:0] sequenceRAMgap =
                              sequenceRAMQ[EVENTCODE_WIDTH+:SEQUENCE_GAP_WIDTH];
wire [EVENTCODE_WIDTH-1:0] sequenceRAMevent = sequenceRAMQ[0+:EVENTCODE_WIDTH];
reg  [EVENTCODE_WIDTH-1:0] precompletionEvent = ~0;

// Enable/disable requests
(*mark_debug=DEBUG*)
reg [1:0] sysSequenceEnableToggle = 0;
(*ASYNC_REG="true"*) reg [1:0] sequenceEnableToggle_m = 0;
(*mark_debug=DEBUG*)
reg [1:0] sequenceEnableToggle = 0, sequenceEnableMatch = 0;
reg [1:0] sysSequenceDisableToggle = 0;
(*ASYNC_REG="true"*) reg [1:0] sequenceDisableToggle_m = 0;
(*mark_debug=DEBUG*)
reg [1:0] sequenceDisableToggle = 0, sequenceDisableMatch = 0;
reg [1:0] sequenceEnabled = 0;
(*mark_debug=DEBUG*)
reg sysStatusForceWEToggle = 0;
(*ASYNC_REG="true"*) reg statusForceWEToggle_m = 0;
reg statusForceWEToggle = 0, statusForceWEMatch = 0;

// State machine
reg [EVENTCODE_WIDTH-1:0] pendingEvent;
reg sequenceActive = 0, sequenceBusy = 0;
reg [1:0] startDelay = 0;

// Statistics
localparam START_REQUEST_COUNTER_WIDTH = 8;
reg [START_REQUEST_COUNTER_WIDTH-1:0] startRequestsIgnored = 0,
                                      startRequestsAccepted = 0;

// Status logic
reg evgStatusBuffWeToggle = 0;

always @(posedge evgTxClk) begin
    sequenceEnableToggle_m <= sysSequenceEnableToggle;
    sequenceEnableToggle   <= sequenceEnableToggle_m;
    sequenceDisableToggle_m <= sysSequenceDisableToggle;
    sequenceDisableToggle   <= sequenceDisableToggle_m;
    statusForceWEToggle_m <= sysStatusForceWEToggle;
    statusForceWEToggle   <= statusForceWEToggle_m;

    // Force write enable to status register. Useful on startup
    // to have a valid initial value
    if (statusForceWEToggle != statusForceWEMatch) begin
        evgStatusBuffWeToggle <= !evgStatusBuffWeToggle;
        statusForceWEMatch <= statusForceWEToggle;
    end

    if (sequenceActive || !evgSequenceStart) begin
        // Enable status can change at startup so
        // hold off requests when starting.
        if (sequenceDisableToggle[1] != sequenceDisableMatch[1]) begin
            sequenceEnabled[1] <= 0;
            evgStatusBuffWeToggle <= !evgStatusBuffWeToggle;
        end
        else if (sequenceEnableToggle[1] != sequenceEnableMatch[1]) begin
            sequenceEnabled[1] <= 1;
            evgStatusBuffWeToggle <= !evgStatusBuffWeToggle;
        end
        if (sequenceDisableToggle[0] != sequenceDisableMatch[0]) begin
            sequenceEnabled[0] <= 0;
            evgStatusBuffWeToggle <= !evgStatusBuffWeToggle;
        end
        else if (sequenceEnableToggle[0] != sequenceEnableMatch[0]) begin
            sequenceEnabled[0] <= 1;
            evgStatusBuffWeToggle <= !evgStatusBuffWeToggle;
        end
        sequenceEnableMatch <= sequenceEnableToggle;
        sequenceDisableMatch <= sequenceDisableToggle;
    end

    if (sequenceActive) begin
        if (evgSequenceStart) begin
            startRequestsIgnored <= startRequestsIgnored + 1;
            evgStatusBuffWeToggle <= !evgStatusBuffWeToggle;
        end
        if (startDelay[0]) begin
            // Sequence read address valid at this point
            startDelay[0] <= 0;
        end
        else if (startDelay[1]) begin
            // Sequence read data valid at this point
            startDelay[1] <= 0;
            gapCounter <= {1'b0, sequenceRAMgap};
            pendingEvent <= sequenceRAMevent;
            readCounter <= readCounter + 1;
        end
        else if (gapCounterDone) begin
            if ((pendingEvent == END_OF_TABLE_EVENT_CODE)
              || readCounterOverflow) begin
                evgSequenceEventTVALID <= 0;
                sequenceBusy <= 0;
                sequenceActive <= 0;
                evgStatusBuffWeToggle <= !evgStatusBuffWeToggle;
            end
            else begin
                if (pendingEvent == precompletionEvent) begin
                    sequenceBusy <= 0;
                    evgStatusBuffWeToggle <= !evgStatusBuffWeToggle;
                end
                evgSequenceEventTVALID <= 1;
                evgSequenceEventTDATA <= pendingEvent;
                gapCounter <= {1'b0, sequenceRAMgap} - 1;
                pendingEvent <= sequenceRAMevent;
                readCounter <= readCounter + 1;
            end
        end
        else begin
            evgSequenceEventTVALID <= 0;
            gapCounter <= gapCounter - 1;
        end
    end
    else begin
        gapCounter <= 0;
        readCounter <= 0;
        startDelay <= 3;
        if (evgSequenceStart) begin
            if (|sequenceEnabled) begin
                sequenceActive <= 1;
                sequenceBusy <= 1;
                seqSelect <= sequenceEnabled[1];
                sequenceEnabled[1] <= 0;
                startRequestsAccepted <= startRequestsAccepted + 1;
                evgStatusBuffWeToggle <= !evgStatusBuffWeToggle;
            end
        end
    end
end

wire [4:0] addressWidth = SEQUENCE_ADDRESS_WIDTH;
wire [31:0] evgStatus = { 3'b0, addressWidth,
                  startRequestsIgnored,
                  startRequestsAccepted,
                  3'b0, sequenceBusy,
                  sequenceActive, seqSelect, sequenceEnabled };

///////////////////////////////////////////////////////////////////////////////
// System clock domain

reg [DPRAM_ADDRESS_WIDTH-1:0] sysWriteAddress;
reg  [SEQUENCE_GAP_WIDTH-1:0] sysGapLatch;
reg                           sysSequenceReadbackSelect;
reg                           sysStatusBuffReadToggle = 0;
reg                           sysStatusBuffReadMatch = 0;

always @(posedge sysClk) begin
    sysSequenceRAMrbk <= sequenceRAM[sysWriteAddress];
    sysSequenceReadback <= sysSequenceReadbackSelect ?
              { {32-EVENTCODE_WIDTH{1'b0}},
                       sysSequenceRAMrbk[0+:EVENTCODE_WIDTH] }:
              { {32-SEQUENCE_GAP_WIDTH{1'b0}},
                       sysSequenceRAMrbk[EVENTCODE_WIDTH+:SEQUENCE_GAP_WIDTH] };
    if (sysCSRstrobe) begin
        case (csrCmdCode)
        SEQ_CSR_CMD_SET_ADDRESS: begin
            sysWriteAddress <= sysGPIO_OUT[DPRAM_ADDRESS_WIDTH-1:0];
            sysSequenceReadbackSelect <=sysGPIO_OUT[24];
            if (sysGPIO_OUT[25]) begin
                precompletionEvent <= sysGPIO_OUT[EVENTCODE_WIDTH-1:0];
            end
        end
        SEQ_CSR_CMD_LATCH_GAP: begin
            sysGapLatch <= sysGPIO_OUT[SEQUENCE_GAP_WIDTH-1:0];
        end
        SEQ_CSR_CMD_WRITE_ENTRY: begin
            sequenceRAM[sysWriteAddress] <=
                                {sysGapLatch, sysGPIO_OUT[EVENTCODE_WIDTH-1:0]};
            sysWriteAddress <= sysWriteAddress + 1;
        end
        default: begin
            if (sysGPIO_OUT[5]) begin
                sysStatusForceWEToggle <= !sysStatusForceWEToggle;
            end
            if (sysGPIO_OUT[4]) begin
                sysStatusBuffReadToggle <= !sysStatusBuffReadToggle;
            end
            if (sysGPIO_OUT[3]) begin
                sysSequenceDisableToggle[1] <= !sysSequenceDisableToggle[1];
            end
            if (sysGPIO_OUT[2]) begin
                sysSequenceDisableToggle[0] <= !sysSequenceDisableToggle[0];
            end
            if (sysGPIO_OUT[1]) begin
                sysSequenceEnableToggle[1] <= !sysSequenceEnableToggle[1];
            end
            if (sysGPIO_OUT[0]) begin
                sysSequenceEnableToggle[0] <= !sysSequenceEnableToggle[0];
            end
        end
        endcase
    end
end

// Status dual buffer
// We want every change in status to be timestamped
// with NTP clock so the processor knows exactly when that happens.
localparam STATUS_BUFF_DATA_WIDTH = 32 + 64;
reg [STATUS_BUFF_DATA_WIDTH-1:0] statusBuff = 0;
reg  sysStatusBuffWeMatch = 0;
wire sysStatusBuffWeToggle;

wire [31:0] sysStatus;
forwardData #(.DATA_WIDTH(32+1))
  forwardSysNTPToEVG(
      .inClk(evgTxClk),
      .inData({evgStatusBuffWeToggle, evgStatus}),
      .outClk(sysClk),
      .outData({sysStatusBuffWeToggle, sysStatus})
);

always @(posedge sysClk) begin
    if (sysStatusBuffWeToggle != sysStatusBuffWeMatch) begin
        statusBuff <= {sysStatus, sysNtpSeconds, sysNtpFraction};
        sysStatusBuffWeMatch <= sysStatusBuffWeToggle;
    end

    if (sysStatusBuffReadToggle != sysStatusBuffReadMatch) begin
        {status, statusNtpSeconds, statusNtpFraction} <= statusBuff;
        sysStatusBuffReadMatch <= sysStatusBuffReadToggle;
    end
end

endmodule
