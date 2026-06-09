// Generate event sequences
//
// Nets with names beginning with sys are in transmitter clock domain.
// All other nets are in transmitter clock domain.

module evgSequencer # (
    parameter SEQUENCE_RAM_CAPACITY  = 2048,
    parameter EVENTCODE_WIDTH        = 8,
    parameter EVENTCAT_WIDTH         = 8,
    parameter EVENTCAT_NUM           = 8,
    parameter DEBUG                  = "false",
    // Don't change these
    parameter SEQUENCE_GAP_CAT_WIDTH = 28
    ) (
    // Processor block connections
    // Some readback values are not in the system clock domain.  The
    // software is aware of this and reads until the values are stable.
    input              sysClk,
    input              sysCSRstrobe,
    input              sysCSRStatusFIFOstrobe,
    input       [31:0] sysGPIO_OUT,

    output      [31:0] status,
    output      [31:0] statusNtpSeconds,
    output      [31:0] statusNtpFraction,
    output      [EVENTCAT_NUM*32-1:0] statusCatDelay,

    output wire [31:0] statusFifo,
    output reg  [31:0] sysSequenceReadback,

    // Synchronization
    input      evgTxClk,
    input      evgSequenceStart,
    input      [EVENTCAT_NUM*SEQUENCE_GAP_CAT_WIDTH-1:0]
               evgCatDelay,

    // NTP timestamp synchronous to evgTxClk
    input       [31:0] evgNtpSeconds,
    input       [31:0] evgNtpFraction,

    // Event requests
    output reg [EVENTCODE_WIDTH-1:0] evgSequenceEventTDATA,
    output reg                       evgSequenceEventTVALID);

generate
if (2**EVENTCODE_WIDTH < EVENTCAT_NUM) begin
    pow2_EVENTCODE_WIDTH_smaller_than_EVENTCAT_NUM err();
end
endgenerate

generate
if (SEQUENCE_GAP_CAT_WIDTH != 28) begin
    SEQUENCE_GAP_CAT_WIDTH_diff_than_28 err2();
end
endgenerate

localparam END_OF_TABLE_EVENT_CODE  = 8'h7F;

localparam SEQ_CSR_CMD_SET_ADDRESS = 2'h1;
localparam SEQ_CSR_CMD_LATCH_GAP   = 2'h2;
localparam SEQ_CSR_CMD_WRITE_ENTRY = 2'h3;
wire [1:0] csrCmdCode = sysGPIO_OUT[31:30];

localparam SEQUENCE_ADDRESS_WIDTH = $clog2(SEQUENCE_RAM_CAPACITY);
localparam SEQUENCE_RAM_DATA_WIDTH = SEQUENCE_GAP_WIDTH + EVENTCODE_WIDTH + EVENTCAT_WIDTH;

localparam SEQUENCE_GAP_WIDTH = 28;
localparam GAP_COUNTER_WIDTH = SEQUENCE_GAP_WIDTH + 1;
reg [GAP_COUNTER_WIDTH-1:0] gapCounter;
wire gapCounterDone = gapCounter[GAP_COUNTER_WIDTH-1];

localparam GAP_CAT_COUNTER_WIDTH = SEQUENCE_GAP_CAT_WIDTH + 1;
reg [GAP_CAT_COUNTER_WIDTH-1:0] gapCatCounter;
wire gapCatCounterDone = gapCatCounter[GAP_CAT_COUNTER_WIDTH-1];

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

wire [EVENTCAT_WIDTH-1:0] sequenceRAMcat =
                              sequenceRAMQ[EVENTCODE_WIDTH+SEQUENCE_GAP_WIDTH+:EVENTCAT_WIDTH];
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
reg [EVENTCAT_WIDTH-1:0] pendingEventCat;
reg sequenceActive = 0, sequenceBusy = 0;
reg [1:0] startDelay = 0;

// Statistics
localparam START_REQUEST_COUNTER_WIDTH = 8;
reg [START_REQUEST_COUNTER_WIDTH-1:0] startRequestsIgnored = 0,
                                      startRequestsAccepted = 0;

// Unblundle category delays
wire [SEQUENCE_GAP_CAT_WIDTH-1:0] evgDelay [0:EVENTCAT_NUM-1];
reg  [SEQUENCE_GAP_CAT_WIDTH-1:0] evgDelayLatch [0:EVENTCAT_NUM-1];
// For readback
wire [32*EVENTCAT_NUM-1:0] evgDelayLatchFlatten;

// For simulation
integer idx;
initial begin
    for(idx = 0; idx < EVENTCAT_NUM; idx = idx+1) begin
        evgDelayLatch[idx] = 0;
    end
end

genvar i;
generate
for(i = 0; i < EVENTCAT_NUM; i = i+1) begin

assign evgDelayLatchFlatten[i*32+:32] = {{32-SEQUENCE_GAP_CAT_WIDTH{1'b0}}, evgDelayLatch[i]};
assign evgDelay[i] = evgCatDelay[i*SEQUENCE_GAP_CAT_WIDTH+:SEQUENCE_GAP_CAT_WIDTH];

end
endgenerate

localparam EVENTCAT_SEL_WIDTH = EVENTCAT_NUM <= 1 ? 1: $clog2(EVENTCAT_NUM);
wire [EVENTCAT_SEL_WIDTH-1:0] evgDelaySel = sequenceRAMcat[EVENTCAT_SEL_WIDTH-1:0];

// Status logic
reg [31:0] evgNtpSecondsLatch = 0;
reg [31:0] evgNtpFractionLatch = 0;

// Status FIFO registers
reg sysStatusFifoAcceptWr = 0;
reg statusFifoWrEvent = 0;
(*ASYNC_REG="true"*) reg statusFifoAcceptWr_m = 0;
reg statusFifoAcceptWr = 0;

always @(posedge evgTxClk) begin
    sequenceEnableToggle_m <= sysSequenceEnableToggle;
    sequenceEnableToggle   <= sequenceEnableToggle_m;
    sequenceDisableToggle_m <= sysSequenceDisableToggle;
    sequenceDisableToggle   <= sequenceDisableToggle_m;
    statusForceWEToggle_m <= sysStatusForceWEToggle;
    statusForceWEToggle   <= statusForceWEToggle_m;

    statusFifoAcceptWr_m <= sysStatusFifoAcceptWr;
    statusFifoAcceptWr   <= statusFifoAcceptWr_m;

    statusFifoWrEvent <= 0;

    // Force write enable to status register. Useful on startup
    // to have a valid initial value
    if (statusForceWEToggle != statusForceWEMatch) begin
        {evgNtpSecondsLatch, evgNtpFractionLatch} <= {evgNtpSeconds, evgNtpFraction};
        statusFifoWrEvent <= 1;
        statusForceWEMatch <= statusForceWEToggle;
    end

    if (sequenceActive || !evgSequenceStart) begin
        // Enable status can change at startup so
        // hold off requests when starting.
        if (sequenceDisableToggle[1] != sequenceDisableMatch[1]) begin
            sequenceEnabled[1] <= 0;
        end
        else if (sequenceEnableToggle[1] != sequenceEnableMatch[1]) begin
            sequenceEnabled[1] <= 1;
        end
        if (sequenceDisableToggle[0] != sequenceDisableMatch[0]) begin
            sequenceEnabled[0] <= 0;
        end
        else if (sequenceEnableToggle[0] != sequenceEnableMatch[0]) begin
            sequenceEnabled[0] <= 1;
        end
        sequenceEnableMatch <= sequenceEnableToggle;
        sequenceDisableMatch <= sequenceDisableToggle;
    end

    if (sequenceActive) begin
        if (evgSequenceStart) begin
            startRequestsIgnored <= startRequestsIgnored + 1;
        end

        if (startDelay[0]) begin
            // Sequence read address valid at this point
            startDelay[0] <= 0;
        end
        else if (startDelay[1]) begin
            // Sequence read data valid at this point
            startDelay[1] <= 0;
            gapCounter <= {1'b0, sequenceRAMgap + evgDelayLatch[evgDelaySel]};
            pendingEvent <= sequenceRAMevent;
            readCounter <= readCounter + 1;
        end
        else if (gapCounterDone) begin
            if ((pendingEvent == END_OF_TABLE_EVENT_CODE)
              || readCounterOverflow) begin
                evgSequenceEventTVALID <= 0;
                sequenceBusy <= 0;
                sequenceActive <= 0;
                {evgNtpSecondsLatch, evgNtpFractionLatch} <= {evgNtpSeconds, evgNtpFraction};
                statusFifoWrEvent <= 1;
            end
            else begin
                if (pendingEvent == precompletionEvent) begin
                    sequenceBusy <= 0;
                    {evgNtpSecondsLatch, evgNtpFractionLatch} <= {evgNtpSeconds, evgNtpFraction};
                    statusFifoWrEvent <= 1;
                end
                evgSequenceEventTVALID <= 1;
                evgSequenceEventTDATA <= pendingEvent;

                gapCounter <= {1'b0, sequenceRAMgap + evgDelayLatch[evgDelaySel]} - 1;
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
                for(idx = 0; idx < EVENTCAT_NUM; idx = idx+1) begin
                    evgDelayLatch[idx] <= evgDelay[idx];
                end

                sequenceActive <= 1;
                sequenceBusy <= 1;
                seqSelect <= sequenceEnabled[1];
                sequenceEnabled[1] <= 0;
                startRequestsAccepted <= startRequestsAccepted + 1;
                {evgNtpSecondsLatch, evgNtpFractionLatch} <= {evgNtpSeconds, evgNtpFraction};
                statusFifoWrEvent <= 1;
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
reg                           sysStatusBuffReadMatch = 0;

always @(posedge sysClk) begin
    sysSequenceRAMrbk <= sequenceRAM[sysWriteAddress];
    sysSequenceReadback <= sysSequenceReadbackSelect ?
              { {16-EVENTCAT_WIDTH{1'b0}},
                sysSequenceRAMrbk[EVENTCODE_WIDTH+SEQUENCE_GAP_WIDTH+:EVENTCAT_WIDTH],
                {16-EVENTCODE_WIDTH{1'b0}},
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
            sequenceRAM[sysWriteAddress] <= {
                sysGPIO_OUT[EVENTCAT_WIDTH+EVENTCODE_WIDTH-1:EVENTCODE_WIDTH],
                sysGapLatch,
                sysGPIO_OUT[EVENTCODE_WIDTH-1:0]};
            sysWriteAddress <= sysWriteAddress + 1;
        end
        default: begin
            if (sysGPIO_OUT[5]) begin
                sysStatusForceWEToggle <= !sysStatusForceWEToggle;
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

localparam STATUS_FIFO_AW = 5;
localparam STATUS_FIFO_USERW = 0;
localparam STATUS_FIFO_DATAW = 32 + 64 + EVENTCAT_NUM*32;
localparam STATUS_FIFO_DW = STATUS_FIFO_USERW + STATUS_FIFO_DATAW;
localparam STATUS_FIFO_MAX = 2**STATUS_FIFO_AW-1;

wire signed [STATUS_FIFO_AW:0] statusFifoWRCount;
wire signed [STATUS_FIFO_AW:0] sysStatusFifoRDCount;
reg sysStatusFifoREGPIO = 0;
wire statusFifoAlmostFull = (statusFifoWRCount >= STATUS_FIFO_MAX-2);
wire statusFifoWE = statusFifoAcceptWr && statusFifoWrEvent && !statusFifoAlmostFull;
wire sysStatusFifoRE;
wire sysStatusFifoEmpty;

genericFifo_2c #(
    .dw(STATUS_FIFO_DW),
    .aw(STATUS_FIFO_AW),
    .fwft(1))
  statusFifo_2c (
    .wr_clk(evgTxClk),
    .din({evgStatus, evgNtpSecondsLatch, evgNtpFractionLatch, evgDelayLatchFlatten}),
    .we(statusFifoWE),
    .full(),
    .wr_count(statusFifoWRCount),

    .rd_clk(sysClk),
    .dout({status, statusNtpSeconds, statusNtpFraction, statusCatDelay}),
    .re(sysStatusFifoRE),
    .empty(sysStatusFifoEmpty),
    .rd_count(sysStatusFifoRDCount)
);

wire sysStatusFifoValid = !sysStatusFifoEmpty;
wire sysStatusFifoAlmostFull = (sysStatusFifoRDCount >= STATUS_FIFO_MAX-2);
assign sysStatusFifoRE = sysStatusFifoValid && sysStatusFifoREGPIO;

//
// Status FIFO CSR
//
assign statusFifo = {{16-(STATUS_FIFO_AW+1){1'b0}}, sysStatusFifoRDCount,
    {16-1-1-1-1{1'b0}}, sysStatusFifoEmpty, sysStatusFifoAlmostFull, sysStatusFifoAcceptWr, sysStatusFifoValid};

always @(posedge sysClk) begin
    sysStatusFifoREGPIO <= 0;

    if (sysCSRStatusFIFOstrobe) begin
        if (sysGPIO_OUT[0]) begin
            sysStatusFifoREGPIO <= 1;
        end

        sysStatusFifoAcceptWr <= sysGPIO_OUT[1];
    end
end

generate
if (DEBUG != "TRUE" && DEBUG != "FALSE" && DEBUG != "true" && DEBUG != "false") begin
    DEBUG_only_TRUE_or_FALSE_SUPPORTED();
end
endgenerate

generate
if (DEBUG == "TRUE" || DEBUG == "true") begin

`ifndef SIMULATE
ila_td256_s4096_cap ila_td256_s4096_cap_inst (
    .clk(evgTxClk),
    .probe0({
        evgSequenceStart,
        sequenceEnabled[0],
        sequenceEnabled[1],
        statusFifoWrEvent,
        evgStatus,
        evgDelay[1],
        evgDelay[2],
        evgDelayLatch[1],
        evgDelayLatch[2],
        evgDelayLatchFlatten[1],
        evgDelayLatchFlatten[2]
    })
);
`endif

end // end if
endgenerate

endmodule
