// Create histograms of input states.
// Align transmitter heartbeat requests with other RF rising edge.
module coincidenceRecorder #(
    parameter CHANNEL_COUNT                = -1,
    parameter CYCLES_PER_ACQUISITION       = -1,
    parameter SAMPLE_CLKS_PER_COINCIDENCE  = -1,
    parameter INPUT_CYCLES_PER_COINCIDENCE = -1,
    parameter TX_CLK_PER_HEARTBEAT         = -1
    ) (
    input         sysClk,
    input         sysCsrStrobe,
    input  [31:0] sysGPIO_OUT,
    output [31:0] sysCsr,

    input                     samplingClk,
    input [CHANNEL_COUNT-1:0] value_a,

    input  txClk,
    output txHeartbeatStrobe);

/* Sanity check -- fake '$error()' */
if (((CYCLES_PER_ACQUISITION + 1) & CYCLES_PER_ACQUISITION) != 0) begin
    CYCLES_PER_ACQUISITION_is_not_one_less_than_a_power_of_2();
end

//////////////////////////////////////////////////////////////////////////////
// Sampling clock domain

/*
 * Count cycles in an acquisition run
 */
localparam CYCLE_COUNT_RELOAD = CYCLES_PER_ACQUISITION - 1;
localparam CYCLE_COUNT_WIDTH = $clog2(CYCLE_COUNT_RELOAD+1) + 1;
reg [CYCLE_COUNT_WIDTH-1:0] cycleCount = ~0;
wire cycleCountDone = cycleCount[CYCLE_COUNT_WIDTH-1];

/*
 * Count samples in a cycle
 * Free running so phase of input signal remains constant between acquisitions.
 */
localparam SAMPLE_COUNTER_WIDTH = $clog2(SAMPLE_CLKS_PER_COINCIDENCE);
reg [SAMPLE_COUNTER_WIDTH-1:0] sampleCounter = 0;
reg firstCycle = 0, firstCycle_d = 0;

/*
 * Sample input signal
 * If (SAMPLE_CLKS_PER_COINCIDENCE > INPUT_CYCLES_PER_COINCIDENCE) the input
 * is aliased to a negative frequency so the rising edge we're looking for
 * appears as a falling edge.  Account for this by inverting the input so
 * that the aliased falling edge appears to be rising.
 * If (SAMPLE_CLKS_PER_COINCIDENCE < INPUT_CYCLES_PER_COINCIDENCE) the input
 * is aliased to a positive frequency so unmodified input can be used.
 */
(*ASYNC_REG="true"*) reg [CHANNEL_COUNT-1:0] value_m = 0;
reg [CHANNEL_COUNT-1:0] value = 0;
always @(posedge samplingClk) begin
    if (SAMPLE_CLKS_PER_COINCIDENCE > INPUT_CYCLES_PER_COINCIDENCE) begin
        value_m <= ~value_a;
    end
    else begin
        value_m <= value_a;
    end
    value   <= value_m;
end

/*
 * Histogram dual-port RAM
 */
localparam SUM_WIDTH = $clog2(CYCLES_PER_ACQUISITION+1);
localparam DPRAM_WIDTH = CHANNEL_COUNT * SUM_WIDTH;
reg [SAMPLE_COUNTER_WIDTH-1:0] sysReadAddress, writeAddress;
wire [SAMPLE_COUNTER_WIDTH-1:0] readAddress = cycleCountDone ? sysReadAddress
                                                             : sampleCounter;
reg writeEnable = 0;
reg [DPRAM_WIDTH-1:0] dpram [0:(1<<SAMPLE_COUNTER_WIDTH)-1], dpramQ;
wire [DPRAM_WIDTH-1:0] writeData;
always @(posedge samplingClk) begin
    dpramQ <= dpram[readAddress];
    if (writeEnable) begin
        dpram[writeAddress] <= writeData;
    end
end
genvar i;
generate
for (i = 0 ; i < CHANNEL_COUNT ; i = i + 1) begin
    assign writeData[i*SUM_WIDTH+:SUM_WIDTH] = {{SUM_WIDTH-1{1'b0}}, value[i]} +
                                  (firstCycle_d ? {SUM_WIDTH{1'b0}}
                                              : dpramQ[i*SUM_WIDTH+:SUM_WIDTH]);
end
endgenerate

/*
 * Acquisition trigger
 */
reg sysStartToggle = 0;
(*ASYNC_REG="true"*) reg startToggle_m = 0;
reg startToggle = 0, startMatch = 0, busy = 0;

/*
 * Coincidence detection
 */
reg sysSampleCountCoincidenceToggle = 0;
(*ASYNC_REG="true"*) reg sampleCountCoincidenceToggle_m = 0;
reg sampleCountCoincidenceToggle = 0, sampleCountCoincidenceToggle_d = 0;
reg [SAMPLE_COUNTER_WIDTH-1:0] sysSampleCountCoincidence = ~0;
reg [SAMPLE_COUNTER_WIDTH-1:0] sampleCountCoincidence = ~0;
reg [3:0] coincidenceStretchCounter = 0;
wire coincidenceStretchActive = coincidenceStretchCounter[3];

/*
 * Acquisition
 */
always @(posedge samplingClk) begin
    /*
     * Account for DPRAM read latency
     */
    writeAddress <= readAddress;
    writeEnable <= !cycleCountDone;
    firstCycle_d <= firstCycle;

    /*
     * Busy/Idle status
     */
    startToggle_m <= sysStartToggle;
    startToggle   <= startToggle_m;
    if (startToggle != startMatch) begin
        busy <= 1;
    end
    else if (cycleCountDone && writeEnable) begin
        busy <= 0;
    end

    /*
     * Acquisition counters
     */
    if (sampleCounter == (SAMPLE_CLKS_PER_COINCIDENCE-1)) begin
        sampleCounter <= 0;
        if (cycleCountDone) begin
            firstCycle <= 1;
            if (startToggle != startMatch) begin
                startMatch <= startToggle;
                cycleCount <= CYCLE_COUNT_RELOAD;
            end
        end
        else begin
            firstCycle <= 0;
            cycleCount <= cycleCount - 1;
        end
    end
    else begin
        sampleCounter <= sampleCounter + 1;
    end

    /*
     * Detect and report coincidence
     */
    sampleCountCoincidenceToggle_m <= sysSampleCountCoincidenceToggle;
    sampleCountCoincidenceToggle   <= sampleCountCoincidenceToggle_m;
    sampleCountCoincidenceToggle_d <= sampleCountCoincidenceToggle;
    if (sampleCountCoincidenceToggle != sampleCountCoincidenceToggle_d) begin
        sampleCountCoincidence <= sysSampleCountCoincidence;
    end
    if (sampleCounter == sampleCountCoincidence) begin
        coincidenceStretchCounter <= ~0;
    end
    else if (coincidenceStretchActive) begin
        coincidenceStretchCounter <= coincidenceStretchCounter - 1;
    end
end

//////////////////////////////////////////////////////////////////////////////
// System clock domain
localparam MUXSEL_WIDTH = $clog2(CHANNEL_COUNT);
reg [MUXSEL_WIDTH-1:0] sysMuxSel = 0;
reg [SUM_WIDTH-1:0] sysReadMux;
reg sysRealignToggle = 0;
always @(posedge sysClk) begin
    if (sysCsrStrobe) begin
        if (sysGPIO_OUT[31]) begin
            sysStartToggle <= !sysStartToggle;
        end
        else if (sysGPIO_OUT[30]) begin
            sysSampleCountCoincidence <= sysGPIO_OUT[0+:SAMPLE_COUNTER_WIDTH];
            sysSampleCountCoincidenceToggle <= !sysSampleCountCoincidenceToggle;
        end
        else if (sysGPIO_OUT[29]) begin
            sysRealignToggle <= !sysRealignToggle;
        end
        else begin
            sysReadAddress <= sysGPIO_OUT[0+:SAMPLE_COUNTER_WIDTH];
            sysMuxSel <= sysGPIO_OUT[24+:MUXSEL_WIDTH];
        end
    end
    sysReadMux <= dpramQ[sysMuxSel*SUM_WIDTH+:SUM_WIDTH];
end
assign sysCsr = { busy, {8-1-MUXSEL_WIDTH{1'b0}}, sysMuxSel,
                  {24-SUM_WIDTH{1'b0}}, sysReadMux };

//////////////////////////////////////////////////////////////////////////////
// Transmiter (EVG) clock domain
// Generate coincidence and heartbeat strobes

localparam TX_HB_COUNTER_RELOAD = TX_CLK_PER_HEARTBEAT - 2;
localparam TX_HB_COUNTER_WIDTH = $clog2(TX_HB_COUNTER_RELOAD+1) + 1;
reg [TX_HB_COUNTER_WIDTH-1:0] txHeartbeatCounter = 0;
assign txHeartbeatStrobe = txHeartbeatCounter[TX_HB_COUNTER_WIDTH-1];

/*
 * Resync when alignment point changes
 */
(*ASYNC_REG="true"*) reg txRealignToggle_m = 0;
reg txRealignToggle = 0, txRealignMatch = 0;

/*
 * Coincidence marker from acquisition domain
 */
(*ASYNC_REG="true"*) reg txCoincidenceMarker_m = 0;
reg txCoincidenceMarker = 0, txCoincidenceMarker_d = 0;

always @(posedge txClk) begin
     txCoincidenceMarker_m <= coincidenceStretchActive;
     txCoincidenceMarker   <= txCoincidenceMarker_m;
     txCoincidenceMarker_d <= txCoincidenceMarker;

    txRealignToggle_m <= sysRealignToggle;
    txRealignToggle   <= txRealignToggle_m;

    if (txRealignToggle != txRealignMatch) begin
        txHeartbeatCounter <= TX_HB_COUNTER_RELOAD;
        if (txCoincidenceMarker != txCoincidenceMarker_d) begin
            txRealignMatch <= !txRealignMatch;
        end
    end
    else begin
        if (txHeartbeatStrobe) begin
            txHeartbeatCounter <= TX_HB_COUNTER_RELOAD;
        end
        else begin
            txHeartbeatCounter <= txHeartbeatCounter - 1;
        end
    end
end

endmodule
