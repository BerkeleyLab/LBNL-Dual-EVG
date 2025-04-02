// Create histograms of input states.
// Align transmitter heartbeat requests with other RF rising edge.
module coincidenceRecorder #(
    parameter CHANNEL_COUNT                = -1,
    parameter CYCLES_PER_ACQUISITION       = -1,
    parameter SAMPLE_CLKS_PER_COINCIDENCE  = -1,
    parameter INPUT_CYCLES_PER_COINCIDENCE = -1,
    parameter TX_CLK_PER_HEARTBEAT         = -1,
    parameter SAMPLE_COUNTER_WIDTH         = $clog2(SAMPLE_CLKS_PER_COINCIDENCE)
    ) (
    input         sysClk,
    input         sysCsrStrobe,
    input  [31:0] sysGPIO_OUT,
    output [31:0] sysCsr,
    output reg    sysRealignToggle = 0,
    input         sysRealignToggleIn,

    input                             samplingClk,
    input [CHANNEL_COUNT-1:0]         refClk,
    output                            coincidenceMarker,
    output [SAMPLE_COUNTER_WIDTH-1:0] sampleCounterDbg,

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
reg [SAMPLE_COUNTER_WIDTH-1:0] sampleCounter = 0;
reg firstCycle = 0, firstCycle_d = 0;

assign sampleCounterDbg = sampleCounter;

/*
 * Sample input signal
 * Because using clock as data is not well defined with the 2 clocks
 * having an "uncontrolable" routing delay, generate a /2 signal using
 * the measured clock
 */
wire [CHANNEL_COUNT-1:0] value;

genvar i;
generate
for(i = 0; i < CHANNEL_COUNT; i = i + 1) begin

reg value_a = 0;
(*ASYNC_REG="true"*) reg value_m = 0, value_d0 = 0;
(*KEEP="true"*) reg value_d1 = 0, value_d2 = 0, value_d3 = 0;

always @(posedge refClk[i]) begin
    value_a <= !value_a;
end

always @(posedge samplingClk) begin
    value_m   <= value_a;
    value_d0  <= value_m;
    value_d1  <= value_d0;
    value_d2  <= value_d1;
    value_d3  <= value_d2;
end

assign value[i] = !(value_d3^value_d2);

end
endgenerate

/*
 * Histogram dual-port RAM
 */
localparam MUXSEL_WIDTH = $clog2(CHANNEL_COUNT);
localparam SUM_WIDTH = $clog2(CYCLES_PER_ACQUISITION+1);
localparam DPRAM_WIDTH = CHANNEL_COUNT * SUM_WIDTH;
wire [SAMPLE_COUNTER_WIDTH-1:0] sampReadAddress;
reg [SAMPLE_COUNTER_WIDTH-1:0] dpramRBAddress = 0;
reg [SAMPLE_COUNTER_WIDTH-1:0] sysReadAddress = 0, writeAddress = 0;
wire [SAMPLE_COUNTER_WIDTH-1:0] readAddress = cycleCountDone ? sampReadAddress
                                                             : sampleCounter;
reg writeEnable = 0;
wire [MUXSEL_WIDTH-1:0] sampMuxSel;
reg [MUXSEL_WIDTH-1:0] muxSel = 0;
reg [DPRAM_WIDTH-1:0] dpram [0:(1<<SAMPLE_COUNTER_WIDTH)-1], dpramQ;
wire [DPRAM_WIDTH-1:0] writeData;
always @(posedge samplingClk) begin
    muxSel <= sampMuxSel;
    dpramRBAddress <= readAddress;
    dpramQ <= dpram[readAddress];
    if (writeEnable) begin
        dpram[writeAddress] <= writeData;
    end
end

generate
for (i = 0 ; i < CHANNEL_COUNT ; i = i + 1) begin
    assign writeData[i*SUM_WIDTH+:SUM_WIDTH] = {{SUM_WIDTH-1{1'b0}}, value[i]} +
                                  (firstCycle_d ? {SUM_WIDTH{1'b0}}
                                              : dpramQ[i*SUM_WIDTH+:SUM_WIDTH]);
end
endgenerate

// Data read by sysClk
reg [MUXSEL_WIDTH-1:0] sampRBMuxSel = 0;
reg [SUM_WIDTH-1:0] sampReadMux = 0;
reg [SAMPLE_COUNTER_WIDTH-1:0] sampRBAddress = 0;
always @(posedge samplingClk) begin
    sampRBMuxSel <= muxSel;
    sampRBAddress <= dpramRBAddress;
    sampReadMux <= dpramQ[muxSel*SUM_WIDTH+:SUM_WIDTH];
end

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

assign coincidenceMarker = coincidenceStretchActive;

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
reg [MUXSEL_WIDTH-1:0] sysMuxSel = 0;
wire [MUXSEL_WIDTH-1:0] sysRBMuxSel;
wire [SUM_WIDTH-1:0] sysReadMux;
wire [SAMPLE_COUNTER_WIDTH-1:0] sysRBAddress;
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
end

// This is slow (1 inClk clocks + 3 outClk clocks)
forwardData #(
    .DATA_WIDTH(MUXSEL_WIDTH+SAMPLE_COUNTER_WIDTH))
  forwardDataToSamp (
    .inClk(sysClk),
    .inData({sysMuxSel, sysReadAddress}),
    .outClk(samplingClk),
    .outData({sampMuxSel, sampReadAddress}));

// This is slow (1 inClk clocks + 3 outClk clocks)
forwardData #(
    .DATA_WIDTH(MUXSEL_WIDTH+SUM_WIDTH+SAMPLE_COUNTER_WIDTH))
  forwardDataToSys (
    .inClk(samplingClk),
    .inData({sampRBMuxSel, sampReadMux, sampRBAddress}),
    .outClk(sysClk),
    .outData({sysRBMuxSel, sysReadMux, sysRBAddress}));

assign sysCsr = { busy, {8-1-MUXSEL_WIDTH{1'b0}}, sysRBMuxSel,
                  {24-SAMPLE_COUNTER_WIDTH-SUM_WIDTH{1'b0}}, sysRBAddress, sysReadMux };

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

    txRealignToggle_m <= sysRealignToggleIn;
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
