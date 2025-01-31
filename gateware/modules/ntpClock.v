// Keep track of NTP time

module ntpClock #(
    parameter CLK_RATE = 125000000,
    parameter DEBUG    = "false"
    ) (
    input                                   sysClk,
    (*mark_debug=DEBUG*) input              writeStrobe,
    input                            [31:0] writeData,
    output wire                             sysPpsToggle,
    output wire                             sysPpsMarker,
    output wire                      [31:0] sysSeconds,
    output wire                      [31:0] sysFraction,
    output wire                      [31:0] sysPosixSeconds,
    output wire                      [31:0] sysPosixSecondsNext,
    output wire                      [31:0] sysStatus,

    input                                   clk,
    input                                   pps_a,
    output reg                              ppsToggle = 0,
    output wire                             ppsMarker,
    (*mark_debug=DEBUG*) output reg  [31:0] seconds,
    (*mark_debug=DEBUG*) output wire [31:0] fraction,
    (*mark_debug=DEBUG*) output reg  [31:0] posixSeconds,
    (*mark_debug=DEBUG*) output reg  [31:0] posixSecondsNext,
    output wire                      [31:0] status);

//
// CSR
//
reg sysCsrToggle = 0;
reg [31:0] sysCsrSeconds = 0;
always @(posedge sysClk) begin
    if (writeStrobe) begin
        sysCsrToggle <= ~sysCsrToggle;
        sysCsrSeconds <= writeData;
    end
end

//
// sysClk to clk
//
wire csrToggle;
wire [31:0] csrSeconds;
forwardData #(.DATA_WIDTH(1+32))
  forwardCSRtoClk (
    .inClk(sysClk),
    .inData({   sysCsrToggle,
                sysCsrSeconds
            }),
    .outClk(clk),
    .outData({  csrToggle,
                csrSeconds
             }));

localparam NTP_SECONDS_AT_POSIX_EPOCH = 32'd2208988800; // 1970 - 1900 seconds
localparam FRACTION_WIDEN = 12;
localparam FILTER_L2_ALPHA = 4;

// Debounce PPS marker
(*ASYNC_REG="true"*) reg pps_m = 0;
reg pps = 0;
reg [8:0] ppsDebounce = 0;
wire ppsDebounceDone = ppsDebounce[8];
(*mark_debug=DEBUG*) reg ppsStrobe;

// Provide a marker wide enough to be seen in other clock domains
reg [7:0] ppsMarkerCounter = 0;
assign ppsMarker = ppsMarkerCounter[7];

// Count clocks per second
localparam PPS_INITIAL_INTERVAL = (CLK_RATE / 100) * 99;
localparam PPS_WINDOW_INTERVAL = CLK_RATE / 50;
localparam CLOCK_COUNTER_WIDTH = $clog2(PPS_INITIAL_INTERVAL+PPS_WINDOW_INTERVAL+1);
reg [CLOCK_COUNTER_WIDTH-1:0] clockCounter = 0;

// Low pass filter clocks per second
localparam FILTER_ACCUMULATOR_WIDTH = CLOCK_COUNTER_WIDTH + FILTER_L2_ALPHA;
(*mark_debug=DEBUG*) reg [FILTER_ACCUMULATOR_WIDTH-1:0] filterAccumulator =
                                                    CLK_RATE << FILTER_L2_ALPHA;
wire [CLOCK_COUNTER_WIDTH-1:0] filteredClocksPerSecond =
             filterAccumulator[FILTER_ACCUMULATOR_WIDTH-1-:CLOCK_COUNTER_WIDTH];

// Validate PPS
reg dividerStart = 0;
localparam PPS_INITIAL_WIDTH = $clog2(PPS_INITIAL_INTERVAL+1)+1;
localparam PPS_WINDOW_WIDTH = $clog2(PPS_WINDOW_INTERVAL+1)+1;
reg [PPS_INITIAL_WIDTH-1:0] ppsInitial = 0;
wire ppsInitialDone = ppsInitial[PPS_INITIAL_WIDTH-1];
reg [PPS_WINDOW_WIDTH-1:0] ppsWindow = 0;
wire ppsWindowDone = ppsWindow[PPS_WINDOW_WIDTH-1];

// Accumlate fractional seconds
localparam FRACTION_ACCUMULATOR_WIDTH = 32 + FRACTION_WIDEN;
localparam FRACTION_INCREMENT_WIDTH = $clog2((1<<30)/(PPS_INITIAL_INTERVAL/4)) +
                                                                 FRACTION_WIDEN;

reg [FRACTION_ACCUMULATOR_WIDTH-1:0] fractionAccumulator;
(*mark_debug=DEBUG*) reg [FRACTION_INCREMENT_WIDTH-1:0] fractionIncrement =
                                   {1'b1, {32+FRACTION_WIDEN{1'b0}}} / CLK_RATE;
wire [FRACTION_ACCUMULATOR_WIDTH:0] nextFractionAccumulator =
                                        fractionAccumulator + fractionIncrement;
wire fractionOverflow = nextFractionAccumulator[FRACTION_ACCUMULATOR_WIDTH];
assign fraction = fractionAccumulator[FRACTION_ACCUMULATOR_WIDTH-1-:32];

// Compute fractional second increment from filtered clocks per second
localparam DIVIDER_BITCOUNT_WIDTH = $clog2(FRACTION_INCREMENT_WIDTH)+1;
reg [DIVIDER_BITCOUNT_WIDTH-1:0] dividerBitsLeft;
wire dividerDone = dividerBitsLeft[DIVIDER_BITCOUNT_WIDTH-1];
localparam DIVIDEND_WIDTH = CLOCK_COUNTER_WIDTH + FRACTION_INCREMENT_WIDTH;
reg [CLOCK_COUNTER_WIDTH:0] dividend;
reg [FRACTION_INCREMENT_WIDTH-1:0] quotient;

// Report validity
reg [2:0] ppsValidCounter = 0;
wire ppsValid = ppsValidCounter[2];
reg secondsValid = 0;
assign status = { ppsToggle, 29'b0, secondsValid, ppsValid };

reg csrStrobe = 0, csrToggle_d1 = 0;
always @(posedge clk) begin
    // sysClk to clk
    csrToggle_d1 <= csrToggle;
    csrStrobe <= csrToggle_d1 ^ csrToggle;

    // Watch for rising PPS edge
    pps_m <= pps_a;
    pps   <= pps_m;
    if (pps) begin
        ppsDebounce <= 0;
        if (ppsDebounceDone) begin
            ppsStrobe <= 1;
        end
        else begin
            ppsStrobe <= 0;
        end
    end
    else begin
        ppsStrobe <= 0;
        if (!ppsDebounceDone) begin
            ppsDebounce <= ppsDebounce + 1;
        end
    end
    if (ppsStrobe && ppsValid) begin
        ppsToggle <= !ppsToggle;
        ppsMarkerCounter <= ~0;
    end
    else if (ppsMarker) begin
        ppsMarkerCounter <= ppsMarkerCounter - 1;
    end

    // Update integer seconds
    if (csrStrobe) begin
        seconds <= csrSeconds;
        posixSeconds <= csrSeconds - NTP_SECONDS_AT_POSIX_EPOCH;
        posixSecondsNext <= csrSeconds - NTP_SECONDS_AT_POSIX_EPOCH + 1;
        if (ppsValid) secondsValid <= 1;
    end
    else begin
        if (!ppsValid) secondsValid <= 0;
        if (ppsStrobe) begin
            seconds <= seconds + 1;
            posixSeconds <= posixSeconds + 1;
            posixSecondsNext <= posixSecondsNext + 1;
        end
    end

    // Update fractional seconds
    if (ppsStrobe) begin
        fractionAccumulator <= 0;
    end
    else if (fractionOverflow) begin
        fractionAccumulator <= ~0;
    end
    else begin
        fractionAccumulator <= fractionAccumulator + fractionIncrement;
    end

    // Measure clock rate
    if (ppsStrobe) begin
        clockCounter <= 1;
        ppsInitial <= PPS_INITIAL_INTERVAL - 1;
        ppsWindow <= PPS_WINDOW_INTERVAL - 1;
        if (ppsInitialDone && !ppsWindowDone) begin
            if (!ppsValid) begin
                ppsValidCounter <= ppsValidCounter + 1;
            end
        end
        else begin
            ppsValidCounter <= 0;
        end
    end
    else begin
        clockCounter <= clockCounter + 1;
        if (ppsInitialDone) begin
            if (ppsWindowDone) begin
                ppsValidCounter <= 0;
            end
            else begin
                ppsWindow <= ppsWindow - 1;
            end
        end
        else begin
            ppsInitial <= ppsInitial - 1;
        end
    end

    // Compute fractional seconds increment

    // First low-pass filter the clock rate measurement
    if (ppsStrobe && ppsInitialDone && !ppsWindowDone) begin
        filterAccumulator <= filterAccumulator -
                             (filterAccumulator >> FILTER_L2_ALPHA) +
                             clockCounter;
        dividerStart <= 1;
    end
    else begin
        dividerStart <= 0;
    end

    // Then compute the fraction increment:
    // fractionIncrement = (2^(32 + FILTER_L2_ALPHA)) / filteredClocksPerSecond
    if (dividerStart) begin
        dividerBitsLeft <= FRACTION_INCREMENT_WIDTH;
        dividend <= {1'b1, {CLOCK_COUNTER_WIDTH-1{1'b0}}};
    end
    else if (!dividerDone) begin
        dividerBitsLeft <= dividerBitsLeft - 1;
        if (dividend >= filteredClocksPerSecond) begin
            dividend <= (dividend - filteredClocksPerSecond) << 1;
            quotient <= (quotient << 1) | 1;
        end
        else begin
            dividend <= dividend << 1;
            quotient <= (quotient << 1);
        end
    end
    else begin
        fractionIncrement <= quotient;
    end
end

//
// clk to sysClk
//
wire sysPpsValid, sysSecondsValid;
forwardData #(.DATA_WIDTH(1+1+1+1+32+32+32+32))
  forwardTimestampToSys (
    .inClk(clk),
    .inData({   ppsValid,
                secondsValid,
                ppsToggle,
                ppsMarker,
                seconds,
                fraction,
                posixSeconds,
                posixSecondsNext
            }),
    .outClk(sysClk),
    .outData({  sysPpsValid,
                sysSecondsValid,
                sysPpsToggle,
                sysPpsMarker,
                sysSeconds,
                sysFraction,
                sysPosixSeconds,
                sysPosixSecondsNext
             }));

assign sysStatus = { sysPpsToggle, 29'b0, sysSecondsValid, sysPpsValid };

endmodule
