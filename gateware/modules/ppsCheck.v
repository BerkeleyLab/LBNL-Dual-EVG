// Validate PPS signal
module ppsCheck #(
    parameter CLK_RATE = -1
    ) (
    input      clk,
    input      pps_a,
    output reg ppsValid = 0);

localparam WATCHDOG_LOW_RELOAD = ((CLK_RATE / 10) * 9) - 2;
localparam WATCHDOG_LOW_WIDTH = $clog2(WATCHDOG_LOW_RELOAD+1) + 1;
reg [WATCHDOG_LOW_WIDTH-1:0] watchdogLow = WATCHDOG_LOW_RELOAD;
wire watchdogLowDone = watchdogLow[WATCHDOG_LOW_WIDTH-1];

localparam WATCHDOG_HIGH_RELOAD = ((CLK_RATE / 10) * 11) - 2;
localparam WATCHDOG_HIGH_WIDTH = $clog2(WATCHDOG_HIGH_RELOAD+1) + 1;
reg [WATCHDOG_HIGH_WIDTH-1:0] watchdogHigh = WATCHDOG_HIGH_RELOAD;
wire watchdogHighDone = watchdogHigh[WATCHDOG_HIGH_WIDTH-1];

localparam DEBOUNCE_RELOAD = (CLK_RATE / 1000) - 2;
localparam DEBOUNCE_WIDTH = $clog2(DEBOUNCE_RELOAD+1) + 1;
reg [DEBOUNCE_WIDTH-1:0] debounce = DEBOUNCE_RELOAD;
wire debounceDone = debounce[DEBOUNCE_WIDTH-1];

(*ASYN_REG="true"*) reg pps_m = 0;
reg pps = 0, pps_strobe = 0;

always @(posedge clk) begin
    // Generate strobe at rising edge of debounced PPS signal
    pps_m <= pps_a;
    pps   <= pps_m;
    if (pps) begin
        if (debounceDone) begin
            pps_strobe <= 1;
        end
        else begin
            pps_strobe <= 0;
        end
        debounce <= DEBOUNCE_RELOAD;
    end
    else begin
        pps_strobe <= 0;
        if (!debounceDone) begin
            debounce  <= debounce - 1;
        end
    end

    // Check that pps_strobe is occurring at the right rate
    if (pps_strobe) begin
        if (watchdogLowDone && !watchdogHighDone) begin
            ppsValid <= 1;
        end
        watchdogLow <= WATCHDOG_LOW_RELOAD;
        watchdogHigh <= WATCHDOG_HIGH_RELOAD;
    end
    else begin
        if (!watchdogLowDone) begin
            watchdogLow <= watchdogLow - 1;
        end
        if (watchdogHighDone) begin
            ppsValid <= 0;
        end
        else begin
            watchdogHigh <= watchdogHigh - 1;
        end
    end
end

endmodule
