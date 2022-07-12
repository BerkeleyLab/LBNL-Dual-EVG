// Provde trigger synchronized to power line input
// Generate fake trigger if no power line signal present
module powerlineTrigger #(
    parameter CLK_RATE             = 100000000,
    parameter NOMINAL_TRIGGER_RATE = 60,
    parameter DEBOUNCE_USEC        = 100
    ) (
    input       clk,
    input       powerline_a,
    output wire trigger,
    output wire powerlineTimeout);

localparam DEBOUNCE_COUNTER_LOAD = $rtoi(((CLK_RATE/1.0e6)*DEBOUNCE_USEC)) - 2;
localparam DEBOUNCE_COUNTER_WIDTH = $clog2(DEBOUNCE_COUNTER_LOAD+1)+1;

localparam WATCHDOG_LIMIT = $rtoi((CLK_RATE*1.02)/NOMINAL_TRIGGER_RATE);
localparam WATCHDOG_RELOAD = WATCHDOG_LIMIT - 2;
localparam WATCHDOG_WIDTH = $clog2(WATCHDOG_RELOAD+1) + 1;
reg [WATCHDOG_WIDTH-1:0] watchdog = WATCHDOG_RELOAD;
assign powerlineTimeout = watchdog[WATCHDOG_WIDTH-1];

localparam FAKE60_LIMIT = $rtoi(CLK_RATE/NOMINAL_TRIGGER_RATE);
localparam FAKE60_RELOAD = FAKE60_LIMIT - 2;
localparam FAKE60_WIDTH = $clog2(FAKE60_RELOAD+1) + 1;
reg [FAKE60_WIDTH-1:0] fake60counter = FAKE60_RELOAD;
assign fake60trigger = fake60counter[FAKE60_WIDTH-1];

(*ASYNC_REG="true"*) reg powerline_m;
reg powerline = 0, powerline_d = 0;

reg [DEBOUNCE_COUNTER_WIDTH-1:0] debounceCounter = 0;
wire debounceCounterDone = debounceCounter[DEBOUNCE_COUNTER_WIDTH-1];
reg debounceCounterDone_d = 0, zeroCrossing = 0;

reg [4:0] triggerStretch = 0;
assign trigger = triggerStretch[4];

always @(posedge clk) begin
    powerline_m <= powerline_a;
    powerline   <= powerline_m;

    if (powerline) begin
        if (!debounceCounterDone) begin
            debounceCounter <= debounceCounter - 1;
        end
    end
    else begin
        debounceCounter <= DEBOUNCE_COUNTER_LOAD;
    end
    debounceCounterDone_d <= debounceCounterDone;
    zeroCrossing <= debounceCounterDone && !debounceCounterDone_d;

    if (zeroCrossing) begin
        triggerStretch <= ~0;
        watchdog <= WATCHDOG_RELOAD;
    end
    else begin
        if (powerlineTimeout) begin
            if (fake60trigger) begin
                fake60counter <= FAKE60_RELOAD;
                triggerStretch <= ~0;
            end
            else begin
                if (trigger) begin
                    triggerStretch <= triggerStretch - 1;
                end
                fake60counter <= fake60counter - 1;
            end
        end
        else begin
            if (trigger) begin
                triggerStretch <= triggerStretch - 1;
            end
            watchdog <= watchdog - 1;
            fake60counter <= FAKE60_RELOAD;
        end
    end
end
endmodule
