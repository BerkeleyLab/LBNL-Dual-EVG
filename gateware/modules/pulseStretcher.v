// Stretch pulse
module pulseStretcher #(
    // clock frequency in Hz
    parameter CLK_FREQUENCY = 100000000,
    // stretched pulse duration in ms
    parameter STRETCH_MS    = 100,
    // "true" = pulse can be asserted before the end of
    // the stretch
    // "false" = pulses are ignored until the end of the
    // stretch
    parameter RETRIGGERABLE = "true"
    ) (
    input      clk,
    input      rst_a,
    input      pulse_a,
    output reg pulseStretch = 0);

localparam COUNTER_RELOAD = $rtoi(((CLK_FREQUENCY/1.0e3)*STRETCH_MS)) - 2;
localparam COUNTER_WIDTH = $clog2(COUNTER_RELOAD+1);

reg [COUNTER_WIDTH:0] stretchCounter = ~0;
wire stretchDone = stretchCounter[COUNTER_WIDTH];

(*ASYNC_REG="true"*) reg pulse_m, pulse;
reg pulse_d, pulsePosedge;

always @(posedge clk or posedge rst_a) begin
    if (rst_a) begin
        pulse_m <= 0;
        pulse <= 0;

        pulse_d <= 0;
        pulsePosedge <= 0;
        stretchCounter <= ~0;
        pulseStretch <= 0;
    end
    else begin
        pulse_m <= pulse_a;
        pulse <= pulse_m;

        pulse_d <= pulse;
        pulsePosedge <= pulse && !pulse_d;

        if (pulsePosedge && ((RETRIGGERABLE == "true") ||
            (RETRIGGERABLE == "false" && !pulseStretch))) begin
            stretchCounter <= COUNTER_RELOAD;
            pulseStretch <= 1;
        end
        else if (!stretchDone) begin
            stretchCounter <= stretchCounter -1;
        end
        else begin
            pulseStretch <= 0;
        end
    end
end

endmodule
