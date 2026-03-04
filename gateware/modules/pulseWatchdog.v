module pulseWatchdog #(
    // In Hz
    parameter SYSCLK_FREQUENCY = 100000000,
    // In Hz
    parameter PULSE_FREQUENCY  = 1,
    parameter DEBUG            = "false"
    ) (
    input      clk,
    input      pulseIn,

    input      sysClk,
    output reg isValid = 0);

localparam SYSCLK_COUNT = (SYSCLK_FREQUENCY + PULSE_FREQUENCY - 1)/ PULSE_FREQUENCY;
localparam UPPER_LIMIT = ((SYSCLK_COUNT * 11) / 10);
localparam LOWER_LIMIT = ((SYSCLK_COUNT *  9) / 10);
(*mark_debug=DEBUG*) reg [$clog2(UPPER_LIMIT+1)-1:0] watchdog;

wire pulse;

pulseSync
  pulseSync (
    .s_clk(clk),
    .s_pulse(pulseIn),
    .d_clk(sysClk),
    .d_pulse(pulse));

always @(posedge sysClk) begin
    if (pulse) begin
        watchdog <= 0;
        if ((watchdog > LOWER_LIMIT)
         && (watchdog < UPPER_LIMIT)) begin
            isValid <= 1;
        end
        else begin
            isValid <= 0;
        end
    end
    else if (watchdog < UPPER_LIMIT) begin
        watchdog <= watchdog + 1;
    end
    else begin
        isValid <= 0;
    end
end
endmodule
