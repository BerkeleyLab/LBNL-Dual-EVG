module pulseSync
(
    input wire s_clk,
    input wire s_pulse,

    input wire d_clk,
    output wire d_pulse
);

// save pulse information on a toggle
reg s_toggle = 0;
always @(posedge s_clk) begin
    if (s_pulse)
        s_toggle <= ~s_toggle;
end

// pass toggle to other domain
(*ASYNC_REG="true"*) reg d_toggle_m, d_toggle;
reg d_toggle_d = 0;
always @(posedge d_clk) begin
    d_toggle_m <= s_toggle;
    d_toggle <= d_toggle_m;
    d_toggle_d <= d_toggle;
end

assign d_pulse = d_toggle_d ^ d_toggle;

endmodule
