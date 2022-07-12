// Does the "real work" for freq_multi_count above, not counting
// the shared logic for the reference counter etc.
module freq_multi_count_sub #(
	parameter NF=8,  // number of frequency counters
	parameter NA=3,  // number of address bits (ceil(log2(NF)))
	parameter gw=4,  // Gray counter width
	parameter cw=3,  // macro-cycle counter width
	parameter rw=24, // reference counter width
	parameter uw=28  // unknown counter width
) (
	// Input clocks
	input [NF-1:0] unk_clk,
	input refclk,
	// Various controls in refclk domain
	input [NA-1:0] addr,
	input [NA-1:0] clksel,
	input ref_carry,
	input squelch,
	// Output in refclk domain
	output [uw-1:0] frequency
);

// One Gray code counter for each input clock
wire [gw-1:0] gray1[0:NF-1];
genvar ix;
generate for (ix=0; ix<NF; ix=ix+1)
	simplest_gray gc(.clk(unk_clk[ix]), .gray(gray1[ix]));
endgenerate

// Transfer those Gray codes to the measurement clock domain
reg [gw-1:0] gray2[0:NF-1];
reg [gw-1:0] gray3=0, gray4=0;
integer jx;
initial for (jx=0; jx<NF; jx=jx+1) gray2[jx] = 0;
always @(posedge refclk) begin
	for (jx=0; jx<NF; jx=jx+1) gray2[jx] <= gray1[jx];  // cross domains here
	gray3 <= gray2[clksel];  // multiplexing step
	gray4 <= gray3;  // probably useless pipeline stage
end

// Figure out how many unk_clk edges happened in the last refclk period
wire [gw-1:0] bin4 = gray4 ^ {1'b0, bin4[gw-1:1]};  // convert Gray to binary
reg [gw-1:0] bin5=0, diff5=0;
reg [uw-1:0] accum=0;  // actual frequency counter
always @(posedge refclk) begin
	bin5 <= bin4;
	diff5 <= bin4 - bin5;
	accum <= squelch ? 0 : accum + diff5;
end

// Create a small RAM to hold the results
// Real-life FPGA RAM initializes to zero, but
// we're content with each entry starting at {uw{1'bx}} in simulation
reg [uw-1:0] freq_arr[0:NF-1], freq_r;
always @(posedge refclk) begin
	if (ref_carry) freq_arr[clksel] <= accum;
	freq_r <= freq_arr[addr];
end
assign frequency = freq_r;

endmodule
