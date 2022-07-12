// Multiplexed-input frequency counter
// FPGA-friendly, since there's no actual multiplexed clock:
//  multiplexing happens in the data path, requiring a small amount
//  of logic in each "unknown" clock domain

// Intended to be useful as a monitor of all the various clocks supplied
// by the hardware in a medium to large FPGA project, without consuming
// more resources than necessary.

module freq_multi_count #(
	parameter NF=8,  // number of frequency counters in a block
	parameter NG=1,  // number of frequency counter blocks
	parameter gw=4,  // Gray counter width
	parameter cw=3,  // macro-cycle counter width
	parameter rw=24, // reference counter width
	parameter uw=28, // unknown counter width

// Sensible choices for gw are 3 (supports f_unk < 6.0 * f_ref) or
// 4 (supports f_unk < 14.0 * f_ref).
// The default combination rw=24, gw=4, and uw=28 make sense together.
// With a 50 MHz reference clock it can in theory measure up to 700 MHz,
// with a resolution of about 3 Hz.

// Designed for power-of-two channel counts, e.g., the 8 you'd get
// by taking default parameters for NF and NG.  Choosing values for NF or NG
// that are not powers of two is possible and will probably work OK.

// All NF*NG results are kept in dual-port RAM, which can be read out at
// any time.  The addr port is formed with $clog2(NG) msb and $clog2(NF) lsb.
// It's OK for NG to be 1, but NF must be at least 2.

// To get decent frequency resolution, the update rate is naturally pretty
// slow.  With default rw=24, NF=8, and a 50 MHz refclk, each channel
// updates every 2.7 seconds.  If the software wants to keep track of updates,
// it can peek at the source_state output port:  the low order $clog2(NF) bits
// represent the channel currently acquiring, which will be the stalest result
// in memory.  The upper cw (default 3) bits of source_state count macro-cycles
// of updating all NF frequency counters in each bank.

// NG > 1 will provide parallel banks of frequency counters, yielding faster
// updates but heavier resource usage compared to simply using a larger NF.
// These parallel banks do share their reference clock logic, so it's "cheaper"
// and/or easier for the synthesizer, compared with just instantiating this
// module multiple times.  If you want independent control over the
// configuration of multiple blocks, multiple instantiations will of course
// also work.

// refclk is used for both the frequency reference and the local-bus
// memory readout.  That output is one refclk cycle delayed from addr,
// in order to clock the internal dual-port RAM.

// If an external reference marker is used, rw must satisfy
//     ((2^rw-5)*refclk_rate) > marker_interval
// The update interval then becomes 2*NF reference marker intervals.

// It's recommended to not override parameters NA_ and NB_.  They are not
// set up as localparams because that triggers toolchain problems.
	parameter NA_=$clog2(NF),  // don't change this!
	parameter NB_=$clog2(NG)   // don't change this!
) (
	// Input clocks
	input [NF*NG-1:0] unk_clk,
	input refclk,
	// Optional reference marker (usually 1 Hz, possibly asynchronous to refclk)
	input refMarker,
	// Result channel selection in refclk domain
	input [NB_+NA_-1:0] addr,
	// Outputs in refclk domain
	output [NA_+cw-1:0] source_state,
	output [uw-1:0] frequency  // read out from dual-port RAM using addr
);

// Reference counter.  Without squelch, the frequency counter would
// accumulate some crap when changing clocks.
// Accounting for squelched cycles, accumulation time is 2^rw-5 refclk cycles.
reg refMarker_m, refMarker_d0, refMarker_d1;
reg [rw-1:0] refcnt=0;
reg ref_carry=0, squelch=0;
always @(posedge refclk) begin
	refMarker_m  <= refMarker;
	refMarker_d0 <= refMarker_m;
	refMarker_d1 <= refMarker_d0;
	if (refMarker_d0 && !refMarker_d1) begin
		squelch <= !squelch;
		if (squelch == 0) ref_carry <= 1;
		refcnt <= 5;
	end
	else begin
		{ref_carry, refcnt} <= refcnt + 1;
		if (ref_carry) squelch <= 1;
		if (refcnt == 4) squelch <= 0;
	end
end

// clksel is used to control clock multiplexing
reg [NA_+cw-1:0] source_count=0, source_count_d=0;
wire [NA_-1:0] clksel = source_count[NA_-1:0];
wire [NA_+cw-1:0] next_state;
generate if (NF == (1<<NA_))
	assign next_state = source_count + 1;  // as originally designed
else begin
	// Kind of a mess, sorry.  Still all combinational.
	wire clksel_carry = clksel == NF-1;
	wire [NA_-1:0] next_clksel = clksel_carry ? 0 : clksel+1;
	wire [cw-1:0] macro = source_count[NA_+cw-1:NA_];
	assign next_state = {macro+clksel_carry, next_clksel};
end
endgenerate

// Back to simple
always @(posedge refclk) begin
	if (ref_carry) source_count <= next_state;
	source_count_d <= source_count;
end
assign source_state = source_count_d;

// Encapsulate one block of counters
wire [uw-1:0] sub_results[0:NG-1];
genvar jx;
generate for (jx=0; jx<NG; jx=jx+1) begin: block
freq_multi_count_sub #(.NA(NA_), .NF(NF), .gw(gw), .uw(uw)) s(
	.unk_clk(unk_clk[jx*NF +: NF]),
	.refclk(refclk),
	.addr(addr[NA_-1:0]),
	.clksel(clksel),
	.ref_carry(ref_carry),
	.squelch(squelch),
	.frequency(sub_results[jx])
);
end
endgenerate

// Select from the array of block results
generate if (NB_ > 0) begin
	reg [NB_-1:0] h_addr;
	always @(posedge refclk) h_addr <= addr[NA_+NB_-1:NA_];
	assign frequency = sub_results[h_addr];  // combinatorial second-stage multiplexing
end else begin
	// Special case that avoids zero-length address
	assign frequency = sub_results[0];
end
endgenerate

endmodule
