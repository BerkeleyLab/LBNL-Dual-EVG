module mod125_reduction #(
    parameter WIDTH = 32
)(
    input  logic                clk,
    input  logic    [WIDTH-1:0] data_in,
    input  logic                valid_in,
    output logic          [6:0] data_out = '0,
    output logic                valid_out = 0
);

// Using a Pseudo-Mersenne reduction:
//
// A = q*128 + r
// A = q*(125 + 3) + r
// A = q*125 + q*3 + r, r <= 7 bits
//
// A (mod 125) = q*3 + r (mod 125)
//
// max(q*3 + r) = max(3*(2^(WIDTH-7)-1) + 2^7-1)
//
// new_width = old_width - 5
//
// Each stage reduces the width by 5 bits. Do it until input
// is 8 bits, which we just do a simple subtraction as the last
// stage

function automatic integer num_stages(input integer width);
    integer new_width = width;
    integer stages = 0;
begin
    while (new_width > 8) begin
        new_width = new_width - 5;
        stages = stages + 1;
    end

    return stages;
end
endfunction

localparam NUM_STAGES = num_stages(WIDTH);

// Rely on the optimizer to trim the unused MSB bits
logic valid_r[0:NUM_STAGES];
logic [WIDTH-1:0] data_r[0:NUM_STAGES];

// For simulation
integer idx;
initial begin
    for (idx = 0; idx <= NUM_STAGES; idx++) begin
        valid_r[idx] = 1'b0;
        data_r[idx] = '0;
    end
end

always_ff @(posedge clk) begin
    valid_r[0] <= valid_in;
    data_r[0] <= data_in;
end

// Actual calculation
genvar i;
generate
for (i = 0; i < NUM_STAGES; i = i + 1) begin

logic [WIDTH-1:0] q;
logic [6:0] r;

assign q = data_r[i] >> 7;   // Upper bits
assign r = data_r[i][6:0];   // Lower 7 bits

// Actual calculation A (mod 125) = q*3 + r
always_ff @(posedge clk) begin
    valid_r[i+1] <= valid_r[i];
    data_r[i+1]  <= (q << 1) + q + r;
end

end
endgenerate

// Final stage is guaranteed to be 8 bits -> [0, 255]
//
// Now, the modulo 125 is easy because we can do subtract 0, 125 or 250
logic [7:0] data_last;
// Systemverilog considers initialization as static initialization
// and not a continous assignment.
assign data_last = data_r[NUM_STAGES][7:0];

always_ff @(posedge clk) begin
    valid_out <= valid_r[NUM_STAGES];

    if (data_last >= 8'd250) begin
        data_out <= data_last - 8'd250;
    end
    else if (data_last >= 8'd125) begin
        data_out <= data_last - 8'd125;
    end
    else begin
        data_out <= data_last[6:0];
    end
end

endmodule
