module mod125_reduction_tb #(
    parameter NUM_TESTS = 10000
);

logic clk;

///////////////////////////////////////////////////////////////////////////////
// Clock generation

initial begin
    clk = 0;
    forever #5 clk = ~clk; // 100 MHz clock
end

///////////////////////////////////////////////////////////////////////////////
// DUTs

logic done_14;
mod125_reduction_test_wrapper #(
    .WIDTH(14),
    .NUM_TESTS(NUM_TESTS)
) mod125_reduction_w14 (
    .clk(clk),
    .done(done_14)
);

logic done_16;
mod125_reduction_test_wrapper #(
    .WIDTH(16),
    .NUM_TESTS(NUM_TESTS)
) mod125_reduction_w16 (
    .clk(clk),
    .done(done_16)
);

logic done_32;
mod125_reduction_test_wrapper #(
    .NUM_TESTS(NUM_TESTS),
    .WIDTH(32)
) mod125_reduction_w32 (
    .clk(clk),
    .done(done_32)
);

///////////////////////////////////////////////////////////////////////////////
// Stimulus

initial begin
    if ($test$plusargs("vcd")) begin
        $dumpfile("mod125_reduction.vcd");
        $dumpvars(5, mod125_reduction_tb);
    end

    for (int i = 0; i < mod125_reduction_w14.dut.NUM_STAGES+1; i++) begin
        $dumpvars(0, mod125_reduction_w14.dut.data_r[i]);
        $dumpvars(0, mod125_reduction_w14.dut.valid_r[i]);
    end

    for (int i = 0; i < mod125_reduction_w16.dut.NUM_STAGES+1; i++) begin
        $dumpvars(0, mod125_reduction_w16.dut.data_r[i]);
        $dumpvars(0, mod125_reduction_w16.dut.valid_r[i]);
    end

    for (int i = 0; i < mod125_reduction_w32.dut.NUM_STAGES+1; i++) begin
        $dumpvars(0, mod125_reduction_w32.dut.data_r[i]);
        $dumpvars(0, mod125_reduction_w32.dut.valid_r[i]);
    end

	wait(done_14 && done_16 && done_32);
end

endmodule
