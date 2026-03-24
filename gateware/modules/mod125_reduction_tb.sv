module mod125_reduction_tb;

parameter WIDTH = 32;
// Number of randomized transactions
parameter NUM_TESTS = 10000;

logic             clk;
logic [WIDTH-1:0] data_in, next_data_in;
logic             valid_in, next_valid_in;
logic [6:0]       data_out;
logic             valid_out;

///////////////////////////////////////////////////////////////////////////////
// Clock generation

initial begin
    clk = 0;
    forever #5 clk = ~clk; // 100 MHz clock
end

///////////////////////////////////////////////////////////////////////////////
// DUT

mod125_reduction #(
    .WIDTH(WIDTH)
) dut (
    .clk(clk),
    .data_in(data_in),
    .valid_in(valid_in),
    .data_out(data_out),
    .valid_out(valid_out)
);

///////////////////////////////////////////////////////////////////////////////
// Stimulus

// Scoreboard. Hold expected results
logic [6:0] expected_queue[$];
logic [6:0] exp_val, actual_val;
logic [6:0] mod_val;
integer errors = 0;

initial begin
    if ($test$plusargs("vcd")) begin
        $dumpfile("mod125_reduction.vcd");
        $dumpvars(5, mod125_reduction_tb);

        for (int i = 0; i < dut.NUM_STAGES+1; i++) begin
            $dumpvars(0, dut.data_r[i]);
            $dumpvars(0, dut.valid_r[i]);
        end
    end

    next_data_in = '0;
    next_valid_in = 0;

    data_in <= '0;
    valid_in <= 0;
    @(posedge clk);

    $display("Starting %0d tests for WIDTH = %0d...", NUM_TESTS, WIDTH);

    for (int i = 0; i < NUM_TESTS; i++) begin
        next_valid_in = $urandom_range(0, 1);
        valid_in <= next_valid_in;

        if (next_valid_in) begin
            next_data_in = $urandom & ((1 << WIDTH) - 1);

            data_in <= next_data_in;
            mod_val = next_data_in % 125;
            expected_queue.push_back(mod_val);
        end

        @(posedge clk);
    end

    valid_in <= 0;
    data_in <= 'x;
    @(posedge clk);

    // Wait for pipeline to drain
    repeat(20) begin
        @(posedge clk);
    end

    if (errors == 0 && expected_queue.size() == 0) begin
        $display("# PASS");
        $finish(0);
    end else begin
        $display("# FAIL, %0d errors", errors);
        $stop(0);
    end
end

// Checker process
always_ff @(posedge clk) begin
    if (valid_out) begin
        if (expected_queue.size() == 0) begin
            $display("Time %0t: Output is valid, but expected queue is empty!", $time);
            errors++;
        end
        else begin
            exp_val <= expected_queue.pop_front();
            actual_val <= data_out;

            if (actual_val !== exp_val) begin
                $display("Time %0t: MISMATCH! Expected: %0d, Got: %0d", $time, exp_val, actual_val);
                errors++;
            end
        end
    end
end

endmodule
