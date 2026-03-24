module mod125_reduction_test_wrapper #(
    parameter WIDTH = 14,
    parameter NUM_TESTS = 10000
) (
    input  logic clk,
    output logic done
);

logic [WIDTH-1:0] data_in, next_data_in;
logic             valid_in, next_valid_in;
logic [6:0]       data_out;
logic             valid_out;

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
logic [WIDTH+6:0] expected_queue[$];
logic [WIDTH+6:0] queue_in, queue_out;
logic [6:0] mod_val;
integer errors = 0;

initial begin
    next_data_in = '0;
    next_valid_in = 0;
    done = 0;

    data_in <= '0;
    valid_in <= 0;
    @(posedge clk);

    $display("%m: starting %0d tests for WIDTH = %0d...", NUM_TESTS, WIDTH);

    for (int i = 0; i < NUM_TESTS; i++) begin
        next_valid_in = $urandom_range(0, 1);
        valid_in <= next_valid_in;

        if (next_valid_in) begin
            next_data_in = $urandom & ((1 << WIDTH) - 1);

            data_in <= next_data_in;
            mod_val = next_data_in % 125;
            queue_in = {next_data_in, mod_val};
            expected_queue.push_back(queue_in);
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
        $display("%m: # PASS");
        $finish(0);
    end else begin
        $display("%m: # FAIL, %0d errors", errors);
        $stop(0);
    end

    done = 1;
end

// Checker process
logic [WIDTH-1:0] input_val;
logic [6:0] exp_val, actual_val;
always_ff @(posedge clk) begin
    if (valid_out) begin
        if (expected_queue.size() == 0) begin
            $display("%m: time %0t: Output is valid, but expected queue is empty!",
                $time);
            errors++;
        end
        else begin
            queue_out = expected_queue.pop_front();
            exp_val <= queue_out[6:0];
            input_val <= queue_out[WIDTH+6:7];
            actual_val <= data_out;

            $display("%m: time %0t: input_val: %0d, exp_val: %0d , actual_val: %0d",
                $time, input_val, exp_val, actual_val);

            if (actual_val !== exp_val) begin
                $display("%m: time %0t: MISMATCH! input_val: %0d, expected: %0d, got: %0d",
                    $time, input_val, exp_val, actual_val);
                errors++;
            end
        end
    end
end

endmodule
