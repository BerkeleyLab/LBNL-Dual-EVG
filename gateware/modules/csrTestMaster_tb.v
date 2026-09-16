`timescale 1ns / 1ns

module csrTestMaster_tb #(
    parameter CSR_DATA_BUS_WIDTH = 32,
    parameter CSR_STROBE_BUS_WIDTH = 8,
    parameter NUMBER_OF_RW_TESTS = 10
);

reg clk;

integer cc;
integer errors = 0;
integer idx = 0;
initial begin
    if ($test$plusargs("vcd")) begin
        $dumpfile("csrTestMaster.vcd");
        $dumpvars(0, csrTestMaster_tb);

        for (idx = 0; idx < NUMBER_OF_RW_TESTS; idx = idx + 1) begin
            $dumpvars(0, csrTestMaster_tb.expected_data_read_results[idx]);
            $dumpvars(0, csrTestMaster_tb.expected_data_write_results[idx]);
            $dumpvars(0, csrTestMaster_tb.expected_sel_read_results[idx]);
            $dumpvars(0, csrTestMaster_tb.expected_sel_write_results[idx]);
        end

        for (idx = 0; idx < CSR_STROBE_BUS_WIDTH; idx = idx + 1) begin
            $dumpvars(0, csrTestMaster_tb.data_dev_to_csr[idx]);
        end

    end

    clk = 0;
    for (cc = 0; cc < 1000; cc = cc+1) begin
        clk = 0; #5;
        clk = 1; #5;
    end

    if (errors==0) begin
        $display("# PASS");
        $finish(0);
    end else begin
        $display("# FAIL");
        $stop(0);
    end
end

csrTestMaster # (
    .CSR_DATA_BUS_WIDTH(CSR_DATA_BUS_WIDTH),
    .CSR_STROBE_BUS_WIDTH(CSR_STROBE_BUS_WIDTH)
) CSR0 (
    .clk(clk)
);

reg [CSR_DATA_BUS_WIDTH-1:0]    expected_data_read_results[0:NUMBER_OF_RW_TESTS-1];
reg [CSR_DATA_BUS_WIDTH-1:0]    expected_data_write_results[0:NUMBER_OF_RW_TESTS-1];
reg [CSR_STROBE_BUS_WIDTH-1:0]  expected_sel_read_results[0:NUMBER_OF_RW_TESTS-1];
reg [CSR_STROBE_BUS_WIDTH-1:0]  expected_sel_write_results[0:NUMBER_OF_RW_TESTS-1];
reg [CSR_DATA_BUS_WIDTH-1:0]    data_read_from_dev = 0;
reg [CSR_DATA_BUS_WIDTH-1:0]    data_dev_to_csr[0:CSR_STROBE_BUS_WIDTH-1];
wire [CSR_STROBE_BUS_WIDTH-1:0] stb_from_csr = CSR0.csr_stb_o;

genvar i;
generate for(i = 0; i < CSR_STROBE_BUS_WIDTH; i = i + 1) begin
    assign CSR0.csr_data_i[(i+1)*CSR_DATA_BUS_WIDTH-1:i*CSR_DATA_BUS_WIDTH] = data_dev_to_csr[i];
end
endgenerate

// stimulus
integer stim_n = 0;
initial begin
    // initialize device data
    for (idx = 0; idx < CSR_STROBE_BUS_WIDTH; idx = idx + 1) begin
        data_dev_to_csr[idx] = 0;
    end

    wait(CSR0.ready);

    // write
    for (stim_n = 0; stim_n < NUMBER_OF_RW_TESTS; stim_n = stim_n + 1) begin
        @(posedge clk);
        expected_sel_write_results[stim_n] = $urandom % CSR_STROBE_BUS_WIDTH;
        expected_data_write_results[stim_n] = $urandom % 1000;

        CSR0.write32(expected_sel_write_results[stim_n], expected_data_write_results[stim_n]);
    end

    //read
    for (stim_n = 0; stim_n < NUMBER_OF_RW_TESTS; stim_n = stim_n + 1) begin
        @(posedge clk);
        expected_sel_read_results[stim_n] = $urandom % CSR_STROBE_BUS_WIDTH;
        data_dev_to_csr[expected_sel_read_results[stim_n]] = $urandom % 1000;
        expected_data_read_results[stim_n] = data_dev_to_csr[expected_sel_read_results[stim_n]];

        CSR0.read32(expected_sel_read_results[stim_n], data_read_from_dev);
    end
end

// check
localparam IDLE                             = 2'd0,
           WRITE_TRANSACTION_IN_PROGRESS    = 2'd1,
           READ_TRANSACTION_IN_PROGRESS     = 2'd2,
           CHECK_TRANSACTION                = 2'd3;
reg [1:0] test_state = IDLE;
reg [CSR_STROBE_BUS_WIDTH-1:0] test_stb = 0;
reg [CSR_DATA_BUS_WIDTH-1:0] test_data = 0;
reg test_rw = 0;
integer result_write_n = 0;
integer result_read_n = 0;
always @(posedge clk) begin

    case(test_state)
    IDLE: begin
        if (CSR0.csr_in_progress) begin
            // save bus signals
            test_rw <= CSR0.csr_rw;
            if (CSR0.csr_rw) begin
                test_stb <= CSR0.csr_stb_o;
                test_data <= CSR0.csr_data_o;
                test_state <= WRITE_TRANSACTION_IN_PROGRESS;
            end else begin
                test_state <= READ_TRANSACTION_IN_PROGRESS;
            end
        end
    end

    WRITE_TRANSACTION_IN_PROGRESS: begin
        if (!CSR0.csr_in_progress) begin
            test_state <= CHECK_TRANSACTION;
        end
    end

    READ_TRANSACTION_IN_PROGRESS: begin
        if (!CSR0.csr_in_progress) begin
            test_state <= CHECK_TRANSACTION;
            test_data <= data_read_from_dev;
            test_stb <= CSR0.csr_stb_o;
        end
    end

    CHECK_TRANSACTION: begin
        // write
        if (test_rw) begin
            if (test_stb != (1 << expected_sel_write_results[result_write_n]) ||
                test_data != expected_data_write_results[result_write_n]) begin
                errors = errors + 1;
            end
            result_write_n = result_write_n + 1;
        //read
        end else begin
            if (test_stb != 0 ||
                test_data != expected_data_read_results[result_read_n]) begin
                errors = errors + 1;
            end
            result_read_n = result_read_n + 1;
        end

        test_state <= IDLE;
    end

    endcase
end

endmodule
