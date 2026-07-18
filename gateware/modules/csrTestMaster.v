module csrTestMaster #(
    parameter CSR_RESET_DELAY = 4,
    // Widths of data/strobe bus
    parameter CSR_DATA_BUS_WIDTH = 32,
    parameter CSR_STROBE_BUS_WIDTH = 32
) (
    input wire clk
);
// these signals make the CSR bus, which can be accessed from outside the module
reg     [CSR_DATA_BUS_WIDTH-1:0]                        csr_data_o = 0;
wire    [CSR_DATA_BUS_WIDTH*CSR_STROBE_BUS_WIDTH-1:0]   csr_data_i;
reg     [CSR_STROBE_BUS_WIDTH-1:0]                      csr_stb_o = 0;

reg [CSR_DATA_BUS_WIDTH-1:0] dummy;
reg csr_verbose = 1;
time last_access_t = 0;

reg csr_rw = 0;
reg csr_in_progress = 0;
wire [CSR_DATA_BUS_WIDTH-1:0] csr_data[0:CSR_STROBE_BUS_WIDTH-1];

genvar i;
generate for(i = 0; i < CSR_STROBE_BUS_WIDTH; i = i + 1) begin
    assign csr_data[i] = csr_data_i[(i+1)*CSR_DATA_BUS_WIDTH-1:i*CSR_DATA_BUS_WIDTH];
end
endgenerate

// ready signal. 1 indicates that CSR_TEST unit is initialized and ready for commands
reg ready = 0;

// generate the reset and ready signals
initial begin
    repeat(CSR_RESET_DELAY)
        @(posedge clk);

    ready <= 1;
end

// enables/disables displaying information about each read/write operation.
task verbose;
  input onoff;
begin
  csr_verbose = onoff;
end
endtask // verbose

task rw_generic;
  input   [$clog2(CSR_STROBE_BUS_WIDTH)-1:0]   sel;
  input   [CSR_DATA_BUS_WIDTH-1:0]             data_i;
  input                                        rw;
  output  [CSR_DATA_BUS_WIDTH-1:0]             data_o;
begin : rw_generic_main

    // Debug information
    if(csr_verbose) begin
        if(rw)
            $display("@%0d: CSR write: sel %d, data %x",
                $time, sel, data_i);
        else
            $display("@%0d: CSR read: sel %d",
                $time, sel);
    end // csr_verbose

    if($time != last_access_t) begin
        @(posedge clk);
    end

    // for external modules can know if that
    // was a read or write
    csr_rw <= rw;
    csr_in_progress <= 1'b1;

    // write
    if(rw) begin
        csr_stb_o[sel] <= 1'b1;
        csr_data_o <= data_i;
    end

    @(posedge clk);

    data_o = csr_data[sel];
    csr_stb_o <= 0;
    csr_in_progress <= 1'b0;
    last_access_t = $time;
  end
endtask // rw_generic

task write32;
    input [$clog2(CSR_STROBE_BUS_WIDTH)-1:0]   sel;
    input [CSR_DATA_BUS_WIDTH-1:0] data_i;
begin
    rw_generic(sel, data_i, 1, dummy);
end
endtask // write32

task read32;
    input [$clog2(CSR_STROBE_BUS_WIDTH)-1:0]   sel;
    output [CSR_DATA_BUS_WIDTH-1:0] data_o;
begin : read32_body
    reg [CSR_DATA_BUS_WIDTH-1:0] rval;
    rw_generic(sel, 0, 0, rval);
    data_o = rval[CSR_DATA_BUS_WIDTH-1:0];
end
endtask // read32

endmodule
