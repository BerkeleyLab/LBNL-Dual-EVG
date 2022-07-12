// Simple wrapper around bedrock i2c_chunk

module i2cHandler #(
    parameter CLK_RATE      = 100000000,
    parameter CHANNEL_COUNT = -1,
    parameter I2C_RATE      = 100000,
    parameter LB_ADDR_WIDTH = 12,
    parameter DEBUG         = "false"
    ) (
    input          clk,
    input          csrStrobe,
    input   [31:0] GPIO_OUT,
    output wire [31:0] status,

    (*mark_debug=DEBUG*) output reg [CHANNEL_COUNT-1:0] scl,
    (*mark_debug=DEBUG*) output reg [CHANNEL_COUNT-1:0] sda_drive,
    (*mark_debug=DEBUG*) input      [CHANNEL_COUNT-1:0] sda_sense);

localparam LB_DATA_WIDTH = 8;

reg [LB_ADDR_WIDTH-1:0] lb_addr;
reg [LB_DATA_WIDTH-1:0] lb_din;
reg lb_write = 0, run_cmd = 0,lb_freeze = 0, rst = 0;

wire [LB_DATA_WIDTH-1:0] lb_dout;
wire err_flag, run_stat, updated;
wire [3:0] addrWidth = LB_ADDR_WIDTH;

assign status = { rst, lb_freeze, run_stat, err_flag,
                  addrWidth,
                  lb_dout,
                  updated, {16-1-LB_ADDR_WIDTH{1'b0}}, lb_addr };

always @(posedge clk) begin
    if (csrStrobe) begin
        lb_addr   <= GPIO_OUT[0+:LB_ADDR_WIDTH];
        lb_din    <= GPIO_OUT[16+:LB_DATA_WIDTH];
        lb_write  <= GPIO_OUT[28];
        run_cmd   <= GPIO_OUT[29];
        lb_freeze <= GPIO_OUT[30];
        rst       <= GPIO_OUT[31];
    end
    else begin
        lb_write <= 0;
    end
end

(*mark_debug=DEBUG*) wire [3:0] hw_config;
(*mark_debug=DEBUG*) wire i2cSCL, i2cSDAdrive, i2cSDAsense;

i2c_chunk #(.initial_file("iicCommandTable.hex"),
            .tick_scale($clog2(CLK_RATE / (14 * I2C_RATE))))
  i2c_chunk (
    .clk(clk),
    .lb_addr(lb_addr),
    .lb_din(lb_din),
    .lb_write(lb_write),
    .lb_dout(lb_dout),
    .run_cmd(run_cmd),
    .freeze(lb_freeze),
    .run_stat(run_stat),
    .updated(updated),
    .err_flag(err_flag),
    .hw_config(hw_config),
    .scl(i2cSCL),
    .sda_drive(i2cSDAdrive),
    .sda_sense(i2cSDAsense),
    .rst(rst),
    .intp(1'b0));

// Multiplex I2C lines
wire [1:0] muxSel = hw_config[2:1];
assign i2cSDAsense = sda_sense[muxSel];
always @(posedge clk) begin
    scl <= ~(1 << muxSel) | (i2cSCL << muxSel);
    sda_drive <= ~(1 << muxSel) | (i2cSDAdrive << muxSel);
end

endmodule
