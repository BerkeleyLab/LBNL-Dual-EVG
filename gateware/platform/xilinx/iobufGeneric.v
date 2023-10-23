// IOBUF generic template for Xilinx

module iobufGeneric (
    // IOBUF facing interal FPGA logic
    input   bufferIn,
    output  bufferOut,
    input   bufferT,
    // IOBUF facing external FPGA logic
    inout   bufferInOut);

IOBUF #(
    .DRIVE(12),
    .IOSTANDARD("DEFAULT"),
    .SLEW("FAST"))
  inst (
    .O(bufferOut),
    .I(bufferIn),
    .IO(bufferInOut),
    .T(bufferT)
);

endmodule
