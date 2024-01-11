// Let processor read events
module evLogger #(
    parameter DEBUG = "false"
    ) (
    input         sysClk,
    input  [31:0] GPIO_OUT,
    input         csrStrobeLogger1,
    output [31:0] statusLogger1,
    input         csrStrobeLogger2,
    output [31:0] statusLogger2,
    output [31:0] sysDataTicksLogger2,

    input        evgTxClk,
    input  [1:0] evgTxCharIsK,
    input [15:0] evgTxData);

///////////////////////////////////////////////////////////////////////////////
// System clock domain
(*mark_debug=DEBUG*) reg        sysReset = 0;
(*mark_debug=DEBUG*) wire       sysRdEnable;
(*mark_debug=DEBUG*) wire       sysRdEmpty;
(*mark_debug=DEBUG*) wire [7:0] sysRdData;
assign sysRdEnable = csrStrobeLogger1 && GPIO_OUT[8];
always @(posedge sysClk) begin
    if (csrStrobeLogger1) begin
        sysReset <= GPIO_OUT[9];
    end
end
assign statusLogger1 = {22'b0, sysReset, sysRdEmpty, sysRdData};

///////////////////////////////////////////////////////////////////////////////
// Event stream clock domain
(*mark_debug=DEBUG*) wire [7:0] evgCode;
(*mark_debug=DEBUG*) reg  [7:0] evgWrData;
(*mark_debug=DEBUG*) reg        evgWrEnable;
(*mark_debug=DEBUG*) wire       evgWrFull;
assign evgCode = evgTxData[7:0];
always @(posedge evgTxClk) begin
    evgWrEnable <= ((evgCode != 0)
                 && (evgCode != 8'h70) /* TOD seconds 0 */
                 && (evgCode != 8'h71) /* TOD seconds 1 */
                 && (evgTxCharIsK[0] == 0)
                 && (evgWrFull == 0));
    evgWrData <= evgCode;
end

`ifndef SIMULATE
evLogFIFO evLogFIFO (
  .rst(sysReset),
  .wr_clk(evgTxClk),
  .wr_en(evgWrEnable),
  .din(evgWrData),
  .full(evgWrFull),
  .rd_clk(sysClk),
  .rd_en(sysRdEnable),
  .dout(sysRdData),
  .empty(sysRdEmpty));
`endif // `ifndef SIMULATE

evFIFO evFIFOtlog (
    .sysClk(sysClk),
    .sysCsrStrobe(csrStrobeLogger2),
    .sysGpioOut(GPIO_OUT),
    .sysCsr(statusLogger2),
    .sysDataTicks(sysDataTicksLogger2),
    .evClk(evgTxClk),
    .evChar(evgTxData[7:0]),
    .evCharIsK(evgTxCharIsK[0]));

endmodule
