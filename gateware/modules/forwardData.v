//
// Forward data from one clock domain to another
//

module forwardData(inClk, inData, outClk, outData);

parameter DATA_WIDTH = 32;

input                   inClk;
input  [DATA_WIDTH-1:0] inData;
input                   outClk;
output [DATA_WIDTH-1:0] outData;

//
// Input clock domain
//
reg [DATA_WIDTH-1:0] inLatch;
reg inReq = 0;
(* ASYNC_REG="TRUE" *) reg inAck_m = 0, inAck = 0;

always @(posedge inClk) begin
    if (inReq == inAck) begin
        inReq <= !inReq;
        inLatch <= inData;
    end
    inAck_m <= outReq_d;
    inAck <= inAck_m;
end

//
// Output clock domain
//
reg [DATA_WIDTH-1:0] outData;
(* ASYNC_REG="TRUE" *) reg outReq_m = 0, outReq = 0;
reg outReq_d = 0;
always @(posedge outClk) begin
    outReq_m <= inReq;
    outReq <= outReq_m;
    outReq_d <= outReq;
    if (outReq != outReq_d) begin
        outData <= inLatch;
    end
end

endmodule
