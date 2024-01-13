// Log event arrival
module evFIFO #(
    parameter ADDR_WIDTH = 10,
    parameter DEBUG      = "false"
    ) (
    input  wire        sysClk,
    input  wire        sysCsrStrobe,
    input  wire [31:0] sysGpioOut,
    output wire [31:0] sysCsr,
    output wire [31:0] sysDataTicks,

    input  wire        evClk,
    input  wire  [7:0] evChar,
    input  wire        evCharIsK);

reg [ADDR_WIDTH-1:0] sysReadAddress = 0, evWriteAddress = 0;
(*mark_debug=DEBUG*) reg sysRunning = 0;
wire [3:0] addrWidth = ADDR_WIDTH;
reg [39:0] dpram[0:(1<<ADDR_WIDTH)-1], dpramQ;
wire [7:0] sysDataEvent = dpramQ[39:32];
assign sysDataTicks = dpramQ[31:0];
assign sysCsr = { sysRunning, {3{1'b0}}, addrWidth,
                  sysDataEvent,
                  {16-ADDR_WIDTH{1'b0}}, evWriteAddress };
always @(posedge sysClk) begin
    if (sysCsrStrobe) begin
        sysReadAddress <= sysGpioOut[ADDR_WIDTH-1:0];
        sysRunning <= sysGpioOut[31];
    end
    dpramQ <= dpram[sysReadAddress];
end

(*ASYNC_REG="true"*) reg evRunning_m;
(*mark_debug=DEBUG*) reg evRunning;
reg        evDPRAMwen;
reg  [7:0] evDPRAMevent;
reg [31:0] evTickCounter = 0;
always @(posedge evClk) begin
    evRunning_m <= sysRunning;
    evRunning   <= evRunning_m;
    evTickCounter <= evTickCounter + 1;
    evDPRAMwen <= evRunning && !evCharIsK && (evChar != 0);
    evDPRAMevent <= evChar;
    if (evDPRAMwen) begin
        dpram[evWriteAddress] <= { evDPRAMevent, evTickCounter };
        evWriteAddress <= evWriteAddress + 1;
    end
    else if (!evRunning) begin
        evWriteAddress <= 0;
    end
end

endmodule
