// Control a Dynamic Reconfiguration Port and provide some control lines
module drpControl #(
    parameter DRP_DATA_WIDTH        = 16,
    parameter DRP_ADDR_WIDTH        = -1,
    parameter RESET_CONTROL_WIDTH   = -1,
    parameter RESET_STATUS_WIDTH    = -1,
    parameter DEBUG                 = "false"
    ) (
    input                      clk,
    (*mark_debug=DEBUG*) input strobe,
    input               [31:0] dataIn,
    output wire         [31:0] dataOut,

    output [RESET_CONTROL_WIDTH-1:0] resetControl,
    input   [RESET_STATUS_WIDTH-1:0] resetStatus,

    (*mark_debug=DEBUG*) output reg                      drp_en = 0,
    (*mark_debug=DEBUG*) output reg                      drp_we = 0,
    (*mark_debug=DEBUG*) input wire                      drp_rdy,
    (*mark_debug=DEBUG*) output reg [DRP_ADDR_WIDTH-1:0] drp_addr,
    (*mark_debug=DEBUG*) output reg [DRP_DATA_WIDTH-1:0] drp_di,
    (*mark_debug=DEBUG*) input wire [DRP_DATA_WIDTH-1:0] drp_do);

(*mark_debug=DEBUG*) reg busy = 0, writing;
(*mark_debug=DEBUG*) reg [DRP_DATA_WIDTH-1:0] data;

assign dataOut = { busy, resetStatus,
                   {32-1-RESET_STATUS_WIDTH-DRP_DATA_WIDTH{1'b0}},
                   data };
            
(*mark_debug=DEBUG*) reg [RESET_CONTROL_WIDTH-1:0] resetControl_r = 0;
assign resetControl = resetControl_r;

always @(posedge clk) begin
    if (strobe && dataIn[31]) begin
        resetControl_r <= dataIn[30-:RESET_CONTROL_WIDTH];
    end
    if (strobe && !dataIn[31]) begin
        busy     <= 1;
        drp_en   <= 1;
        drp_we   <= dataIn[30];
        writing  <= dataIn[30];
        drp_addr <= dataIn[16+:DRP_ADDR_WIDTH];
        drp_di   <= dataIn[0+:DRP_DATA_WIDTH];
        data     <= dataIn[0+:DRP_DATA_WIDTH];
    end
    else begin
        drp_en <= 0;
        drp_we <= 0;
        if (drp_rdy) begin
            busy <= 0;
            if (!writing) data <= drp_do;
        end
    end
end

endmodule

