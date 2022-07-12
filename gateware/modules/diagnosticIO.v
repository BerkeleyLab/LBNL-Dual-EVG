// Diagnostic and miscellaneous I/O
// For now outputs are driven by software, but it may be useful to
// provide some input ports to this module to forward to outputs.

module diagnosticIO #(
    parameter INPUT_WIDTH         = -1,
    parameter OUTPUT_WIDTH        = -1,
    parameter OUTPUT_SELECT_WIDTH = -1
    ) (
    input                                sysClk,
    input                                csrStrobe,
    input                         [31:0] GPIO_OUT,
    output wire                   [31:0] status,
    input                                auxSwitch_n,
    input              [INPUT_WIDTH-1:0] diagnosticIn,
    output reg        [OUTPUT_WIDTH-1:0] diagnosticOut = 0,
    output reg [OUTPUT_SELECT_WIDTH-1:0] diagnosticOutputSelect = 0);

assign status = { {16{1'b0}},
                  auxSwitch_n, {8-1{1'b0}},
                  {4-OUTPUT_SELECT_WIDTH{1'b0}}, diagnosticOutputSelect,
                  {4-OUTPUT_WIDTH{1'b0}}, diagnosticOut,
                  {4-INPUT_WIDTH{1'b0}}, diagnosticIn };

always @(posedge sysClk) begin
    if (csrStrobe) begin
        diagnosticOut <= GPIO_OUT[4+:OUTPUT_WIDTH];
        diagnosticOutputSelect <= GPIO_OUT[8+:OUTPUT_SELECT_WIDTH];
    end
end
endmodule
