// Trivial bit-banging connection to (Q)SPI bootstrap flash memory

module spiFlashBitBang #(
    parameter DEBUG = "false"
    ) (
    input              sysClk,
    input       [31:0] sysGPIO_OUT,
    input              sysCSRstrobe,
    output wire [31:0] sysStatus,

    (*mark_debug=DEBUG*) output reg spiFlashClk = 0,
    (*mark_debug=DEBUG*) output reg spiFlashMOSI = 0,
    (*mark_debug=DEBUG*) output reg spiFlashCS_B = 1,
    (*mark_debug=DEBUG*) input      spiFlashMISO);

always @(posedge sysClk) begin
    if (sysCSRstrobe) begin
        if (sysGPIO_OUT[0]) begin
            spiFlashClk <= 1'b1;
        end
        else if (sysGPIO_OUT[1]) begin
            spiFlashClk <= 1'b0;
        end
        if (sysGPIO_OUT[2]) begin
            spiFlashCS_B <= 1'b1;
        end
        else if (sysGPIO_OUT[3]) begin
            spiFlashCS_B <= 1'b0;
        end
        if (sysGPIO_OUT[4]) begin
            spiFlashMOSI <= 1'b1;
        end
        else if (sysGPIO_OUT[5]) begin
            spiFlashMOSI <= 1'b0;
        end
    end
end

assign sysStatus = { {24{1'b0}},
            1'b0, spiFlashMISO,
            1'b0, spiFlashMOSI,
            1'b0, spiFlashCS_B,
            1'b0, spiFlashClk };

endmodule
