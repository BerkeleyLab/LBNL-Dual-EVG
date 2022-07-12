// Generate software-iniated event requests

module evgSoftwareTrigger #(
    parameter  EVENTCODE_WIDTH = 8
    ) (
    // Processor block connections
    input              sysClk,
    input              sysCSRstrobe,
    input       [31:0] sysGPIO_OUT,
    output wire [31:0] status,

    // Event requests
    input                            evgTxClk,
    output reg [EVENTCODE_WIDTH-1:0] evgSoftwareEventTDATA,
    output reg                       evgSoftwareEventTVALID = 0,
    input                            evgSoftwareEventTREADY);

reg sysToggle = 0;
reg [EVENTCODE_WIDTH-1:0] sysEvent;
(*ASYNC_REG="true"*) reg evgToggle_m = 0;
reg evgToggle = 0, evgToggle_d = 0;
reg evgAck = 0;

always @(posedge evgTxClk) begin
    evgToggle_m <= sysToggle;
    evgToggle   <= evgToggle_m;
    evgToggle_d <= evgToggle;
    if (evgSoftwareEventTVALID) begin
        if (evgSoftwareEventTREADY) begin
            evgSoftwareEventTVALID <= 0;
            evgAck <= evgToggle;
        end
    end
    else if (evgToggle != evgToggle_d) begin
        evgSoftwareEventTVALID <= 1;
        evgSoftwareEventTDATA <= sysEvent;
    end
end

///////////////////////////////////////////////////////////////////////////////
// System clock domain
reg sysBusy = 0;
(*ASYNC_REG="true"*) reg sysAck_m = 0;
reg sysAck = 0;

always @(posedge sysClk) begin
    sysAck_m <= evgAck;
    sysAck   <= sysAck_m;
    if (sysBusy) begin
        if (sysAck == sysToggle) begin
            sysBusy <= 0;
        end
    end
    else if (sysCSRstrobe) begin
        sysBusy <= 1;
        sysToggle <= !sysToggle;
        sysEvent <= sysGPIO_OUT[EVENTCODE_WIDTH-1:0];
    end
end

assign status = { {32-1-EVENTCODE_WIDTH{1'b0}}, sysBusy, sysEvent };

endmodule
