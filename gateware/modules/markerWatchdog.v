module markerWatchdog #(
    parameter SYSCLK_FREQUENCY = 100000000,
    parameter DEBUG            = "false"
    ) (
    input      sysClk,
    input      evrMarker,
    output reg isValid = 0);

localparam UPPER_LIMIT = ((SYSCLK_FREQUENCY * 11) / 10);
localparam LOWER_LIMIT = ((SYSCLK_FREQUENCY *  9) / 10);
(*mark_debug=DEBUG*) reg [$clog2(LOWER_LIMIT+1)-1:0] watchdog;
(*mark_debug=DEBUG*) reg marker_m, marker, marker_d;

always @(posedge sysClk) begin
    marker_m <= evrMarker;
    marker   <= marker_m;
    marker_d <= marker;
    if (marker && !marker_d) begin
        watchdog <= 0;
        if ((watchdog > LOWER_LIMIT)
         && (watchdog < UPPER_LIMIT)) begin
            isValid <= 1;
        end
        else begin
            isValid <= 0;
        end
    end
    else if (watchdog < UPPER_LIMIT) begin
        watchdog <= watchdog + 1;
    end
    else begin
        isValid <= 0;
    end
end
endmodule
