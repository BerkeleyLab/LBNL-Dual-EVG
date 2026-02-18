// Create heartbeat strobes
module heartbeatGenerator #(
    parameter TX_CLK_PER_HEARTBEAT = -1
    ) (
    input     txClk,

    input     sampCoincidenceMarker,
    input     sysRealignToggleIn,

    output    txHeartbeatStrobe);

localparam TX_HB_COUNTER_RELOAD = TX_CLK_PER_HEARTBEAT - 2;
localparam TX_HB_COUNTER_WIDTH = $clog2(TX_HB_COUNTER_RELOAD+1) + 1;
reg [TX_HB_COUNTER_WIDTH-1:0] txHeartbeatCounter = 0;
assign txHeartbeatStrobe = txHeartbeatCounter[TX_HB_COUNTER_WIDTH-1];

/*
 * Resync when alignment point changes
 */
(*ASYNC_REG="true"*) reg txRealignToggle_m = 0;
reg txRealignToggle = 0, txRealignMatch = 0;

/*
 * Coincidence marker from acquisition domain
 */
(*ASYNC_REG="true"*) reg txCoincidenceMarker_m = 0;
reg txCoincidenceMarker = 0, txCoincidenceMarker_d = 0;

always @(posedge txClk) begin
    txCoincidenceMarker_m <= sampCoincidenceMarker;
    txCoincidenceMarker   <= txCoincidenceMarker_m;
    txCoincidenceMarker_d <= txCoincidenceMarker;

    txRealignToggle_m <= sysRealignToggleIn;
    txRealignToggle   <= txRealignToggle_m;

    if (txRealignToggle != txRealignMatch) begin
        txHeartbeatCounter <= TX_HB_COUNTER_RELOAD;
        if (txCoincidenceMarker != txCoincidenceMarker_d) begin
            txRealignMatch <= !txRealignMatch;
        end
    end
    else begin
        if (txHeartbeatStrobe) begin
            txHeartbeatCounter <= TX_HB_COUNTER_RELOAD;
        end
        else begin
            txHeartbeatCounter <= txHeartbeatCounter - 1;
        end
    end
end

endmodule
