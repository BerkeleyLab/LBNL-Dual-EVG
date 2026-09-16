// Create heartbeat strobes
module heartbeatGenerator #(
    parameter TX_CLK_PER_HEARTBEAT = -1
    ) (
    // synced with sampling clock
    input     sampCoincidenceMarker,

    // synced with sys clock
    input     sysRealignToggleIn,

    // synced with tx clock
    input     txClk,
    output    txCoincidenceMarker,
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
(*ASYNC_REG="true"*) reg txCoincIntMarker_m = 0, txCoincIntMarker = 0;
reg txCoincIntMarker_d = 0;

always @(posedge txClk) begin
    txCoincIntMarker_m <= sampCoincidenceMarker;
    txCoincIntMarker   <= txCoincIntMarker_m;
    txCoincIntMarker_d <= txCoincIntMarker;

    txRealignToggle_m <= sysRealignToggleIn;
    txRealignToggle   <= txRealignToggle_m;

    if (txRealignToggle != txRealignMatch) begin
        txHeartbeatCounter <= TX_HB_COUNTER_RELOAD;
        if (txCoincIntMarker && !txCoincIntMarker_d) begin
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

assign txCoincidenceMarker = txCoincIntMarker;

endmodule
