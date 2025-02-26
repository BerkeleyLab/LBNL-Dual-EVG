module measureLatency #(
    parameter SAMPLING_CLOCK_RATE = 500000000,
    parameter DEBUG               = "false"
    ) (
    input         sysClk,
    output [31:0] sysLatency,

    input samplingClk,
    input rxValid,
    input ping,
    input echo
    );

// Limit measurement to about 2 us
localparam TICK_LIMIT = SAMPLING_CLOCK_RATE / 500000;
localparam LATENCY_WIDTH = $clog2(TICK_LIMIT+1);

// Measurement resolution ~0.5 ns.
localparam LATENCY_WIDEN = $clog2(2000000000 / SAMPLING_CLOCK_RATE);

///////////////////////////////////////////////////////////////////////////////
// Sampling clock domain
localparam COUNTER_WIDTH = LATENCY_WIDTH + 1;
reg [COUNTER_WIDTH-1:0] sampleCounter;
(*mark_debug=DEBUG*) reg [LATENCY_WIDTH-1:0] rawLatency;
wire sampleCounterOverflow = sampleCounter[COUNTER_WIDTH-1];
reg measuring = 0, newValueToggle = 0;

(*ASYNC_REG="true"*) reg valid_m = 0, ping_m = 0, echo_m = 0;
reg valid, ping_d0 = 0, ping_d1 = 0, echo_d0 = 0, echo_d1 = 0;
reg pingStrobe = 0, echoStrobe = 0;

always @(posedge samplingClk) begin
    valid_m <= rxValid;
    valid   <= valid_m;
    ping_m  <= ping;
    ping_d0 <= ping_m;
    ping_d1 <= ping_d0;
    echo_m  <= echo;
    echo_d0 <= echo_m;
    echo_d1 <= echo_d0;
    pingStrobe <= ping_d0 && !ping_d1;
    echoStrobe <= echo_d0 && !echo_d1;
    if (measuring) begin
        sampleCounter <= sampleCounter - 1;
        if (pingStrobe || !valid) begin
            measuring <= 0;
        end
        else if (echoStrobe || sampleCounterOverflow) begin
            measuring <= 0;
            rawLatency <= sampleCounter[LATENCY_WIDTH-1:0];
            newValueToggle <= !newValueToggle;
        end
    end
    else begin
        sampleCounter <= TICK_LIMIT;
        measuring <= pingStrobe && valid;
    end
end

///////////////////////////////////////////////////////////////////////////////
// System clock domain
localparam RESULT_WIDTH = LATENCY_WIDEN + LATENCY_WIDTH;
localparam FILTER_WIDTH = RESULT_WIDTH + 3;
localparam FILTER_WIDEN = FILTER_WIDTH - LATENCY_WIDTH;
reg [FILTER_WIDTH-1:0] filter = 0;

(*ASYNC_REG="true"*) reg sysNewValueToggle_m = 0;
reg sysNewValueToggle = 0, sysNewValueToggle_d = 0;
(*mark_debug=DEBUG*) reg sysNewValueStrobe = 0;
always @(posedge sysClk) begin
    sysNewValueToggle_m <= newValueToggle;
    sysNewValueToggle   <= sysNewValueToggle_m;
    sysNewValueToggle_d <= sysNewValueToggle;
    sysNewValueStrobe <= sysNewValueToggle ^ sysNewValueToggle_d;
    if (sysNewValueStrobe) begin
        filter <= filter - (filter >> FILTER_WIDEN) + (TICK_LIMIT - rawLatency);
    end
end

assign sysLatency = { {16-LATENCY_WIDTH{1'b0}},
                      rawLatency,
                      {16-RESULT_WIDTH{1'b0}},
                      filter[FILTER_WIDTH-1-:RESULT_WIDTH] };
endmodule
