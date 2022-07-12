/*
 * Copyright 2021, Lawrence Berkeley National Laboratory
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS
 * AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Fan tachometer
 */
module fanTachCore #(
    parameter CLK_FREQUENCY = -1,
    parameter FAN_COUNT     = 1,
    parameter MUXSEL_WIDTH = (FAN_COUNT == 1) ? 1 : $clog2(FAN_COUNT)
    ) (
    input                    clk,

    input [MUXSEL_WIDTH-1:0] muxSel,
    output signed     [15:0] value,

    input    [FAN_COUNT-1:0] tachs_a);

// Check once every 50 microseconds -- makes debouncing easier
localparam SAMPLE_FREQUENCY = 20000;
localparam CLK_COUNTER_RELOAD = (CLK_FREQUENCY / SAMPLE_FREQUENCY) - 2;
localparam CLK_COUNTER_WIDTH = $clog2(CLK_COUNTER_RELOAD+1) + 1;
reg [CLK_COUNTER_WIDTH-1:0] clkCounter = CLK_COUNTER_RELOAD;
wire clkCounterDone = clkCounter[CLK_COUNTER_WIDTH-1];

// Update once every four seconds
localparam TICK_COUNTER_RELOAD = (SAMPLE_FREQUENCY * 4) - 2;
localparam TICK_COUNTER_WIDTH = $clog2(TICK_COUNTER_RELOAD+1) + 1;
reg [TICK_COUNTER_WIDTH-1:0] tickCounter = TICK_COUNTER_RELOAD;
wire tickCounterDone = tickCounter[TICK_COUNTER_WIDTH-1];
reg latch = 0;

// Count pulses -- assume 30,000 RPM max, 4 pulses per revolution, two edges
localparam MAX_COUNT = (30000 * 4 * 2) / 60;
localparam COUNTER_WIDTH = $clog2(MAX_COUNT+1) + 1;
reg [(FAN_COUNT*COUNTER_WIDTH)-1:0] results = 0;

// Multiplex results
reg signed [COUNTER_WIDTH-1:0] muxVal;
assign value = muxVal;

always @(posedge clk) begin
    if (clkCounterDone) begin
        clkCounter <= CLK_COUNTER_RELOAD;
        if (tickCounterDone) begin
            tickCounter <= TICK_COUNTER_RELOAD;
            latch <= 1;
        end
        else begin
            tickCounter <= tickCounter - 1;
            latch <= 0;
        end
    end
    else begin
        clkCounter <= clkCounter - 1;
        latch <= 0;
    end
    muxVal <= results[muxSel*COUNTER_WIDTH+:COUNTER_WIDTH];
end

genvar i;
generate
for (i = 0 ; i < FAN_COUNT ; i = i + 1) begin
    (*ASYNC_REG="true"*) reg tach_m = 0;
    reg tach_d0 = 0, tach_d1 = 0, tach_Old = 0;
    reg [COUNTER_WIDTH-1:0] counter = 0;
    wire counterOverflow = counter[COUNTER_WIDTH-1];
    always @(posedge clk) begin
        if (clkCounterDone) begin
            tach_m  <= tachs_a[i];
            tach_d0 <= tach_m;
            tach_d1 <= tach_d0;
            if ((tach_d0 == tach_d1) && (tach_d0 != tach_Old)) begin
                tach_Old <= tach_d0;
                if (!counterOverflow) begin
                    counter <= counter + 1;
                end
            end
        end
        else if (latch) begin
            results[i*COUNTER_WIDTH+:COUNTER_WIDTH] <= counter;
            counter <= 0;
        end
    end
end
endgenerate
endmodule
