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
 * Wrapper to provide generic register access to fan tachometer
 */

module fanTach #(
    parameter CLK_FREQUENCY = 100000,
    parameter FAN_COUNT     = 1
    ) (
    input                 clk,
    input                 csrStrobe,
    input          [31:0] GPIO_OUT,
    output signed  [31:0] value,
    input [FAN_COUNT-1:0] tachs_a);

localparam MUXSEL_WIDTH = (FAN_COUNT == 1) ? 1 : $clog2(FAN_COUNT);
reg [MUXSEL_WIDTH-1:0] muxSel = 0;
wire signed [15:0] coreValue;
assign value = coreValue;

always @(posedge clk) begin
    if (csrStrobe) begin
        muxSel <= GPIO_OUT[MUXSEL_WIDTH-1:0] ;
    end
end

fanTachCore #(
    .CLK_FREQUENCY(CLK_FREQUENCY),
    .FAN_COUNT(FAN_COUNT))
  fanTachCore_i (
    .clk(clk),
    .muxSel(muxSel),
    .value(coreValue),
    .tachs_a(tachs_a));

endmodule
