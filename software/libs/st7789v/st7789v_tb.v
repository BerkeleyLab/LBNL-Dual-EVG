/*
 * Copyright 2020, Lawrence Berkeley National Laboratory
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

`timescale 1 ns / 1ns

module st7789v_tb;

localparam CLK_RATE = 100000000;

localparam CSR_RESET                    = 32'h80000000;
localparam CSR_READ                     = 32'h40000000;
localparam CSR_READ_IS_32_BIT           = 32'h20000000;

localparam REG_7  = 8'hAB;

reg clk = 1, csrStrobe = 0, dataStrobe = 0;
reg [31:0] GPIO_OUT = {32{1'bx}};
wire [31:0] statusReg, readData;
wire busy = statusReg[30];

wire DISPLAY_RESET_N, DISPLAY_CMD_N;
wire DISPLAY_CLK, DISPLAY_CS_N, DISPLAY_SDA_O, DISPLAY_SDA_T, DISPLAY_SDA_I;
wire DISPLAY_MOSI = DISPLAY_SDA_T ? 1'bz : DISPLAY_SDA_O;

always begin
    #5 clk = !clk;
end

ST7789V #(.CLK_RATE(CLK_RATE))
  st7789v (.clk(clk),
           .csrStrobe(csrStrobe),
           .dataStrobe(dataStrobe),
           .gpioOut(GPIO_OUT),
           .status(statusReg),
           .readData(readData),
           .DISPLAY_RESET_N(DISPLAY_RESET_N),
           .DISPLAY_CMD_N(DISPLAY_CMD_N),
           .DISPLAY_CLK(DISPLAY_CLK),
           .DISPLAY_CS_N(DISPLAY_CS_N),
           .DISPLAY_SDA_O(DISPLAY_SDA_O),
           .DISPLAY_SDA_T(DISPLAY_SDA_T),
           .DISPLAY_SDA_I(DISPLAY_SDA_I));

// Fake device (multi-byte read requires a dummy cycle at the beginning)
reg [32:0] deviceSend = {33{1'bz}};
assign DISPLAY_SDA_I = DISPLAY_SDA_T ? deviceSend[32] : 1'bx;
reg [31:0] dataReceived = {32{1'bz}}, matchData = {32{1'bx}};
reg [7:0] commandReceived = {8{1'bx}}, matchCommand = {8{1'bx}};
integer failed = 0;
integer shiftEnable = 0;
always @(negedge DISPLAY_CS_N) begin
    commandReceived = {8{1'bx}};
    dataReceived = {32{1'bz}};
    deviceSend = {33{1'bz}};
end
always @(posedge DISPLAY_CS_N) begin
    if ((commandReceived != matchCommand) || (dataReceived != matchData)) begin
        $display("Expect command %x, data %x, got %x:%x   FAIL",
                        matchCommand, matchData, commandReceived, dataReceived);
        failed = 1;
    end
end
always @(posedge DISPLAY_CLK) begin
    if (DISPLAY_CMD_N) begin
        shiftEnable = 1;
        dataReceived = {dataReceived[30:0], DISPLAY_MOSI};
    end
    else begin
        shiftEnable = 0;
        commandReceived = {commandReceived[6:0], DISPLAY_MOSI};
    end
end
always @(posedge DISPLAY_CMD_N) begin
    deviceSend <= (commandReceived == 9) ? {1'bz, 32'h12345678}
                                        : {commandReceived, {25{1'bz}}};
end
always @(negedge DISPLAY_CLK) begin
    if (shiftEnable) begin
        deviceSend <= {deviceSend[31:0], 1'bz};
    end
end

initial begin
    $dumpfile("st7789v_tb.lxt");
    $dumpvars(0, st7789v_tb);

    #40 ;
    writeCSR(CSR_RESET);
    #40 ;
    writeCSR(0);
    #40 ;
    readRegister(7);
    readRegister(9);
    #100 ;

    // Write single value
    writeData(1, 0, 16'h9876);
    matchCommand = 8'h98;
    matchData = {{24{1'bx}}, 8'h76};
    #100 ;
    while (DISPLAY_CS_N == 0) # 1;
    #40 ;

    // Write single value another way
    writeData(0, 0, {8'h12, {8{1'bz}}});
    writeData(1, 1, {8'h34, {8{1'bz}}});
    matchCommand = 8'h12;
    matchData = {{24{1'bx}}, 8'h34};
    #100 ;
    while (DISPLAY_CS_N == 0) # 1;
    #40 ;

    // Write two single values back to back
    writeData(0, 0, {8'hFE, {8{1'bz}}});
    writeData(1, 1, {8'hDC, {8{1'bz}}});
    writeData(0, 0, {8'hBA, {8{1'bz}}});
    writeData(1, 1, {8'h98, {8{1'bz}}});
    matchCommand = 8'hFE;
    matchData = {{24{1'bx}}, 8'hDC};
    #100 ;
    while (DISPLAY_CS_N == 0) # 1;
    #100 ;
    matchCommand = 8'hBA;
    matchData = {{24{1'bx}}, 8'h98};
    while (DISPLAY_CS_N == 0) # 1;

    // Write double value
    writeData(0, 1, {8'hAB, {8{1'bx}}});
    writeData(0, 0, 16'hCDEF);
    writeData(1, 0, 16'h1234);
    matchCommand = 8'hAB;
    matchData = 32'hCDEF1234;
    #100 ;
    while (DISPLAY_CS_N == 0) # 1;
    #40 ;

    // Write double value with gap
    writeData(0, 1, {8'hAB, {8{1'bx}}});
    writeData(0, 0, 16'hCAFE);
    #4000 ;
    writeData(1, 0, 16'hF00D);
    matchCommand = 8'hAB;
    matchData = 32'hCAFEF00D;
    #100 ;
    while (DISPLAY_CS_N == 0) # 1;
    #40 ;

    $display("%s", failed ? "FAIL" : "PASS");
    $finish;
end

task readRegister;
    input [7:0] r;
    reg [31:0] expect;
    begin
        expect = (r == 9) ? 32'h12345678 : {{24{1'bx}}, r};
        writeCSR(CSR_READ | ((r == 9) ? CSR_READ_IS_32_BIT : 0) | r);
        while (busy) # 1;
        if (readData != expect) begin
            failed = 1;
            $display("Register %02x.  Expect %x, got %x   FAIL", r, expect, readData);
        end
    end
endtask

task writeCSR;
    input [31:0] csr;
    begin
        GPIO_WRITE(1, 0, csr);
    end
endtask

task writeData;
    input isLast;
    input isByte;
    input [15:0] value;
    begin
        GPIO_WRITE(0, 1, {14'b0, isLast, isByte, value});
    end
endtask

task GPIO_WRITE;
    input csrEnable, dataEnable;
    input [31:0] value;
    begin
        @(posedge clk) begin
            csrStrobe <= csrEnable;
            dataStrobe <= dataEnable;
            GPIO_OUT <= value;
        end
        @(posedge clk) begin
            csrStrobe <= 0;
            dataStrobe <= 0;
            GPIO_OUT <= 32'bx;
        end
        @(posedge clk) ;
    end
endtask

endmodule
