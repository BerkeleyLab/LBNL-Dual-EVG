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

// SITRONIX ST7789V Display Controller
// 4-line 8-bit serial interface (SDA pin is bidirectional) IM[3:0]=4'b0110
// The EXTRA_WRITE_CYCLE_NS and EXTRA_READ_CYCLE_NS parameters provide a 
// mechanism for adding extra time to account for level shifter latency.  This
// is usually necessary only for read cycles so the default value for write
// cycle extension is 0.

module ST7789V #(
    parameter CLK_RATE                    = 100000000,
    parameter EXTRA_WRITE_CYCLE_NS        = 0,
    parameter EXTRA_READ_CYCLE_NS         = 100,
    parameter COMMAND_QUEUE_ADDRESS_WIDTH = 11,
    parameter DEBUG                       = "false"
    ) (
    input              clk,
    input              csrStrobe,
    input              dataStrobe,
    input       [31:0] gpioOut,
    output wire [31:0] status,
    output reg  [31:0] readData,

    (*mark_debug=DEBUG*) output reg  DISPLAY_BACKLIGHT_ENABLE = 1,
    (*mark_debug=DEBUG*) output reg  DISPLAY_RESET_N = 0,
    (*mark_debug=DEBUG*) output reg  DISPLAY_CMD_N = 0,
    (*mark_debug=DEBUG*) output reg  DISPLAY_CLK = 0,
    (*mark_debug=DEBUG*) output reg  DISPLAY_CS_N = 1,
    (*mark_debug=DEBUG*) output wire DISPLAY_SDA_O,
    (*mark_debug=DEBUG*) output reg  DISPLAY_SDA_T = 0,
    (*mark_debug=DEBUG*) input       DISPLAY_SDA_I);

function integer ns2ticks;
    input integer ns;
    begin
        ns2ticks = (((ns) * (CLK_RATE/100)) + 9999999) / 10000000;
    end
endfunction

// Minimum SPI clock cycle time read=160 ns, write=80 ns.
localparam READ_CYCLE_TICKS      = ns2ticks(160+EXTRA_READ_CYCLE_NS);
localparam WRITE_CYCLE_TICKS     = ns2ticks(80+EXTRA_WRITE_CYCLE_NS);
localparam CS_ENABLE_TICKS       = ns2ticks(20);
localparam CS_READ_DISABLE_TICKS = ns2ticks(60);
localparam CS_PAUSE_TICKS        = ns2ticks(40);

localparam COMMAND_SHIFTREG_WIDTH = 16;
localparam READ_RELOAD            = ((READ_CYCLE_TICKS + 1) / 2) - 2;
localparam TICK_COUNTER_WIDTH     = $clog2(READ_RELOAD+1);
localparam WRITE_RELOAD           = ((WRITE_CYCLE_TICKS + 1) / 2) - 2;
localparam CS_ENABLE_RELOAD       = CS_ENABLE_TICKS - 2;
localparam CS_READ_DISABLE_RELOAD = CS_READ_DISABLE_TICKS - 3;
localparam CS_PAUSE_RELOAD        = CS_PAUSE_TICKS - 2;

// Command queue
localparam COMMAND_QUEUE_WIDTH = 18;
localparam COMMAND_QUEUE_DATA_WIDTH = COMMAND_QUEUE_WIDTH - 2;
reg [COMMAND_QUEUE_ADDRESS_WIDTH-1:0] commandQueueHead=0, commandQueueTail=0;
reg [COMMAND_QUEUE_ADDRESS_WIDTH-1:0] commandQueueFreeCount;
wire [COMMAND_QUEUE_ADDRESS_WIDTH-1:0] commandQueueNextHead=commandQueueHead+1;
reg [COMMAND_QUEUE_WIDTH-1:0]
            commandQueue[0:(1<<COMMAND_QUEUE_ADDRESS_WIDTH)-1], commandQueueOut;
wire [COMMAND_QUEUE_DATA_WIDTH-1:0] commandQueueData =
                                   commandQueueOut[0+:COMMAND_QUEUE_DATA_WIDTH];
wire commandQueueByteFlag = commandQueueOut[COMMAND_QUEUE_DATA_WIDTH+0];
wire commandQueueLastFlag = commandQueueOut[COMMAND_QUEUE_DATA_WIDTH+1];
reg fifoFull = 0, fifoEmpty = 1;
always @(posedge clk) begin
    fifoEmpty <= (commandQueueHead == commandQueueTail);
    fifoFull <= (commandQueueNextHead == commandQueueTail);
    commandQueueOut <= commandQueue[commandQueueTail];
    if (dataStrobe) begin
        commandQueue[commandQueueHead] <= gpioOut[0+:COMMAND_QUEUE_WIDTH];
        commandQueueHead <= commandQueueHead + 1;
    end
end

// State machine
localparam S_IDLE     = 0,
           S_TRANSFER = 1,
           S_FINISH   = 2,
           S_PAUSE    = 3;
(*mark_debug=DEBUG*) reg [1:0] state = S_IDLE;
reg busy = 0, isLast, isRead, isSingleCommandByte;

// Counters are one bit wider to allow MSB to act as completion flag
localparam BIT_COUNTER_8  = 8 - 2;
reg [6:0] bitCounter;
wire bitCounterDone = bitCounter[6];
reg [4:0] commandBitCounter;
wire commandDone = commandBitCounter[4];
reg [TICK_COUNTER_WIDTH:0] tickCounter, tickCounterReload;
wire tick = tickCounter[TICK_COUNTER_WIDTH];

reg [COMMAND_SHIFTREG_WIDTH-1:0] commandShiftReg;
assign DISPLAY_SDA_O = commandShiftReg[COMMAND_SHIFTREG_WIDTH-1];

wire [4:0] commandQueueAddressWidth = COMMAND_QUEUE_ADDRESS_WIDTH;
assign status = { 1'b0, busy, 4'b0, DISPLAY_BACKLIGHT_ENABLE, 1'b0,
                  commandQueueAddressWidth, fifoFull,
                  {18-COMMAND_QUEUE_ADDRESS_WIDTH{1'b0}},commandQueueFreeCount};

always @(posedge clk) begin
    commandQueueFreeCount<= commandQueueTail - commandQueueNextHead;
    if (csrStrobe) begin
        DISPLAY_RESET_N <= !gpioOut[31];
        if (gpioOut[24]) DISPLAY_BACKLIGHT_ENABLE <= 0;
        else if (gpioOut[25]) DISPLAY_BACKLIGHT_ENABLE <= 1;
    end
    if (DISPLAY_RESET_N == 0) begin
        commandQueueTail <= commandQueueHead;
        state <= S_IDLE;
    end
    else if (state == S_IDLE) begin
        tickCounter <= CS_ENABLE_RELOAD;
        tickCounterReload <= WRITE_RELOAD;
        commandBitCounter <= BIT_COUNTER_8;
        DISPLAY_CMD_N <= 0;
        DISPLAY_SDA_T <= 0;
        DISPLAY_CLK <= 0;
        if (csrStrobe && gpioOut[30]) begin
            busy <= 1;
            // Some multi-byte readback requires extra cycle
            bitCounter <= BIT_COUNTER_8 + (gpioOut[29] ? 33 :
                                          (gpioOut[28] ? 32 :
                                          (gpioOut[27] ? 25 : 8)));
            commandShiftReg <= {gpioOut[7:0], {COMMAND_SHIFTREG_WIDTH-8{1'b0}}};
            isLast <= 1;
            isSingleCommandByte <= 0;
            isRead <= 1;
            DISPLAY_CS_N <= 0;
            state <= S_TRANSFER;
        end
        else if (!fifoEmpty) begin
            busy <= 1;
            bitCounter <= BIT_COUNTER_8 + (commandQueueByteFlag ? 0 : 8);
            commandShiftReg <= commandQueueData;
            isLast <= commandQueueLastFlag;
            isSingleCommandByte <= commandQueueLastFlag && commandQueueByteFlag;
            isRead <= 0;
            DISPLAY_CS_N <= 0;
            commandQueueTail <= commandQueueTail + 1;
            state <= S_TRANSFER;
        end
        else begin
            busy <= 0;
        end
    end
    else begin
        if (tick) begin
            case (state)
            S_TRANSFER: begin
                tickCounter <= tickCounterReload;
                if (DISPLAY_CLK) begin
                    DISPLAY_CMD_N <= commandDone && !isSingleCommandByte;
                    DISPLAY_SDA_T <= commandDone && isRead;
                    if (!commandDone) begin
                        commandBitCounter <= commandBitCounter - 1;
                    end
                    if (bitCounterDone) begin
                        if (isLast) begin
                            DISPLAY_CLK <= !DISPLAY_CLK;
                            tickCounter <= CS_READ_DISABLE_RELOAD;
                            state <= S_FINISH;
                        end
                        else if (!fifoEmpty) begin
                            DISPLAY_CLK <= !DISPLAY_CLK;
                            bitCounter <= BIT_COUNTER_8 +
                                                 (commandQueueByteFlag ? 0 : 8);
                            commandShiftReg <= commandQueueData;
                            isLast <= commandQueueLastFlag;
                            commandQueueTail <= commandQueueTail + 1;
                        end
                    end
                    else begin
                        DISPLAY_CLK <= !DISPLAY_CLK;
                        bitCounter <= bitCounter - 1;
                        commandShiftReg <=
                            {commandShiftReg[COMMAND_SHIFTREG_WIDTH-2:0], 1'b0};
                    end
                end
                else begin
                    DISPLAY_CLK <= !DISPLAY_CLK;
                    if (commandDone && isRead) tickCounterReload <= READ_RELOAD;
                    readData <= { readData[30:0], DISPLAY_SDA_I };
                end
            end
            S_FINISH: begin
                DISPLAY_CS_N <= 1;
                tickCounter <= CS_PAUSE_RELOAD;
                state <= S_PAUSE;
            end
            S_PAUSE: begin
                state <= S_IDLE;
            end
            default: state <= S_IDLE;
            endcase
        end
        else begin
            tickCounter <= tickCounter - 1;
        end
    end
end

endmodule
