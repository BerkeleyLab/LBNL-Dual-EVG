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

/*
 * MGT clock crosspoint switch
 */

#include <stdio.h>
#include <stdint.h>
#include <xil_io.h>
#include <xparameters.h>
#include "iicProc.h"
#include "mgtClkSwitch.h"
#include "util.h"

#define MGTCLK_SWITCH_ADDRESS   0x48    /* 7 bit address */

#define REG_XPT_RESET   0x00
#define REG_XPT_CONFIG  0x40
#define REG_XPT_UPDATE  0x41

static void
setReg(int reg, int value)
{
    uint8_t cv = value;
    iicProcWrite(MGTCLK_SWITCH_ADDRESS, reg, &cv, 1);
}

static int
getReg(int reg)
{
    uint8_t cv;
    if (!iicProcRead(MGTCLK_SWITCH_ADDRESS, reg, &cv, 1)) {
        return -1;
    }
    return cv;
}

static void
clkConnect(int outputIndex, int inputIndex)
{
    setReg(REG_XPT_CONFIG, ((inputIndex & 0x7) << 4) |
                                          (outputIndex & 0x7));
}

static void
outputEnable(int outputIndex, int enable)
{
    setReg(0xC0 + (8 * outputIndex), enable ? 0x20 : 0x00);
}

static int
setMuxToClockBus(void)
{
    if (!iicProcSetMux(IIC_MUX_PORT_MGTCLK_SWITCH)) {
        warn("Can't set IIC MUX");
        return 0;
    }
    return 1;
}

void
mgtClkSwitchInit(void)
{
    int output, input;

    iicProcTakeControl();
    if (!setMuxToClockBus()) {
        iicProcRelinquishControl();
        return;
    }
    setReg(REG_XPT_RESET, 0x1);
    microsecondSpin(0x10);
    setReg(REG_XPT_RESET, 0x0);
    for (output = 0 ; output < 8 ; output++) {
        input = -1;
        switch (output) {
        case MGT_CLK_SWITCH_OUTPUT_MGTCLK2: /* Bank 115 REFCLK0 */
            input = MGT_CLK_SWITCH_INPUT_FMC1_GBTCLK0;
            break;

        case MGT_CLK_SWITCH_OUTPUT_MGTCLK3: /* Bank 115 REFCLK1 */
            input = MGT_CLK_SWITCH_INPUT_FMC2_GBTCLK0;
            break;
        }
        if (input >= 0) {
            clkConnect(output, input);
            outputEnable(output, 1);
        }
        else {
            outputEnable(output, 0);
        }
    }
    setReg(REG_XPT_UPDATE, 0x1);
    iicProcRelinquishControl();
}

void
mgtClkSwitchDump(void)
{
    int i;
    static const uint8_t reg[] = {
        0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
        0x58, 0x59, 0x5A, 0x5B,
        0x80, 0x88, 0x90, 0x98, 0xA0, 0xA8, 0xB0, 0xB8,
        0xC0, 0xC8, 0xD0, 0xD8, 0xE0, 0xE8, 0xF0, 0xF8,
        0x23,
        0x83, 0x8B, 0x93, 0x9B, 0xA3, 0xAB, 0xB3, 0xBB,
        0x84, 0x8C, 0x94, 0x9C, 0xA4, 0xAC, 0xB4, 0xBC,
        0x85, 0x8D, 0x95, 0x9D, 0xA5, 0xAD, 0xB5, 0xBD,
        0xC1, 0xC9, 0xD1, 0xD9, 0xE1, 0xE9, 0xF1, 0xF9,
        0xC2, 0xCA, 0xD2, 0xDA, 0xE2, 0xEA, 0xF2, 0xFA,
        0xC3, 0xCB, 0xD3, 0xDB, 0xE3, 0xEB, 0xF3, 0xFB };
    iicProcTakeControl();
    if (!setMuxToClockBus()) {
        iicProcRelinquishControl();
        return;
    }
    for (i = 0 ; i < sizeof reg ; i++) {
        printf("%02X %02X\n", reg[i], getReg(reg[i]));
    }
    iicProcRelinquishControl();
}
