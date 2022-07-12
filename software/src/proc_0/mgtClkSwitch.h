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

#ifndef _MGTCLKSWITCH_H_
#define _MGTCLKSWITCH_H_

#define MGT_CLK_SWITCH_OUTPUT_MGTCLK0 0
#define MGT_CLK_SWITCH_OUTPUT_MGTCLK1 1
#define MGT_CLK_SWITCH_OUTPUT_MGTCLK2 4
#define MGT_CLK_SWITCH_OUTPUT_MGTCLK3 5

#define MGT_CLK_SWITCH_INPUT_EXT_CLK0       0
#define MGT_CLK_SWITCH_INPUT_EXT_CLK1       1
#define MGT_CLK_SWITCH_INPUT_FPGA_REFCLK0   2
#define MGT_CLK_SWITCH_INPUT_SI570          3
#define MGT_CLK_SWITCH_INPUT_FMC1_GBTCLK0   4
#define MGT_CLK_SWITCH_INPUT_FMC1_GBTCLK1   5
#define MGT_CLK_SWITCH_INPUT_FMC2_GBTCLK0   6
#define MGT_CLK_SWITCH_INPUT_FMC2_GBTCLK1   7

void mgtClkSwitchInit(void);
void mgtClkSwitchDump(void);

#endif /* _MGTCLKSWITCH_H_ */
