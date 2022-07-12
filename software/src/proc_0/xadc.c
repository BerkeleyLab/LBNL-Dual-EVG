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
 * Read XADC system monitor
 */
#include <stdio.h>
#include <xil_io.h>
#include <xparameters.h>
#include "xadc.h"
#include "util.h"

#define In32(offset)    Xil_In32(XPAR_XADC_WIZ_0_BASEADDR+(offset))

#define R_TEMP          0x200 /* On-chip Temperature */
#define R_VCCINT        0x204 /* FPGA VCCINT */
#define R_VCCAUX        0x208 /* FPGA VCCAUX */
#define R_VBRAM         0x218 /* FPGA VBRAM */
#define R_CFR0          0x300 /* Configuration Register 0 */
#define R_CFR1          0x304 /* Configuration Register 1 */
#define R_CFR2          0x308 /* Configuration Register 2 */
#define R_SEQ00         0x320 /* Seq Reg 00 -- Channel Selection */
#define R_SEQ01         0x324 /* Seq Reg 01 -- Channel Selection */
#define R_SEQ02         0x328 /* Seq Reg 02 -- Average Enable */
#define R_SEQ03         0x32C /* Seq Reg 03 -- Average Enable */
#define R_SEQ04         0x330 /* Seq Reg 04 -- Input Mode Select */
#define R_SEQ05         0x334 /* Seq Reg 05 -- Input Mode Select */
#define R_SEQ06         0x338 /* Seq Reg 06 -- Acquisition Time Select */
#define R_SEQ07         0x33C /* Seq Reg 07 -- Acquisition Time Select */

/*
 * Update the ADC readings
 */
uint32_t *
xadcUpdate(uint32_t *buf)
{
    *buf++ = (In32(R_VCCINT) << 16) | In32(R_TEMP);
    *buf++ = (In32(R_VBRAM) << 16) | In32(R_VCCAUX);
    return buf;
}

/*
 * Return FPGA temperature in units of 0.1 degree C
 * Avoid floating point arithmetic
 */
int
xadcGetFPGAtemp(void)
{
    return ((In32(R_TEMP) * 5040) >> 16) - 2732;
}
