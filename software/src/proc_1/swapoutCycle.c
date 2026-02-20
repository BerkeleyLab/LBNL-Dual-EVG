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

#include <stdio.h>
#include "swapoutCycle.h"
#include "gpio.h"
#include "util.h"

void
swapoutCycleEnable(int offset)
{
    GPIO_WRITE(GPIO_IDX_SWAPOUT_CYCLE_CSR, CSR_SWAPOUT_W_OFFSET_W(offset));
}

void
swapoutAlignSetAlignSel(int sel)
{
    if (sel >= CFG_EVG2_HEARTBEAT_COUNT) {
        return;
    }

    GPIO_WRITE(GPIO_IDX_SWAPOUT_ALIGN_CSR, CSR_SWAPOUT_ALIGN_W_SET_ALIGN_SEL |
            CSR_SWAPOUT_ALIGN_W_SEL_W(sel));
}

void
swapoutAlignSetHeartbeatSel(int sel)
{
    if (sel >= CFG_EVG2_HEARTBEAT_COUNT) {
        return;
    }

    GPIO_WRITE(GPIO_IDX_SWAPOUT_ALIGN_CSR, CSR_SWAPOUT_ALIGN_W_SET_HEARTBEAT_SEL |
            CSR_SWAPOUT_ALIGN_W_SEL_W(sel));
}

int
swapoutAlignSetSel(unsigned int idx, int sel)
{
    switch (idx) {
        case 0:
            swapoutAlignSetAlignSel(sel);
            break;

        case 1:
            swapoutAlignSetHeartbeatSel(sel);
            break;

        default:
            return -1;
    }

    return 0;
}

int
swapoutAlignGetAlignSel(void)
{
    uint32_t reg = GPIO_READ(GPIO_IDX_SWAPOUT_ALIGN_CSR);

    return CSR_SWAPOUT_ALIGN_R_COUNTER_SEL_R(reg);
}

int
swapoutAlignGetHbSel(void)
{
    uint32_t reg = GPIO_READ(GPIO_IDX_SWAPOUT_ALIGN_CSR);

    return CSR_SWAPOUT_ALIGN_R_HB_SEL_R(reg);
}

int
swapoutAlignFetchStatus(uint32_t *ap)
{
    int idx = 0;
    ap[idx++] = GPIO_READ(GPIO_IDX_SWAPOUT_ALIGN_CSR);
    return idx;
}
