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
#include "evg.h"
#include "gpio.h"
#include "mgt.h"
#include "util.h"

#define CSR_W_ENABLE_RESETS     0x80000000
#define CSR_RW_GT_TX_RESET      0x40000000
#define CSR_RW_GT_RX_RESET      0x20000000
#define CSR_RW_CPLL_RESET       0x10000000

#define CSR_W_DRP_WE            0x40000000
#define CSR_W_DRP_ADDR_SHIFT    16
#define CSR_R_DRP_BUSY          0x80000000
#define CSR_DRP_DATA_MASK       0xFFFF

#define CSR_R_RX_ALIGNED        0x08000000
#define CSR_R_TX_FSM_RESET_DONE 0x04000000
#define CSR_R_RX_FSM_RESET_DONE 0x02000000
#define CSR_R_TX_RESET_DONE     0x01000000
#define CSR_R_RX_RESET_DONE     0x00800000
#define CSR_R_CPLL_LOCKED       0x00400000

/*
 * Receiver alignment/loopback
 */

static void
writeResets(int mgtBitmap, uint32_t resets)
{
    resets |= CSR_W_ENABLE_RESETS;
    if (mgtBitmap & 0x1) {
        GPIO_WRITE(GPIO_IDX_EVG_1_0_DRP_CSR, CSR_W_ENABLE_RESETS | resets);
    }
    if (mgtBitmap & 0x2) {
        GPIO_WRITE(GPIO_IDX_EVG_2_0_DRP_CSR, CSR_W_ENABLE_RESETS | resets);
    }
    if (debugFlags & DEBUGFLAG_SHOW_TX_RESETS) {
        printf("TX resets:%x 1:%08X 2:%08X\n", mgtBitmap,
                                             GPIO_READ(GPIO_IDX_EVG_1_0_DRP_CSR),
                                             GPIO_READ(GPIO_IDX_EVG_2_0_DRP_CSR));
    }
}

void
mgtTxReset(int mgtBitmap)
{
    uint32_t then, seconds;
    static uint32_t whenWarned;

    writeResets(mgtBitmap, CSR_RW_GT_TX_RESET | CSR_RW_GT_RX_RESET | CSR_RW_CPLL_RESET);
    microsecondSpin(200);
    writeResets(mgtBitmap, CSR_RW_GT_TX_RESET | CSR_RW_GT_RX_RESET);
    microsecondSpin(200);
    seconds = GPIO_READ(GPIO_IDX_SECONDS_SINCE_BOOT);
    then = MICROSECONDS_SINCE_BOOT();
    while (((mgtBitmap & 0x1)
          && !(GPIO_READ(GPIO_IDX_EVG_1_0_DRP_CSR) & CSR_R_CPLL_LOCKED))
        || ((mgtBitmap & 0x2)
          && !(GPIO_READ(GPIO_IDX_EVG_2_0_DRP_CSR) & CSR_R_CPLL_LOCKED))) {
        if ((MICROSECONDS_SINCE_BOOT() - then) > 100000) {
            if ((seconds - whenWarned) > 5) {
                warn("MGT CPLL lock fail %x %X %X", mgtBitmap,
                                             GPIO_READ(GPIO_IDX_EVG_1_0_DRP_CSR),
                                             GPIO_READ(GPIO_IDX_EVG_2_0_DRP_CSR));
                whenWarned = seconds;
            }
            break;
        }
    }
    writeResets(mgtBitmap, 0);
    then = MICROSECONDS_SINCE_BOOT();
    while (((mgtBitmap & 0x1)
          && !(GPIO_READ(GPIO_IDX_EVG_1_0_DRP_CSR) & CSR_R_TX_RESET_DONE))
        || ((mgtBitmap & 0x2)
          && !(GPIO_READ(GPIO_IDX_EVG_2_0_DRP_CSR) & CSR_R_TX_RESET_DONE))) {
        if ((MICROSECONDS_SINCE_BOOT() - then) > 100000) {
            if ((seconds - whenWarned) > 5) {
                warn("MGT Tx reset fail %x %08X %08X", mgtBitmap,
                                             GPIO_READ(GPIO_IDX_EVG_1_0_DRP_CSR),
                                             GPIO_READ(GPIO_IDX_EVG_2_0_DRP_CSR));
                whenWarned = seconds;
            }
            break;
        }
    }
}

/*
 * Unsync temporarily
 */
void
mgtTxStutter(int evgNumber)
{
    switch(evgNumber) {
    default: return;
    case 1:  mgtTxReset(0x1);  break;
    case 2:  mgtTxReset(0x2);  break;
    }
}

void
mgtInit(void)
{
    mgtTxReset(0x3);
}

void
mgtCrank(void)
{
    static int nextCheck = 0;
    if (nextCheck == (EVG_COUNT - 1)) {
        nextCheck = 0;
    }
    else {
        nextCheck++;
    }
    if (debugFlags & DEBUGFLAG_TX_RESET) {
        debugFlags &= ~DEBUGFLAG_TX_RESET;
        mgtTxReset(0x3);
        evgAlign();
    }
}

int
mgtFetchLatency(unsigned int evgIdx)
{
    int shift;
    uint32_t mask, csr, latency;
    static struct {
        uint16_t csrIdx;
        uint16_t latencyIdx;
    } rxLoopbackInfo[EVG_COUNT] = {
        { GPIO_IDX_EVG_1_0_DRP_CSR, GPIO_IDX_EVG_1_0_LATENCY },
        { GPIO_IDX_EVG_2_0_DRP_CSR, GPIO_IDX_EVG_2_0_LATENCY },
    };
    if (evgIdx >= EVG_COUNT) {
        return -1;
    }
    shift = 8 * evgIdx;
    mask = 0xFF << shift;
    if ((sharedMemory->loopbackRequest & mask) !=
        (sharedMemory->loopbackActual & mask)) {
        return -1;
    }
    csr = GPIO_READ(rxLoopbackInfo[evgIdx].csrIdx);
    if (csr & CSR_R_RX_ALIGNED) {
        latency = GPIO_READ(rxLoopbackInfo[evgIdx].latencyIdx);
        return latency & 0xFFFF;
    }
    return 0;
}
