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
#include "eyescanTarget.h"

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
#define CSR_R_CPLL_LOSS_OF_LOCK 0x00200000

#define REG(base,chan)  ((base) + (GPIO_IDX_PER_MGTWRAPPER * (chan)))
#define MGT_RESET_WAITING_TIME  100000 // us

/*
 * Receiver alignment/loopback
 */

static void
writeResets(int mgtBitmap, uint32_t resets)
{
    int evg, lane;
    uint32_t csrIdx;
    uint32_t csrBaseIdx;

    resets |= CSR_W_ENABLE_RESETS;

    for (evg = 0 ; evg < EVG_COUNT; evg++) {
        csrBaseIdx = ((mgtBitmap & (0x1 << evg)) == 1)? GPIO_IDX_EVG_1_0_DRP_CSR :
                        (((mgtBitmap & (0x1 << evg)) == 2)? GPIO_IDX_EVG_2_0_DRP_CSR : 0);

        if (!csrBaseIdx) {
            continue;
        }

        for (lane = 0 ; lane < EYESCAN_LANECOUNT/2; lane++) {
            csrIdx = REG(csrBaseIdx, lane);
            GPIO_WRITE(csrIdx, CSR_W_ENABLE_RESETS | resets);

            if (debugFlags & DEBUGFLAG_SHOW_TX_RESETS) {
                printf("TX resets - EVG%d Lane:%d 1:%08X 2:%08X\n", evg+1, lane,
                        GPIO_READ(REG(GPIO_IDX_EVG_1_0_DRP_CSR, lane)),
                        GPIO_READ(REG(GPIO_IDX_EVG_2_0_DRP_CSR, lane)));
            }
        }
    }
}

int
mgtLossOfLock(int mgtBitmap)
{
    int evg, lane, lol;
    int mgtBit = 0, mgtLOLBitmap = 0;
    uint32_t csrBaseIdx = 0;
    uint32_t csrIdx = 0;

    for (evg = 0 ; evg < EVG_COUNT; evg++) {
        mgtBit = 0x1 << evg;
        csrBaseIdx = (((mgtBitmap & mgtBit) == 1)? GPIO_IDX_EVG_1_0_DRP_CSR :
                        ((mgtBitmap & mgtBit) == 2)? GPIO_IDX_EVG_2_0_DRP_CSR : 0);

        if (!csrBaseIdx) {
            continue;
        }

        for (lane = 0 ; lane < EYESCAN_LANECOUNT/2; lane++) {
            csrIdx = REG(csrBaseIdx, lane);
            lol = GPIO_READ(csrIdx) & CSR_R_CPLL_LOSS_OF_LOCK;
            if (lol) {
                mgtLOLBitmap |= mgtBit;
            }
        }
    }

    return mgtLOLBitmap;
}

void
mgtTxReset(int mgtBitmap)
{
    uint32_t then, seconds;
    int evg, lane, locked, txResetDone;
    uint32_t csrIdx;
    uint32_t csrBaseIdx;
    uint32_t reg1Value, reg2Value;
    static uint32_t whenWarned;

    writeResets(mgtBitmap, CSR_RW_GT_TX_RESET | CSR_RW_GT_RX_RESET | CSR_RW_CPLL_RESET);
    microsecondSpin(200);
    writeResets(mgtBitmap, CSR_RW_GT_TX_RESET | CSR_RW_GT_RX_RESET);
    microsecondSpin(200);
    seconds = GPIO_READ(GPIO_IDX_SECONDS_SINCE_BOOT);
    then = MICROSECONDS_SINCE_BOOT();

    for (evg = 0 ; evg < EVG_COUNT; evg++) {
        csrBaseIdx = ((mgtBitmap & (0x1 << evg)) == 1)? GPIO_IDX_EVG_1_0_DRP_CSR :
                        (((mgtBitmap & (0x1 << evg)) == 2)? GPIO_IDX_EVG_2_0_DRP_CSR : 0);

        if (!csrBaseIdx) {
            continue;
        }

        for (lane = 0 ; lane < EYESCAN_LANECOUNT/2; lane++) {
            csrIdx = REG(csrBaseIdx, lane);
            locked = GPIO_READ(csrIdx) & CSR_R_CPLL_LOCKED;
            reg1Value = GPIO_READ(REG(GPIO_IDX_EVG_1_0_DRP_CSR, lane));
            reg2Value = GPIO_READ(REG(GPIO_IDX_EVG_2_0_DRP_CSR, lane));

            while (!locked) {
                if ((MICROSECONDS_SINCE_BOOT() - then) > MGT_RESET_WAITING_TIME) {
                    if ((seconds - whenWarned) > 5) {
                        warn("MGT CPLL lock fail - EVG%d Lane:%d ResetCmd:0x%x [reg 0x%08X - 0x%08X]",
                             evg+1, lane, mgtBitmap, reg1Value, reg2Value);
                        whenWarned = seconds;
                    }
                    break;
                }
                locked = GPIO_READ(csrIdx) & CSR_R_CPLL_LOCKED;
                reg1Value = GPIO_READ(REG(GPIO_IDX_EVG_1_0_DRP_CSR, lane));
                reg2Value = GPIO_READ(REG(GPIO_IDX_EVG_2_0_DRP_CSR, lane));
            }
        }
    }

    writeResets(mgtBitmap, 0);
    then = MICROSECONDS_SINCE_BOOT();

    for (evg = 0 ; evg < EVG_COUNT; evg++) {
        csrBaseIdx = ((mgtBitmap & (0x1 << evg)) == 1)? GPIO_IDX_EVG_1_0_DRP_CSR :
                        (((mgtBitmap & (0x1 << evg)) == 2)? GPIO_IDX_EVG_2_0_DRP_CSR : 0);

        if (!csrBaseIdx) {
            continue;
        }

        for (lane = 0 ; lane < EYESCAN_LANECOUNT/2; lane++) {
            csrIdx = REG(csrBaseIdx, lane);
            txResetDone = GPIO_READ(csrIdx) & CSR_R_TX_RESET_DONE;
            reg1Value = GPIO_READ(REG(GPIO_IDX_EVG_1_0_DRP_CSR, lane));
            reg2Value = GPIO_READ(REG(GPIO_IDX_EVG_2_0_DRP_CSR, lane));

            while (!txResetDone) {
                if ((MICROSECONDS_SINCE_BOOT() - then) > MGT_RESET_WAITING_TIME) {
                    if ((seconds - whenWarned) > 5) {
                        warn("MGT Tx reset fail - EVG%d Lane:%d ResetCmd:0x%x [reg 0x%08X - 0x%08X]",
                             evg+1, lane, mgtBitmap, reg1Value, reg2Value);
                        whenWarned = seconds;
                    }
                    break;
                }
                txResetDone = GPIO_READ(csrIdx) & CSR_R_TX_RESET_DONE;
                reg1Value = GPIO_READ(REG(GPIO_IDX_EVG_1_0_DRP_CSR, lane));
                reg2Value = GPIO_READ(REG(GPIO_IDX_EVG_2_0_DRP_CSR, lane));
            }
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
    static uint32_t whenChecked;
    uint32_t seconds;
    int mgtLOLBitmap = 0;

    if (debugFlags & DEBUGFLAG_TX_RESET) {
        debugFlags &= ~DEBUGFLAG_TX_RESET;
        mgtTxReset(0x3);
        evgAlign();
    }

    if (!(debugFlags & DEBUGFLAG_NO_RESYNC_ON_LOL)) {
        /* Check all EVGs LOL every few seconds */
        seconds = GPIO_READ(GPIO_IDX_SECONDS_SINCE_BOOT);
        if ((seconds - whenChecked) > 4) {
            whenChecked = GPIO_READ(GPIO_IDX_SECONDS_SINCE_BOOT);
            if ((mgtLOLBitmap = mgtLossOfLock(0x3))) {
                warn("MGT CPLL LOL detected on 0x%08X at %u seconds. Resetting EVG(s)",
                        mgtLOLBitmap, whenChecked);
                mgtTxReset(mgtLOLBitmap);
                evgAlign();
            }
        }
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
