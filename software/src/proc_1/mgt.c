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

int
mgtReset(int mgtBitmap)
{
    int good = 1;
    uint32_t then, seconds;
    int evg, lane, resetDone;
    uint32_t csrBaseIdx;
    uint32_t csr;
    uint32_t reg1Value, reg2Value;
    static uint32_t whenWarned;

    // We don't need to pull CPLL_RESET because it is not doing anything in
    // the FPGA anyway. TX_RESET is enough and will internally reset the CPLL
    writeResets(mgtBitmap, CSR_RW_GT_TX_RESET | CSR_RW_GT_RX_RESET);
    microsecondSpin(200);
    writeResets(mgtBitmap, 0);

    seconds = GPIO_READ(GPIO_IDX_SECONDS_SINCE_BOOT);
    then = MICROSECONDS_SINCE_BOOT();

    for (evg = 0 ; evg < EVG_COUNT; evg++) {
        csrBaseIdx = ((mgtBitmap & (0x1 << evg)) == 1)? GPIO_IDX_EVG_1_0_DRP_CSR :
                        (((mgtBitmap & (0x1 << evg)) == 2)? GPIO_IDX_EVG_2_0_DRP_CSR : 0);

        if (!csrBaseIdx) {
            continue;
        }

        for (lane = 0 ; lane < EYESCAN_LANECOUNT/2; lane++) {
            csr = GPIO_READ(REG(csrBaseIdx, lane));
            resetDone = csr & CSR_R_RESET_DONE_MASK;
            reg1Value = GPIO_READ(REG(GPIO_IDX_EVG_1_0_DRP_CSR, lane));
            reg2Value = GPIO_READ(REG(GPIO_IDX_EVG_2_0_DRP_CSR, lane));

            while (resetDone != CSR_R_RESET_DONE) {
                if ((MICROSECONDS_SINCE_BOOT() - then) > MGT_RESET_WAITING_TIME) {
                    if ((seconds - whenWarned) > 5) {
                        warn("MGT Tx/Rx reset fail - EVG%d Lane:%d ResetCmd:0x%x [reg 0x%08X - 0x%08X]",
                             evg+1, lane, mgtBitmap, reg1Value, reg2Value);
                        whenWarned = seconds;
                    }

                    good = 0;
                    break;
                }
                csr = GPIO_READ(REG(csrBaseIdx, lane));
                resetDone = csr & CSR_R_RESET_DONE_MASK;
                reg1Value = GPIO_READ(REG(GPIO_IDX_EVG_1_0_DRP_CSR, lane));
                reg2Value = GPIO_READ(REG(GPIO_IDX_EVG_2_0_DRP_CSR, lane));
            }
        }
    }

    return good;
}

int mgtLOLState(int mgtBitmap, int lane)
{
    int evg;
    uint32_t csrBaseIdx;
    uint32_t csr;

    if (lane > EYESCAN_LANECOUNT/2-1) {
        return -1;
    }

    for (evg = 0 ; evg < EVG_COUNT; evg++) {
        csrBaseIdx = ((mgtBitmap & (0x1 << evg)) == 1)? GPIO_IDX_EVG_1_0_DRP_CSR :
                        (((mgtBitmap & (0x1 << evg)) == 2)? GPIO_IDX_EVG_2_0_DRP_CSR : 0);

        if (!csrBaseIdx) {
            continue;
        }

        csr = GPIO_READ(REG(csrBaseIdx, lane));
        return CSR_R_LOL_STATE_R(csr);
    }

    return -1;
}

/*
 * Unsync temporarily
 */
void
mgtTxStutter(int evgNumber)
{
    switch(evgNumber) {
    default: return;
    case 1:  mgtReset(0x1);  break;
    case 2:  mgtReset(0x2);  break;
    }
}

void
mgtInit(void)
{
    mgtReset(0x3);
}

void
mgtCrank(void)
{
    static uint32_t whenChecked;
    static int reportedLOL;
    static unsigned int successfulRstNeeded;
    static int mgtLOLBitmap;
    uint32_t seconds;

    if (debugFlags & DEBUGFLAG_TX_RESET) {
        debugFlags &= ~DEBUGFLAG_TX_RESET;

        if(mgtReset(0x3)) {
            printf("MGT forced reset (DEBUGFLAG_TX_RESET) succeeded on 0x%08X\n",
                    0x3);
            reportedLOL = 0;
            evgAlign();
        }
    }

    /* always Read LOL state so other processor is aware */
    sharedMemory->lolState = mgtLOLState(sharedMemory->lolStateMGTBitmap,
            sharedMemory->lolStateMGTLane);

    if (!(debugFlags & DEBUGFLAG_NO_RESYNC_ON_LOL)) {
        /* Check all EVGs LOL every few seconds */
        seconds = GPIO_READ(GPIO_IDX_SECONDS_SINCE_BOOT);
        if ((seconds - whenChecked) > 2) {
            whenChecked = GPIO_READ(GPIO_IDX_SECONDS_SINCE_BOOT);

            if (!successfulRstNeeded) {
                if ((mgtLOLBitmap = mgtLossOfLock(0x3))) {
                    // On loss of lock, we need 1 reset to re-establish the
                    // CPLL lock reliability and another for relible clock
                    // itself
                    successfulRstNeeded = 2;

                    if (!reportedLOL) {
                        reportedLOL = 1;
                        warn("MGT CPLL LOL detected on 0x%08X at %u seconds",
                                mgtLOLBitmap, whenChecked);
                    }
                }
            }

            if (successfulRstNeeded) {
                if(mgtReset(mgtLOLBitmap)) {
                    successfulRstNeeded--;
                }
                else {
                    successfulRstNeeded = 2;
                }

                if (!successfulRstNeeded) {
                    printf("MGT reset succeeded on 0x%08X at %u seconds\n",
                            mgtLOLBitmap, whenChecked);
                    reportedLOL = 0;
                    evgAlign();
                }
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

int
mgtFetchStatus(uint32_t *ap)
{
    int evg, lane;
    int idx = 0;
    uint32_t csrIdx;
    uint32_t csrBaseIdx;
    uint32_t csr = 0;
    uint32_t evgStatus = 0;
    uint32_t laneStatus = 0;

    for (evg = 0 ; evg < EVG_COUNT; evg++) {
        evgStatus = 0;
        csrBaseIdx = ((0x1 << evg) == 1)? GPIO_IDX_EVG_1_0_DRP_CSR :
                        (((0x1 << evg) == 2)? GPIO_IDX_EVG_2_0_DRP_CSR : 0);

        if (!csrBaseIdx) {
            continue;
        }

        for (lane = 0 ; lane < EYESCAN_LANECOUNT/2; lane++) {
            laneStatus = 0;
            csrIdx = REG(csrBaseIdx, lane);
            csr = GPIO_READ(csrIdx);

            if (csr & CSR_R_RX_ALIGNED) {
                laneStatus |= 0x1;
            }
            if (csr & CSR_R_TX_FSM_RESET_DONE) {
                laneStatus |= 0x2;
            }
            if (csr & CSR_R_RX_FSM_RESET_DONE) {
                laneStatus |= 0x4;
            }
            if (csr & CSR_R_CPLL_LOCKED) {
                laneStatus |= 0x8;
            }
            if (csr & CSR_R_CPLL_LOSS_OF_LOCK) {
                laneStatus |= 0x10;
            }

            evgStatus |= (laneStatus << (8*lane));
        }

        ap[idx++] = evgStatus;
    }

    return idx;
}
