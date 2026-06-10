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
#include "injectionCycle.h"
#include "gpio.h"
#include "util.h"

#define MINIMUM_INJECTION_PERIOD_MILLISECONDS  1000
#define MAXIMUM_INJECTION_PERIOD_MILLISECONDS  ((1UL << 16) - 1)

static struct injCycle {
    uint16_t     csrIdx;
    uint16_t     csrAlignIdx;
    uint16_t     csrTargetStatusIdx;
    uint16_t     csrTargetStatus2Idx;
    uint32_t     minInjPeriod;
    uint32_t     maxInjPeriod;
    uint32_t     baseInterval;
    uint32_t     maxExtension;
    unsigned int arTgtBucket;
} injCycle = {
    .csrIdx = GPIO_IDX_INJECTION_CYCLE_CSR,
    .csrAlignIdx = GPIO_IDX_INJECTION_ALIGN_CSR,
    .csrTargetStatusIdx = GPIO_IDX_INJECTION_TARGET_CSR,
    .csrTargetStatus2Idx = GPIO_IDX_INJECTION_TARGET2_CSR,
    .minInjPeriod = MINIMUM_INJECTION_PERIOD_MILLISECONDS,
    .maxInjPeriod = MAXIMUM_INJECTION_PERIOD_MILLISECONDS,
    .baseInterval = MINIMUM_INJECTION_PERIOD_MILLISECONDS,
    .maxExtension = MAXIMUM_INJECTION_PERIOD_MILLISECONDS -
        MINIMUM_INJECTION_PERIOD_MILLISECONDS,
    .arTgtBucket = 0,
};

static struct injCycle *injp = &injCycle;

void
injectionCycleEnable(int enable)
{
    GPIO_WRITE(injp->csrIdx, enable ?
                        CSR_INJ_W_ENABLE_TIMED_CYCLES : CSR_INJ_W_DISABLE_TIMED_CYCLES);
}

void
injectionCycleManualTrigger(void)
{
    GPIO_WRITE(injp->csrIdx, CSR_INJ_W_MANUAL_TRIGGER);
}

void
injectionCycleExtendInterval(int milliseconds)
{
    if (milliseconds < 0) {
        milliseconds = 0;
    }
    else if (milliseconds > injp->maxExtension) {
        milliseconds = injp->maxExtension;
    }
    milliseconds += injp->baseInterval;
    GPIO_WRITE(injp->csrIdx, CSR_INJ_W_SET_CYCLE_MILLISECONDS |
                                                            (milliseconds - 2));
}

void
injectionCycleSetBaseInterval(int milliseconds)
{
    if (milliseconds < injp->minInjPeriod) {
        milliseconds = injp->minInjPeriod;
    }
    else if (milliseconds > injp->maxInjPeriod) {
        milliseconds = injp->maxInjPeriod;
    }
    injp->baseInterval = milliseconds;
    injp->maxExtension = injp->maxInjPeriod - injp->baseInterval;
    injectionCycleExtendInterval(0);
}

int
injectionCycleFetchStatus(uint32_t *ap)
{
    int idx = 0;
    ap[idx++] = GPIO_READ(injp->csrIdx);
    return idx;
}

void
injectionAlignSetAlignSel(int sel)
{
    if (sel >= CFG_EVG1_HEARTBEAT_COUNT) {
        return;
    }

    GPIO_WRITE(injp->csrAlignIdx, CSR_INJ_ALIGN_W_SET_ALIGN_SEL |
            CSR_INJ_ALIGN_W_SEL_W(sel));
}

void
injectionAlignSetHeartbeatSel(int sel)
{
    if (sel >= CFG_EVG1_HEARTBEAT_COUNT) {
        return;
    }

    GPIO_WRITE(injp->csrAlignIdx, CSR_INJ_ALIGN_W_SET_HEARTBEAT_SEL |
            CSR_INJ_ALIGN_W_SEL_W(sel));
}

int
injectionAlignSetSel(unsigned int idx, int sel)
{
    switch (idx) {
        case 0:
            injectionAlignSetAlignSel(sel);
            break;

        case 1:
            injectionAlignSetHeartbeatSel(sel);
            break;

        default:
            return -1;
    }

    return 0;
}

int
injectionAlignGetAlignSel(void)
{
    uint32_t reg = GPIO_READ(injp->csrAlignIdx);

    return CSR_INJ_ALIGN_R_COUNTER_SEL_R(reg);
}

int
injectionAlignGetHbSel(void)
{
    uint32_t reg = GPIO_READ(injp->csrAlignIdx);

    return CSR_INJ_ALIGN_R_HB_SEL_R(reg);
}

int
injectionAlignFetchStatus(uint32_t *ap)
{
    int idx = 0;
    ap[idx++] = GPIO_READ(injp->csrAlignIdx);
    return idx;
}

/*
 * Implementing the first part of:
 * bBR =[(43*[(5*bAR) (mod 304)](mod125) + 72*iAR,BR(mod 125)] (mod 125) :
 * bBR =[(43*[(5*bAR) (mod 304)](mod125)
 */

int
injectionTargetSetRfCoincTerm(unsigned int arBucket)
{
    unsigned int rfCoincIdx = 0;
    unsigned int rfCoincTerm = 0;

    if (arBucket >= CFG_EVG1_BR_AR_COINC_PER_RF_COINC) {
        return -1;
    }

    injp->arTgtBucket = arBucket;

    rfCoincIdx = (5 * injp->arTgtBucket) % CFG_EVG1_BR_AR_COINC_PER_RF_COINC;
    rfCoincTerm = (43 * rfCoincIdx) % CFG_EVG1_BR_AR_ALIGN_PER_BR_AR_COINC;

    GPIO_WRITE(injp->csrTargetStatusIdx,
            CSR_TGT_RF_COINC_IDX_W(rfCoincIdx) | CSR_TGT_RF_COINC_TERM_W(rfCoincTerm));

    if (debugFlags & DEBUGFLAG_INJ_CYCLE) {
        injectionTargetDisplay();
    }

    return 0;
}

uint32_t
injectionTargetStatus(void)
{
    return GPIO_READ(injp->csrTargetStatusIdx);
}

uint32_t
injectionTargetStatus2(void)
{
    return GPIO_READ(injp->csrTargetStatus2Idx);
}

void
injectionTargetDisplay(void)
{
    uint32_t reg = injectionTargetStatus();
    int rfCoincIdx = CSR_TGT_RF_COINC_IDX_R(reg);
    int rfCoincTerm = CSR_TGT_RF_COINC_TERM_R(reg);

    printf("Injection Target: rfCoincIdx: %d rfCoincTerm: %d\n",
            rfCoincIdx, rfCoincTerm);
}

void
injectionTarget2Display(void)
{
    uint32_t reg = injectionTargetStatus2();
    int brBucket = CSR_TGT2_BR_BUCKET_R(reg);
    int alignCount = CSR_TGT2_ALIGN_COUNT_R(reg);

    printf("Injection Target: brBucket: %d alignCount: %d\n",
            brBucket, alignCount);
}
