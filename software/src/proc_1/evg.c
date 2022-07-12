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
#include <stdlib.h>
#include "evg.h"
#include "evgProtocol.h"
#include "gpio.h"
#include "injectionCycle.h"
#include "mgt.h"
#include "systemParameters.h"
#include "util.h"

#define SEQ_CSR_CMD_SET_ADDRESS         (1UL << 30)
#define SEQ_CSR_CMD_LATCH_GAP           (2UL << 30)
#define SEQ_CSR_CMD_WRITE_ENTRY         (3UL << 30)
#define SEQ_CSR_ADDRESS_WIDTH_MASK      0x1F000000
#define SEQ_CSR_ADDRESS_WIDTH_SHIFT     24
#define SEQ_CSR_IGNORED_CYCLES_MASK     0xFF0000
#define SEQ_CSR_W_RBK_MUX_SEL           (1UL << 24)
#define SEQ_CSR_W_SET_PRECOMP_EVENT     (1UL << 25)
#define SEQ_CSR_IGNORED_CYCLES_SHIFT    16
#define SEQ_CSR_ACCEPTED_CYCLES_MASK    0xFF00
#define SEQ_CSR_ACCEPTED_CYCLES_SHIFT   8
#define SEQ_CSR_DISABLE_SEQ(n)          (0x4<<(n))
#define SEQ_CSR_ENABLE_SEQ(n)           (0x1<<(n))

#define MONITOR_CHANNELS_PER_EVG    2

static struct evgInfo {
    uint16_t    csrIdx;
    uint16_t    rbkIdx;
    uint16_t    hwIdx;
    uint16_t    swIdx;
    uint16_t    capacity;
    uint16_t    pkNumber;
    uint16_t    writeCount;
    uint16_t    length;
    int16_t     precompletionEvent;
    uint8_t     evgIndex;
    uint8_t     evgNumber;
    uint8_t     isWriting;
    uint8_t     isValid;
} evgs[EVG_COUNT] = {
    { .csrIdx   = GPIO_IDX_EVG_1_SEQ_CSR,
      .rbkIdx   = GPIO_IDX_EVG_1_SEQ_RBK,
      .hwIdx    = GPIO_IDX_EVG_1_HW_CSR,
      .swIdx    = GPIO_IDX_EVG_1_SW_CSR,
      .length   = CFG_EVG1_CLK_PER_RF_COINCIDENCE,
      .evgIndex  = 0,
      .evgNumber = 1 },
    { .csrIdx   = GPIO_IDX_EVG_2_SEQ_CSR,
      .rbkIdx   = GPIO_IDX_EVG_2_SEQ_RBK,
      .hwIdx    = GPIO_IDX_EVG_2_HW_CSR,
      .swIdx    = GPIO_IDX_EVG_2_SW_CSR,
      .length   = CFG_EVG2_CLK_PER_RF_COINCIDENCE,
      .evgIndex  = 1,
      .evgNumber = 2 },
};

static int
convertSequence(const char *csv, uint32_t *dest, int16_t *precompletionEvent)
{
    char *endp;
    int period, gap, event = 0;
    int idx = 0;
    int line = 2;

    period = strtol(csv, &endp, 10);
    if (((*endp != ',') && (*endp != '\n'))
     || (period < CFG_MINIMUM_INJECTION_CYCLE_MS)
     || (period > CFG_MAXIMUM_INJECTION_CYCLE_MS)) {
        printf("Bad period (%d ms)\n", period);
        return 0;
    }
    csv = endp;
    while (*csv != '\n') {
        if (!*csv) {
            printf("No gap,event lines\n");
            return 0;
        }
        csv++;
    }
    while (*csv) {
        gap = strtol(csv, &endp, 10);
        if ((*endp != ',')
         || (gap < 0)
         || (gap > (1<<28))) {
            printf("Line %d: Bad gap\n", line);
            return 0;
        }
        csv = endp + 1;
        event = strtol(csv, &endp, 10);
        if (((*endp != ',') && (*endp != '\n'))
         || (event <= 0)
         || (event > 255)) {
            printf("Line %d: Bad event\n", line);
            return 0;
        }
        if (*endp == ',') {
            endp++;
            while (*endp == ' ') endp++;
            if (*endp == '*') {
                *precompletionEvent = event;
                endp++;
            }
            while (*endp == ' ') endp++;
            if (*endp != '\n') {
                printf("Line %d: Bad precompletion marker\n", line);
                return 0;
            }
        }
        csv = endp + 1;
        if (gap < EVG_PROTOCOL_WAVEFORM_SINGLE_WORD_DELAY_LIMIT) {
            dest[idx++] = (gap << 8) |  event;
        }
        else {
            dest[idx++] =
                   (EVG_PROTOCOL_WAVEFORM_SINGLE_WORD_DELAY_LIMIT << 8) | event;
            dest[idx++] = gap;
        }
        line++;
    }
    if (event != EVG_PROTOCOL_WAVEFORM_END_OF_TABLE_EVENT_CODE) {
        printf("No end-of-sequence event\n");
        return 0;
    }
    printf("%10d\n", period);
    injectionCycleSetBaseInterval(period);
    return idx;
}

static void
fillDefaultSequence(struct evgInfo *evgp)
{
    int n;
    /* Convert in place -- binary form is never longer than ASCII */
    if ((n = convertSequence((const char *)sharedMemory->udpTFTP.txBuf,
                               (uint32_t *)sharedMemory->udpTFTP.txBuf,
                               &evgp->precompletionEvent)) <= 0) {
        warn("Default sequence CSV invalid!");
        return;
    }
    debugFlags |= DEBUGFLAG_STASH_SEQUENCE;
    evgStashSequence(0, 0, n, (uint32_t *)sharedMemory->udpTFTP.txBuf);
    debugFlags &= ~DEBUGFLAG_STASH_SEQUENCE;
    if (evgp->precompletionEvent > 0) {
        GPIO_WRITE(evgs[0].csrIdx, SEQ_CSR_CMD_SET_ADDRESS |
                                   SEQ_CSR_W_SET_PRECOMP_EVENT |
                                   evgp->precompletionEvent);
    }
    evgEnableSequence(0, 1);
}

/*
 * Ask other processor to measure phase of reference and PLL output (Tx) clocks.
 * Slow, but not used during normal operation.
 */
static void
findPhase(void)
{
    sharedMemory->requestCoincidenceMeasurement = 1;
    while (sharedMemory->requestCoincidenceMeasurement) {
        microsecondSpin(10);
    }
}

/*
 * Align a single MGT
 */
static int
align(struct evgInfo *evgp)
{
    int pass = 0;
    int phaseOffset, phaseError;
    for (;;) {
        pass++;
        findPhase();
        phaseOffset = sharedMemory->pllPhaseOffset[evgp->evgIndex];
        phaseError = sharedMemory->pllPhaseError[evgp->evgIndex];
        if (abs(phaseError) < (evgp->length/40)) {
            printf("EVG %d PLL Tx:Ref offset %d (want %d), pass %d.\n",
                   evgp->evgNumber, phaseOffset,
                   sharedMemory->systemParameters.pllPhaseShift[evgp->evgIndex],
                   pass);
            return 1;
        }
        if (debugFlags & DEBUGFLAG_COINCIDENCE) {
            printf("EVG %d PLL Tx:Ref offset %d error %d\n",
                                      evgp->evgNumber, phaseOffset, phaseError);
        }
        if ((pass % 100) == 0) {
            printf("Aligning EVG %d Tx:Ref (pass %d).\n", evgp->evgNumber,pass);
        }
        if (pass >= 1000) {
            warn("Can't align EVG %d -- Tx:Ref %d, want %d", evgp->evgNumber,
                  phaseOffset,
                  sharedMemory->systemParameters.pllPhaseShift[evgp->evgIndex]);
            return 0;
        }
        mgtTxStutter(evgp->evgNumber);
    }
}

/*
 * Align EVG Tx clocks with reference clocks.
 */
int
evgAlign(void)
{
    struct evgInfo *evgp;
    int ret = 1;
    for (evgp = evgs ; evgp < &evgs[EVG_COUNT] ; evgp++) {
        if (!align(evgp)) {
            ret = 0;
        }
    }
    if (!sharedMemory->requestAlignment) {
        sharedMemory->requestAlignment = 1;
    }
    return ret;
}

void
evgInit(void)
{
    int i;
    struct evgInfo *evgp;
    for (i = 0, evgp = evgs ; i < EVG_COUNT ; i++, evgp++) {
        uint32_t csr = GPIO_READ(evgp->csrIdx);
        int addressWidth = (csr & SEQ_CSR_ADDRESS_WIDTH_MASK) >>
                                                    SEQ_CSR_ADDRESS_WIDTH_SHIFT;
        evgp->capacity = (1 << addressWidth);
        if (evgp->capacity != CFG_SEQUENCE_RAM_CAPACITY) {
            warn("CFG_SEQUENCE_RAM_CAPACITY:%d  Firmware:%d",
                                     CFG_SEQUENCE_RAM_CAPACITY, evgp->capacity);
        }
        evgp->precompletionEvent = -1;
    }
    evgAlign();
    fillDefaultSequence(&evgs[0]);
}

void
evgShowAlignment(void)
{
    struct evgInfo *evgp;
    for (evgp = evgs ; evgp < &evgs[EVG_COUNT] ; evgp++) {
        printf("EVG %d Tx:Ref offset: %d\n", evgp->evgNumber,
                                  sharedMemory->pllPhaseOffset[evgp->evgIndex]);
    }
}

static struct evgInfo *
evgPtr(unsigned int idx)
{
    if (idx >= EVG_COUNT) return NULL;
    return &evgs[idx];
}

static int
addEntry(struct evgInfo *evgp, uint32_t delay, int evCode)
{
    if (debugFlags & DEBUGFLAG_STASH_SEQUENCE) {
        printf("%10d, %d%s\n", delay, evCode,
                             (evCode == evgp->precompletionEvent) ? ", *" : "");
    }
    if (evgp->writeCount >= evgp->capacity) return 0;
    GPIO_WRITE(evgp->csrIdx, SEQ_CSR_CMD_LATCH_GAP | delay);
    GPIO_WRITE(evgp->csrIdx, SEQ_CSR_CMD_WRITE_ENTRY | evCode);
    evgp->writeCount++;
    return 1;
}

int
evgStashSequence(unsigned int idx, int pkNumber, int count,const uint32_t *seqp)
{
    struct evgInfo *evgp = evgPtr((idx >> 4) & 0xF);
    int i;

    if ((evgp == NULL) || (count == 0)) {
        return 0;
    }
    if (pkNumber == 0) {
        int sequenceSelect = (idx & 0xF) * evgp->capacity;
        evgEnableSequence(idx, 0);
        GPIO_WRITE(evgp->csrIdx, SEQ_CSR_CMD_SET_ADDRESS | sequenceSelect);
        evgp->pkNumber = 0;
        evgp->writeCount = 0;
        evgp->isWriting = 1;
        evgp->isValid = 0;
    }
    else if (!evgp->isWriting || (pkNumber != evgp->pkNumber)) {
        return 0;
    }
    for (i = 0 ; i < count ; i++, seqp++) {
        uint32_t delay = *seqp >> 8;
        int evCode = *seqp & 0xFF;
        if (delay == EVG_PROTOCOL_WAVEFORM_SINGLE_WORD_DELAY_LIMIT) {
            i++;
            if (i == count) {
                return 0;
            }
            delay = *++seqp;
        }
        /*
         * Maximum firmware value is 2^28-1
         */
        while (delay > ((1<<28)-1)) {
            if (!addEntry(evgp, ((1<<28)-1), 0)) return 0;
            delay -= ((1<<28)-1);
        }
        if (!addEntry(evgp, delay, evCode)) return 0;
        if (evCode == EVG_PROTOCOL_WAVEFORM_END_OF_TABLE_EVENT_CODE) {
            evgp->isWriting = 0;
            evgp->isValid = 1;
            return 1;
        }
    }
    evgp->pkNumber++;
    return 1;
}

int
evgEnableSequence(unsigned int idx, int enable)
{
    struct evgInfo *evgp = evgPtr((idx >> 4) & 0xF);
    int sequence;
    uint32_t cmd;
    if (evgp == NULL) return 0;
    sequence = (idx & 0xF);
    if (enable) {
        if (evgp->isWriting || !evgp->isValid) return 0;
        cmd = SEQ_CSR_ENABLE_SEQ(sequence);
    }
    else {
        cmd = SEQ_CSR_DISABLE_SEQ(sequence);
    }
    GPIO_WRITE(evgp->csrIdx, cmd);
    return 1;
}

/*
 * Account for EVG->SYS clock domain crossing -- read until stable
 */
uint32_t
evgSequencerStatus(unsigned int idx)
{
    struct evgInfo *evgp = evgPtr(idx);
    int csrIdx;
    uint32_t r0, r1;

    if (evgp == NULL) return 0;
    csrIdx = evgp->csrIdx;
    r0 = GPIO_READ(csrIdx);
    for (;;) {
        r1 = GPIO_READ(csrIdx);
        if (r1 == r0) return r1;
        r0 = r1;
    }
}
        
void
evgSoftwareTrigger(unsigned int idx, int eventCode)
{
    struct evgInfo *evgp = evgPtr(idx);
    int pass = 0;
    if (evgp == NULL) return;
    while (GPIO_READ(evgp->swIdx) & 0x100) {
        if (++pass == 10) {
            printf("E%d SW BUSY!\n", idx);
            return;
        }
    }
    GPIO_WRITE(evgp->swIdx, eventCode & 0xFF);
}

void
evgHardwareTrigger(unsigned int idx, int eventCode)
{
    struct evgInfo *evgp = evgPtr(idx >> 4);
    int chan = idx & 0xF;
    if ((evgp == NULL) || (chan >= CFG_HARDWARE_TRIGGER_COUNT))return;
    GPIO_WRITE(evgp->hwIdx, (1 << 31) | (chan << 8) | (eventCode & 0xFF));
}

int
evgFetchCoincidenceStatus(uint32_t *ap)
{
    int i;
    int idx = 0;
    int coincidenceIndex = 0;
    struct evgInfo *evgp;

    for (i = 0, evgp = evgs ; i < EVG_COUNT ; i++, evgp++) {
        for (int c = 0 ; c < EVG_COINCIDENCE_COUNT ; c++) {
            ap[idx++] = sharedMemory->coincidence[coincidenceIndex++];
        }
        ap[idx++] = sharedMemory->referenceFrequency[i];
    }
    ap[idx++] = sharedMemory->isAligned;
    sharedMemory->requestCoincidenceMeasurement = 1;
    return idx;
}

void
evgDumpSequence(int evg, int seq)
{
    struct evgInfo *evgp = evgPtr(evg - 1);
    int i;
    int event;
    int gap = 0;

    if ((evgp == NULL)
     || (seq < 0)
     || (seq > 1)) {
        return;
    }
    printf("EVG %d  SEQ %d:\n", evg, seq);
    for (i = 0 ; i < evgp->capacity ; i++) {
        uint32_t cmd = SEQ_CSR_CMD_SET_ADDRESS | (seq * evgp->capacity) | i;
        GPIO_WRITE(evgp->csrIdx, cmd);
        gap += GPIO_READ(evgp->rbkIdx);
        GPIO_WRITE(evgp->csrIdx, cmd | SEQ_CSR_W_RBK_MUX_SEL);
        event = GPIO_READ(evgp->rbkIdx);
        if (event != 0) {
            printf("%8d,%3d\n", gap, event);
            gap = 0;
        }
        if (event == EVG_PROTOCOL_WAVEFORM_END_OF_TABLE_EVENT_CODE) {
            break;
        }
    }
}
