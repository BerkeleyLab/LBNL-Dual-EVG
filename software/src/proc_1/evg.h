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
 * Event generator support
 */

#ifndef _EVG_H_
#define _EVG_H_

#include <stdint.h>
#include "reg_macros.h"

#define EVG_COUNT               2
#define EVG_COINCIDENCE_COUNT   2

#define EVG_STATUS_SEQUENCE_0_ENABLED  (1UL << 0)
#define EVG_STATUS_SEQUENCE_1_ENABLED  (1UL << 1)
#define EVG_STATUS_MAP_1_ACTIVE        (1UL << 2)
#define EVG_STATUS_SEQUENCER_ACTIVE    (1UL << 3)

#define EVG_STATUS_START_COUNT_SHIFT   8
#define EVG_STATUS_START_COUNT_MASK    (0xFF<<EVG_STATUS_START_COUNT_SHIFT)

#define EVG_STATUS_MISSED_COUNT_SHIFT  16
#define EVG_STATUS_MISSED_COUNT_MASK   (0xFF<<EVG_STATUS_MISSED_COUNT_SHIFT)

/* CSR definitions */

#define SEQ_CSR_CMD_SET_ADDRESS         (1UL << 30)
#define SEQ_CSR_CMD_LATCH_GAP           (2UL << 30)
#define SEQ_CSR_CMD_WRITE_ENTRY         (3UL << 30)

#define SEQ_CSR_DELAY_SIZE              28
#define SEQ_CSR_DELAY_SHIFT             0
#define SEQ_CSR_DELAY_MASK              REG_GEN_MASK(SEQ_CSR_DELAY_SHIFT, \
                                            SEQ_CSR_DELAY_SIZE)
#define SEQ_CSR_DELAY_R(reg)            REG_GEN_READ(reg, SEQ_CSR_DELAY_SHIFT, \
                                            SEQ_CSR_DELAY_SIZE)
#define SEQ_CSR_DELAY_W(value)          REG_GEN_WRITE(value, SEQ_CSR_DELAY_SHIFT, \
                                            SEQ_CSR_DELAY_SIZE)

#define SEQ_CSR_EVCODE_SIZE             8
#define SEQ_CSR_EVCODE_SHIFT            0
#define SEQ_CSR_EVCODE_MASK             REG_GEN_MASK(SEQ_CSR_EVCODE_SHIFT, \
                                            SEQ_CSR_EVCODE_SIZE)
#define SEQ_CSR_EVCODE_W(value)         REG_GEN_WRITE(value, SEQ_CSR_EVCODE_SHIFT, \
                                            SEQ_CSR_EVCODE_SIZE)

#define SEQ_CSR_CAT_SIZE                8
#define SEQ_CSR_CAT_SHIFT               8
#define SEQ_CSR_CAT_MASK                REG_GEN_MASK(SEQ_CSR_CAT_SHIFT, \
                                               SEQ_CSR_CAT_SIZE)
#define SEQ_CSR_CAT_W(value)            REG_GEN_WRITE(value, SEQ_CSR_CAT_SHIFT, \
                                               SEQ_CSR_CAT_SIZE)

#define SEQ_CSR_ADDRESS_WIDTH_MASK      0x1F000000
#define SEQ_CSR_ADDRESS_WIDTH_SHIFT     24
#define SEQ_CSR_IGNORED_CYCLES_MASK     0xFF0000
#define SEQ_CSR_IGNORED_CYCLES_SHIFT    16

#define SEQ_CSR_W_RBK_MUX_SEL           (1UL << 24)
#define SEQ_CSR_W_SET_PRECOMP_EVENT     (1UL << 25)

#define SEQ_CSR_ACCEPTED_CYCLES_MASK    0xFF00
#define SEQ_CSR_ACCEPTED_CYCLES_SHIFT   8

#define SEQ_CSR_FORCE_UPDATE_STATUS_REG 0x20
#define SEQ_CSR_FLIP_STATUS_REG         0x10
#define SEQ_CSR_DISABLE_SEQ(n)          (0x4<<(n))
#define SEQ_CSR_ENABLE_SEQ(n)           (0x1<<(n))

#define SEQ_CSR_RD_STATUS_FIFO_RD_COUNT        0x3F0000
#define SEQ_CSR_RD_STATUS_FIFO_RD_COUNT_SHIFT  16

#define SEQ_CSR_RD_STATUS_FIFO_EMPTY           0x8
#define SEQ_CSR_RD_STATUS_FIFO_ALMOST_FULL     0x4
#define SEQ_CSR_RD_STATUS_FIFO_ACCEPT_WR       0x2
#define SEQ_CSR_RD_STATUS_FIFO_VALID           0x1

#define SEQ_CSR_WR_STATUS_FIFO_ACCEPT_WR       0x2
#define SEQ_CSR_WR_STATUS_FIFO_RE              0x1

void evgInit(void);
void evgCrank(void);
int evgAlign(void);
void evgShowAlignment(void);

int evgStashSequence(unsigned int idx, int isFirst, int count,
        const uint32_t *seqp);
int evgEnableSequence(unsigned int idx, int enable);
uint32_t evgSequencerStatus(unsigned int idx, uint32_t *seconds,
        uint32_t *fraction, uint32_t *catDelay, unsigned int numCatDelay);

void evgSoftwareTrigger(unsigned int idx, int eventCode);
void evgHardwareTrigger(unsigned int idx, int eventCode);

void evgDumpSequence(int evg, int seq);
int evgFetchCoincidenceStatus(uint32_t *ap);

#endif /* _EVG_H_ */
