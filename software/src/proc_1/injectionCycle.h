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
 * Control/monitor injection cycle
 */

#ifndef _INJECTION_CYCLE_H_
#define _INJECTION_CYCLE_H_

#include <stdint.h>
#include "util.h"

/*
 * Definitions for CSR
 */

#define CSR_INJ_W_SET_CYCLE_MILLISECONDS            (1UL << 31)
#define CSR_INJ_W_MANUAL_TRIGGER                    (1UL << 7)
#define CSR_INJ_W_DISABLE_TIMED_CYCLES              (1UL << 1)
#define CSR_INJ_W_ENABLE_TIMED_CYCLES               (1UL << 0)

#define CSR_INJ_R_POWER_LINE_VALID                  0x80000000
#define injectionCycleIsPowerLineValid()            ((GPIO_READ(GPIO_IDX_INJECTION_CYCLE_CSR) & \
                                                        CSR_INJ_R_POWER_LINE_VALID) != 0)

/*
 * Definitions for Alignment CSR W
 */

#define CSR_INJ_ALIGN_W_SET_ALIGN_SEL               (1UL << 31)

#define CSR_INJ_ALIGN_W_SET_HEARTBEAT_SEL           (1UL << 30)

#define CSR_INJ_ALIGN_W_SEL_SIZE                    3
#define CSR_INJ_ALIGN_W_SEL_SHIFT                   0
#define CSR_INJ_ALIGN_W_SEL_MASK                    REG_GEN_MASK(CSR_INJ_ALIGN_W_SEL_SHIFT, \
                                                    CSR_INJ_ALIGN_W_SEL_SIZE)
#define CSR_INJ_ALIGN_W_SEL_W(value)                REG_GEN_WRITE(value, CSR_INJ_ALIGN_W_SEL_SHIFT, \
                                                    CSR_INJ_ALIGN_W_SEL_SIZE)

/*
 * Definitions for Alignment CSR R
 */

#define CSR_INJ_ALIGN_R_COUNTER_SYNCED_SIZE         8
#define CSR_INJ_ALIGN_R_COUNTER_SYNCED_SHIFT        0
#define CSR_INJ_ALIGN_R_COUNTER_SYNCED_MASK         REG_GEN_MASK(CSR_INJ_ALIGN_R_COUNTER_SYNCED_SHIFT, \
                                                    CSR_INJ_ALIGN_R_COUNTER_SYNCED_SIZE)
#define CSR_INJ_ALIGN_R_COUNTER_SYNCED_R(reg)       REG_GEN_READ(reg, CSR_INJ_ALIGN_R_COUNTER_SYNCED_SHIFT, \
                                                    CSR_INJ_ALIGN_R_COUNTER_SYNCED_SIZE)

#define CSR_INJ_ALIGN_R_COUNTER_SEL_LATCH_SIZE      4
#define CSR_INJ_ALIGN_R_COUNTER_SEL_LATCH_SHIFT     8
#define CSR_INJ_ALIGN_R_COUNTER_SEL_LATCH_MASK      REG_GEN_MASK(CSR_INJ_ALIGN_R_COUNTER_SEL_LATCH_SHIFT, \
                                                    CSR_INJ_ALIGN_R_COUNTER_SEL_LATCH_SIZE)
#define CSR_INJ_ALIGN_R_COUNTER_SEL_LATCH_R(reg)    REG_GEN_READ(reg, CSR_INJ_ALIGN_R_COUNTER_SEL_LATCH_SHIFT, \
                                                    CSR_INJ_ALIGN_R_COUNTER_SEL_LATCH_SIZE)

#define CSR_INJ_ALIGN_R_COUNTER_SEL_SIZE            4
#define CSR_INJ_ALIGN_R_COUNTER_SEL_SHIFT           12
#define CSR_INJ_ALIGN_R_COUNTER_SEL_MASK            REG_GEN_MASK(CSR_INJ_ALIGN_R_COUNTER_SEL_SHIFT, \
                                                       CSR_INJ_ALIGN_R_COUNTER_SEL_SIZE)
#define CSR_INJ_ALIGN_R_COUNTER_SEL_R(reg)          REG_GEN_READ(reg, CSR_INJ_ALIGN_R_COUNTER_SEL_SHIFT, \
                                                       CSR_INJ_ALIGN_R_COUNTER_SEL_SIZE)

#define CSR_INJ_ALIGN_R_HB_SEL_SIZE                 4
#define CSR_INJ_ALIGN_R_HB_SEL_SHIFT                16
#define CSR_INJ_ALIGN_R_HB_SEL_MASK                 REG_GEN_MASK(CSR_INJ_ALIGN_R_HB_SEL_SHIFT, \
                                                            CSR_INJ_ALIGN_R_HB_SEL_SIZE)
#define CSR_INJ_ALIGN_R_HB_SEL_R(reg)               REG_GEN_READ(reg, CSR_INJ_ALIGN_R_HB_SEL_SHIFT, \
                                                            CSR_INJ_ALIGN_R_HB_SEL_SIZE)

void injectionCycleEnable(int enable);
void injectionCycleManualTrigger(void);
void injectionCycleSetBaseInterval(int milliseconds);
void injectionCycleExtendInterval(int milliseconds);
int injectionCycleFetchStatus(uint32_t *ap);
void injectionAlignSetAlignSel(int sel);
void injectionAlignSetHeartbeatSel(int sel);
int injectionAlignSetSel(unsigned int idx, int sel);
int injectionAlignGetAlignSel(void);
int injectionAlignGetHbSel(void);
int injectionAlignGetCSR(void);

#endif /* _INJECTION_CYCLE_H_ */
