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
 * Control/monitor swapout cycle
 */

#ifndef _SWAPOUT_CYCLE_H_
#define _SWAPOUT_CYCLE_H_

#include <stdint.h>
#include "util.h"

/*
 * Definitions for CSR
 */

#define CSR_SWAPOUT_R_NOT_SYNCED                        0x80000000
#define swapoutIsUnsynched()                            ((GPIO_READ(GPIO_IDX_SWAPOUT_CYCLE_CSR) & \
                                                            CSR_SWAPOUT_R_NOT_SYNCED) != 0)

#define CSR_SWAPOUT_W_OFFSET_SIZE                       16
#define CSR_SWAPOUT_W_OFFSET_SHIFT                      0
#define CSR_SWAPOUT_W_OFFSET_MASK                       REG_GEN_MASK(CSR_SWAPOUT_W_OFFSET_SHIFT, \
                                                            CSR_SWAPOUT_W_OFFSET_SIZE)
#define CSR_SWAPOUT_W_OFFSET_W(value)                   REG_GEN_WRITE(value, CSR_SWAPOUT_W_OFFSET_SHIFT, \
                                                            CSR_SWAPOUT_W_OFFSET_SIZE)

/*
 * Definitions for Alignment CSR W
 */

#define CSR_SWAPOUT_ALIGN_W_SET_ALIGN_SEL               (1UL << 31)

#define CSR_SWAPOUT_ALIGN_W_SET_HEARTBEAT_SEL           (1UL << 30)

#define CSR_SWAPOUT_ALIGN_W_SEL_SIZE                    3
#define CSR_SWAPOUT_ALIGN_W_SEL_SHIFT                   0
#define CSR_SWAPOUT_ALIGN_W_SEL_MASK                    REG_GEN_MASK(CSR_SWAPOUT_ALIGN_W_SEL_SHIFT, \
                                                            CSR_SWAPOUT_ALIGN_W_SEL_SIZE)
#define CSR_SWAPOUT_ALIGN_W_SEL_W(value)                REG_GEN_WRITE(value, CSR_SWAPOUT_ALIGN_W_SEL_SHIFT, \
                                                            CSR_SWAPOUT_ALIGN_W_SEL_SIZE)

/*
 * Definitions for Alignment CSR R
 */

#define CSR_SWAPOUT_ALIGN_R_COUNTER_SYNCED_SIZE         8
#define CSR_SWAPOUT_ALIGN_R_COUNTER_SYNCED_SHIFT        0
#define CSR_SWAPOUT_ALIGN_R_COUNTER_SYNCED_MASK         REG_GEN_MASK(CSR_SWAPOUT_ALIGN_R_COUNTER_SYNCED_SHIFT, \
                                                            CSR_SWAPOUT_ALIGN_R_COUNTER_SYNCED_SIZE)
#define CSR_SWAPOUT_ALIGN_R_COUNTER_SYNCED_R(reg)       REG_GEN_READ(reg, CSR_SWAPOUT_ALIGN_R_COUNTER_SYNCED_SHIFT, \
                                                            CSR_SWAPOUT_ALIGN_R_COUNTER_SYNCED_SIZE)

#define CSR_SWAPOUT_ALIGN_R_COUNTER_SEL_LATCH_SIZE      4
#define CSR_SWAPOUT_ALIGN_R_COUNTER_SEL_LATCH_SHIFT     8
#define CSR_SWAPOUT_ALIGN_R_COUNTER_SEL_LATCH_MASK      REG_GEN_MASK(CSR_SWAPOUT_ALIGN_R_COUNTER_SEL_LATCH_SHIFT, \
                                                            CSR_SWAPOUT_ALIGN_R_COUNTER_SEL_LATCH_SIZE)
#define CSR_SWAPOUT_ALIGN_R_COUNTER_SEL_LATCH_R(reg)    REG_GEN_READ(reg, CSR_SWAPOUT_ALIGN_R_COUNTER_SEL_LATCH_SHIFT, \
                                                            CSR_SWAPOUT_ALIGN_R_COUNTER_SEL_LATCH_SIZE)

#define CSR_SWAPOUT_ALIGN_R_COUNTER_SEL_SIZE            4
#define CSR_SWAPOUT_ALIGN_R_COUNTER_SEL_SHIFT           12
#define CSR_SWAPOUT_ALIGN_R_COUNTER_SEL_MASK            REG_GEN_MASK(CSR_SWAPOUT_ALIGN_R_COUNTER_SEL_SHIFT, \
                                                            CSR_SWAPOUT_ALIGN_R_COUNTER_SEL_SIZE)
#define CSR_SWAPOUT_ALIGN_R_COUNTER_SEL_R(reg)          REG_GEN_READ(reg, CSR_SWAPOUT_ALIGN_R_COUNTER_SEL_SHIFT, \
                                                            CSR_SWAPOUT_ALIGN_R_COUNTER_SEL_SIZE)

#define CSR_SWAPOUT_ALIGN_R_HB_SEL_SIZE                 4
#define CSR_SWAPOUT_ALIGN_R_HB_SEL_SHIFT                16
#define CSR_SWAPOUT_ALIGN_R_HB_SEL_MASK                 REG_GEN_MASK(CSR_SWAPOUT_ALIGN_R_HB_SEL_SHIFT, \
                                                            CSR_SWAPOUT_ALIGN_R_HB_SEL_SIZE)
#define CSR_SWAPOUT_ALIGN_R_HB_SEL_R(reg)               REG_GEN_READ(reg, CSR_SWAPOUT_ALIGN_R_HB_SEL_SHIFT, \
                                                            CSR_SWAPOUT_ALIGN_R_HB_SEL_SIZE)

void swapoutCycleEnable(int offset);
void swapoutAlignSetAlignSel(int sel);
void swapoutAlignSetHeartbeatSel(int sel);
int swapoutAlignSetSel(unsigned int idx, int sel);
int swapoutAlignGetAlignSel(void);
int swapoutAlignGetHbSel(void);
int swapoutAlignFetchStatus(uint32_t *ap);

#endif /* _SWAPOUT_CYCLE_H_ */
