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
 * High-speed serial (Multi-Gigabit) transceivers
 */

#ifndef _MGT_H_
#define _MGT_H_

#include "util.h"

#define CSR_W_ENABLE_RESETS     0x80000000
#define CSR_RW_GT_TX_RESET      0x40000000
#define CSR_RW_GT_RX_RESET      0x20000000
#define CSR_RW_GT_LOL_ACK       0x10000000

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

#define CSR_R_LOL_STATE_SIZE    3
#define CSR_R_LOL_STATE_SHIFT   18
#define CSR_R_LOL_STATE_MASK    REG_GEN_MASK(CSR_R_LOL_STATE_SHIFT, CSR_R_LOL_STATE_SIZE)
#define CSR_R_LOL_STATE_R(reg)  REG_GEN_READ(reg, CSR_R_LOL_STATE_SHIFT, CSR_R_LOL_STATE_SIZE)

#define CSR_R_LOL_RST_COUNTER_SIZE    2
#define CSR_R_LOL_RST_COUNTER_SHIFT   16
#define CSR_R_LOL_RST_COUNTER_MASK    REG_GEN_MASK(CSR_R_LOL_RST_COUNTER_SHIFT, CSR_R_LOL_RST_COUNTER_SIZE)
#define CSR_R_LOL_RST_COUNTER_R(reg)  REG_GEN_READ(reg, CSR_R_LOL_RST_COUNTER_SHIFT, CSR_R_LOL_RST_COUNTER_SIZE)

#define CSR_R_RESET_DONE        (CSR_R_TX_FSM_RESET_DONE | \
                                    CSR_R_RX_FSM_RESET_DONE | \
                                    CSR_R_TX_RESET_DONE | \
                                    CSR_R_RX_RESET_DONE | \
                                    CSR_R_CPLL_LOCKED)

void mgtInit(void);
int mgtLossOfLock(int mgtBitmap);
int mgtReset(int mgtBitmap);
void mgtTxStutter(int evgNumber);
int mgtFetchLatency(unsigned int evgIdx);
void mgtCrank();
int mgtFetchStatus(uint32_t *ap);
int mgtLOLState(int mgtBitmap, int lane);
int mgtLOLRstCounter(int mgtBitmap, int lane);

#endif /* _MGT_H_ */
