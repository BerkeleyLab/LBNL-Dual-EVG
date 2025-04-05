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
 * Event generator coincidence detection
 */

#ifndef _EVG_COINCIDENCE_H_
#define _EVG_COINCIDENCE_H_

#include "util.h"

/* CSR read */

#define DATA_HIST_SIZE              10
#define DATA_HIST_SHIFT             0
#define DATA_HIST_MASK              REG_GEN_MASK(DATA_HIST_SHIFT, DATA_HIST_SIZE)
#define DATA_HIST_R(reg)            REG_GEN_READ(reg, DATA_HIST_SHIFT, DATA_HIST_SIZE)

#define ADDRESS_RB_SIZE             11
#define ADDRESS_RB_SHIFT            DATA_HIST_SIZE
#define ADDRESS_RB_MASK             REG_GEN_MASK(ADDRESS_RB_SHIFT, ADDRESS_RB_SIZE)
#define ADDRESS_RB_R(reg)           REG_GEN_READ(reg, ADDRESS_RB_SHIFT, ADDRESS_RB_SIZE)

#define MUX_SEL_RB_SIZE             1
#define MUX_SEL_RB_SHIFT            24
#define MUX_SEL_RB_MASK             REG_GEN_MASK(MUX_SEL_RB_SHIFT, MUX_SEL_RB_SIZE)
#define MUX_SEL_RB_R(reg)           REG_GEN_READ(reg, MUX_SEL_RB_SHIFT, MUX_SEL_RB_SIZE)

#define BUSY_STATUS_SIZE            1
#define BUSY_STATUS_SHIFT           31
#define BUSY_STATUS_MASK            REG_GEN_MASK(BUSY_STATUS_SHIFT, BUSY_STATUS_SIZE)
#define CSR_R_BUSY                  BUSY_STATUS_MASK

/* CSR write */

#define ADDRESS_WR_SIZE             10
#define ADDRESS_WR_SHIFT            0
#define ADDRESS_WR_MASK             REG_GEN_MASK(ADDRESS_WR_SHIFT, ADDRESS_WR_SIZE)
#define ADDRESS_WR_W(value)         REG_GEN_WRITE(value, ADDRESS_WR_SHIFT, ADDRESS_WR_SIZE)

#define MUX_SEL_WR_SIZE             1
#define MUX_SEL_WR_SHIFT            24
#define MUX_SEL_WR_MASK             REG_GEN_MASK(MUX_SEL_WR_SHIFT, MUX_SEL_WR_SIZE)
#define MUX_SEL_WR_W(value)         REG_GEN_WRITE(value, MUX_SEL_WR_SHIFT, MUX_SEL_WR_SIZE)

#define REALIGN_WR_SIZE             1
#define REALIGN_WR_SHIFT            29
#define REALIGN_WR_MASK             REG_GEN_MASK(REALIGN_WR_SHIFT, REALIGN_WR_SIZE)
#define CSR_W_REALIGN               REALIGN_WR_MASK

#define SET_COINCIDENCE_WR_SIZE     1
#define SET_COINCIDENCE_WR_SHIFT    30
#define SET_COINCIDENCE_WR_MASK     REG_GEN_MASK(SET_COINCIDENCE_WR_SHIFT, SET_COINCIDENCE_WR_SIZE)
#define CSR_W_SET_COINCIDENCE       SET_COINCIDENCE_WR_MASK

#define START_WR_SIZE               1
#define START_WR_SHIFT              31
#define START_WR_MASK               REG_GEN_MASK(START_WR_SHIFT, START_WR_SIZE)
#define CSR_W_START                 START_WR_MASK

void evgCoincidenceInit(void);
void evgCoincidenceCrank(void);
void evgCoincidenceShow(int showData);

#endif /* _EVG_COINCIDENCE_H_ */
