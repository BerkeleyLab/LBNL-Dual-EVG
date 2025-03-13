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
 * Utility routines
 */

#ifndef _UTIL_H_
#define _UTIL_H_

#include <stdint.h>
#include <xil_io.h>
#include "sharedMemory.h"

/*
 * Allow code to refer  to printf without actually pulling it in
 */
#define printf(...) xil_printf(__VA_ARGS__)

/*
 * Diagnostics
 */
#define DEBUGFLAG_EPICS             0x1
#define DEBUGFLAG_TFTP              0x2
#define DEBUGFLAG_BOOT_FLASH        0x4
#define DEBUGFLAG_TOD               0x8
#define DEBUGFLAG_GPS               0x10
#define DEBUGFLAG_STASH_SEQUENCE    0x20
#define DEBUGFLAG_SHOW_SEQUENCES    0x40
#define DEBUGFLAG_SHOW_TX_RESETS    0x80
#define DEBUGFLAG_SHOW_RX_ALIGNER   0x100
#define DEBUGFLAG_COINCIDENCE       0x200
#define DEBUGFLAG_SHOW_COINCIDENCE  0x400
#define DEBUGFLAG_PLOT_COINCIDENCE  0x800
#define DEBUGFLAG_IIC_PROC          0x1000
#define DEBUGFLAG_IIC_FMC           0x2000
#define DEBUGFLAG_IIC_FMC_REG       0x4000
#define DEBUGFLAG_IIC_SCAN          0x8000
#define DEBUGFLAG_DISPLAY_NEXT_PAGE 0x10000
#define DEBUGFLAG_SEQ_STATUS_FIFO   0x200000
#define DEBUGFLAG_FORCE_MEAS_ON_LOA 0x400000 // Force coincidence measurement on lost alignment state
#define DEBUGFLAG_DUMP_SCREEN       0x1000000
#define DEBUGFLAG_DUMP_MGT_SWITCH   0x2000000
#define DEBUGFLAG_DUMP_CROSSPOINT   0x4000000
#define DEBUGFLAG_NO_RESYNC_ON_LOL  0x8000000
#define DEBUGFLAG_TX_RESET          0x40000000

void warn(const char *fmt, ...);
void bswap32(uint32_t *b, int n);
void microsecondSpin(unsigned int us);
void showReg(unsigned int i);
void resetFPGA(int bootAlternateImage);
void checkForReset(void);
int resetRecoverySwitchPressed(void);
int displaySwitchPressed(void);
void civil_from_days(int posixDays, int *year, int *month, int *day);

#endif /* _UTIL_H_ */
