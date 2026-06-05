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
 * Indices into the big general purpose I/O block.
 * Used to generate Verilog parameter statements too, so be careful with
 * the syntax:
 *     Spaces only (no tabs).
 *     Index defines must be valid Verilog parameter expressions.
 * The createVerilogIDX.sh script must be run after changess are made
 * to this file or to config.h.
 */

#ifndef _GPIO_H_
#define _GPIO_H_

#define GPIO_IDX_COUNT 256

#define GPIO_IDX_FIRMWARE_BUILD_DATE      0 // Firmware build POSIX seconds (R)
#define GPIO_IDX_MICROSECONDS_SINCE_BOOT  1 // Microseconds since boot (R)
#define GPIO_IDX_SECONDS_SINCE_BOOT       2 // Seconds since boot (R)
#define GPIO_IDX_FREQ_MONITOR_CSR         3 // Frequency measurement CSR
#define GPIO_IDX_USER_GPIO_CSR            4 // Reset/recovery switch and such
#define GPIO_IDX_I2C_CHUNK_CSR            5 // State machine for I2C I/O
#define GPIO_IDX_EVG_SYNC_CSR             6 // Heartbeat synchronization
#define GPIO_IDX_NET_CONFIG_CSR           7 // Network configuration
#define GPIO_IDX_NET_TX_CSR               8 // Network packet transmission
#define GPIO_IDX_NET_RX_CSR               9 // Network packet reception
#define GPIO_IDX_NET_RX_DATA             10 // Network packet received data
#define GPIO_IDX_FMC1_DIAGNOSTIC         11 // FMC1 miscellaneous I/O
#define GPIO_IDX_FMC2_DIAGNOSTIC         12 // FMC2 miscellaneous I/O
#define GPIO_IDX_FMC1_FIREFLY            13 // FMC1 Firefly control/status
#define GPIO_IDX_FMC2_FIREFLY            14 // FMC2 Firefly control/status
#define GPIO_IDX_QSPI_FLASH_CSR          15 // Boot flash I/O
#define GPIO_IDX_UNUSED_1                16 // Unused
#define GPIO_IDX_UNUSED_2                17 // Unused
#define GPIO_IDX_FAN_TACHOMETERS         18 // LCD Data
#define GPIO_IDX_EVG_1_COINC_CSR         19 // EVG 1 coincidence measure/control
#define GPIO_IDX_EVG_1_SEQ_CSR           20 // EVG 1 sequencer control
#define GPIO_IDX_EVG_1_SEQ_RBK           21 // EVG 1 sequencer readback
#define GPIO_IDX_EVG_1_HW_CSR            22 // EVG 1 hardware trigger control
#define GPIO_IDX_EVG_1_SW_CSR            23 // EVG 1 software trigger control
#define GPIO_IDX_EVG_2_COINC_CSR         24 // EVG 2 coincidence measure/control
#define GPIO_IDX_EVG_2_SEQ_CSR           25 // EVG 2 sequencer control
#define GPIO_IDX_EVG_2_SEQ_RBK           26 // EVG 2 sequencer readback
#define GPIO_IDX_EVG_2_HW_CSR            27 // EVG 2 hardware trigger control
#define GPIO_IDX_EVG_2_SW_CSR            28 // EVG 2 software trigger control
#define GPIO_IDX_INJECTION_CYCLE_CSR     29 // Injection cycle control
#define GPIO_IDX_INJECTION_ALIGN_CSR     30 // Injection align control
#define GPIO_IDX_INJECTION_TARGET_CSR    31 // Injection target selection control
#define GPIO_IDX_SWAPOUT_CYCLE_CSR       32 // Swapout cycle control
#define GPIO_IDX_SWAPOUT_ALIGN_CSR       33 // Swapout align control
#define GPIO_IDX_NTP_SERVER_STATUS       34 // NTP server status (R)
#define GPIO_IDX_NTP_SERVER_SECONDS      35 // NTP server seconds (R/W)
#define GPIO_IDX_NTP_SERVER_FRACTION     36 // NTP fractional seconds (R)
#define GPIO_IDX_MMC_MAILBOX             37 // Communicate with MMC
#define GPIO_IDX_EVG_1_DISP_LOG_CSR      38 // EVG 1 display event logger
#define GPIO_IDX_EVG_2_DISP_LOG_CSR      39 // EVG 2 display event logger
#define GPIO_IDX_GITHASH                 40 // Git 32-bit hash
#define GPIO_IDX_EVG_1_TLOG_CSR          41 // EVG 1 tlog event logger control
#define GPIO_IDX_EVG_1_TLOG_TICKS        42 // EVG 1 tlog event logger ticks
#define GPIO_IDX_EVG_2_TLOG_CSR          43 // EVG 2 tlog event logger control
#define GPIO_IDX_EVG_2_TLOG_TICKS        44 // EVG 2 tlog event logger ticks
#define GPIO_IDX_EVG_1_SEQ_SECONDS_CSR   45 // EVG 1 sequencer status seconds
#define GPIO_IDX_EVG_1_SEQ_FRACTION_CSR  46 // EVG 1 sequencer status fraction
#define GPIO_IDX_EVG_2_SEQ_SECONDS_CSR   47 // EVG 2 sequencer status seconds
#define GPIO_IDX_EVG_2_SEQ_FRACTION_CSR  48 // EVG 2 sequencer status fraction
#define GPIO_IDX_NTP_SERVER_F2_STATUS    49 // NTP server F2 status (R)
#define GPIO_IDX_NTP_SERVER_F2_SECONDS   50 // NTP server F2 seconds (R/W)
#define GPIO_IDX_NTP_SERVER_F2_FRACTION  51 // NTP F2 fractional seconds (R)
#define GPIO_IDX_EVG_1_SEQ_STATUS_FIFO_CSR 52 // EVG 1 sequencer status FIFO
#define GPIO_IDX_EVG_2_SEQ_STATUS_FIFO_CSR 53 // EVG 2 sequencer status FIFO
#define GPIO_IDX_EVG_1_CLK_GEN_1_CSR     54 // EVG 1 Clk Generation 1 status
#define GPIO_IDX_EVG_1_CLK_GEN_2_CSR     55 // EVG 1 Clk Generation 2 status
#define GPIO_IDX_EVG_1_CLK_GEN_3_CSR     56 // EVG 1 Clk Generation 3 status
#define GPIO_IDX_EVG_1_CLK_GEN_4_CSR     57 // EVG 1 Clk Generation 4 status
#define GPIO_IDX_EVG_1_CLK_GEN_5_CSR     58 // EVG 1 Clk Generation 4 status
#define GPIO_IDX_EVG_1_CLK_GEN_6_CSR     59 // EVG 1 Clk Generation 4 status
#define GPIO_IDX_EVG_2_CLK_GEN_1_CSR     60 // EVG 2 Clk Generation 1 status
#define GPIO_IDX_EVG_2_CLK_GEN_2_CSR     61 // EVG 2 Clk Generation 2 status
#define GPIO_IDX_EVG_2_CLK_GEN_3_CSR     62 // EVG 2 Clk Generation 3 status

// Per EVG 1 MGT
#define GPIO_IDX_EVG_1_0_DRP_CSR         128 // EVG 1 ID 0 transceiver DRP access
#define GPIO_IDX_EVG_1_0_LATENCY         129 // EVG 1 ID 0 round-trip latency

#define GPIO_IDX_PER_MGTWRAPPER          (GPIO_IDX_EVG_1_0_LATENCY-GPIO_IDX_EVG_1_0_DRP_CSR+1)

// Per EVG 2 MGT
#define GPIO_IDX_EVG_2_0_DRP_CSR         (GPIO_IDX_EVG_1_0_DRP_CSR+16) // EVG 2 ID 0 transceiver DRP access
#define GPIO_IDX_EVG_2_0_LATENCY         (GPIO_IDX_EVG_1_0_LATENCY+16) // EVG 2 ID 0 round-trip latency

// Per EVG 1 category. Maximum of 32
#define GPIO_IDX_EVG_1_0_CAT_DELAY_RBK_0    160 // EVG 1 ID 0 category delay RBK

// Per EVG 2 category. Maximum of 32
#define GPIO_IDX_EVG_2_0_CAT_DELAY_RBK_0    192 // EVG 2 ID 0 category delay RBK

#include <xil_io.h>
#include <xparameters.h>
#include <assert.h>
#include "config.h"

static_assert(CFG_EVENT_CAT_NUM < 8,
        "CFG_EVENT_CAT_NUM cannot be greater than 8");

#define GPIO_READ(i)    Xil_In32(XPAR_AXI_LITE_GENERIC_REG_0_BASEADDR+(4*(i)))
#define GPIO_WRITE(i,x) Xil_Out32(XPAR_AXI_LITE_GENERIC_REG_0_BASEADDR+(4*(i)),(x))
#define MICROSECONDS_SINCE_BOOT()   GPIO_READ(GPIO_IDX_MICROSECONDS_SINCE_BOOT)

#endif
