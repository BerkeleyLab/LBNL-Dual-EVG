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

#define GPIO_IDX_COUNT 64

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
#define GPIO_IDX_DISPLAY_CSR             16 // LCD CSR
#define GPIO_IDX_DISPLAY_DATA            17 // LCD Data
#define GPIO_IDX_FAN_TACHOMETERS         18 // LCD Data
#define GPIO_IDX_EVG_1_COINC_CSR         19 // EVG 1 coincidence measure/control
#define GPIO_IDX_EVG_1_SEQ_CSR           20 // EVG 1 sequencer control
#define GPIO_IDX_EVG_1_SEQ_RBK           21 // EVG 1 sequencer readback
#define GPIO_IDX_EVG_1_HW_CSR            22 // EVG 1 hardware trigger control
#define GPIO_IDX_EVG_1_SW_CSR            23 // EVG 1 software trigger control
#define GPIO_IDX_EVG_1_DRP_CSR           24 // EVG 1 transceiver DRP access
#define GPIO_IDX_EVG_1_LATENCY           25 // EVG 1 round-trip latency
#define GPIO_IDX_EVG_2_COINC_CSR         26 // EVG 2 coincidence measure/control
#define GPIO_IDX_EVG_2_SEQ_CSR           27 // EVG 2 sequencer control
#define GPIO_IDX_EVG_2_SEQ_RBK           28 // EVG 2 sequencer readback
#define GPIO_IDX_EVG_2_HW_CSR            29 // EVG 2 hardware trigger control
#define GPIO_IDX_EVG_2_SW_CSR            30 // EVG 2 software trigger control
#define GPIO_IDX_EVG_2_DRP_CSR           31 // EVG 2 transceiver DRP access
#define GPIO_IDX_EVG_2_LATENCY           32 // EVG 1 round-trip latency
#define GPIO_IDX_INJECTION_CYCLE_CSR     33 // Injection cycle control
#define GPIO_IDX_SWAPOUT_CYCLE_CSR       34 // Swapout cycle control
#define GPIO_IDX_NTP_SERVER_STATUS       35 // NTP server status (R)
#define GPIO_IDX_NTP_SERVER_SECONDS      36 // NTP server seconds (R/W)
#define GPIO_IDX_NTP_SERVER_FRACTION     37 // NTP fractional seconds (R)
#define GPIO_IDX_MMC_MAILBOX             38 // Communicate with MMC
#define GPIO_IDX_EVG_1_LOG_CSR           39 // EVG 1 event logger
#define GPIO_IDX_EVG_2_LOG_CSR           40 // EVG 2 event logger

#include <xil_io.h>
#include <xparameters.h>
#include "config.h"

#define GPIO_READ(i)    Xil_In32(XPAR_AXI_LITE_GENERIC_REG_0_BASEADDR+(4*(i)))
#define GPIO_WRITE(i,x) Xil_Out32(XPAR_AXI_LITE_GENERIC_REG_0_BASEADDR+(4*(i)),(x))
#define MICROSECONDS_SINCE_BOOT()   GPIO_READ(GPIO_IDX_MICROSECONDS_SINCE_BOOT)

#endif
