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
 * Support for optional GPS receiver as time of day and pulse per second source.
 */
#include <stdio.h>
#include <stdint.h>
#include "gpio.h"
#include "gps.h"
#include "NMEA.h"
#include "util.h"

#define REG_R_RBR       0x1000   /* Receiver Buffer Register */
#define REG_W_THR       0x1000   /* Transmitter Holding Register */
#define REG_RW_IER      0x1004   /* Interupt Enable Regisger */
#define REG_R_IIR       0x1008   /* Interrupt Identification Register */
#define REG_W_FCR       0x1008   /* FIFO Control Register */
#define REG_RW_LCR      0x100C   /* Line Control Regisger */
#define REG_RW_MCR      0x1010   /* Modem Control Regisger */
#define REG_RW_LSR      0x1014   /* Line Status Regisger */
#define REG_RW_MSR      0x1018   /* Modem Status Regisger */
#define REG_RW_MSR      0x1018   /* Modem Status Regisger */

#define REG_RW_DLL      0x1000   /* Divisor Latch LSB (when LCR(7)=1) */
#define REG_RW_DLM      0x1004   /* Divisor Latch MSB (when LCR(7)=1) */
#define REG_R_FCR       0x1008   /* FIFO Control Register (when LCR(7)=1) */

#define LCR_DLAB        0x80    /* Divisor latch access bit */
#define LCR_WLS8        0x03    /* Eight data bits */
#define MCR_OUT1N       0x04    /* Control OUT1N pin */
#define FCR_RXFIFO_RST  0x02    /* Reset receiver FIFO */
#define FCR_FIFOEN      0x01    /* FIFO enable */
#define LSR_FE          0x08    /* Framing error */
#define LSR_PE          0x04    /* Parity error */
#define LSR_OE          0x02    /* Overrun error */
#define LSR_DR          0x01    /* Data ready */

#define WREG(r,v) Xil_Out32(XPAR_GPS_UART_BASEADDR+(r), (v))
#define RREG(r)    Xil_In32(XPAR_GPS_UART_BASEADDR+(r))

#define BAUD        9600
#define DIVISOR     (XPAR_GPS_UART_CLOCK_FREQ_HZ/(16 * (BAUD)))

static void
init16550(void)
{
    /* 9600 8-N-1 */
    WREG(REG_RW_LCR, LCR_DLAB|LCR_WLS8);
    WREG(REG_RW_DLL, DIVISOR & 0xFF);
    WREG(REG_RW_DLM, DIVISOR >> 8);
    WREG(REG_W_FCR, FCR_RXFIFO_RST | FCR_FIFOEN);
    WREG(REG_RW_LCR, LCR_WLS8);
}

static int
poll(void)
{
    if (RREG(REG_RW_LSR) & LSR_DR) {
        return RREG(REG_R_RBR) & 0xFF;
    }
    else {
        return -1;
    }
}

void
gpsInit(void)
{
    init16550();
}

void
gpsFlush(void)
{
    WREG(REG_W_FCR, FCR_RXFIFO_RST | FCR_FIFOEN);
}

static int haveNewTime;
static uint32_t newTimePosixSeconds, newTimeFraction;
void
gpsCheck(uint32_t *posixSeconds, uint32_t *fraction)
{
    int c;
    static int oldCount = -1;
    while ((c = poll()) >= 0) {
        NMEAconsume(c);
        if (haveNewTime) {
            *posixSeconds = newTimePosixSeconds;
            *fraction = newTimeFraction;
            haveNewTime = 0;
            break;
        }
    }
    if ((debugFlags & DEBUGFLAG_GPS) && (oldCount != NMEAsatellitesInView())) {
        int n = NMEAsatellitesInView();
        oldCount = n;
        printf("Satellites now in view: %d\n", n);
    }
}

void
NMEAerror(const char *message)
{
    if (debugFlags & DEBUGFLAG_GPS) {
        printf("GPS error: \"%s\"\n", message);
    }
}

void
NMEAcallback(const char *sentence)
{
    if (debugFlags & DEBUGFLAG_GPS) {
        printf("GPS: \"%s\"\n", sentence);
    }
}

void
NMEAtime(unsigned int posixSeconds, unsigned int fractionalSeconds) {
    newTimePosixSeconds = posixSeconds;
    newTimeFraction = fractionalSeconds;
    haveNewTime = 1;
}
