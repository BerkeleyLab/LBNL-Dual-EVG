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
#include <stdarg.h>
#include <string.h>
#include "console.h"
#include "display.h"
#include "gpio.h"
#include "util.h"

#define USER_GPIO_CSR_RESET_BUTTON      0x80000000
#define USER_GPIO_CSR_DISPLAY_BUTTON    0x40000000
#define USER_GPIO_CSR_GPS_PPS_VALID     0x2
#define USER_GPIO_CSR_FMC_PPS_VALID     0x1

static char msgBuf[120];

/*
 * Employ some gross hacks to avoid using vprintf (and thereby
 * bloating the text size by about 60 kB).
 */
void
fatal(const char *fmt, ...)
{
    va_list args;
    unsigned int now, then = MICROSECONDS_SINCE_BOOT();
    unsigned int limit = 0;
    static int enterCount = 0;
    unsigned int a[4];

    enterCount++;
    for (;;) {
        if (((now = MICROSECONDS_SINCE_BOOT()) - then) >= limit) {
            va_start(args, fmt);
            a[0] = va_arg(args, unsigned int);
            a[1] = va_arg(args, unsigned int);
            a[2] = va_arg(args, unsigned int);
            a[3] = va_arg(args, unsigned int);
            sprintf(msgBuf, fmt, a[0], a[1], a[2], a[3]);
            va_end(args);
            printf("*** Fatal error: %s\n", msgBuf);
            displayShowFatal(msgBuf);
            then = now;
            limit = 10000000;
        }
        if (enterCount == 1) {
            consoleCheck();
            checkForReset();
        }
    }
}

void
warn(const char *fmt, ...)
{
    va_list args;
    unsigned int a[4];

    va_start(args, fmt);
    a[0] = va_arg(args, unsigned int);
    a[1] = va_arg(args, unsigned int);
    a[2] = va_arg(args, unsigned int);
    a[3] = va_arg(args, unsigned int);
    sprintf(msgBuf, fmt, a[0], a[1], a[2], a[3]);
    va_end(args);
    printf("*** Warning: %s\n", msgBuf);
    displayShowWarning(msgBuf);
}

void
microsecondSpin(unsigned int us)
{
    uint32_t then;
    then = MICROSECONDS_SINCE_BOOT();
    while ((uint32_t)((MICROSECONDS_SINCE_BOOT()) - then) <= us) continue;
}

/*
 * Show register contents
 */
void
showReg(unsigned int i)
{
    int r;

    if (i >= GPIO_IDX_COUNT) return;
    r = GPIO_READ(i);
    printf("  R%d = %04X:%04X %11d\n", i, (r>>16)&0xFFFF, r&0xFFFF, r);
}

/*
 * See if reset switch has been pressed for a while
 */
void
checkForReset(void)
{
    uint32_t now = MICROSECONDS_SINCE_BOOT();
    static uint32_t then;
    static uint32_t beenReleased = 0;
    if (resetRecoverySwitchPressed()) {
        if (beenReleased && ((MICROSECONDS_SINCE_BOOT() - then) > 2000000)) {
            resetFPGA(displaySwitchPressed());
        }
    }
    else {
        beenReleased = 1;
        then = now;
    }
    if (sharedMemory->resetFPGA) {
        resetFPGA(displaySwitchPressed());
    }
}

/*
 * Is reset/recovery switch currently pressed
 */
int
resetRecoverySwitchPressed(void)
{
    return GPIO_READ(GPIO_IDX_USER_GPIO_CSR) & USER_GPIO_CSR_RESET_BUTTON;
}

/*
 * Is display switch pressed
 */
int
displaySwitchPressed(void)
{
    return GPIO_READ(GPIO_IDX_USER_GPIO_CSR) & USER_GPIO_CSR_DISPLAY_BUTTON;
}

/*
 * Convert number of days since POSIX epoch to year, month, day
 * Ref: http://howardhinnant.github.io/date_algorithms.html#civil_from_days
 */
void
civil_from_days(int posixDays, int *year, int *month, int *day)
{
    posixDays += 719468;
    const int era =  posixDays / 146097;
    const unsigned doe = (posixDays - era * 146097);              // [0, 146096]
    const unsigned yoe = (doe-doe/1460+doe/36524-doe/146096)/365; // [0, 399]
    const int y = yoe + era * 400;
    const unsigned doy = doe - (365*yoe + yoe/4 - yoe/100);       // [0, 365]
    const unsigned mp = (5*doy + 2)/153;                          // [0, 11]
    const unsigned d = doy - (153*mp+2)/5 + 1;                    // [1, 31]
    const unsigned m = mp + (mp < 10 ? 3 : -9);                   // [1, 12]
    *year = y;
    *month = m;
    *day = d;
}

/*
 * Write to the ICAP instance to force a warm reboot
 * Command sequence from UG470
 */
static void
writeICAP(int value)
{
    Xil_Out32(XPAR_HWICAP_0_BASEADDR+0x100, value); /* Write FIFO */
}
void
resetFPGA(int bootAlternateImage)
{
    printf("====== FPGA REBOOT ======\n\n");
    microsecondSpin(50000);
    writeICAP(0xFFFFFFFF); /* Dummy word */
    writeICAP(0xAA995566); /* Sync word */
    writeICAP(0x20000000); /* Type 1 NO-OP */
    writeICAP(0x30020001); /* Type 1 write 1 to Warm Boot STart Address Reg */
    writeICAP(bootAlternateImage ? 0x00700000
                                 : 0x00000000); /* Warm boot start addr */
    writeICAP(0x20000000); /* Type 1 NO-OP */
    writeICAP(0x30008001); /* Type 1 write 1 to CMD */
    writeICAP(0x0000000F); /* IPROG command */
    writeICAP(0x20000000); /* Type 1 NO-OP */
    Xil_Out32(XPAR_HWICAP_0_BASEADDR+0x10C, 0x1);   /* Initiate WRITE */
    microsecondSpin(1000000);
    printf("====== FPGA REBOOT FAILED ======\n");
}
