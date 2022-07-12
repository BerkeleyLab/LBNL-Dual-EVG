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
#include <stdint.h>
#include "display.h"
#include "drawEventLog.h"
#include "evg.h"
#include "evio.h"
#include "fmcIO.h"
#include "gpio.h"
#include "injectionCycle.h"
#include "st7789v.h"
#include "tod.h"
#include "util.h"
#include "xadc.h"

/*
 * Special display modes for internal use
 */
#define DISPLAY_MODE_UPDATE -1
#define DISPLAY_MODE_FETCH  -2

/*
 * Update date and time
 * Draw as little as possible
 */
static void
drawTime(int redraw)
{
    uint32_t now = GPIO_READ(GPIO_IDX_NTP_SERVER_SECONDS) - NTP_POSIX_OFFSET;
    static uint32_t then;
    int year, month, day;
    int n, c;
    static char cbuf[12]; /* YYYMMDDhhmms */
    char *cp;
    static short yBase;

    if (yBase == 0) {
        yBase = DISPLAY_HEIGHT - st7789vCharHeight;
    }
    if (redraw) {
        st7789vSetCharacterRGB(ST7789V_WHITE, ST7789V_BLACK);
        st7789vDrawChar((0 * st7789vCharWidth), yBase, '2');
        st7789vDrawChar((4 * st7789vCharWidth), yBase, '-');
        st7789vDrawChar((7 * st7789vCharWidth), yBase, '-');
        st7789vDrawChar((13 * st7789vCharWidth), yBase, ':');
        st7789vDrawChar((16 * st7789vCharWidth), yBase, ':');
        st7789vShowString((19 * st7789vCharWidth), yBase, " UTC");
        then = now - 1;
        memset(cbuf, 0, sizeof cbuf);
    }
    if (now == then) return;
    if (now != (then + 1)) redraw = 1;
    then = now;
    cp = cbuf + 11;
    st7789vSetCharacterRGB(ST7789V_WHITE, ST7789V_BLACK);

    n = now % 60;
    c = (n % 10) + '0';
    st7789vDrawChar(18 * st7789vCharWidth, yBase, c);
    c = (n / 10) + '0';
    if ((c == *cp) && !redraw) return;
    *cp-- = c;
    st7789vDrawChar(17 * st7789vCharWidth, yBase, c);
    if ((n != 0) && !redraw) return;

    n = (now / 60) % 60;
    c = (n % 10) + '0';
    if ((c == *cp) && !redraw) return;
    *cp-- = c;
    st7789vDrawChar(15 * st7789vCharWidth, yBase, c);
    c = (n / 10) + '0';
    if ((c == *cp) && !redraw) return;
    *cp-- = c;
    st7789vDrawChar(14 * st7789vCharWidth, yBase, c);
    if ((n != 0) && !redraw) return;

    n = (now / 3600) % 24;
    c = (n % 10) + '0';
    if ((c == *cp) && !redraw) return;
    *cp-- = c;
    st7789vDrawChar(12 * st7789vCharWidth, yBase, c);
    c = (n / 10) + '0';
    if ((c == *cp) && !redraw) return;
    *cp-- = c;
    st7789vDrawChar(11 * st7789vCharWidth, yBase, c);
    if ((n != 0) && !redraw) return;

    civil_from_days(now / 86400, &year, &month, &day);
    c = (day % 10) + '0';
    if ((c == *cp) && !redraw) return;
    *cp-- = c;
    st7789vDrawChar(9 * st7789vCharWidth, yBase, c);
    c = (day / 10) + '0';
    if ((c == *cp) && !redraw) return;
    *cp-- = c;
    st7789vDrawChar(8 * st7789vCharWidth, yBase, c);
    if ((day != 1) && !redraw) return;

    c = (month % 10) + '0';
    if ((c == *cp) && !redraw) return;
    *cp-- = c;
    st7789vDrawChar(6 * st7789vCharWidth, yBase, c);
    c = (month / 10) + '0';
    if ((c == *cp) && !redraw) return;
    *cp-- = c;
    st7789vDrawChar(5 * st7789vCharWidth, yBase, c);
    if ((month != 1) && !redraw) return;

    c = (year % 10) + '0';
    if ((c == *cp) && !redraw) return;
    *cp-- = c;
    st7789vDrawChar(3 * st7789vCharWidth, yBase, c);
    c = ((year / 10) % 10) + '0';
    if ((c == *cp) && !redraw) return;
    *cp-- = c;
    st7789vDrawChar(2 * st7789vCharWidth, yBase, c);
    c = ((year / 100) % 10) + '0';
    if ((c == *cp) && !redraw) return;
    *cp = c;
    st7789vDrawChar(1 * st7789vCharWidth, yBase, c);
}

/*
 * Account for EVG->SYS clock domain crossing -- read until stable
 */
uint32_t
evgSequencerStatus(unsigned int idx)
{
    int csrIdx = idx ? GPIO_IDX_EVG_2_SEQ_CSR : GPIO_IDX_EVG_1_SEQ_CSR;
    uint32_t r0, r1;

    r0 = GPIO_READ(csrIdx);
    for (;;) {
        r1 = GPIO_READ(csrIdx);
        if (r1 == r0) return r1;
        r0 = r1;
    }
}

static void
drawEVGstatus(int redrawAll)
{
    int i;
    int xBase;
    int xInc = DISPLAY_WIDTH / 2;
    static uint32_t oldStatus[2];
    static uint32_t whenStarted[2];

    if (redrawAll) {
        st7789vSetCharacterRGB(ST7789V_WHITE, ST7789V_BLACK);
        for (i = 0, xBase = 0 ; i < EVG_COUNT ;  i++, xBase += xInc) {
            st7789vShowString(xBase+1*st7789vCharWidth, 0, "EVG");
            st7789vDrawChar  (xBase+5*st7789vCharWidth, 0, '1' + i);
            st7789vShowString(xBase,   st7789vCharHeight, "TRG:");
            st7789vShowString(xBase, 2*st7789vCharHeight, "SEQ:");
        }
    }
    for (i = 0, xBase = 0 ; i < EVG_COUNT ; i++, xBase += xInc) {
        uint32_t status = evgSequencerStatus(i) & (EVG_STATUS_MAP_1_ACTIVE |
                                                   EVG_STATUS_START_COUNT_MASK);
        uint32_t now = MICROSECONDS_SINCE_BOOT();
        uint32_t diff;
        if (redrawAll) {
            diff = EVG_STATUS_MAP_1_ACTIVE;
            if (status & EVG_STATUS_SEQUENCER_ACTIVE) {
                diff |= EVG_STATUS_START_COUNT_MASK;
            }
        }
        else {
            diff = status ^ oldStatus[i];
        }
        if (diff) {
            oldStatus[i] = status;
            if (diff & EVG_STATUS_START_COUNT_MASK) {
                st7789vFlood(xBase + (5 * st7789vCharWidth) + 1,
                             st7789vCharHeight + 2,
                             st7789vCharWidth - 2,
                             st7789vCharHeight - 4, ST7789V_GREEN);
                whenStarted[i] = now ? now : ~0UL;
            }
            if (diff & EVG_STATUS_MAP_1_ACTIVE) {
                st7789vSetCharacterRGB(ST7789V_WHITE, ST7789V_BLACK);
                st7789vDrawChar(xBase + (5 * st7789vCharWidth),
                                2 * st7789vCharHeight,
                                (status & EVG_STATUS_MAP_1_ACTIVE) ? '1' : '0');
            }
        }
        if (whenStarted[i] && ((now - whenStarted[i]) > 350000)) {
            whenStarted[i] = 0;
            st7789vFlood(xBase + (5 * st7789vCharWidth) + 1,
                         st7789vCharHeight + 2,
                         st7789vCharWidth - 2,
                         st7789vCharHeight - 4, ST7789V_BLACK);
        }
    }
}

static void
drawHardwareTriggers(int redrawAll)
{
    int i;
    int xBase;
    int xInc = DISPLAY_WIDTH / 2;
    int yBase = 3 * st7789vCharHeight;
    int mask = ~(~0U << CFG_HARDWARE_TRIGGER_COUNT);
    const int width = 8;
    static int8_t oldStatus[EVG_COUNT];

    if (redrawAll) {
        st7789vSetCharacterRGB(ST7789V_WHITE, ST7789V_BLACK);
        for (i = 0, xBase = 0 ; i < EVG_COUNT ; i++, xBase += xInc) {
            char cbuf[10];
            sprintf(cbuf, "HW[1-%d]:", CFG_HARDWARE_TRIGGER_COUNT);
            st7789vShowString(xBase, yBase, cbuf);
            st7789vFlood(xBase + (8*st7789vCharWidth) + (st7789vCharWidth/2),
                         yBase + 1,
                         (CFG_HARDWARE_TRIGGER_COUNT * width) + 1,
                         st7789vCharHeight - 2, ST7789V_WHITE);
        }
    }
    for (i = 0, xBase = 0 ; i < EVG_COUNT ; i++, xBase += xInc) {
        int status = (GPIO_READ(i ? GPIO_IDX_EVG_2_HW_CSR :
                                    GPIO_IDX_EVG_1_HW_CSR)
                                           >> FMCIO_CSR_HW_STATUS_SHIFT) & mask;
        int diff = redrawAll ? mask : (status ^ oldStatus[i]);
        if (diff) {
            int x = xBase + (8 * st7789vCharWidth) + (st7789vCharWidth / 2) + 1;
            int b = 0x1;
            oldStatus[i] = status;
            while (diff) {
                if (diff & b) {
                    st7789vFlood(x, yBase + 2, width - 1, st7789vCharHeight - 4,
                                    status & b ? ST7789V_GREEN : ST7789V_BLACK);
                    diff &= ~b;
                }
                b <<= 1;
                x += width;
            }
        }
    }
}

static void
drawReferenceFrequencies(int redrawAll)
{
    int i;
    int xBase;
    int yBase = 4 * st7789vCharHeight;
    int xInc = DISPLAY_WIDTH / 2;
    static int displayFrequency[EVG_COUNT];

    for (i = 0, xBase = 0 ; i < EVG_COUNT ; i++, xBase += xInc) {
        int f = sharedMemory->referenceFrequency[i];
        if (f > 999999999) f = 999999999;
        if (redrawAll) {
            st7789vSetCharacterRGB(ST7789V_WHITE, ST7789V_BLACK);
            st7789vDrawChar(xBase+(0*st7789vCharWidth), yBase, 'F');
            st7789vShowString(xBase+(1*st7789vCharWidth), yBase+4, "in");
            st7789vDrawChar(xBase+(3*st7789vCharWidth), yBase, ':');
        }
        if (redrawAll || (displayFrequency[i] != f)) {
            char cbuf[20];
            st7789vSetCharacterRGB(ST7789V_WHITE, ST7789V_BLACK);
            displayFrequency[i] = f;
            sprintf(cbuf, "%3d.%06d", f / 1000000, f % 1000000);
            st7789vShowString(xBase + (9*st7789vCharWidth/2), yBase, cbuf);
        }
    }
}

static void
drawAlignmentStatus(int redrawAll)
{
    int xBase = 7 * st7789vCharWidth;
    int yBase = 5 * st7789vCharHeight;
    int isAligned = sharedMemory->isAligned;
    static int wasAligned = -1;
    if (redrawAll || (isAligned != wasAligned)) {
        if (isAligned) {
            st7789vSetCharacterRGB(ST7789V_GREEN, ST7789V_BLACK);
            st7789vShowString(xBase, yBase, " ALIGNED  ");
        }
        else {
            st7789vSetCharacterRGB(ST7789V_RED, ST7789V_BLACK);
            st7789vShowString(xBase, yBase, "MISALIGNED");
        }
    }
}

static void
drawTimingMarkers(int redrawAll)
{
    int i;
    const int xBase = 0, yBase = 9 * st7789vCharHeight;
    static uint8_t wasPPSvalid, was60HzValid;

    if (redrawAll) {
        st7789vSetCharacterRGB(ST7789V_WHITE, ST7789V_BLACK);
        st7789vShowString(xBase+st7789vCharWidth, yBase, "Markers");
        st7789vShowString(xBase+st7789vCharWidth, yBase+1*st7789vCharHeight,
                                                                       "1 Hz:");
        st7789vShowString(xBase                 , yBase+2*st7789vCharHeight,
                                                                      "60 Hz:");
    }
    i =((GPIO_READ(GPIO_IDX_NTP_SERVER_STATUS)&NTP_SERVER_STATUS_PPS_VALID)!=0);
    if (redrawAll || (wasPPSvalid != i)) {
        wasPPSvalid = i;
        st7789vFlood(xBase + 7*st7789vCharWidth + 1,
                     yBase + 1*st7789vCharHeight + 2,
                     st7789vCharWidth - 2,
                     st7789vCharHeight - 4, i ? ST7789V_GREEN : ST7789V_BLACK);
    }
    i = injectionCycleIsPowerLineValid();
    if (redrawAll || (was60HzValid != i)) {
        was60HzValid = i;
        st7789vFlood(xBase + 7*st7789vCharWidth + 1,
                     yBase + 2*st7789vCharHeight + 2,
                     st7789vCharWidth - 2,
                     st7789vCharHeight - 4, i ? ST7789V_GREEN : ST7789V_BLACK);
    }
}

static void
drawTemperatures(int redrawAll)
{
    int i;
    const int xBarBase = DISPLAY_WIDTH - 100 - st7789vCharWidth;
    const int xLabelBase = xBarBase - ((9 * st7789vCharWidth) / 2);
    static int oldColors[5];
    static int oldTemperatures[sizeof oldColors / sizeof oldColors[0]];
    const int yLabelBase = DISPLAY_HEIGHT -
           ((4 + (sizeof oldColors / sizeof oldColors[0])) * st7789vCharHeight);
    const int yBarSize = 7;
    const int yBarBase = yLabelBase + (2 * st7789vCharHeight) + 4;
    const int yellowThreshold = 60;
    const int redThreshold = 68;
    const int hysteresis = 2;
    static uint32_t whenDrawn = 0;
    uint32_t now = MICROSECONDS_SINCE_BOOT();

    if (redrawAll) {
        st7789vSetCharacterRGB(ST7789V_WHITE, ST7789V_BLACK);
        st7789vShowString(DISPLAY_WIDTH-(15*st7789vCharWidth),
                          yLabelBase, "Temperature (C)");
        st7789vShowString(xBarBase+1-(st7789vCharWidth/2),
                          yLabelBase+1*st7789vCharHeight, "0");
        st7789vShowString(xBarBase+51-st7789vCharWidth,
                          yLabelBase+1*st7789vCharHeight, "50");
        st7789vShowString(DISPLAY_WIDTH-(3*st7789vCharWidth),
                          yLabelBase+1*st7789vCharHeight, "100");
        st7789vShowString(xLabelBase, yLabelBase+2*st7789vCharHeight, "FPGA");
        st7789vShowString(xLabelBase, yLabelBase+3*st7789vCharHeight, "1 Tx");
        st7789vShowString(xLabelBase, yLabelBase+4*st7789vCharHeight, "1 Rx");
        st7789vShowString(xLabelBase, yLabelBase+5*st7789vCharHeight, "2 Tx");
        st7789vShowString(xLabelBase, yLabelBase+6*st7789vCharHeight, "2 Rx");
        for (i = 0 ; i < 5 ; i++) {
            int x = xBarBase + 25 * i;
            int y = yLabelBase + (2 * st7789vCharHeight) - 2;
            st7789vFlood(x, y, 1, 5, ST7789V_WHITE);
        }
        for (i = 0 ; i < (sizeof oldColors / sizeof oldColors[0]) ; i++) {
            oldTemperatures[i] = 0;
        }
        whenDrawn = now - 2500000;
    }
    if ((now - whenDrawn) < 2000000) {
        return;
    }
    whenDrawn = now;
    for (i = 0 ; i < (sizeof oldColors / sizeof oldColors[0]) ; i++) {
        int color, temperature;
        int yBase = yBarBase + (i * st7789vCharHeight);
        switch(i) {
        case 0: temperature = (xadcGetFPGAtemp() + 5) / 10; break;
        case 1: temperature = evioWarmestTransmitter(0);    break;
        case 2: temperature = evioWarmestReceiver(0);       break;
        case 3: temperature = evioWarmestTransmitter(1);    break;
        case 4: temperature = evioWarmestReceiver(1);       break;
        default: temperature = 0; break;
        }
        color = oldColors[i];
        if (temperature > 100) temperature = 100;
        if (temperature == oldTemperatures[i]) {
            continue;
        }
        if (temperature > oldTemperatures[i]) {
            if (temperature > redThreshold) {
                color = ST7789V_RED;
            }
            else if (temperature > yellowThreshold) {
                color = ST7789V_YELLOW;
            }
            else {
                color = ST7789V_GREEN;
            }
            if (color != oldColors[i]) {
                oldTemperatures[i] = 0;
            }
            st7789vFlood(xBarBase + oldTemperatures[i],
                      yBase, temperature - oldTemperatures[i], yBarSize, color);
        }
        else {
            if (temperature < (yellowThreshold - hysteresis)) {
                color = ST7789V_GREEN;
            }
            else if (temperature < (redThreshold - hysteresis)) {
                color = ST7789V_YELLOW;
            }
            else {
                color = ST7789V_RED;
            }
            if (color != oldColors[i]) {
                st7789vFlood(xBarBase, yBase, temperature, yBarSize, color);
            }
            st7789vFlood(xBarBase + temperature, yBase,
                     oldTemperatures[i] - temperature, yBarSize, ST7789V_BLACK);
        }
        oldTemperatures[i] = temperature;
        oldColors[i] = color;
    }
}

void
displayShowNetworkAddress(void)
{
    if (sharedMemory->systemParameters.netConfig.isDefault) {
        st7789vSetCharacterRGB(ST7789V_WHITE, ST7789V_BLACK);
    }
    else {
        st7789vSetCharacterRGB(ST7789V_BLACK, ST7789V_WHITE);
    }
    st7789vShowString(0, DISPLAY_HEIGHT - (2 * st7789vCharHeight),
        formatIP((void *)&sharedMemory->systemParameters.netConfig.np.address));
}

static int
displayRefresh(int newMode)
{
    int buttonState = displaySwitchPressed();
    uint32_t secondsNow = GPIO_READ(GPIO_IDX_SECONDS_SINCE_BOOT);
    uint32_t microsecondsNow = GPIO_READ(GPIO_IDX_MICROSECONDS_SINCE_BOOT);
    static enum { pageFirst,
                  pageStatus,
                  pageLogEv1,
                  pageLogEv2,
                  pageLast } newPage = pageFirst, currentPage = pageLast;
    int isPressed, newPress;
    int redrawAll = 0;
    uint32_t pressedMicroseconds, releasedSeconds;
    static int buttonOldState;
    static int displayMode = DISPLAY_MODE_STARTUP;
    static int wasPressed, isBlanked;
    static uint32_t microsecondsWhenButtonStateChanged;
    static uint32_t secondsWhenButtonStateChanged;

    if (newMode == DISPLAY_MODE_FETCH) return displayMode;
    if (buttonState != buttonOldState) {
        microsecondsWhenButtonStateChanged = microsecondsNow;
        secondsWhenButtonStateChanged = secondsNow;
        buttonOldState = buttonState;
    }
    if ((microsecondsNow - microsecondsWhenButtonStateChanged) > 20000) {
        isPressed = buttonState;
        newPress = (isPressed && !wasPressed);
        wasPressed = isPressed;
    }
    else {
        newPress = 0;
        isPressed = wasPressed;
    }
    if (isPressed) {
        pressedMicroseconds = microsecondsNow - microsecondsWhenButtonStateChanged;
        releasedSeconds = 0;
    }
    else {
        releasedSeconds = secondsNow - secondsWhenButtonStateChanged;
        pressedMicroseconds = 0;
        if (releasedSeconds > DISPLAY_ENABLE_SECONDS) {
            st7789vBacklightEnable(0);
            isBlanked = 1;
        }
    }
    if ((displayMode == DISPLAY_MODE_STARTUP)
     || (pressedMicroseconds > 1000000)) {
        newMode = DISPLAY_MODE_PAGES;
        newPage = pageFirst;
    }
    if (debugFlags & DEBUGFLAG_DISPLAY_NEXT_PAGE) {
        debugFlags &= ~DEBUGFLAG_DISPLAY_NEXT_PAGE;
        if (displayMode != DISPLAY_MODE_PAGES) {
            newMode = DISPLAY_MODE_PAGES;
            newPage = pageFirst;
        }
        secondsWhenButtonStateChanged = secondsNow;
        newPress = 1;
    }
    if (newPage == pageFirst) {
        newPage++;
    }
    if (newMode >= 0) {
        if (displayMode != newMode) {
            displayMode = newMode;
            currentPage = pageLast;
            redrawAll = 1;
        }
    }
    if ((displayMode == DISPLAY_MODE_PAGES) && (newPage != currentPage)) {
        currentPage = newPage;
        redrawAll = 1;
    }
    if (redrawAll) {
        st7789vFlood(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT,
               displayMode == DISPLAY_MODE_FATAL ? ST7789V_RED : ST7789V_BLACK);
    }

    if (displayMode != DISPLAY_MODE_FATAL) {
        /*
         * Common to all non-fatal display modes
         */
        drawTime(redrawAll);
        if (redrawAll) {
            displayShowNetworkAddress();
        }

        /*
         * Mode-specific displays
         */
        switch (displayMode) {
        case DISPLAY_MODE_PAGES:
            switch(currentPage) {
            case pageStatus:
                drawEVGstatus(redrawAll);
                drawHardwareTriggers(redrawAll);
                drawAlignmentStatus(redrawAll);
                drawReferenceFrequencies(redrawAll);
                drawTimingMarkers(redrawAll);
                drawTemperatures(redrawAll);
                break;
            case pageLogEv1:
                drawEventLog(0, redrawAll);
                break;
            case pageLogEv2:
                drawEventLog(1, redrawAll);
                break;
            default: break;
            }
            break;

        case DISPLAY_MODE_WARNING:
            if (redrawAll) {
                displayShowWarning(NULL);
            }
            break;
        }
    }
    if (newPress) {
        if (isBlanked) {
            st7789vBacklightEnable(1);
            isBlanked = 0;
        }
        else {
            if (++newPage >= pageLast) {
                newPage = pageFirst;
            }
        }
    }
    return displayMode;
}

void
displayUpdate(void)
{
    displayRefresh(DISPLAY_MODE_UPDATE);
}

void
displaySetMode(int mode)
{
    displayRefresh(mode);
}

int
displayGetMode(void)
{
    return displayRefresh(DISPLAY_MODE_FETCH);
}

void
displayShowFatal(const char *msg)
{
    displaySetMode(DISPLAY_MODE_FATAL);
    st7789vShowText(4, 4, DISPLAY_WIDTH - 4, DISPLAY_HEIGHT - 4,
                                               ST7789V_BLACK, ST7789V_RED, msg);
}

void
displayShowWarning(const char *msg)
{
    int y;
    static char cbuf[400];
    static unsigned short msgs[13];
    static int charIndex, msgIndex, yNext;
    static int displayYlast, displayYsize;

    if (displayYsize == 0) {
        displayYsize = DISPLAY_HEIGHT - (2 * st7789vCharHeight);
        displayYlast = displayYsize - st7789vCharHeight;
    }
    if (msg) {
        if (displayGetMode() != DISPLAY_MODE_WARNING) {
            displaySetMode(DISPLAY_MODE_WARNING);
        }
        if ((charIndex >= sizeof(cbuf))
         || (msgIndex >= (sizeof msgs / sizeof msgs[0]))
         || (yNext > displayYlast)) {
            st7789vFlood(2, 0, DISPLAY_WIDTH - 2, displayYsize, ST7789V_YELLOW);
            charIndex = 0;
            msgIndex = 0;
            yNext = 0;
        }
        if ((charIndex < sizeof(cbuf))
         && (msgIndex < (sizeof msgs / sizeof msgs[0]))
         && (yNext <= displayYlast)) {
            int spaceRemaining = sizeof(cbuf) - charIndex;
            int len = strlen(msg) + 1;
            if (len > spaceRemaining) len = spaceRemaining;
            memcpy(&cbuf[charIndex], msg, len);
            msgs[msgIndex++] = charIndex;
            charIndex += len;
            cbuf[charIndex - 1] = '\0';
            yNext = st7789vShowText(2, yNext,
                       DISPLAY_WIDTH - 2, displayYsize - yNext,
                       ST7789V_BLACK, ST7789V_YELLOW, msg);
        }
        return;
    }
    st7789vFlood(0, 0, 2, displayYsize, ST7789V_YELLOW);
    y = 0;
    for (int m = 0 ; m < msgIndex ; m++) {
        y = st7789vShowText(2, y, DISPLAY_WIDTH - 2, displayYsize - y,
                                 ST7789V_BLACK, ST7789V_YELLOW, &cbuf[msgs[m]]);
        if (y >= yNext) break;
    }
    if (y < displayYsize)  {
        st7789vFlood(2, y, DISPLAY_WIDTH - 2, displayYsize - y, ST7789V_YELLOW);
    }
}

void
displayShowStartup(const char *msg)
{
    static int yBase;
    if ((displayRefresh(DISPLAY_MODE_FETCH) == DISPLAY_MODE_STARTUP)
     && ((yBase + st7789vCharHeight) <= DISPLAY_HEIGHT)) {
        if (yBase == 0) {
            st7789vFlood(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, ST7789V_BLACK);
        }
        st7789vShowString(0, yBase, msg);
        yBase += st7789vCharHeight;
    }
    printf("%s\n", msg);
}
