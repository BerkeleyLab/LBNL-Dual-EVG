/*
 * Show events as they occur
 */

#include <stdio.h>
#include <stdint.h>
#include "display.h"
#include "drawEventLog.h"
#include "evio.h"
#include "gpio.h"
#include "st7789v.h"
#include "util.h"

#define DISPLAY_USEC    700000

#define MAX_EVENT_CODE      0x7E
#define EVCODES_PER_LINE    10
#define CHARS_PER_EVCODE    3
#define EVCODE_HB           122
#define EVCODE_C0(x)        ((int)((x%10)+48))
#define EVCODE_C1(x)        ((int)((x/10)%10+48))
#define EVCODE_C2(x)        ((int)((x/100)%100+48))


#define CSR_R_EVCODE_MASK   0xFF
#define CSR_R_FIFO_EMPTY    0x100
#define CSR_W_READ_FIFO     0x100
#define CSR_RW_RESET        0x200

struct evInfo {
    struct evInfo *forw;
    struct evInfo *back;
    int16_t        xBase;
    int16_t        yBase;
    uint8_t        c0;
    uint8_t        c1;
    uint8_t        c2;
    uint8_t        floodWidth;
    uint32_t       whenOn;
};

void
drawEventLog(int evgIdx, int redrawAll)
{
    uint32_t csr;
    int evCode;
    uint32_t now;
    struct evInfo *evp;
    static struct evInfo evTable[MAX_EVENT_CODE];
    static struct evInfo *displayHead, *displayTail;
    static int beenHere;
    static int csrIdx;

    if (!beenHere) {
        /* Initialize table */
        beenHere = 1;
        for (evCode = 1 ; evCode <= MAX_EVENT_CODE ; evCode++) {
            evp = &evTable[evCode-1];
            evp->xBase = (evCode % EVCODES_PER_LINE) * CHARS_PER_EVCODE *
                                                               st7789vCharWidth;
            evp->yBase = (evCode / EVCODES_PER_LINE) * st7789vCharHeight;
            if (evCode < 10) {
                evp->xBase += st7789vCharWidth;
                evp->floodWidth = st7789vCharWidth;
            }
            else if (evCode < 100) {
                evp->xBase += st7789vCharWidth / 2;
                evp->floodWidth = 2 * st7789vCharWidth;
            }
            else {
                evp->floodWidth = 3 * st7789vCharWidth;
            }
            evp->c0 = (evCode % 10) + '0';
            if (evCode >= 10) evp->c1 = ((evCode / 10) % 10) + '0';
            if (evCode >= 100) evp->c2 = ((evCode / 100) % 10) + '0';
        }
    }
    if (redrawAll) {
        int lines = (MAX_EVENT_CODE + EVCODES_PER_LINE - 1) / EVCODES_PER_LINE;
        st7789vFlood(0, 0, EVCODES_PER_LINE*st7789vCharWidth,
                                        lines*st7789vCharHeight, ST7789V_BLACK);
        st7789vSetCharacterRGB(ST7789V_BLACK, ST7789V_WHITE);
        st7789vShowString(0, 0, "EVG");
        st7789vDrawChar(3*st7789vCharWidth, 0, '1' + evgIdx);
        st7789vSetCharacterRGB(ST7789V_WHITE, ST7789V_BLACK);
        while (displayHead) {
            displayHead->whenOn = 0;
            displayHead = displayHead->forw;
        }
        displayTail = NULL;
        csrIdx = evgIdx ? GPIO_IDX_EVG_2_LOG_CSR : GPIO_IDX_EVG_1_LOG_CSR;
        GPIO_WRITE(csrIdx, CSR_RW_RESET);
        microsecondSpin(1);
        GPIO_WRITE(csrIdx, 0);
    }
    if (!csrIdx) return;
    while (!((csr = GPIO_READ(csrIdx)) & CSR_R_FIFO_EMPTY)) {
        GPIO_WRITE(csrIdx, CSR_W_READ_FIFO);
        evCode = csr & CSR_R_EVCODE_MASK;
        if ((evCode <= 0) || (evCode > MAX_EVENT_CODE)) {
            continue;
        }
        evp = &evTable[evCode-1];
        now = MICROSECONDS_SINCE_BOOT();
        if (now == 0) now = 1;
        if (evp->whenOn) {
            /* Already on display (and list) so remove from list */
            if (evp->forw) {
                evp->forw->back = evp->back;
            }
            else {
                displayTail = evp->back;
            }
            if (evp->back) {
                evp->back->forw = evp->forw;
            }
            else {
                displayHead = evp->forw;
            }
        }
        else {
            /* Display event code */
            int xBase = evp->xBase;
            if (evp->c2) {
                st7789vDrawChar(xBase, evp->yBase, evp->c2);
                xBase += st7789vCharWidth;
            }
            if (evp->c1) {
                st7789vDrawChar(xBase, evp->yBase, evp->c1);
                xBase += st7789vCharWidth;
            }
            st7789vDrawChar(xBase, evp->yBase, evp->c0);
        }
        /* Hearbeat icon flashing */
        if (evCode == EVCODE_HB) {
            drawHeartbeatIndicator(evgIdx, 1);
        }
        /* Link on to tail */
        evp->forw = NULL;
        if (displayHead == NULL) {
            displayHead = evp;
        }
        evp->back = displayTail;
        if (displayTail) {
            displayTail->forw = evp;
        }
        displayTail = evp;
        evp->whenOn = now;
    }
    /* Remove stale events from display and from list */
    now = MICROSECONDS_SINCE_BOOT();
    while (displayHead && ((int32_t)(now-displayHead->whenOn) > DISPLAY_USEC)) {
        st7789vFlood(displayHead->xBase, displayHead->yBase,
                     displayHead->floodWidth, st7789vCharHeight, ST7789V_BLACK);
        if(displayHead->c2 == EVCODE_C2(EVCODE_HB) &&
                displayHead->c1 == EVCODE_C1(EVCODE_HB) &&
                displayHead->c0 == EVCODE_C0(EVCODE_HB)) {
            drawHeartbeatIndicator(evgIdx, 0);
        }
        displayHead->whenOn = 0;
        displayHead = displayHead->forw;
        if (displayHead) {
            displayHead->back = NULL;
        }
        else {
            displayTail = NULL;
        }
        now = MICROSECONDS_SINCE_BOOT();
    }
}
