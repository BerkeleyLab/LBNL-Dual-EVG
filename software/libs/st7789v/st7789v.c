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
 * Communicate with Sitronix ST7789V display controller.
 */
#include <stdio.h>
#include <stdint.h>
#include "gpio.h"
#include "lv_font.h"
#include "st7789v.h"
#include "util.h"

#define CMD_RDDID       0x04
#define CMD_RDDST       0x09
#define CMD_RDDPM       0x0A
#define CMD_RDDMADCTL   0x0B
#define CMD_RDDCOLMOD   0x0C
#define CMD_RDDIM       0x0D
#define CMD_RDDSM       0x0E
#define CMD_RDDSRDDSDR  0x0F
#define CMD_SLPIN       0x10
#define CMD_SLPOUT      0x11
#define CMD_GAMSET      0x26
#define CMD_DISPON      0x29
#define CMD_CASET       0x2A
#define CMD_RASET       0x2B
#define CMD_RAMWR       0x2C
#define CMD_RAMRD       0x2E
#define CMD_MADCTL      0x36
#define CMD_COLMOD      0x3A
#define CMD_WRDISBV     0x51
#define CMD_RDDISBV     0x52
#define CMD_WRCTRLD     0x53
#define CMD_RDCTRLD     0x54
#define CMD_RDCABC      0x56
#define CMD_WRCABCMB    0x5E
#define CMD_RDCABCMB    0x5F
#define CMD_RDABCSDR    0x68
#define CMD_RDID1       0xDA
#define CMD_RDID2       0xDB
#define CMD_RDID3       0xDC

#define MADCTL_MY       0x80 /* Mirror Y */
#define MADCTL_MX       0x40 /* Mirror X */
#define MADCTL_MV       0x20 /* Exchange X/Y */

#define COL_COUNT   320
#define ROW_COUNT   240

#define CSR_RESET               0x80000000
#define CSR_W_READ              0x40000000
#define CSR_R_BUSY              0x40000000
#define CSR_W_READ_33           0x20000000
#define CSR_W_READ_32           0x10000000
#define CSR_W_READ_25           0x08000000
#define CSR_R_BACKLIGHT_ENABLED 0x02000000
#define CSR_W_BACKLIGHT_ENABLE  0x02000000
#define CSR_W_BACKLIGHT_DISABLE 0x01000000
#define CSR_R_ADDR_WIDTH_MASK   0xF80000
#define CSR_R_ADDR_WIDTH_SHIFT  19
#define CSR_R_FIFO_FULL         0x40000
#define CSR_R_FREE_SPACE_MASK   0x3FFFF

#define IO_BYTE 0x10000
#define IO_LAST 0x20000

#define READ_CSR()    GPIO_READ(GPIO_IDX_DISPLAY_CSR)
#define WRITE_CSR(v)  GPIO_WRITE(GPIO_IDX_DISPLAY_CSR, (v))
#define READ_DATA()   GPIO_READ(GPIO_IDX_DISPLAY_DATA)
#define WRITE_DATA(v)  GPIO_WRITE(GPIO_IDX_DISPLAY_DATA, (v))

extern const lv_font_t systemFont;
int st7789vCharWidth, st7789vCharHeight;
static int pixelsPerCharacter;

static void
writeData(uint32_t value)
{
    while ((READ_CSR() & CSR_R_FIFO_FULL)) continue;
    GPIO_WRITE(GPIO_IDX_DISPLAY_DATA, value);
}

void
st7789vInit(void)
{
    st7789vCharWidth = systemFont.monospace,
    st7789vCharHeight = systemFont.h_px;
    pixelsPerCharacter = st7789vCharWidth * st7789vCharHeight;
    WRITE_CSR(CSR_RESET | CSR_W_BACKLIGHT_DISABLE);
    microsecondSpin(100);
    WRITE_CSR(0);
    microsecondSpin(500000);
    st7789vCommand(CMD_SLPOUT);
    microsecondSpin(500000);
    st7789vWriteRegister(CMD_GAMSET, 0x8);      /* Emperically determined value*/
    st7789vWriteRegister(CMD_COLMOD, 0x65);     /* 16 bits per pixel */
    st7789vWriteRegister(CMD_WRCTRLD, 0x24);    /* Brightness en, backlight on */
    st7789vWriteRegister(CMD_WRCABCMB, 0xFF);   /* Full brightness */
    st7789vWriteRegister(CMD_WRDISBV, 0xFF);    /* Full brightness */
    st7789vWriteRegister(CMD_MADCTL, MADCTL_MY | MADCTL_MV);
    st7789vFlood(0, 0, COL_COUNT, ROW_COUNT, 0);
    st7789vCommand(CMD_DISPON);
    st7789vAwaitCompletion();
    st7789vBacklightEnable(1);
}

void
st7789vAwaitCompletion(void)
{
    int aw = (READ_CSR() & CSR_R_ADDR_WIDTH_MASK) >> CSR_R_ADDR_WIDTH_SHIFT;
    int capacity = (1 << aw) - 1;
    while ((READ_CSR() & CSR_R_FREE_SPACE_MASK) < capacity) continue;
}

void
st7789vBacklightEnable(int enable)
{
    if (enable) {
        WRITE_CSR(CSR_W_BACKLIGHT_ENABLE);
    }
    else {
        WRITE_CSR(CSR_W_BACKLIGHT_DISABLE);
    }
}

void
st7789vShow(void)
{
    int i;

    static const struct {
        char   *name;
        uint8_t r;
      } regTable[] = {
            { "RDDST",          CMD_RDDST       },
            { "RDDPM",          CMD_RDDPM       },
            { "RDDMADCTL",      CMD_RDDMADCTL   },
            { "RDDCOLMOD",      CMD_RDDCOLMOD   },
            { "RDDIM",          CMD_RDDIM       },
            { "RDDSM",          CMD_RDDSM       },
            { "RDDSRDDSDR",     CMD_RDDSRDDSDR  },
            { "RDDISBV",        CMD_RDDISBV     },
            { "RDCTRLD",        CMD_RDCTRLD     },
            { "RDCABC",         CMD_RDCABC      },
            { "RDCABCMB",       CMD_RDCABCMB    },
            { "RDABCSDR",       CMD_RDABCSDR    },
            { "RDID1",          CMD_RDID1       },
            { "RDID2",          CMD_RDID2       },
            { "RDID3",          CMD_RDID3       },
        };
    printf("ST7789V Display Controller:\n");
    for (i = 0 ; i < sizeof regTable / sizeof regTable[0] ; i++) {
        int r = regTable[i].r;
        printf("    %02x%13s: %02x\n", r, regTable[i].name, st7789vReadRegister(r));
    }
}

int
st7789vReadRegister(int r)
{
    uint32_t flag, mask;
    r &= 0xFF;
    switch(r) {
    case CMD_RDDST: flag = CSR_W_READ_33; mask = 0xFFFFFFFF; break;
    case CMD_RAMRD: flag = CSR_W_READ_32; mask = 0xFFFFFF;   break;
    case CMD_RDDID: flag = CSR_W_READ_25; mask = 0xFFFFFF;   break;
    default:        flag = 0;             mask = 0xFF;       break;
    }
    while (READ_CSR() & CSR_R_BUSY) continue;
    WRITE_CSR(CSR_W_READ | flag | r);
    while (READ_CSR() & CSR_R_BUSY) continue;
    return READ_DATA() & mask;
}

void
st7789vCommand(int c)
{
    writeData(IO_LAST | IO_BYTE | ((c & 0xFF) << 8));
}

void
st7789vWriteRegister(int r, int v)
{
    writeData(IO_LAST | ((r & 0xFF) << 8) | (v & 0xFF));
}

/*
 * Fill a rectangle with left upper corner at (xs, ys)
 */
void
st7789vFlood(int xs, int ys, int width, int height, int value)
{
    int n = width * height;
    int xe = xs + width - 1;
    int ye = ys + height - 1;
    int nfree = READ_CSR() & CSR_R_FREE_SPACE_MASK;

    value &= 0xFFFF;
    if (nfree >= (n + 7)) {
        WRITE_DATA(IO_BYTE | (CMD_CASET << 8));
        WRITE_DATA(xs & 0xFFFF);
        WRITE_DATA(IO_LAST | (xe & 0xFFFF));
        WRITE_DATA(IO_BYTE | (CMD_RASET << 8));
        WRITE_DATA(ys & 0xFFFF);
        WRITE_DATA(IO_LAST | (ye & 0xFFFF));
        WRITE_DATA(IO_BYTE | (CMD_RAMWR << 8));
        while (--n) {
            WRITE_DATA(value);
        }
        WRITE_DATA(IO_LAST | value);
    }
    else {
        writeData(IO_BYTE | (CMD_CASET << 8));
        writeData(xs & 0xFFFF);
        writeData(IO_LAST | (xe & 0xFFFF));
        writeData(IO_BYTE | (CMD_RASET << 8));
        writeData(ys & 0xFFFF);
        writeData(IO_LAST | (ye & 0xFFFF));
        writeData(IO_BYTE | (CMD_RAMWR << 8));
        while (--n) {
            writeData(value);
        }
        writeData(IO_LAST | value);
    }
}

void
st7789vDrawRectangle(int xs, int ys, int width, int height, const uint16_t *data)
{
    int n = width * height;
    int xe = xs + width - 1;
    int ye = ys + height - 1;
    int nfree = READ_CSR() & CSR_R_FREE_SPACE_MASK;
    const uint16_t *last = data + n - 1;

    if (nfree >= (n + 7)) {
        WRITE_DATA(IO_BYTE | (CMD_CASET << 8));
        WRITE_DATA(xs & 0xFFFF);
        WRITE_DATA(IO_LAST | ( xe & 0xFFFF));
        WRITE_DATA(IO_BYTE | (CMD_RASET << 8));
        WRITE_DATA(ys & 0xFFFF);
        WRITE_DATA(IO_LAST | (ye & 0xFFFF));
        WRITE_DATA(IO_BYTE | (CMD_RAMWR << 8));
        while (data != last) {
            WRITE_DATA(*data++);
        }
        WRITE_DATA(IO_LAST | *data);
    }
    else {
        writeData(IO_BYTE | (CMD_CASET << 8));
        writeData(xs & 0xFFFF);
        writeData(IO_LAST | ( xe & 0xFFFF));
        writeData(IO_BYTE | (CMD_RASET << 8));
        writeData(ys & 0xFFFF);
        writeData(IO_LAST | (ye & 0xFFFF));
        writeData(IO_BYTE | (CMD_RAMWR << 8));
        while (data != last) {
            writeData(*data++);
        }
        writeData(IO_LAST | *data);
    }
}

int
st7789vColorAtPixel(int x, int y)
{
    uint32_t v;
    x &= 0xFFFF;
    y &= 0xFFFF;
    /* Must read back in 18 bit per pixel mode */
    st7789vWriteRegister(CMD_COLMOD, 0x66);
    writeData(IO_BYTE | (CMD_CASET << 8));
    writeData(x);
    writeData(IO_LAST | x);
    writeData(IO_BYTE | (CMD_RASET << 8));
    writeData(y);
    writeData(IO_LAST | y);
    v = st7789vReadRegister(CMD_RAMRD);
    /* Restore 16 bit per pixel mode */
    st7789vWriteRegister(CMD_COLMOD, 0x65);
    return v;
}

void
st7789vTestPattern(void)
{
    int ix, iy, xs, ys;
    static const uint16_t v[] = { 0x8000,  0x0400, 0x0010, 0x8410 };
    const int xStep = COL_COUNT / 5;
    const int yStep = ROW_COUNT / (sizeof v / sizeof v[0]);

    for (iy = 0, ys = 0 ; iy < sizeof v / sizeof v[0] ; iy++, ys += yStep) {
        int b = v[iy];
        for (ix = 0, xs = 0 ; ix < 5 ; ix++, xs += xStep, b >>= 1) {
            st7789vFlood(xs, ys, xStep, yStep, b);
        }
    }
}

/*
 * Text display
 */
#define FONT_MAX_BPP 4
static uint16_t intensities[1<<FONT_MAX_BPP];
void
st7789vSetCharacterRGB(int foreground, int background)
{
    int charForegroundComponent[3], charBackgroundComponent[3];
    int mask = (1 << systemFont.bpp) - 1;
    int intensity;
    static int oldForeground, oldBackground;
    if ((foreground == oldForeground) && (background == oldBackground)) {
        return;
    }
    oldForeground = foreground;
    oldBackground = background;
    if (systemFont.bpp > FONT_MAX_BPP) {
        printf("%d bits per pixel!\n", systemFont.bpp);
    }
    charForegroundComponent[0] = (foreground >> 11) & 0x1F;
    charForegroundComponent[1] = (foreground >> 6) & 0x1F;
    charForegroundComponent[2] = foreground & 0x1F;
    charBackgroundComponent[0] = (background >> 11) & 0x1F;
    charBackgroundComponent[1] = (background >> 6) & 0x1F;
    charBackgroundComponent[2] = background & 0x1F;
    for (intensity = 0 ; intensity <= mask ; intensity++) {
        int i, d[3];
        for (i = 0 ; i < 3 ; i++) {
            int f = charForegroundComponent[i];
            int b = charBackgroundComponent[i];
            d[i] = (((f - b) * intensity) / mask) + b;
            if (d[i] > 31) d[i] = 31;
            if (d[i] < 0) d[i] = 0;
        }
        intensities[intensity] = (d[0] << 11) | (d[1] << 6) | d[2];
    }
}

/*
 * Draw a character with left upper corner at (x, y)
 */
void
st7789vDrawChar(int x, int y, int c)
{
    int mask = (1 << systemFont.bpp) - 1;
    int rowsRemaining, width, leftPad, rightPad;
    int i, shift, intensity;
    const lv_font_glyph_dsc_t *dsc;
    const uint8_t *glyph;
    if ((c < systemFont.unicode_first)
    || (c > systemFont.unicode_last)
    || ((dsc = &systemFont.glyph_dsc[c-systemFont.unicode_first])->w_px == 0)) {
        glyph = NULL;
    }
    else {
        width = dsc->w_px;
        glyph = systemFont.glyph_bitmap + dsc->glyph_index;
        leftPad = (systemFont.monospace - width) / 2;
        rightPad = systemFont.monospace - (leftPad + width);
    }

    /*
     * Set display range
     */
    while ((READ_CSR() & CSR_R_FREE_SPACE_MASK) < (pixelsPerCharacter+7)) {
        continue;
    }
    WRITE_DATA(IO_BYTE | (CMD_CASET << 8));
    WRITE_DATA(x & 0xFFFF);
    WRITE_DATA(IO_LAST | ((x + systemFont.monospace - 1) & 0xFFFF));
    WRITE_DATA(IO_BYTE | (CMD_RASET << 8));
    WRITE_DATA(y & 0xFFFF);
    WRITE_DATA(IO_LAST | ((y + systemFont.h_px - 1) & 0xFFFF));
    WRITE_DATA(IO_BYTE | (CMD_RAMWR << 8));

    /*
     * Handle non-printable
     */
    if (glyph == NULL) {
        i = pixelsPerCharacter;
        while (--i >= 0) {
            WRITE_DATA(intensities[0]);
        }
        WRITE_DATA(IO_LAST | intensities[0]);
        return;
    }
    rowsRemaining = systemFont.h_px;
    for (;;) {
        shift = 8 - systemFont.bpp;
        i = leftPad;
        while (--i >= 0) {
            WRITE_DATA(intensities[0]);
        }
        i = width;
        for (;;) {
            intensity = intensities[(*glyph >> shift) & mask];
            if (--i <= 0) {
                break;
            }
            WRITE_DATA(intensity);
            shift -= systemFont.bpp;
            if (shift < 0) {
                glyph++;
                shift = 8 - systemFont.bpp;
            }
        }
        if (--rowsRemaining == 0) {
            if (rightPad == 0) {
                WRITE_DATA(IO_LAST | intensity);
            }
            else {
                WRITE_DATA(intensity);
                i = rightPad;
                while (--i > 0) {
                    WRITE_DATA(intensities[0]);
                }
                WRITE_DATA(IO_LAST | intensities[0]);
            }
            return;
        }
        WRITE_DATA(intensity);
        i = rightPad;
        while (--i >= 0) {
            WRITE_DATA(intensities[0]);
        }
        glyph++;
    }
}

void
st7789vShowString(int xBase, int yBase, const char *str)
{
    while (*str && (xBase <= (COL_COUNT - st7789vCharWidth))) {
        st7789vDrawChar(xBase, yBase, *str);
        str++;
        xBase += st7789vCharWidth;
    }
}

int
st7789vShowText(int xBase, int yBase, int width, int height,
                int foreground, int background, const char *str)
{
    int x, y;
    int newline;

    x = xBase;
    y = yBase;
    if ((width < st7789vCharWidth) || (height < st7789vCharHeight)) return y;
    st7789vSetCharacterRGB(foreground, background);
    for (;;) {
        /* Clear remainder of line */
        if ((x > xBase) && (x < (xBase + width))) {
            st7789vFlood(x, y - st7789vCharHeight,
                            (xBase + width) - x, st7789vCharHeight, background);
        }
        x = xBase;

        /* Remove blank lines and leading space */
        while ((*str == ' ') || (*str == '\n')) str++;
        if ((*str == '\0') || ((y + st7789vCharHeight) > (yBase + height))) break;

        /* Emit until the first space */
        while ((*str != '\0') && (*str != ' ') && (*str != '\n')) {
            st7789vDrawChar(x, y, *str);
            str++;
            x += st7789vCharWidth;
            if ((x + st7789vCharWidth) > (xBase + width)) {
                break;
            }
        }
        if ((*str == '\n') || (*str != ' ')) {
            y += st7789vCharHeight;
            continue;
        }

        /* Emit text that fits */
        newline = 0;
        for (;;) {
            int i = 0;
            while (str[i] == ' ') {
                if ((x + (i * st7789vCharWidth)) > (xBase + width)) {
                    newline = 1;
                    break;
                }
                i++;
            }
            if (newline) break;
            while (str[i] != ' ') {
                if (str[i] == '\0') {
                    break;
                }
                if ((x + (i * st7789vCharWidth)) > (xBase + width)) {
                    newline = 1;
                    break;
                }
                i++;
            }
            if (newline) break;
            while (i--) {
                st7789vDrawChar(x, y, *str);
                str++;
                x += st7789vCharWidth;
            }
            if ((*str == '\0') || (*str == '\n')) {
                break;
            }
        }
        y += st7789vCharHeight;
    }
    if (y < (yBase + height)) {
        st7789vFlood(xBase, y, width, yBase + height - y, background);
    }
    return y;
}


/*
 * Produce NetPBM PPM dump of screen contents
 * Warning -- Takes a long time!
 */
void
st7789vDumpScreen(void)
{
    int x, y;
    printf("P3\n%d %d 255\n", COL_COUNT, ROW_COUNT);
    for (y = 0 ; y < ROW_COUNT ; y++) {
        for (x = 0 ; x < COL_COUNT ; x++) {
            int v = st7789vColorAtPixel(x, y);
            printf("%d %d %d\n", (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF);
        }
    }
}

#ifdef ST7789_GRAB_SCREEN
/*
 * Produce file of NETBPM portable pixmap screen image
 */
#ifdef __MICROBLAZE__
#include "ff.h"
#else
#include "ffs.h"
#endif
static int
writeImage(FIL *fp)
{
    int r, c;

    for (r = 0 ; r < ROW_COUNT ; r++) {
        for (c = 0 ; c < COL_COUNT ; c++) {
            int v = st7789vColorAtPixel(c, r);
            UINT nWritten;
            char cbuf[3];
            cbuf[0] = v >> 16;
            cbuf[1] = v >> 8;
            cbuf[2] = v;
            if ((f_write(fp, cbuf, 3, &nWritten) != FR_OK) || (nWritten != 3)) {
                return 0;
            }
        }
    }
    return 1;
}

int
st7789vGrabScreen(void)
{
    FRESULT fr;
    FIL fil;
    UINT nWritten;
    int l;
    int ret = 1;
    char cbuf[20];

    fr = f_open(&fil, "/SCREEN.ppm", FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        return 0;
    }
    l = sprintf(cbuf, "P6\n%d %d 255\n", COL_COUNT, ROW_COUNT);
    if ((f_write(&fil, cbuf, l, &nWritten) != FR_OK) || (nWritten != l)
     || !writeImage(&fil)) {
        ret = 0;
    }
    if (f_close(&fil) != FR_OK) {
        ret = 0;
    }
    return ret;
}
#endif
