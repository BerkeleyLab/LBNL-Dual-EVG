/*
 * Copyright 2022, Lawrence Berkeley National Laboratory
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
 * Stubs for ST7789V display controller API
 */

#include <stdio.h>
#include <stdint.h>

int st7789vCharWidth, st7789vCharHeight;
static int pixelsPerCharacter;

void
st7789vInit(void)
{
    st7789vCharWidth = 0,
    st7789vCharHeight = 0;
    pixelsPerCharacter = st7789vCharWidth * st7789vCharHeight;
}

void
st7789vAwaitCompletion(void)
{}

void
st7789vBacklightEnable(int enable)
{}

void
st7789vShow(void)
{}

int
st7789vReadRegister(int r)
{
    return 0;
}

void
st7789vCommand(int c)
{}

void
st7789vWriteRegister(int r, int v)
{}

void
st7789vFlood(int xs, int ys, int width, int height, int value)
{}

void
st7789vDrawRectangle(int xs, int ys, int width, int height, const uint16_t *data)
{}

int
st7789vColorAtPixel(int x, int y)
{
    return 0;
}

void
st7789vTestPattern(void)
{}

/*
 * Text display
 */
#define FONT_MAX_BPP 4
static uint16_t intensities[1<<FONT_MAX_BPP];
void
st7789vSetCharacterRGB(int foreground, int background)
{}

/*
 * Draw a character with left upper corner at (x, y)
 */
void
st7789vDrawChar(int x, int y, int c)
{}

void
st7789vShowString(int xBase, int yBase, const char *str)
{}

int
st7789vShowText(int xBase, int yBase, int width, int height,
                int foreground, int background, const char *str)
{
    return 0;
}

void
st7789vDumpScreen(void)
{}

#ifdef ST7789_GRAB_SCREEN
#include "ffs.h"
static int
writeImage(FIL *fp)
{
    return 0;
}

int
st7789vGrabScreen(void)
{
    return 0;
}
#endif
