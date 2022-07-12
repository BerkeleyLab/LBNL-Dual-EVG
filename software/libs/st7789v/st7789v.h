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
 * Communicate with Sitronix ST7789V display controller
 */
#ifndef _ST7789V_H_
#define _ST7789V_H_

/*
 * Documentation claims R/G/B 5/6/5 but setting the least signficant bit
 * of the green component affects all three components so instead limit
 * palette to 5/5/5.
 */
#define ST7789V_COLOUR(r,g,b) (((r)<<11)|((g)<<6)|(b))

#define ST7789V_RED     ST7789V_COLOUR(31,0,0)
#define ST7789V_GREEN   ST7789V_COLOUR(0,31,0)
#define ST7789V_BLUE    ST7789V_COLOUR(0,0,31)
#define ST7789V_CYAN    ST7789V_COLOUR(0,31,31)
#define ST7789V_MAGENTA ST7789V_COLOUR(31,0,31)
#define ST7789V_YELLOW  ST7789V_COLOUR(31,31,0)
#define ST7789V_BLACK   ST7789V_COLOUR(0,0,0)
#define ST7789V_WHITE   ST7789V_COLOUR(31,31,31)

#define ST7789V_ORANGE  ST7789V_COLOUR(31,15,0)
#define ST7789V_PURPLE  ST7789V_COLOUR(16,0,16)
#define ST7789V_BROWN   ST7789V_COLOUR(38,19,0)

extern int st7789vCharWidth, st7789vCharHeight;

void st7789vInit(void);
void st7789vAwaitCompletion(void);
void st7789vShow(void);
void st7789vBacklightEnable(int enable);

int st7789vReadRegister(int r);
void st7789vCommand(int c);
void st7789vWriteRegister(int r, int v);

void st7789vFlood(int xs, int ys, int width, int height, int value);
void st7789vDrawRectangle(int xs, int ys, int width, int height, const uint16_t *data);

int st7789vColorAtPixel(int x, int y);
void st7789vTestPattern(void);

void st7789vSetCharacterRGB(int foreground, int background);
void st7789vDrawChar(int x, int y, int c);
void st7789vShowString(int xBase, int yBase, const char *str);
int st7789vShowText(int xBase, int yBase, int width, int height,
                    int foreground, int background, const char *str);

void st7789vDumpScreen(void);
#ifdef ST7789_GRAB_SCREEN
int st7789vGrabScreen(void);
#endif

#endif  /* _ST7789V_H_ */
