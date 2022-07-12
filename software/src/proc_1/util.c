
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "gpio.h"
#include "ipc.h"
#include "util.h"

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
    outbyte(ASCII_SO);
    printf(fmt, a[0], a[1], a[2], a[3]);
    printf("\n");
    va_end(args);
}

/*
 * 32-bit endian swap
 * Assume that we're using GCC
 */
void
bswap32(uint32_t *b, int n)
{
    while (n--) {
        *b = __builtin_bswap32(*b);
        b++;
    }
}

void
microsecondSpin(unsigned int us)
{
    uint32_t then;
    then = MICROSECONDS_SINCE_BOOT();
    while ((uint32_t)((MICROSECONDS_SINCE_BOOT()) - then) <= us) continue;
}
