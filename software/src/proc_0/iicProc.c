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
 * Communicate with IIC devices directly from processor
 * Simple, slow, bit-banging interface intended for occasional use
 */
#include <stdio.h>
#include <stdint.h>
#include <xil_io.h>
#include <xparameters.h>
#include "iicChunk.h"
#include "iicProc.h"
#include "util.h"

#define FMC_COUNT               2
#define FMC_PRODUCT_NAME_LENGTH 12
static char productNames[FMC_COUNT][FMC_PRODUCT_NAME_LENGTH];

/*
 * 7 bit addresses
 */
#define QSFP_CONTROL_PORT_EXPANDER_ADDRESS  0x22
#define MGTCLK_SWITCH_ADDRESS               0x48
#define FMC1_EEPROM_ADDRESS                 0x50
#define FMC2_EEPROM_ADDRESS                 0x51
#define QSPF2_ADDRESS                       0x50

/*
 * FMC IPMI EEPROMs
 */
#define EEPROM_PAGE_SIZE 16

/*
 * Bit-banging
 */
#define IIC_SDA_RBK 0x8
#define IIC_ENABLE  0x4
#define IIC_SDA     0x2
#define IIC_SCL     0x1
#define GPIO_OUT(x) Xil_Out32(XPAR_IIC_PROC_GPIO_BASEADDR, (x))
#define GPIO_IN()   Xil_In32(XPAR_IIC_PROC_GPIO_BASEADDR)

static int shadow;

void
iicProcTakeControl(void)
{
    iicChunkSuspend();
    shadow = IIC_ENABLE | IIC_SDA | IIC_SCL;
    GPIO_OUT(shadow);
}

void
iicProcRelinquishControl(void)
{
    shadow = IIC_SDA | IIC_SCL;
    GPIO_OUT(shadow);
    iicChunkResume();
}

static void SET_SCL(void) { shadow |= IIC_SCL;  GPIO_OUT(shadow); }
static void CLR_SCL(void) { shadow &= ~IIC_SCL; GPIO_OUT(shadow); }
static void SET_SDA(void) { shadow |= IIC_SDA;  GPIO_OUT(shadow); }
static void CLR_SDA(void) { shadow &= ~IIC_SDA; GPIO_OUT(shadow); }
static int READ_SDA(void) { return ((GPIO_IN() & IIC_SDA_RBK) != 0); }

static void
SEND_START(void)
{
    if (debugFlags & DEBUGFLAG_IIC_PROC) {
        printf("START\n");
    }
    shadow = IIC_ENABLE | IIC_SDA | IIC_SCL;
    GPIO_OUT(shadow);
    microsecondSpin(5);
    if (!READ_SDA()) printf("iicProc can't START -- SDA stuck LO\n");
    CLR_SDA();
    microsecondSpin(5);
    if (READ_SDA()) printf("iicProc can't START -- SDA stuck HI\n");
    CLR_SCL();
    microsecondSpin(5);
}

static void
SEND_STOP(void)
{
    if (debugFlags & DEBUGFLAG_IIC_PROC) {
        printf("STOP\n");
    }
    CLR_SDA();
    microsecondSpin(5);
    SET_SCL();
    microsecondSpin(5);
    if (READ_SDA()) printf("iicProc can't STOP -- SDA stuck HI\n");
    SET_SDA();
    microsecondSpin(5);
    if (!READ_SDA()) printf("iicProc can't STOP -- SDA stuck LO\n");
}

static void
WRITE_BIT(int b)
{
     if (b)
        SET_SDA();
     else
        CLR_SDA();
    microsecondSpin(5);
    SET_SCL();
    microsecondSpin(5);
    CLR_SCL();
}

static int
READ_BIT(void)
{
    int b;
    SET_SDA();
    microsecondSpin(5);
    SET_SCL();
    microsecondSpin(5);
    b = READ_SDA();
    CLR_SCL();
    return b;
}

/*
 * Return ACK status
 */
static int
writeByte(int byte)
{
    int b = byte;
    int i, ack;
    for (i = 0 ; i < 8 ; i++) {
        WRITE_BIT(b & 0x80);
        b <<= 1;
    }
    ack = !READ_BIT();
    if (debugFlags & DEBUGFLAG_IIC_PROC) {
        printf("W %02X %c\n", byte, ack ? 'A' : 'N');
    }
    return ack;
}

static int
readByte(int sendAck)
{
    int i, byte;
    for (i = 0, byte = 0 ; i < 8 ; i++) {
        byte <<= 1;
        byte |= READ_BIT();
    }
    WRITE_BIT(!sendAck);
    if (debugFlags & DEBUGFLAG_IIC_PROC) {
        printf("R %02X %c\n", byte, sendAck ? 'A' : 'N');
    }
    return byte;
}

/*
 * Send with no trailing STOP
 */
static int
iicSend(int device, int subaddress, uint8_t *buf, int n)
{
    SEND_START();
    if (!writeByte(device << 1)) {
        return 0;
    }
    if (subaddress >= 0) {
        if (!writeByte(subaddress)) {
            return 0;
        }
    }
    while (n--) {
        if (!writeByte(*buf++)) {
            return 0;
        }
    }
    return 1;
}

int
iicProcRead(int device, int subaddress, uint8_t *buf, int n)
{
    if (subaddress >= 0) {
        if (!iicSend(device, subaddress, NULL, 0)) {
            SEND_STOP();
            return 0;
        }
    }
    SEND_START();
    if (!writeByte((device << 1) | 0x1)) {
        SEND_STOP();
        return 0;
    }
    while (n--) {
        *buf++ = readByte(n);
    }
    SEND_STOP();
    return 1;
}

int
iicProcWrite(int device, int subaddress, uint8_t *buf, int n)
{
    int r;

    r = iicSend(device, subaddress, buf, n);
    SEND_STOP();
    return r;
}

int
iicProcSetMux(int port)
{
    if (!iicProcWrite(IIC_MUX_ADDRESS, 1 << port, NULL, 0)) {
        printf("Can't set MUX to port %d.\n", port);
        return 0;
    }
    return 1;
}

/*
 * Read n bytes from beginning of EEPROM
 * Assume a 256 byte device (single IIC address)
 */
int
iicProcReadFMC_EEPROM(int fmcIndex, uint8_t *buf, int n)
{
    int r;

    iicProcTakeControl();
    if (!iicProcSetMux(IIC_MUX_PORT_FMC1 + fmcIndex)) {
        iicProcRelinquishControl();
        return 0;
    }
    r = iicProcRead(FMC1_EEPROM_ADDRESS + fmcIndex, 0, buf, n);
    iicProcRelinquishControl();
    return r;
}

/*
 * Write n bytes at beginning of EEPROM
 * Assume a 256 byte device (single IIC address)
 */
int
iicProcWriteFMC_EEPROM(int fmcIndex, uint8_t *buf, int n)
{
    int subaddress = 0;

    iicProcTakeControl();
    if (!iicProcSetMux(IIC_MUX_PORT_FMC1 + fmcIndex)) {
        iicProcRelinquishControl();
        return 0;
    }
    if (n) {
        if (!iicSend(FMC1_EEPROM_ADDRESS + fmcIndex, -1, NULL, 0)) {
            SEND_STOP();
            iicProcRelinquishControl();
            return 0;
        }
        for (;;) {
            int pass = 0;
            int nSend = n;
            int i;
            if (nSend > EEPROM_PAGE_SIZE) nSend = EEPROM_PAGE_SIZE;
            if (!writeByte(subaddress)) {
                SEND_STOP();
                iicProcRelinquishControl();
                return 0;
            }
            for (i = 0 ; i < nSend ; i++) {
                if (!writeByte(*buf++)) {
                    SEND_STOP();
                    iicProcRelinquishControl();
                    return 0;
                }
            }
            SEND_STOP();
            /*
             * Poll for completion
             */
            while (!iicSend(FMC1_EEPROM_ADDRESS + fmcIndex, -1, NULL, 0)) {
                if (++pass > 30) {
                    SEND_STOP();
                    iicProcRelinquishControl();
                    return 0;
                }
            }
            subaddress += nSend;
            n -= nSend;
            if (n == 0) {
                SEND_START(); // This is what the flow chart shows....
                SEND_STOP();
                break;
            }
        }
    }
    iicProcRelinquishControl();
    return 1;
}

void
iicProcInit(void)
{
    int fmcIndex;
    int strIndex;
    static const char *strNames[] = { "Manufacturer",
                                      "Product name",
                                      "Serial number",
                                      "Part number" };
    for (fmcIndex = 0 ; fmcIndex < FMC_COUNT ; fmcIndex++) {
        uint8_t cbuf[256], *cp;
        const uint8_t *boardAreaBase;
        int l, boardAreaLength;
        if (!iicProcReadFMC_EEPROM(fmcIndex, cbuf, sizeof cbuf)) {
            printf("Can't read FMC %d EEPROM!\n", fmcIndex + 1);
            continue;
        }
        if (cbuf[0] != 0x01) {
            printf("FMC %d header format:%d!\n", fmcIndex + 1, cbuf[0]);
            continue;
        }
        if ((cbuf[3] == 0) || (cbuf[3] >= 256/8)) {
            printf("FMC %d board info offset:%d!\n", fmcIndex + 1, cbuf[3]);
            continue;
        }
        boardAreaBase = cp = &cbuf[cbuf[3]*8];
        if (cp[0] != 0x01) {
            printf("FMC %d board area format:%d!\n", fmcIndex + 1, cp[0]);
            continue;
        }
        boardAreaLength = cp[1] * 8;
        if ((boardAreaLength == 0) || (boardAreaLength >= 256)) {
            printf("FMC %d board area length:%d!\n", fmcIndex + 1, cp[1]);
            continue;
        }
        cp += 6;

        printf("FMC %d:\n", fmcIndex + 1);
        for (strIndex = 0 ; strIndex < (sizeof strNames / sizeof strNames[0]) ;
                                                                   strIndex++) {
            int i;
            const char *strName = strNames[strIndex];
            printf("    %-14s", strName);
            if ((*cp & 0xC0) != 0xC0) {
                printf("Not ASCII!\n");
                break;
            }
            l = *cp & 0x3F;
            if ((&cp[l] - boardAreaBase) >= boardAreaLength) {
                printf("Too long!\n");
                 break;
            }
            printf("\"");
            cp++;
            for (i = 0 ; i < l ; i++) {
                char c = (char)(cp[i]);
                printf("%c", c);
                if ((strIndex == 1) && (i < (FMC_PRODUCT_NAME_LENGTH-1))) {
                    productNames[fmcIndex][i] = c;
                }
            }
            printf("\"\n");
            cp += l;
        }
    }
}

const char *
iicProcFMCproductType(int fmcIndex)
{
    if ((fmcIndex < 0) || (fmcIndex >= FMC_COUNT)) {
        return "?";
    }
    return productNames[fmcIndex];
}

/*
 * Scan I2C buses
 */
void
iicProcScan(void)
{
    int m, a;
    const char *sep;
    iicProcTakeControl();
    printf("IIC Devices (7 bit addresses (hex))\n");
    for (m = 0 ; m < 8 ; m++) {
        printf("Bus %d:", m);
        if (!iicProcWrite(IIC_MUX_ADDRESS, 1 << m, NULL, 0)) {
            iicProcRelinquishControl();
            printf("iicProc can't set IIC MUX\n");
            return;
        }
        sep = "";
        for (a = 1 ; a < 127 ; a++) {
            if (iicProcWrite(a, -1, NULL, 0)) {
                printf("%s %02X", sep, a);
                sep = ",";
            }
        }
        printf("\n");
    }
    iicProcRelinquishControl();
}
