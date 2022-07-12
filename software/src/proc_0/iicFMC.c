/*
 * Xilinx AXI IIC to FMC
 * FMC slot selected by GPO line.
 */
#include <stdio.h>
#include <stdint.h>
#include <xparameters.h>
#include "iicFMC.h"
#include "gpio.h"
#include "util.h"

#define IIC_REG_ISR             0x020
#define IIC_REG_SOFTR           0x040
#define IIC_REG_CR              0x100
#define IIC_REG_SR              0x104
#define IIC_REG_TX_FIFO         0x108
#define IIC_REG_RX_FIFO         0x10C
#define IIC_REG_RX_FIFO_PIRQ    0x120
#define IIC_REG_GPO             0x124

#define ISR_TX_FIFO_HALF_EMPTY  0x80
#define ISR_NOT_ADDRESSED       0x40
#define ISR_ADDRESSED           0x20
#define ISR_BUS_NOT_BUSY        0x10
#define ISR_RX_FIFO_FULL        0x8
#define ISR_TX_FIFO_EMPTY       0x4
#define ISR_TX_ERROR            0x2
#define ISR_ARB_LOST            0x1

#define CR_TX_FIFO_RESET    0x2
#define CR_TX_EN            0x1

#define SR_TX_FIFO_EMPTY    0x80
#define SR_RX_FIFO_EMPTY    0x40
#define SR_BB               0x04

#define TX_FIFO_START       0x100
#define TX_FIFO_STOP        0x200

static void
iicWriteReg(int reg, int value)
{
    if (debugFlags & DEBUGFLAG_IIC_FMC_REG) {
        printf("%X <= %X\n", reg, value);
    }
    Xil_Out32(XPAR_EVIO_IIC_BASEADDR + reg, value);
}

static int
iicReadReg(int reg)
{
    int value = Xil_In32(XPAR_EVIO_IIC_BASEADDR + reg);
    if (debugFlags & DEBUGFLAG_IIC_FMC_REG) {
        printf("%X => %X\n", reg, value);
    }
    return value;
}

static void
showTransfer(const char *dir, int address7, int subaddress,
                                              const unsigned char *buf, int len)
{
   int gpo = iicFMCgetGPO();
    printf("EVIO %s %X:%02X", dir, gpo, address7);
    if (subaddress >= 0) printf("[%02X]", subaddress);
    printf(":");
    while (len--) {
        printf(" %02X", *buf++);
    }
    printf("\n");
}

void
iicFMCinit(void)
{
   int gpo = iicFMCgetGPO();
    int sr;
    if (debugFlags & DEBUGFLAG_IIC_FMC) {
        printf("iicFMCinit()\n");
    }
    iicWriteReg(IIC_REG_SOFTR, 0xA); /* Soft reset */
    iicWriteReg(IIC_REG_CR, CR_TX_EN);
    iicWriteReg(IIC_REG_RX_FIFO_PIRQ, 0x0F); /* Full RX FIFO */
    sr = iicReadReg(IIC_REG_SR);
    if ((sr & (SR_TX_FIFO_EMPTY|SR_RX_FIFO_EMPTY|SR_BB)) !=
                                          (SR_TX_FIFO_EMPTY|SR_RX_FIFO_EMPTY)) {
        warn("Init IIC %X SR:%02X\n", iicReadReg(IIC_REG_GPO), sr);
    }
    iicFMCsetGPO(gpo);
}

static void
awaitReady(void)
{
    uint32_t then = MICROSECONDS_SINCE_BOOT();
    int sr;
    sr = iicReadReg(IIC_REG_SR);
    if ((sr & (SR_TX_FIFO_EMPTY|SR_RX_FIFO_EMPTY|SR_BB)) == SR_RX_FIFO_EMPTY) {
        iicWriteReg(IIC_REG_CR, CR_TX_FIFO_RESET | CR_TX_EN);
        iicWriteReg(IIC_REG_CR, CR_TX_EN);
    }
    for (;;) {
        sr = iicReadReg(IIC_REG_SR);
        if ((sr & (SR_TX_FIFO_EMPTY|SR_RX_FIFO_EMPTY|SR_BB)) ==
                                          (SR_TX_FIFO_EMPTY|SR_RX_FIFO_EMPTY)) {
            break;
        }
        if ((MICROSECONDS_SINCE_BOOT() - then) > 200) {
            warn("EVIO IIC not ready GPO:%X SR:%02X ISR:%02X",
                                                       iicReadReg(IIC_REG_GPO),
                                                       iicReadReg(IIC_REG_SR),
                                                       iicReadReg(IIC_REG_ISR));
            iicFMCinit();
            break;
        }
    }
    if (iicReadReg(IIC_REG_ISR) & ISR_TX_ERROR) {
        iicWriteReg(IIC_REG_ISR, ISR_TX_ERROR);
    }
}

int
iicFMCsend(int address7, int subaddress, const unsigned char *buf, int len)
{
    int i;
    uint32_t whenStarted;
    int address8 = address7 << 1;
    awaitReady();
    if (debugFlags & DEBUGFLAG_IIC_FMC) {
        showTransfer("Tx", address7, subaddress, buf, len);
    }
    iicWriteReg(IIC_REG_TX_FIFO, TX_FIFO_START | address8);
    if (subaddress >= 0) {
        iicWriteReg(IIC_REG_TX_FIFO, subaddress);
    }
    for (i = 1 ; i < len ; i++) {
        iicWriteReg(IIC_REG_TX_FIFO, *buf++);
    }
    iicWriteReg(IIC_REG_TX_FIFO, TX_FIFO_STOP | *buf);
    whenStarted = MICROSECONDS_SINCE_BOOT();
    while ((iicReadReg(IIC_REG_SR) & (SR_TX_FIFO_EMPTY | SR_BB)) !=
                                                             SR_TX_FIFO_EMPTY) {
        if ((MICROSECONDS_SINCE_BOOT() - whenStarted) > 100000) {
            warn("EVIO IIC Tx %X:%X timeout SR:%02X ISR:%02X",
                               iicReadReg(IIC_REG_GPO), address7,
                               iicReadReg(IIC_REG_SR), iicReadReg(IIC_REG_ISR));
            return -1;
        }
    }
    if (iicReadReg(IIC_REG_ISR) & ISR_TX_ERROR) {
        warn("EVIO IIC Tx %X:%X error SR:%02X ISR:%02X",
                               iicReadReg(IIC_REG_GPO), address7,
                               iicReadReg(IIC_REG_SR), iicReadReg(IIC_REG_ISR));
        iicWriteReg(IIC_REG_ISR, ISR_TX_ERROR);
        return -1;
    }
    return 0;
}

int
iicFMCrecv(int address7, int subaddress, unsigned char *buf, int len)
{
    uint32_t whenStarted = MICROSECONDS_SINCE_BOOT();
    int address8 = address7 << 1;
    unsigned char *cp = buf;
    int nLeft = len;
    awaitReady();
    iicWriteReg(IIC_REG_TX_FIFO, TX_FIFO_START | address8);
    iicWriteReg(IIC_REG_TX_FIFO, subaddress);
    iicWriteReg(IIC_REG_TX_FIFO, TX_FIFO_START | address8 | 0x1);
    iicWriteReg(IIC_REG_TX_FIFO, TX_FIFO_STOP | len);
    while (nLeft--) {
        while (iicReadReg(IIC_REG_SR) & SR_RX_FIFO_EMPTY) {
            if ((MICROSECONDS_SINCE_BOOT() - whenStarted) > 100000) {
                warn("EVIO IIC Rx %X:%X timeout SR:%02X ISR:%02X",
                               iicReadReg(IIC_REG_GPO), address7,
                               iicReadReg(IIC_REG_SR), iicReadReg(IIC_REG_ISR));
                return -1;
            }
        }
        *cp++ = iicReadReg(IIC_REG_RX_FIFO);
    }
    if (debugFlags & DEBUGFLAG_IIC_FMC) {
        showTransfer("Rx", address7, subaddress, buf, len);
    }
    return 0;
}

int
iicFMCgetGPO(void)
{
    return iicReadReg(IIC_REG_GPO);
}

void
iicFMCsetGPO(int gpo)
{
    iicWriteReg(IIC_REG_GPO, gpo);
}
