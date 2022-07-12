/*
 * Support for EVIO FMC card components:
 *  ADN4605 40x40 crosspoint switch
 *  Firefly optical I/O modules
 */

#include <stdio.h>
#include <stdint.h>
#include <xparameters.h>
#include "display.h"
#include "evio.h"
#include "iicFMC.h"
#include "iicProc.h"
#include "gpio.h"
#include "util.h"

#define EVIO_XCVR_COUNT         3
#define CHANNELS_PER_FIREFLY    12

/*
 * Crosspoint switch port assignments
 *   Port 0 is unused.
 *   Ports [1:36] correspond to front panel fiber pairs [1:36].
 *   Port 37 is the FPGA MGT.
 *   Port 38 is the FPGA MGT reference clock (output only).
 *   Port 39 is the FPGA fabric reference clock (output only).
 */
#define XPOINT_IO_UNUSED        0
#define XPOINT_IO_FIRST_FIREFLY 1
#define XPOINT_IO_LAST_FIREFLY  (XPOINT_IO_FIRST_FIREFLY + \
                                   (EVIO_XCVR_COUNT * CHANNELS_PER_FIREFLY) - 1)
#define XPOINT_IO_FMC_DP0       (XPOINT_IO_LAST_FIREFLY + 1)
#define XPOINT_OUT_FMC_GBTCLK0  (XPOINT_IO_FMC_DP0 + 1)
#define XPOINT_OUT_FMC_CLK0_M2C (XPOINT_OUT_FMC_GBTCLK0 + 1)
#define XPOINT_CHANNEL_COUNT 40

#define FIREFLY_REG_PWR_LO      14
#define FIREFLY_REG_TEMPERATURE 22
#define FIREFLY_REG_VCC_HI      26
#define FIREFLY_REG_VCC_LO      27
#define FIREFLY_REG_RX_CHANNEL_DISABLE_HI   52
#define FIREFLY_REG_RX_CHANNEL_DISABLE_LO   53
#define FIREFLY_REG_RX_OUTPUT_DISABLE_HI    54
#define FIREFLY_REG_RX_OUTPUT_DISABLE_LO    55

static struct fireflyStatus {
    uint8_t     isPresent;
    int8_t      temperature;
    uint16_t    vcc;
    uint16_t    rxLowPower;
    uint16_t    rxEnable;
} fireflyStatus[CFG_EVIO_FMC_COUNT][EVIO_XCVR_COUNT][2]; /* [0]=Tx, [1]=Rx */
static unsigned int fmcPresent;

/*
 * Crosspoint switch
 */
#define XP_REG_RESET                0x00
# define XP_RESET 0x1
#define XP_REG_UPDATE               0x01
# define XP_UPDATE 0x1
#define XP_REG_MAP_SELECT           0x02
#define XP_REG_TX_HEADROOM          0xBB
#define XP_REG_OUTPUT_CONNECT(n)    (0x04+(n))
#define XP_REG_OUTPUT_STATUS(n)     (0x54+(n))
#define XP_REG_RX_SIGN_CONTROL(n)   (0xCB+((n)/8))
#define XP_REG_TX_SIGN_CONTROL(n)   (0xA9+((n)/8))
# define XP_SIGN_BIT(n)             (0x1<<((n)%8))
#define XP_REG_OUTPUT_ENABLE(n)     (0xB0+((n)/4))
# define XP_OUTPUT_ENABLE_BITS(n)   (0x3<<(((n)%4)*2))
# define XP_OUTPUT_ENABLED(n,v)     (((v)>>(((n)%4)*2))&0x3)
#define XP_REG_TX_TERM_CONTROL(n)   (0xBC+((n)/20))
#define XP_REG_RX_TERM_CONTROL(n)   (0xD0+((n)/20))
# define XP_TERM_ENABLE_BIT(n)      (0x1<<(((n)/4)%5))
#define XP_REG_RX_EQ_BOOST(n)       (0xC0+((n)/4))
# define XP_RX_EQ_BOOST_WR(n,v)     ((v)<<(((n)%4)*2))
# define XP_RX_EQ_BOOST_RD(n,v)     (((v)>>(((n)%4)*2)) & 0x3)
# define XP_RX_EQ_DISABLE_RX 0x0
# define XP_RX_EQ_3_DB       0x1
# define XP_RX_EQ_MASK       0x3

/*
 * Card device IIC addresses (7-bit)
 */
#define IIC_ADDRESS_ADN4605    0x48
#define IIC_ADDRESS_FIREFLY_TX 0x50
#define IIC_ADDRESS_FIREFLY_RX 0x54

static void
iicSelect(int fmcIndex, int fireflyIndex)
{
    int sel = fmcIndex;
    if (fireflyIndex >= 0) {
        sel |= 1 << (fireflyIndex + 1);
    }
    iicFMCsetGPO(sel);
}

static int
xpWriteReg(int fmcIndex, int reg, int value)
{
    unsigned char v = value;
    iicSelect(fmcIndex, -1);
    return (iicFMCsend(IIC_ADDRESS_ADN4605, reg, &v, 1) >= 0);
}

static int
xpReadReg(int fmcIndex, int reg)
{
    unsigned char v;
    iicSelect(fmcIndex, -1);
    if (iicFMCrecv(IIC_ADDRESS_ADN4605, reg, &v, 1) < 0) return -1;
    return v;
}

/*
 * Drive specified output from specified input
 */
static int
xpOutFromIn(int fmcIndex, unsigned int outputIndex, unsigned int inputIndex)
{
    if ((outputIndex >= XPOINT_CHANNEL_COUNT)
     || (inputIndex >= XPOINT_CHANNEL_COUNT)) return 0;
    return (xpWriteReg(fmcIndex, XP_REG_OUTPUT_CONNECT(outputIndex), inputIndex)
         && xpWriteReg(fmcIndex, XP_REG_UPDATE, XP_UPDATE));
}

/*
 * Enable specified output
 */
static int
xpEnableOutput(int fmcIndex, unsigned int outputIndex)
{
    int r, v, b;
    if (outputIndex >= XPOINT_CHANNEL_COUNT) return 0;
    r = XP_REG_OUTPUT_ENABLE(outputIndex);
    b = XP_OUTPUT_ENABLE_BITS(outputIndex);
    v = xpReadReg(fmcIndex, r);
    if (v < 0) return 0;
    v |= b;
    return xpWriteReg(fmcIndex, r, v);
}

/*
 * Set receiver equalization
 */
static int
xpSetRxEqualization(int fmcIndex, unsigned int inputIndex, int eq)
{
    int r, v, bits, mask;
    r = XP_REG_RX_EQ_BOOST(inputIndex);
    bits = XP_RX_EQ_BOOST_WR(inputIndex, eq);
    mask = XP_RX_EQ_BOOST_WR(inputIndex, XP_RX_EQ_MASK);
    v = xpReadReg(fmcIndex, r);
    if (v < 0) return 0;
    v = (v & ~mask) | (bits & mask);
    return xpWriteReg(fmcIndex, r, v);
}

static int
xpReset(int fmcIndex)
{
    return xpWriteReg(fmcIndex, XP_REG_RESET, XP_RESET);
}

static void
fireflyRxLoopbackEnable(int fmcIndex, int receiverChannel)
{
    int xcvrIndex, loopbackTransceiver = -1;
    uint8_t cbuf[4];

    if ((receiverChannel > XPOINT_IO_FIRST_FIREFLY)
     && (receiverChannel <= XPOINT_IO_LAST_FIREFLY)) {
        loopbackTransceiver = (receiverChannel - XPOINT_IO_FIRST_FIREFLY) /
                                                           CHANNELS_PER_FIREFLY;
    }

    /*
     * Disable everything but the RF reference(first) and loopback channel.
     */
    for (xcvrIndex = 0 ; xcvrIndex < EVIO_XCVR_COUNT ; xcvrIndex++) {
        struct fireflyStatus *fs = &fireflyStatus[fmcIndex][xcvrIndex][1];
        fs->rxEnable = (xcvrIndex == 0) ? 0x1 : 0x0;
        if (xcvrIndex == loopbackTransceiver) {
            fs->rxEnable |= 1 << ((receiverChannel - XPOINT_IO_FIRST_FIREFLY) %
                                                          CHANNELS_PER_FIREFLY);
        }
        if (fs->isPresent) {
            int bitmap = ~fs->rxEnable & ((1 << CHANNELS_PER_FIREFLY) - 1);
            cbuf[0] = bitmap >> 8;
            cbuf[1] = bitmap;
            cbuf[2] = cbuf[0];
            cbuf[3] = cbuf[1];
            iicSelect(fmcIndex, xcvrIndex);
            iicFMCsend(IIC_ADDRESS_FIREFLY_RX,
                                    FIREFLY_REG_RX_CHANNEL_DISABLE_HI, cbuf, 4);
        }
    }
}

void
evioInit(void)
{
    int fmcIndex, xpChannel, isRx;
    int i;

    /*
     * Check which FMC cards are installed
     */
    for (fmcIndex = 0 ; fmcIndex < CFG_EVIO_FMC_COUNT ; fmcIndex++) {
        const char *name = iicProcFMCproductType(fmcIndex);
        if (strcmp(name, "EVIO") == 0) {
            fmcPresent |= 1 << fmcIndex;
        }
    }
    if (fmcPresent != ((1 << CFG_EVIO_FMC_COUNT) - 1)) {
        const char *cp = "Critical -- Missing/Improper FMC card";
        displayShowStartup(cp);
        warn(cp);
    }

    /*
     * Set up crosspoint switches
     */
    for (fmcIndex = 0 ; fmcIndex < CFG_EVIO_FMC_COUNT ; fmcIndex++) {
        uint32_t presentBits = ~GPIO_READ(GPIO_IDX_FMC1_FIREFLY + fmcIndex);
        int txCount = 0;
        if ((fmcPresent & (0x1 << fmcIndex)) == 0) {
            continue;
        }

        /*
         * Detect firefly modules
         */
        for (i = 0 ; i < EVIO_XCVR_COUNT ; i++) {
            for (isRx = 0 ; isRx < 2 ; isRx++) {
                fireflyStatus[fmcIndex][i][isRx].isPresent =
                                          presentBits >> ((i * 2) + isRx) & 0x1;
            }
            if (fireflyStatus[fmcIndex][i][0].isPresent) {
                txCount++;
            }
        }
        if (!fireflyStatus[fmcIndex][0][1].isPresent) {
            warn("Critical -- FMC %d has no RF reference receiver", fmcIndex+1);
        }
        if (txCount == 0) {
            warn("Critical -- FMC %d has no transmitters", fmcIndex + 1);
        }

        /*
         * Reset crosspoint -- disables all I/O, enables all terminations
         */
        if (!xpReset(fmcIndex)) {
            warn("Critical -- FMC %d bad crosspoint switch", fmcIndex + 1);
            continue;
        }

        /*
         * Route EVG reference clock
         */
        xpSetRxEqualization(fmcIndex, XPOINT_IO_FIRST_FIREFLY, XP_RX_EQ_3_DB);
        xpOutFromIn(fmcIndex, XPOINT_IO_FIRST_FIREFLY, XPOINT_IO_FIRST_FIREFLY);
        xpOutFromIn(fmcIndex, XPOINT_OUT_FMC_GBTCLK0, XPOINT_IO_FIRST_FIREFLY);
        xpOutFromIn(fmcIndex, XPOINT_OUT_FMC_CLK0_M2C, XPOINT_IO_FIRST_FIREFLY);
        xpEnableOutput(fmcIndex, XPOINT_IO_FIRST_FIREFLY);
        xpEnableOutput(fmcIndex, XPOINT_OUT_FMC_GBTCLK0);
        xpEnableOutput(fmcIndex, XPOINT_OUT_FMC_CLK0_M2C);

        /*
         * Route event links
         */
        xpSetRxEqualization(fmcIndex, XPOINT_IO_FMC_DP0, XP_RX_EQ_3_DB);
        for (xpChannel = XPOINT_IO_FIRST_FIREFLY + 1 ;
                            xpChannel <= XPOINT_IO_LAST_FIREFLY ; xpChannel++) {
            int ffIdx;
            ffIdx = (xpChannel-XPOINT_IO_FIRST_FIREFLY) / CHANNELS_PER_FIREFLY;
            if (fireflyStatus[fmcIndex][ffIdx][0].isPresent) {
                xpOutFromIn(fmcIndex, xpChannel, XPOINT_IO_FMC_DP0);
                xpEnableOutput(fmcIndex, xpChannel);
            }
        }

        /*
         * Loop back data for baseline latency measurement
         */
        xpOutFromIn(fmcIndex, XPOINT_IO_FMC_DP0, XPOINT_IO_FMC_DP0);
        xpEnableOutput(fmcIndex, XPOINT_IO_FMC_DP0);
        sharedMemory->loopbackActual |= XPOINT_IO_FMC_DP0 << (8 * fmcIndex);
        fireflyRxLoopbackEnable(fmcIndex, XPOINT_IO_UNUSED);
    }
    sharedMemory->loopbackRequest = sharedMemory->loopbackActual;
}

/*
 * Watch for loopback change requests
 * Disable unused crosspoint receiver inputs
 */
static int
updateLoopback(void)
{
    int newLoopback = sharedMemory->loopbackRequest;
    int oldLoopback = sharedMemory->loopbackActual;
    if (newLoopback != oldLoopback) {
        int fmcIndex = 0;
        int shift = 0;
        int mask = 0xFF;
        while (fmcIndex < CFG_EVIO_FMC_COUNT) {
            if (fmcPresent & (0x1 << fmcIndex)) {
                if ((newLoopback & mask) != (oldLoopback & mask)) {
                    int oldIn = (oldLoopback >> shift) & 0xFF;
                    int newIn = (newLoopback >> shift) & 0xFF;
                    if ((oldIn >  XPOINT_IO_FIRST_FIREFLY)
                     && (oldIn <= XPOINT_IO_LAST_FIREFLY)) {
                        xpSetRxEqualization(fmcIndex,oldIn,XP_RX_EQ_DISABLE_RX);
                    }
                    if ((newIn >  XPOINT_IO_FIRST_FIREFLY)
                     && (newIn <= XPOINT_IO_LAST_FIREFLY)) {
                        xpSetRxEqualization(fmcIndex, newIn, XP_RX_EQ_3_DB);
                    }
                    else {
                        newIn = XPOINT_IO_FMC_DP0;
                    }
                    xpOutFromIn(fmcIndex, XPOINT_IO_FMC_DP0, newIn);
                    fireflyRxLoopbackEnable(fmcIndex, newIn);
                }
            }
            fmcIndex++;
            mask <<= 8;
            shift += 8;
        }
        microsecondSpin(500);
        sharedMemory->loopbackActual = newLoopback;
        return 1;
    }
    return 0;
}

void
evioCrank(void)
{
    static int isActive;
    static int fmcIndex, xcvrIndex, isRx;
    static uint32_t whenIdle;

    if (updateLoopback()) return;
    if (isActive) {
        struct fireflyStatus *fs = &fireflyStatus[fmcIndex][xcvrIndex][isRx];
        if (fs->isPresent) {
            uint8_t cbuf[FIREFLY_REG_VCC_LO - FIREFLY_REG_PWR_LO + 1];
            iicSelect(fmcIndex, xcvrIndex);
            iicFMCrecv(isRx ? IIC_ADDRESS_FIREFLY_RX : IIC_ADDRESS_FIREFLY_TX,
                                   FIREFLY_REG_PWR_LO, cbuf,
                                   FIREFLY_REG_VCC_LO - FIREFLY_REG_PWR_LO + 1);
            fs->temperature = cbuf[FIREFLY_REG_TEMPERATURE-FIREFLY_REG_PWR_LO];
            fs->vcc = (cbuf[FIREFLY_REG_VCC_HI - FIREFLY_REG_PWR_LO] << 8)
                     | cbuf[FIREFLY_REG_VCC_LO - FIREFLY_REG_PWR_LO];
            if (isRx) {
                int i, idx = 0, bit = 0x40;
                int bitmap = 0;
                for (i = 0 ; i < CHANNELS_PER_FIREFLY ; i++) {
                    bitmap = (bitmap << 1) | ((cbuf[idx] & bit) != 0);
                    bit >>= 2;
                    if (bit == 0) {
                        idx++;
                        bit = 0x40;
                    }
                }
                fs->rxLowPower = bitmap;
            }
        }
        if (isRx) {
            isRx = 0;
            if (xcvrIndex == (EVIO_XCVR_COUNT - 1)) {
                xcvrIndex = 0;
                if (fmcIndex == (CFG_EVIO_FMC_COUNT - 1)) {
                    fmcIndex = 0;
                    isActive = 0;
                    whenIdle = MICROSECONDS_SINCE_BOOT();
                }
                else {
                    fmcIndex++;
                }
            }
            else {
                xcvrIndex++;
            }
        }
        else {
            isRx = 1;
        }
    }
    else {
        if ((MICROSECONDS_SINCE_BOOT() - whenIdle) > 10000000) {
            isActive = 1;
        }
    }
}

uint32_t *
evioFetchSysmon(uint32_t *ap)
{
    int fmcIndex, xcvrIndex, isRx;
    struct fireflyStatus *fs;
    for (fmcIndex = 0 ; fmcIndex < CFG_EVIO_FMC_COUNT ; fmcIndex++) {
        for (xcvrIndex = 0 ; xcvrIndex < EVIO_XCVR_COUNT ; xcvrIndex++) {
            for (isRx = 0 ; isRx < 2 ; isRx++) {
                fs = &fireflyStatus[fmcIndex][xcvrIndex][isRx];
                *ap++ = (fs->temperature << 16) | fs->vcc;
            }
        }
    }
    for (fmcIndex = 0 ; fmcIndex < CFG_EVIO_FMC_COUNT ; fmcIndex++) {
        for (xcvrIndex = 0 ; xcvrIndex < EVIO_XCVR_COUNT ; xcvrIndex++) {
            fs = &fireflyStatus[fmcIndex][xcvrIndex][1];
            *ap++ = (fs->rxLowPower << 16) | fs->rxEnable;
        }
    }
    return ap;
}

int
evioWarmestTransmitter(int fmcIndex)
{
    int xcvrIndex;
    int highTemp = 0;
    for (xcvrIndex = 0 ; xcvrIndex < EVIO_XCVR_COUNT ; xcvrIndex++) {
        struct fireflyStatus *fs = &fireflyStatus[fmcIndex][xcvrIndex][0];
        if (fs->temperature > highTemp) {
            highTemp = fs->temperature;
        }
    }
    return highTemp;
}

int
evioWarmestReceiver(int fmcIndex)
{
    int xcvrIndex;
    int highTemp = 0;
    for (xcvrIndex = 0 ; xcvrIndex < EVIO_XCVR_COUNT ; xcvrIndex++) {
        struct fireflyStatus *fs = &fireflyStatus[fmcIndex][xcvrIndex][1];
        if (fs->temperature > highTemp) {
            highTemp = fs->temperature;
        }
    }
    return highTemp;
}

static void
fireflyDumpPage(int isRx, int base)
{
    int i, j;
    uint8_t cbuf[16];
    for (i = 0 ; i < 128 ; i += 16) {
        iicFMCrecv((isRx ? IIC_ADDRESS_FIREFLY_RX : IIC_ADDRESS_FIREFLY_TX),
                                                            base + i, cbuf, 16);
        printf("%3d:", base + i);
        for (j = 0 ; j < 16 ; j++) {
            printf(" %02X", cbuf[j]);
        }
        printf("  ");
        for (j = 0 ; j < 16 ; j++) {
            int c = cbuf[j];
            if ((c < ' ') || (c > '~')) c = ' ';
            printf("%c", c);
        }
        printf("\n");
    }
}

static void
fireflyShowAll(void)
{
    int fmcIdx, xcvrIdx, isRx, pgIdx;
    static const uint8_t pgMap[] = { 0x00, 0x01, 0x0B };
    for (fmcIdx = 0 ; fmcIdx < CFG_EVIO_FMC_COUNT ; fmcIdx++) {
      for (isRx = 0 ; isRx < 2 ; isRx++) {
        for (xcvrIdx = 0 ; xcvrIdx < EVIO_XCVR_COUNT ; xcvrIdx++) {
            struct fireflyStatus *fs = &fireflyStatus[fmcIdx][xcvrIdx][isRx];
            if (!fs->isPresent) {
                continue;
            }
            printf("FMC:%d  XCVR:%d  %cx\n", fmcIdx + 1, xcvrIdx, isRx?'R':'T');
            iicSelect(fmcIdx, xcvrIdx);
            fireflyDumpPage(isRx, 0);
            for (pgIdx = 0 ; pgIdx < sizeof pgMap/sizeof pgMap[0] ; pgIdx++) {
                printf(" Page %02X:\n", pgMap[pgIdx]);
                iicFMCsend((isRx ? IIC_ADDRESS_FIREFLY_RX :
                                IIC_ADDRESS_FIREFLY_TX), 127, &pgMap[pgIdx], 1);
                microsecondSpin(600000);
                fireflyDumpPage(isRx, 128);
            }
            iicFMCsend((isRx ? IIC_ADDRESS_FIREFLY_RX :
                               IIC_ADDRESS_FIREFLY_TX), 127, &pgMap[0], 1);
        }
      }
    }
}

void
evioShowCrosspointRegisters(void)
{
    int fmcIndex, i;
    uint8_t txAssignment, txSign, txEnReg, txEn, txTerm;
    uint8_t rxEqReg, rxEq, rxSign, rxTerm;

    for (fmcIndex = 0 ; fmcIndex < CFG_EVIO_FMC_COUNT ; fmcIndex++) {
        if ((fmcPresent & (0x1 << fmcIndex)) == 0) {
            continue;
        }
        printf("FMC %d: Map %d enabled, TxHeadroom %sabled\n", fmcIndex + 1,
                        xpReadReg(fmcIndex, XP_REG_MAP_SELECT),
                        xpReadReg(fmcIndex, XP_REG_TX_HEADROOM) ? "en" : "dis");
        printf("Output Source  TXen TXsign TXterm  RXeq RXsign RXterm\n");
        for (i = 0 ; i < XPOINT_CHANNEL_COUNT ; i++) {
            txAssignment = xpReadReg(fmcIndex, XP_REG_OUTPUT_STATUS(i));
            txSign = ((xpReadReg(fmcIndex, XP_REG_TX_SIGN_CONTROL(i)) &
                                                          XP_SIGN_BIT(i)) != 0);
            rxSign = ((xpReadReg(fmcIndex, XP_REG_RX_SIGN_CONTROL(i)) &
                                                          XP_SIGN_BIT(i)) != 0);
            txEnReg = xpReadReg(fmcIndex, XP_REG_OUTPUT_ENABLE(i));
            txEn = XP_OUTPUT_ENABLED(i, txEnReg);
            txTerm = ((xpReadReg(fmcIndex, XP_REG_TX_TERM_CONTROL(i)) &
                                                   XP_TERM_ENABLE_BIT(i)) != 0);
            rxTerm = ((xpReadReg(fmcIndex, XP_REG_RX_TERM_CONTROL(i)) &
                                                   XP_TERM_ENABLE_BIT(i)) != 0);
            rxEqReg = xpReadReg(fmcIndex, XP_REG_RX_EQ_BOOST(i));
            rxEq = XP_RX_EQ_BOOST_RD(i, rxEqReg);
            printf("%4d     %2d      %d     %d      %d      %d     %d      %d\n",
                                          i, txAssignment, txEn, txSign, txTerm,
                                          rxEq, rxSign, rxTerm);
        }
    }
    for (fmcIndex = 0 ; fmcIndex < CFG_EVIO_FMC_COUNT ; fmcIndex++) {
        for (i = 0 ; i < EVIO_XCVR_COUNT ; i++) {
            if(fireflyStatus[fmcIndex][i][1].isPresent) {
                printf("FMC%d Firefly %d low Rx power: %03X\n", fmcIndex + 1, i,
                                      fireflyStatus[fmcIndex][i][1].rxLowPower);
            }
        }
    }
    fireflyShowAll();
}
