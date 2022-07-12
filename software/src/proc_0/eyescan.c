/*
 * Perform a transceiver eye scan
 *
 * This code is based on the example provided in Xilinx application note
 * XAPP743, "Eye Scan with MicroBlaze Processor MCS Application Note".
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "eyescan.h"
#include "gpio.h"
#include "util.h"

#define PRESCALE_CODE  5

#if EYESCAN_TRANSCEIVER_WIDTH == 40
# define DRP_REG_ES_SDATA_MASK0_VALUE   0x0000
# define DRP_REG_ES_SDATA_MASK1_VALUE   0x0000
#elif EYESCAN_TRANSCEIVER_WIDTH == 20
# define DRP_REG_ES_SDATA_MASK0_VALUE   0xFFFF
# define DRP_REG_ES_SDATA_MASK1_VALUE   0x000F
#else
# error "Invalid EYESCAN_TRANSCEIVER_WIDTH value"
#endif

#define VRANGE         128
#define VSTRIDE        8

#define DRP_REG_ES_QUAL_MASK0     0x031
#define DRP_REG_ES_QUAL_MASK1     0x032
#define DRP_REG_ES_QUAL_MASK2     0x033
#define DRP_REG_ES_QUAL_MASK3     0x034
#define DRP_REG_ES_QUAL_MASK4     0x035
#define DRP_REG_ES_SDATA_MASK0    0x036
#define DRP_REG_ES_SDATA_MASK1    0x037
#define DRP_REG_ES_SDATA_MASK2    0x038
#define DRP_REG_ES_SDATA_MASK3    0x039
#define DRP_REG_ES_SDATA_MASK4    0x03A
#define DRP_REG_ES_PS_VOFF        0x03B
#define DRP_REG_ES_HORZ_OFFSET    0x03C
#define DRP_REG_ES_CSR            0x03D
#define DRP_REG_ES_ERROR_COUNT    0x14F
#define DRP_REG_ES_SAMPLE_COUNT   0x150
#define DRP_REG_ES_STATUS         0x151
#define DRP_REG_PMA_RSV2          0x082
#define DRP_REG_TXOUT_RXOUT_DIV   0x088

#define ES_CSR_CONTROL_RUN      0x01
#define ES_CSR_CONTROL_ARM      0x02
#define ES_CSR_CONTROL_TRIG0    0x04
#define ES_CSR_CONTROL_TRIG1    0x08
#define ES_CSR_CONTROL_TRIG2    0x10
#define ES_CSR_CONTROL_TRIG3    0x20
#define ES_CSR_EYE_SCAN_EN      0x100
#define ES_CSR_ERRDET_EN        0x200

#define ES_STATUS_DONE          0x1
#define ES_STATUS_STATE_MASK    0xE
#define ES_STATUS_STATE_WAIT    0x0
#define ES_STATUS_STATE_RESET   0x2
#define ES_STATUS_STATE_END     0x4
#define ES_STATUS_STATE_COUNT   0x6
#define ES_STATUS_STATE_READ    0x8
#define ES_STATUS_STATE_ARMED   0xA

#define ES_PS_VOFF_PRESCALE_MASK  0xF800
#define ES_PS_VOFF_PRESCALE_SHIFT 11
#define ES_PS_VOFF_VOFFSET_MASK   0x1FF

#define DRP_CSR_W_WE            (1U << 30)
#define DRP_CSR_W_ADDR_SHIFT    16
#define DRP_CSR_R_BUSY          (1U << 31)
#define DRP_CSR_RW_DATA_MASK    0xFFFF

static const char *laneNames[] = EYESCAN_LANE_NAMES;

static enum eyescanFormat {
    FMT_ASCII_ART,
    FMT_NUMERIC,
    FMT_RAW
} eyescanFormat;

static int
drp_wait(uint32_t csrIdx)
{
    uint32_t csr;
    int pass = 0;
    while ((csr = GPIO_READ(csrIdx)) & DRP_CSR_R_BUSY) {
        if (++pass > 10) {
            return -1;
        }
        microsecondSpin(5);
    }
    return csr & DRP_CSR_RW_DATA_MASK;
}

static void
drp_write(uint32_t csrIdx, int regOffset, int value)
{
    GPIO_WRITE(csrIdx, DRP_CSR_W_WE | (regOffset << DRP_CSR_W_ADDR_SHIFT) |
                                      (value & DRP_CSR_RW_DATA_MASK));
    drp_wait(csrIdx);
}

static int
drp_read(uint32_t csrIdx, int regOffset)
{
    GPIO_WRITE(csrIdx, regOffset << DRP_CSR_W_ADDR_SHIFT);
    return drp_wait(csrIdx);
}

static void
drp_rmw(uint32_t csrIdx, int regOffset, int mask, int value)
{
    uint16_t v = drp_read(csrIdx, regOffset);
    v &= ~mask;
    v |= value & mask;
    drp_write(csrIdx, regOffset, v);
}

static void
drp_set(uint32_t csrIdx, int regOffset, int bits)
{
    drp_rmw(csrIdx, regOffset, bits, bits);
}

static void
drp_clr(uint32_t csrIdx, int regOffset, int bits)
{
    drp_rmw(csrIdx, regOffset, bits, 0);
}

static void
drp_showReg(uint32_t csrIdx, const char *msg)
{
    int i;
    static const uint16_t regMap[] = {
        DRP_REG_ES_QUAL_MASK0,  DRP_REG_ES_QUAL_MASK1,  DRP_REG_ES_QUAL_MASK2,
        DRP_REG_ES_QUAL_MASK3,  DRP_REG_ES_QUAL_MASK4,  DRP_REG_ES_SDATA_MASK0,
        DRP_REG_ES_SDATA_MASK1, DRP_REG_ES_SDATA_MASK2, DRP_REG_ES_SDATA_MASK3,
        DRP_REG_ES_SDATA_MASK4, DRP_REG_ES_PS_VOFF,     DRP_REG_ES_HORZ_OFFSET,
        DRP_REG_ES_CSR,         DRP_REG_ES_ERROR_COUNT, DRP_REG_ES_SAMPLE_COUNT,
        DRP_REG_ES_STATUS, DRP_REG_PMA_RSV2, DRP_REG_TXOUT_RXOUT_DIV };
    printf("\nEYE SCAN REGISTERS AT %d (%s):\n", csrIdx, msg);
    for (i = 0 ; i < (sizeof regMap / sizeof regMap[0]) ; i++) {
        int reg = drp_read(csrIdx, regMap[i]);
        if (reg < 0) {
            printf("  %03X: <DRP lockup>\n", regMap[i]);
        }
        else {
            printf("  %03X: %04X\n", regMap[i], reg);
        }
    }
}

/*
 * Eye scan initialization may change the value of PMA_RSV2[5].
 * This means that a subsequent PMA reset is required.
 * Ensure that main() invokes eye scan initialization before transceiver
 * initialization since the latter peforms the required a PMA reset.
 */
void
eyescanInit(void)
{
    int lane, r;
    uint32_t csrIdx;

    for (lane = 0 ; lane < EYESCAN_LANECOUNT ; lane++) {
        csrIdx = lane ? GPIO_IDX_EVG_2_DRP_CSR : GPIO_IDX_EVG_1_DRP_CSR;
        /* Enable eye scan if necessary. PMA_RSV2[5] */
        if ((drp_read(csrIdx, DRP_REG_PMA_RSV2) & (1U << 5)) == 0) {
            drp_set(csrIdx, DRP_REG_PMA_RSV2, 1U << 5);
        }

        /* Enable statistical eye scan */
        drp_set(csrIdx, DRP_REG_ES_CSR, ES_CSR_EYE_SCAN_EN | ES_CSR_ERRDET_EN);

        /* Set prescale */
        drp_rmw(csrIdx, DRP_REG_ES_PS_VOFF, ES_PS_VOFF_PRESCALE_MASK,
                                    PRESCALE_CODE << ES_PS_VOFF_PRESCALE_SHIFT);

        /* Set 80 bit ES_SDATA_MASK to check 40 or 20 bit data */
        drp_write(csrIdx, DRP_REG_ES_SDATA_MASK0, DRP_REG_ES_SDATA_MASK0_VALUE);
        drp_write(csrIdx, DRP_REG_ES_SDATA_MASK1, DRP_REG_ES_SDATA_MASK1_VALUE);
        drp_write(csrIdx, DRP_REG_ES_SDATA_MASK2, 0xFF00);
        drp_write(csrIdx, DRP_REG_ES_SDATA_MASK3, 0xFFFF);
        drp_write(csrIdx, DRP_REG_ES_SDATA_MASK4, 0xFFFF);

        /* Enable all bits in ES_QUAL_MASK (count all bits) */
        for (r = DRP_REG_ES_QUAL_MASK0 ; r <= DRP_REG_ES_QUAL_MASK4 ; r++) {
            drp_write(csrIdx, r, 0xFFFF);
        }
    }
}

/* 2 characters per horizontal position except for last */
#define PLOT_WIDTH (33 * 2 - 1)
static int
xBorderCrank(int lane)
{
    const int titleOffset = 2;
    static int i = PLOT_WIDTH + 1, nameIndex, laneIndex;
    char c;

    if (eyescanFormat == FMT_RAW) return 0;
    if (lane >= 0) {
        i = -1;
        nameIndex = 0;
        laneIndex = lane;
    }
    if (i <= PLOT_WIDTH) {
        if (i == -1) {
            printf("+");
        }
        else if (i == PLOT_WIDTH) {
                printf("+\n");
        }
        else {
            if (i == PLOT_WIDTH / 2) {
                c = '+';
            }
            else if ((laneIndex < (sizeof laneNames/sizeof laneNames[0]))
                  && (i >= titleOffset)
                  && (laneNames[laneIndex][nameIndex] != '\0')) {
                c = laneNames[laneIndex][nameIndex++];
            }
            else {
                c = '-';
            }
            printf("%c", c);
        }
        i++;
    }
    return (i <= PLOT_WIDTH);
}

/*
 * Map log2err(errorCount) onto grey scale or alphanumberic value.
 * Special cases for 0 which is the only value that maps to ' ' and for
 * full scale error (131070) which is the only value that maps to '@'.
 */
static int
plotChar(int errorCount)
{
    int i, c;

    if (errorCount <= 0)           c = ' ';
    else if (errorCount >= 131070) c = '@';
    else {
        int log2err = 31 - __builtin_clz(errorCount);
        if (eyescanFormat == FMT_NUMERIC) {
            c = (log2err <= 9) ? '0' + log2err : 'A' - 10 + log2err;
        }
        else {
            i = 1 + ((log2err * 8) / 18);
            c = " .:-=?*#%@"[i];
        }
    }
    return c;
}

static int
eyescanStep(int lane)
{
    static int eyescanActive, eyescanAcquiring, eyescanLane;
    static int passCount;
    static int hRange, hStride;
    static int hOffset, vOffset, utFlag;
    static int errorCount;
    static uint32_t csrIdx;

    if ((lane >= 0) && !eyescanActive) {
        csrIdx = lane ? GPIO_IDX_EVG_2_DRP_CSR : GPIO_IDX_EVG_1_DRP_CSR;
        /* Want 32 horizontal offsets on either side of csrIdxline */
        int rxDiv = 1 << (drp_read(csrIdx, DRP_REG_TXOUT_RXOUT_DIV) & 0x3);
        hRange = 32 * rxDiv; 
        hStride = 2 * rxDiv;
        hOffset = -hRange;
        vOffset = -VRANGE;
        utFlag = 0;
        errorCount = 0;
        xBorderCrank(lane);
        eyescanLane = lane;
        eyescanAcquiring = 0;
        eyescanActive = 1;
        return 1;
    }
    if (xBorderCrank(-1)) {
        return 1;
    }
    if (eyescanActive) {
        if (eyescanAcquiring) {
            int status = drp_read(csrIdx, DRP_REG_ES_STATUS);
            if ((status & ES_STATUS_DONE) || (++passCount > 10000000)) {
                drp_clr(csrIdx, DRP_REG_ES_CSR, ES_CSR_CONTROL_RUN);
                eyescanAcquiring = 0;
                if (status == (ES_STATUS_STATE_END | ES_STATUS_DONE)) {
                    char border = vOffset == 0 ? '+' : '|';
                    errorCount += drp_read(csrIdx, DRP_REG_ES_ERROR_COUNT);
                    utFlag = !utFlag;
                    if (!utFlag) {
                        if (eyescanFormat == FMT_RAW) {
                            printf(" %6d", errorCount);
                            border = ' ';
                        }
                        else {
                            char c;
                            printf("%c", (hOffset == -hRange) ? border : ' ');
                            if ((errorCount==0) && (hOffset==0) && (vOffset==0))
                                c = '+';
                            else
                                c = plotChar(errorCount);
                            printf("%c", c);
                        }
                        errorCount = 0;
                        hOffset += hStride;
                        if (hOffset > hRange) {
                            printf("%c\n", border);
                            hOffset = -hRange;
                            vOffset += VSTRIDE;
                        }
                    }
                }
                else {
                    drp_showReg(csrIdx, "SCAN FAILURE");
                    eyescanActive = 0;
                }
            }
        }
        else if (vOffset > VRANGE) {
            xBorderCrank(eyescanLane);
            eyescanActive = 0;
        }
        else {
            int hSign;
            int vSign, vAbs;
            
            hSign = (hOffset < 0) ? 1 << 11 : 0;
            if (vOffset < 0) {
                vAbs = -vOffset;
                vSign = 1 << 7;
            }
            else {
                vAbs = vOffset;
                vSign = 0;
            }
            if (vAbs > 127) vAbs = 127;
            drp_rmw(csrIdx, DRP_REG_ES_PS_VOFF, ES_PS_VOFF_VOFFSET_MASK,
                                        (utFlag ? (1 << 8) : 0) | vSign | vAbs);
            drp_write(csrIdx, DRP_REG_ES_HORZ_OFFSET, hSign | (hOffset & 0x7FF));
            drp_set(csrIdx, DRP_REG_ES_CSR, ES_CSR_CONTROL_RUN);
            passCount = 0;
            eyescanAcquiring = 1;
        }
    }
    return eyescanActive;
}

int
eyescanCrank(void)
{
    return eyescanStep(-1);
}

int
eyescanCommand(int argc, char **argv)
{
    int i;
    char *endp;
    enum eyescanFormat format = FMT_ASCII_ART;
    int evg = 0;

    for (i = 1 ; i < argc ; i++) {
        if (argv[i][0] == '-') {
            switch (argv[i][1]) {
            case 'n': format = FMT_NUMERIC;   break;
            case 'r': format = FMT_RAW;       break;
            default: return 1;
            }
            if (argv[i][2] != '\0') return 1;
        }
        else {
            evg = strtol(argv[i], &endp, 0);
            if ((*endp != '\0') || (evg <= 0) || (evg > EYESCAN_LANECOUNT))
                return 1;
        }
    }
    if (evg <= 0) evg = 1;
    eyescanFormat = format;
    eyescanStep(evg - 1);
    return 0;
}
