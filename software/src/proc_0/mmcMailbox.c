/*
 * Communicate with microcontroller
 */

#include <stdio.h>
#include <stdint.h>
#include <xparameters.h>
#include "gpio.h"
#include "mmcMailbox.h"
#include "util.h"

#define MMC_MAILBOX_CAPACITY    (1 << 11)

#define CSR_WRITE_ENABLE    (1UL << 31)
#define CSR_ADDR_SHIFT      8
#define CSR_DATA_MASK       0xFF

/*
 * Mailboxes
 *
 * Page 2, location 0 controls MGT4-7 routing:
 *    MUX3   MUX2   MUX1      MGT4         MGT5         MGT6         MGT7
 *      0      0      0     FCM2-DP0     FMC2-DP1     FMC2-DP2     FMC1-DP1
 *      0      0      1     FCM2-DP0     FMC2-DP1     FMC1-DP0     FMC1-DP1
 *      0      1      0     FCM2-DP0     FMC2-DP1     FMC2-DP2     FMC2-DP3
 *      0      1      1     FCM2-DP0     FMC2-DP1     FMC1-DP0     FMC2-DP3
 *      1      X      X    QSFP2:4/9    QSFP2:1/12   QSFP2:2/11   QSFP2:3/10
 *
 * Tile 115-0 -- MGT_[RT]X_4 -- MGT_[RT]X_4_FMC2 -- FMC2-DP0
 *                           or MGT_[RT]X_4_QSFP -- QSFP2_[RT]X_3 -- QSFP:3/10
 *
 * Tile 115-1 -- MGT_[RT]X_5 -- MGT_[RT]X_5_FMC2 -- FMC2-DP1
 *                           or MGT_[RT]X_5_QSFP -- QSFP2_[RT]X_0 -- QSFP:1/12
 *
 * Tile 115-2 -- MGT_[RT]X_6 -- MGT_[RT]X_6_FMC -- MGT_[RT]X_6_FMC2 -- FMC2-DP2
 *                                              or MGT_[RT]X_6_FMC1 -- FMC1-DP0
 *                           or MGT_[RT]X_6_QSFP -- QSFP2_[RT]X_1 -- QSFP:2/11
 *
 * Tile 115-3 -- MGT_[RT]X_7 -- MGT_[RT]X_7_FMC -- MGT_[RT]X_7_FMC1 -- FMC1-DP1
 *                                              or MGT_[RT]X_7_FMC2 -- FMC2-DP3
 *                           or MGT_[RT]X_7_QSFP -- QSFP2_[RT]X_2 -- QSFP:4/9
 *
 * QSFP:x/y specify fibers x and y on the 'squid'.
 *
 * Bank 115-[0-3] are X0Y[0-3]
 * Bank 116-[0-3] are X0Y[4-7]
 *
 * QSFP1 is J17, QSFP2 is J8.
 */
#define MADDR_MGT_CONFIG    0x20  /* Page 2, location 0 */
#define MADDR_PG3_COUNT     0x30
#define MADDR_U29_TEMP      0x34
#define MADDR_U28_TEMP      0x36
#define MADDR_MMC_BUILD     0x3C
#define MADDR_PG4_COUNT     0x4A

/*
 * Page 2, location 0 controls MGT4-7 routing and FMC power.
 *   4 pairs of bits: High bit enables configuration bit update.
 *                    Low bit is new state or "no change" if high bit clear.
 */
# define MGT_CONFIG_SET_MUX3       0xC0
# define MGT_CONFIG_CLR_MUX3       0x80
# define MGT_CONFIG_SET_MUX2       0x30
# define MGT_CONFIG_CLR_MUX2       0x20
# define MGT_CONFIG_SET_MUX1       0x0C
# define MGT_CONFIG_CLR_MUX1       0x08
# define MGT_CONFIG_SET_FMC        0x03
# define MGT_CONFIG_CLR_FMC        0x02
/* MGT6:FMC1-DP0, MGT4:FMC2-DP0, FMC on */
# define MGT_CONFIG_FMC (MGT_CONFIG_CLR_MUX3 | MGT_CONFIG_SET_MUX2 | \
                         MGT_CONFIG_SET_MUX1 | MGT_CONFIG_SET_FMC)
/* MGT6:QSFP2:2/11, MGT4:QSFP2:3/10, FMC on */
# define MGT_CONFIG_QSFP (MGT_CONFIG_SET_MUX3 | MGT_CONFIG_SET_FMC)

/* MGTMUX config masks the result with 0x55 on the MMC side */
# define MGT_CONFIG_RESULT_MASK    0x55

/*
 * MMC usually updates the register map every 1s.
 * So, set the timeout to 5s
 */
#define MMC_UPDATE_TIMEOUT         5000000

static int initDone;

void
mmcMailboxWrite(unsigned int address, int value)
{
    if (address < MMC_MAILBOX_CAPACITY) {
        GPIO_WRITE(GPIO_IDX_MMC_MAILBOX, (address << CSR_ADDR_SHIFT) |
                                    CSR_WRITE_ENABLE | (value & CSR_DATA_MASK));
    }
}

static void
mmcMailboxWriteAndWait(unsigned int address, int value, int result)
{
    uint32_t then;
    mmcMailboxWrite(address, value);
    then = MICROSECONDS_SINCE_BOOT();
    while ((GPIO_READ(GPIO_IDX_MMC_MAILBOX) & CSR_DATA_MASK) != result) {
        if ((MICROSECONDS_SINCE_BOOT() - then) > MMC_UPDATE_TIMEOUT) {
            warn("mmcMailboxWriteAndWait(0x%02x) timed out", address);
            return;
        }
    }
}

int
mmcMailboxRead(unsigned int address)
{
    if (address < MMC_MAILBOX_CAPACITY) {
        GPIO_WRITE(GPIO_IDX_MMC_MAILBOX, (address << CSR_ADDR_SHIFT));
        return GPIO_READ(GPIO_IDX_MMC_MAILBOX) & CSR_DATA_MASK;
    }
    return -1;
}

static int
mmcMailboxRead16(unsigned int address)
{
    int16_t v0 = (mmcMailboxRead(address) << 8) | mmcMailboxRead(address+1);

    for (int i = 0; i < 500; i++) {
        int16_t v = (mmcMailboxRead(address) << 8) | mmcMailboxRead(address+1);
        if (v == v0) return v;
        v0 = v;
    }
}

static void
showLM75temperature(int id, int address)
{
    int v = mmcMailboxRead16(address);
    printf("  U%d: %d.%d C\n", id, v / 2, (v & 0x1) * 5);
}

int
getU28temperature(void) {
    int t = mmcMailboxRead16(MADDR_U28_TEMP);
    return t*5 + (t&0x1)*5;
}

int
getU29temperature(void) {
    int t = mmcMailboxRead16(MADDR_U29_TEMP);
    return t*5 + (t&0x1)*5;
}

uint16_t
getMMCPG3Count()
{
    return mmcMailboxRead16(MADDR_PG3_COUNT);
}

uint16_t
getMMCPG4Count()
{
    return mmcMailboxRead16(MADDR_PG4_COUNT);
}

int
mmcMailboxIsInit()
{
    return initDone;
}

int
getMMCfirmware(void) {
    uint32_t c=0;
    for (uint8_t i = 0 ; i < 4 ; i++) {
        c |= mmcMailboxRead(MADDR_MMC_BUILD + i)<<(24-8*i);
    }
    return c;
}

static void
showMMCfirmware(void)
{
    int i;
    printf("  Firmware: ");
    for (i = 0 ; i < 4 ; i++) {
        int c = mmcMailboxRead(MADDR_MMC_BUILD + i);
        printf("%02X", c);
    }
    printf("\n");
}

static int
mmcMailboxIsValid()
{
    uint32_t then, check;
    uint16_t counter;

    counter = getMMCPG3Count();
    check = then = MICROSECONDS_SINCE_BOOT();
    while (getMMCPG3Count() == counter) {
        if ((check = (MICROSECONDS_SINCE_BOOT() - then)) > MMC_UPDATE_TIMEOUT) {
            warn("mmcMailboxIsValid() timed out reading PG3 count");
            return 0;
        }
    }

    printf("MMC mailbox valid after %u ms\n", check/1000);
    return 1;
}

int
mmcMailboxInit(void)
{
    int mgtCfg = (CFG_MGT_FMC_OR_QSFP == 0)? MGT_CONFIG_FMC : MGT_CONFIG_QSFP;

    if(!mmcMailboxIsValid()) {
        return 0;
    }

    mmcMailboxWriteAndWait(MADDR_MGT_CONFIG, mgtCfg,
            mgtCfg & MGT_CONFIG_RESULT_MASK);
    microsecondSpin(100);
    printf("Microcontroller:\n");
    showLM75temperature(28, MADDR_U28_TEMP);
    showLM75temperature(29, MADDR_U29_TEMP);
    showMMCfirmware();

    initDone = 1;
    return 1;
}

uint32_t *
mmcMailboxFetchSysmon(uint32_t *ap)
{
    *ap++ = (mmcMailboxRead16(MADDR_U29_TEMP) << 16) |
            (mmcMailboxRead16(MADDR_U28_TEMP) & 0xFFFF);
    return ap;
}
