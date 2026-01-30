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
 * Wrapper around SPI flash support https://github.com/pellepl/spiflash_driver
 */
#include <stdio.h>
#include <xparameters.h>
#include "bootFlash.h"
#include "gpio.h"
#include "util.h"

#ifdef BOOT_FLASH_SMALL_SECTORS_AT_TOP
# define BOOT_FLASH_LO_SECTOR_COUNT 32
# define BOOT_FLASH_LO_SECTOR_SIZE  (4*1024)
# define BOOT_FLASH_HI_SECTOR_SIZE  (64*1024)
#else
# define BOOT_FLASH_LO_SECTOR_COUNT 254
# define BOOT_FLASH_LO_SECTOR_SIZE  (64*1024)
# define BOOT_FLASH_HI_SECTOR_SIZE  (4*1024)
#endif

#ifdef BOOT_FLASH_32MB_VARIANT
# define BOOT_FLASH_SIZE             (32*1024*1024)
#else
# define BOOT_FLASH_SIZE             (16*1024*1024)
#endif

#define BOOT_FLASH_BIG_SECTOR_SIZE  (64*1024)

#define BOOT_FLASH_WP_SECTOR_SIZE   (BOOT_FLASH_SIZE / 64)

#define SPIF_DBG(...) if(debugFlags&DEBUGFLAG_BOOT_FLASH)printf("SPIFL:" __VA_ARGS__)

#define CSR_W_CLK_SET  0x1
#define CSR_W_CLK_CLR  0x2
#define CSR_W_CS_B_SET 0x4
#define CSR_W_CS_B_CLR 0x8
#define CSR_W_MOSI_SET 0x10
#define CSR_W_MOSI_CLR 0x20

#define CSR_R_CLK      0x1
#define CSR_R_CS_B     0x4
#define CSR_R_MOSI     0x10
#define CSR_R_MISO     0x40

#include "s25flxxxs.h"
#include "spiflash.h"

static struct bootFlash_t {
    spiflash_t spif;
    uint32_t wp_start;
    uint32_t wp_size;
} bootFlash = {
    .wp_start = 0,
    // Assume the lower half of the flash is protected
    .wp_size = BOOT_FLASH_SIZE / 2
};

/*
 * Can't name the source spiflash.c since then the Xilinx tools insist
 * on compiling it as well, resulting in all sorts of duplicate symbols.
 */
#include "spiflash.inc"

static int
spiFlashTxRx(struct spiflash_s *spi, const uint8_t *tx_data, uint32_t tx_len,
                                           uint8_t *rx_data, uint32_t rx_len)
{
    if(debugFlags & DEBUGFLAG_BOOT_FLASH) {
        printf("spiFlashTxRx: W %d %d", tx_len, rx_len);
    }

    while (tx_len--) {
        int w = *tx_data++;
        int b;

        if(debugFlags & DEBUGFLAG_BOOT_FLASH) {
            printf(" %02X", w);
        }

        for (b = 0x80 ; b != 0 ; b >>= 1) {
            GPIO_WRITE(GPIO_IDX_QSPI_FLASH_CSR,
                   ((w & b) ? CSR_W_MOSI_SET : CSR_W_MOSI_CLR) | CSR_W_CLK_CLR);
            GPIO_WRITE(GPIO_IDX_QSPI_FLASH_CSR, CSR_W_CLK_SET);
        }
    }
    while (rx_len) {
        int r = 0;
        int b;
        for (b = 0x80 ; b != 0 ; b >>= 1) {
            GPIO_WRITE(GPIO_IDX_QSPI_FLASH_CSR, CSR_W_CLK_SET);
            GPIO_WRITE(GPIO_IDX_QSPI_FLASH_CSR, CSR_W_CLK_CLR);
            /*
             * The assumption is that there is enough delay between
             * the clock going low and the data being read.
             * Add another CSR_W_CLK_CLR operation here
             * if that assumption is invalid.
             */
            if (GPIO_READ(GPIO_IDX_QSPI_FLASH_CSR) & CSR_R_MISO) {
                r |= b;
            }
        }
        rx_len--;
        *rx_data++ = r;

        if(debugFlags & DEBUGFLAG_BOOT_FLASH) {
            printf(" (%02X)", r);
        }
    }

    GPIO_WRITE(GPIO_IDX_QSPI_FLASH_CSR, CSR_W_CLK_CLR);

    if(debugFlags & DEBUGFLAG_BOOT_FLASH) {
        printf("\n");
    }

    return SPIFLASH_OK;
}

static void
spiFlashCS(struct spiflash_s *spi, uint8_t cs)
{
    if(debugFlags & DEBUGFLAG_BOOT_FLASH) {
        printf("spiFlashCS %d\n", cs);
    }

    GPIO_WRITE(GPIO_IDX_QSPI_FLASH_CSR, cs ? CSR_W_CS_B_CLR : CSR_W_CS_B_SET);
}

static void
spiFlashWait(struct spiflash_s *spi, uint32_t ms)
{
    microsecondSpin(ms * 1000);
}

static int
spiFlashWPAddress(uint32_t *start, uint32_t *size)
{
    int ret = SPIFLASH_OK;
    uint32_t wp_start = 0;
    uint32_t wp_size = 0;
    uint8_t reg_sr1 = 0, reg_cr1 = 0;
    uint8_t bp = 0;
    uint8_t tbprot = 0;

    ret = SPIFLASH_read_reg(&bootFlash.spif, S25FLXXXS_SR1_REG, &reg_sr1);
    ret |= SPIFLASH_read_reg(&bootFlash.spif, S25FLXXXS_CR1_REG, &reg_cr1);
    if(ret != SPIFLASH_OK) {
        return ret;
    }

    bp = S25FLXXXS_SR1_BP_R(reg_sr1);
    tbprot = S25FLXXXS_CR1_TBPROT_R(reg_cr1);

    if(debugFlags & DEBUGFLAG_BOOT_FLASH) {
        printf("spiFlashWPAddress: BP:0x%02X TBPROT:0x%02X\n", bp, tbprot);
    }

    if (bp == 0) {
        *start = 0;
        *size = 0;
        return SPIFLASH_OK;
    }

    wp_size = (BOOT_FLASH_WP_SECTOR_SIZE * (1 << (bp-1)));
    if (tbprot == 0) {
        wp_start = BOOT_FLASH_SIZE - wp_size;
    }
    else {
        wp_start = 0;
    }

    *start = wp_start;
    *size = wp_size;

    if(debugFlags & DEBUGFLAG_BOOT_FLASH) {
        printf("spiFlashWPAddress: WP start:0x%08X WP size:0x%08X\n", *start, *size);
    }

    return SPIFLASH_OK;
}

static int
spiFlashIsWriteOk(uint32_t address, uint32_t length)
{
    uint32_t wp_start = bootFlash.wp_start;
    uint32_t wp_size = bootFlash.wp_size;
    uint32_t wp_end = wp_start + wp_size;

    if (wp_size == 0) {
        return SPIFLASH_OK;
    }

    if (((address >= wp_start) && (address < wp_end)) ||
            ((address <= wp_start) && ((address + length) > wp_end))) {
        return SPIFLASH_ERR_BAD_STATE;
    }

    return SPIFLASH_OK;
}

void
bootFlashInit(void)
{
    uint32_t id;
    static const spiflash_hal_t spiFlashHAL = {
        ._spiflash_spi_txrx = spiFlashTxRx,
        ._spiflash_spi_cs = spiFlashCS,
        ._spiflash_wait     = spiFlashWait,
    };
    static const spiflash_cmd_tbl_t spiFlashCMD = SPIFLASH_CMD_TBL_STANDARD;
    static const spiflash_config_t spiFlashCFG = {
      .sz = BOOT_FLASH_SIZE,
      .page_sz = 256,
      .addr_sz = 3,
      .addr_dummy_sz = 0, // Single line data
      .addr_endian = SPIFLASH_ENDIANNESS_BIG,
      .sr_write_ms = 10,
      .page_program_ms = 2,
      .block_erase_4_ms = 50,
      .block_erase_8_ms = 0,
      .block_erase_16_ms = 0,
      .block_erase_32_ms = 0,
      .block_erase_64_ms = 200,
      .chip_erase_ms = 30000
    };
    SPIFLASH_init(&bootFlash.spif, &spiFlashCFG, &spiFlashCMD, &spiFlashHAL, NULL,
                                                    SPIFLASH_SYNCHRONOUS, NULL);
    /* Dummy read since first transaction doesn't seem to work */
    SPIFLASH_read_jedec_id(&bootFlash.spif, &id);
    if (SPIFLASH_read_jedec_id(&bootFlash.spif, &id) == SPIFLASH_OK) {
        printf("Boot flash JEDEC ID: 0x%X\n", (unsigned int)id);
    }
    else {
        warn("Can't read boot flash ID");
    }
    if (SPIFLASH_read_product_id(&bootFlash.spif, &id) == SPIFLASH_OK) {
        printf("Boot flash product ID: 0x%X\n", (unsigned int)id);
    }

    uint32_t wp_start = 0;
    uint32_t wp_size = 0;
    if (!(spiFlashWPAddress(&wp_start, &wp_size) == SPIFLASH_OK)) {
        warn("Can't read WP bits. Assuming lower half is WP");
        wp_start = 0;
        wp_size = BOOT_FLASH_SIZE / 2;
    }

    bootFlash.wp_start = wp_start;
    bootFlash.wp_size = wp_size;

    printf("Boot flash WP start:size 0x%08X:0x%08X\n",
            bootFlash.wp_start, bootFlash.wp_size);
}

int
bootFlashRead(uint32_t address, uint32_t length, void *buf)
{
    return SPIFLASH_fast_read(&bootFlash.spif, address, length, buf);
}

/*
 * The following function imposes some constraints on how it is invoked.
 *  - The first write to a sector must begin at the first address of the sector.
 *  - Writes must not span a sector boundary.
 * The TFTP server meets these constraints.
 */
int
bootFlashWrite(uint32_t address, uint32_t length, const void *buf)
{
    int ret = SPIFLASH_OK;

    ret = spiFlashIsWriteOk(address, length);
    if (ret != SPIFLASH_OK) {
        return ret;
    }

    uint32_t sectorSize =
            (address < (BOOT_FLASH_LO_SECTOR_COUNT*BOOT_FLASH_LO_SECTOR_SIZE)) ?
                          BOOT_FLASH_LO_SECTOR_SIZE : BOOT_FLASH_HI_SECTOR_SIZE;
    if ((address % sectorSize) == 0) {
        ret = SPIFLASH_erase(&bootFlash.spif, address, sectorSize);
        if (ret != SPIFLASH_OK) {
            return ret;
        }
    }
    return SPIFLASH_write(&bootFlash.spif, address, length, buf);
}

void
bfErase(void)
{
    printf("SPIFLASH_chip_erase %d\n",  SPIFLASH_chip_erase(&bootFlash.spif));
}
