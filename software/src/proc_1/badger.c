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
 * Badger ethernet-in-fabric I/O for bwudp
 */
#include <stdio.h>
#include "bwudp.h"
#include "gpio.h"

#define CONFIG_CSR_ENABLE_RX_ENABLE 0x80000000
#define CONFIG_CSR_RX_ENABLE        0x40000000
#define CONFIG_CSR_ADDRESS_MASK     (0xF << CONFIG_CSR_ADDRESS_SHIFT)
#define CONFIG_CSR_ADDRESS_SHIFT    8
#define CONFIG_CSR_DATA_MASK        0xFF

#define TX_CSR_W_START          0x80000000
#define TX_CSR_R_BUSY           0x80000000
#define TX_CSR_R_TOGGLE_A       0x40000000
#define TX_CSR_R_TOGGLE_B       0x20000000
#define TX_CSR_ADDRESS_MASK     (0x7FF << TX_CSR_ADDRESS_SHIFT)
#define TX_CSR_ADDRESS_SHIFT    16
#define TX_CSR_DATA_MASK        0xFFFF

#define RX_CSR_R_MAC_BANK           0x1
#define RX_CSR_R_MAC_HBANK          0x2
#define RX_CSR_W_MAC_HBANK_TOGGLE   0x2

void
bwudpInitializeInterface(const uint8_t *ethernetAddress,
                         const uint8_t *ipv4Addr)
{
    int i;
    int a = 0;

    /* Configure MAC address */
    for (i = 0 ; i < 6 ; i++, a++) {
        GPIO_WRITE(GPIO_IDX_NET_CONFIG_CSR, (a << CONFIG_CSR_ADDRESS_SHIFT) |
                                                            ethernetAddress[i]);
    }

    /* Configure IPv4 address */
    for (i = 0 ; i < 4 ; i++, a++) {
        GPIO_WRITE(GPIO_IDX_NET_CONFIG_CSR, (a << CONFIG_CSR_ADDRESS_SHIFT) |
                                                                   ipv4Addr[i]);
    }

    /* Enable packet reception */
    GPIO_WRITE(GPIO_IDX_NET_CONFIG_CSR, CONFIG_CSR_ENABLE_RX_ENABLE |
                                        CONFIG_CSR_RX_ENABLE);
}

void
bwudpSendFrame(const void *frame, int length)
{
    const uint16_t *p16 = frame;
    /* Minimum is 60 since value doesn't include final four FCS octets */
    int frameLength = length < 60 ? 60 : length;
    int index = 0, limit;

    while (GPIO_READ(GPIO_IDX_NET_TX_CSR) & TX_CSR_R_BUSY) continue;
    GPIO_WRITE(GPIO_IDX_NET_TX_CSR, frameLength);
    limit = (length + 1) >> 1;
    while (index < limit) {
        index++;
        GPIO_WRITE(GPIO_IDX_NET_TX_CSR, (index << TX_CSR_ADDRESS_SHIFT) | *p16);
        p16++;
    }
    GPIO_WRITE(GPIO_IDX_NET_TX_CSR, TX_CSR_W_START);
}

int
bwudpFetchFrame(void *frame)
{
    uint32_t csr = GPIO_READ(GPIO_IDX_NET_RX_CSR);
    uint32_t length_status;
    int length;
    uint16_t *dst = frame;
    uint16_t *limit;
    uint32_t index = 1;

    if (((csr & RX_CSR_R_MAC_BANK) != 0) != ((csr & RX_CSR_R_MAC_HBANK) != 0)) {
        return 0;
    }
    GPIO_WRITE(GPIO_IDX_NET_RX_CSR, RX_CSR_W_MAC_HBANK_TOGGLE);
    GPIO_WRITE(GPIO_IDX_NET_RX_DATA, 0);
    length_status = GPIO_READ(GPIO_IDX_NET_RX_DATA);
    /* Length is encoded in a rather unusual fashion and describes
     * full frame, including ethernet header, payload and FCS. */
    length = (((length_status & 0xF00) >> 1) | (length_status & 0x7F));
    /* Don't include FCS in copy or returned count */
    length -= 4;
    /* Minimum is ARP or 0-payload UDP.  Maximum is full ethernet payload. */
    if ((length < (14 + 20 + 8)) || (length > (14 + 1500))) {
        return 0;
    }
    limit = dst + ((length + 1) >> 1);
    while (dst < limit) {
        uint32_t v;
        GPIO_WRITE(GPIO_IDX_NET_RX_DATA, index);
        v = GPIO_READ(GPIO_IDX_NET_RX_DATA);
        /* We're a little-endian architecture */
        *dst++ = v;
        if (dst >= limit) break;
        *dst++ = v >> 16;
        index++;
    }
    return length;
}
