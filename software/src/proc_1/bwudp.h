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
 * Ultra-lightweight networking wrapper around badger low-level I/O
 */

#ifndef _BWUDP_H_
#define _BWUDP_H_

#include <stdint.h>
#include "bwudp_config.h"

#ifndef BWUDP_INTERFACE_CAPACITY
# define BWUDP_INTERFACE_CAPACITY 1
#endif
#if BWUDP_INTERFACE_CAPACITY > 1
# define BWUDP_INTERFACE_INDEX int interfaceIndex,
#else
# define BWUDP_INTERFACE_INDEX 
# define BWUDP_SERVER_INTERFACE
#endif

/*
 * Assume a little-endian machine
 */
#define ntohs(x) __builtin_bswap16(x)
#define ntohl(x) __builtin_bswap32(x)
#define htons(x) __builtin_bswap16(x)
#define htonl(x) __builtin_bswap32(x)

struct bwudpStatistics {
    uint32_t    accepted;
    uint32_t    badProtocol;
    uint32_t    mangled;
    uint32_t    rejected;
    uint32_t    arp;
};

typedef struct ethernetMAC {
    uint8_t  a[6];
} ethernetMAC;

typedef struct ipv4Address {
    uint8_t  a[4];
} ipv4Address;

int bwudpRegisterInterface(BWUDP_INTERFACE_INDEX
                           ethernetMAC *ethernetMAC,
                           ipv4Address *address,
                           ipv4Address *netmask,
                           ipv4Address *gateway);
void bwudpCrank(void);

typedef void *bwudpHandle;
typedef void (*bwudpCallback)(bwudpHandle handle, char *payload, int length);

int bwudpRegisterServer(BWUDP_INTERFACE_INDEX int port,bwudpCallback callback);
bwudpHandle bwudpCreateClient(ipv4Address *serverAddress, int serverPort,
                                         int localPort, bwudpCallback callback);
void bwudpSend(bwudpHandle handle, const char *payload, int length);

#if BWUDP_INTERFACE_CAPACITY > 1
const struct bwudpStatistics *bwudpStatistics(int interfaceIndex);
#else
const struct bwudpStatistics *bwudpStatistics(void);
#endif

/*
 * Functions to be supplied by driver
 */
void bwudpInitializeInterface(BWUDP_INTERFACE_INDEX
                    const uint8_t *ethernetAddress, const uint8_t *ipv4Address);

void bwudpSendFrame(BWUDP_INTERFACE_INDEX const void *frame, int length);
int bwudpFetchFrame(BWUDP_INTERFACE_INDEX void *frame);

#endif /* _BWUDP_H_ */
