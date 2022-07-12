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

#include <stdio.h>
#include <stdlib.h>
#include <xparameters.h>
#include "console.h"
#include "tftp.h"
#include "util.h"

const struct sysNetConfig netDefault = {
    .isDefault = 1,
    .ethernetMAC.a[0] = 0xAA,
    .ethernetMAC.a[1] = 'L',
    .ethernetMAC.a[2] = 'B',
    .ethernetMAC.a[3] = 'N',
    .ethernetMAC.a[4] = 'L',
    .ethernetMAC.a[5] = 0x04,
    .np.address.a[0] = 192,
    .np.address.a[1] = 168,
    .np.address.a[2] =   1,
    .np.address.a[3] = 129,
    .np.netmask.a[0] = 255,
    .np.netmask.a[1] = 255,
    .np.netmask.a[2] = 255,
    .np.netmask.a[3] =   0,
    .np.gateway.a[0] = 192,
    .np.gateway.a[1] = 168,
    .np.gateway.a[2] =   1,
    .np.gateway.a[3] =   1,
};


static int
checksum(void)
{
    int i, sum = 0xCAFEF00D;
    const int *ip = (int *)&sharedMemory->systemParameters;

    for (i = 0 ; i < ((sizeof sharedMemory->systemParameters -
                    sizeof sharedMemory->systemParameters.checksum) / sizeof(*ip)) ; i++)
        sum += *ip++ + i;
    if (sum == 0) sum = 0xABCD0341;
    return sum;
}

void systemParametersUpdateChecksum(void)
{
    sharedMemory->systemParameters.checksum = checksum();
}

/*
 * Read and process values on system startup.
 * Perform sanity check on parameters read from flash.
 * If they aren't good then assign default values.
 */
void
systemParametersReadback(void)
{
    if ((tftpReadSystemParameters(sizeof sharedMemory->systemParameters,
                (struct systemParameters *)&sharedMemory->systemParameters) < 0)
     || (checksum() != sharedMemory->systemParameters.checksum)) {
        printf("\n====== ASSIGNING DEFAULT PARAMETERS ===\n\n");
        sharedMemory->systemParameters.netConfig = netDefault;
        sharedMemory->systemParameters.ntpHost.a[0] = 0;
        sharedMemory->systemParameters.ntpHost.a[1] = 0;
        sharedMemory->systemParameters.ntpHost.a[2] = 0;
        sharedMemory->systemParameters.ntpHost.a[3] = 0;
        sharedMemory->systemParameters.pllPhaseShift[0] = 0;
        sharedMemory->systemParameters.pllPhaseShift[1] = 0;
        sharedMemory->systemParameters.startupDebugFlags = 0;
    }
    debugFlags = sharedMemory->systemParameters.startupDebugFlags;
}

/*
 * Update flash
 */
void
systemParametersStash(void)
{
    systemParametersUpdateChecksum();
    if (tftpWriteSystemParameters(sizeof(struct systemParameters),
              (struct systemParameters *)&sharedMemory->systemParameters) < 0) {
        printf("Unable to write system parameteres to flash.\n");
    }
}

/*
 * Serializer/deserializers
 * Note -- Format routines share common static buffer.
 * Go through some contortions to avoid calling printf and bloating executable by >60 kB.
 */
static char cbuf[40];

char *
formatMAC(const void *val)
{
    const uint8_t *addr = (const uint8_t *)val;
    sprintf(cbuf, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2],
                                                   addr[3], addr[4], addr[5]);
    return cbuf;
}

int
parseMAC(const char *str, void *val)
{
    const char *cp = str;
    int i = 0;
    long l;
    char *endp;

    for (;;) {
        l = strtol(cp, &endp, 16);
        if ((l < 0) || (l > 255))
            return -1;
        *((uint8_t*)val + i) = l;
        if (++i == 6)
            return endp - str;
        if (*endp++ != ':')
            return -1;
        cp = endp;
    }
}

char *
formatIP(const void *val)
{
    const uint8_t *addr = val;
    sprintf(cbuf, "%d.%d.%d.%d", addr[0], addr[1], addr[2], addr[3]);
    return cbuf;
}

int
parseIP(const char *str, void *val)
{
    const char *cp = str;
    uint8_t *addr = val;
    int i = 0;
    long l;
    char *endp;

    for (;;) {
        l = strtol(cp, &endp, 10);
        if ((l < 0) || (l > 255))
            return -1;
        *addr++ = l;
        if (++i == 4) {
            return endp - str;
        }
        if (*endp++ != '.')
            return -1;
        cp = endp;
    }
}

void
showNetworkConfig(const struct sysNetParms *np)
{
    printf("   IP ADDR: %s\n", formatIP(&np->address));
    printf("  NET MASK: %s\n", formatIP(&np->netmask));
    printf("   GATEWAY: %s\n", formatIP(&np->gateway));
}

void
showNTPserver(const ipv4Address *ntpHost)
{
    printf("NTP SERVER: %s\n", formatIP(ntpHost));
}
