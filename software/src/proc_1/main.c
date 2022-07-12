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
#include <stdint.h>
#include "epics.h"
#include "evg.h"
#include "fmcIO.h"
#include "gpio.h"
#include "ipc.h"
#include "mgt.h"
#include "platform.h"
#include "softwareBuildDate.h"
#include "systemParameters.h"
#include "tod.h"
#include "util.h"

int
main(void)
{
    /* Set up infrastructure */
    init_platform();

    /* Wait for other processor to set things up */
    while(!sharedMemory->enableEPICSprocessor) continue;

    /*
     * Start things
     */
    mgtInit();
    evgInit();
    bwudpRegisterInterface(
           (ethernetMAC *)&sharedMemory->systemParameters.netConfig.ethernetMAC,
           (ipv4Address *)&sharedMemory->systemParameters.netConfig.np.address,
           (ipv4Address *)&sharedMemory->systemParameters.netConfig.np.netmask,
           (ipv4Address *)&sharedMemory->systemParameters.netConfig.np.gateway);
    ipcInit();
    epicsInit();
    todInit();

    /*
     * Main processing loop
     */
    for (;;) {
        bwudpCrank();
        sequencerStatusPublisherCrank();
        ipcCrank();
        todCrank();
        mgtCrank();
        if (debugFlags & DEBUGFLAG_SHOW_SEQUENCES) {
            int e, s;
            debugFlags &= ~DEBUGFLAG_SHOW_SEQUENCES;
            for (e = 1 ; e <= EVG_COUNT ; e++) {
                for (s = 0 ; s < 2 ; s++) {
                    evgDumpSequence(e, s);
                    bwudpCrank();
                }
            }
        }
    }

    /* Never reached */
    cleanup_platform();
    return 0;
}
