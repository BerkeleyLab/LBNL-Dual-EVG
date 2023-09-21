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
#include "bootFlash.h"
#include "console.h"
#include "display.h"
#include "evgCoincidence.h"
#include "evio.h"
#include "eyescan.h"
#include "gpio.h"
#include "iicChunk.h"
#include "iicFMC.h"
#include "iicProc.h"
#include "platform.h"
#include "mgtClkSwitch.h"
#include "mmcMailbox.h"
#include "softwareBuildDate.h"
#include "st7789v.h"
#include "sysmon.h"
#include "tftp.h"
#include "util.h"

/*
 * Configuration sanity check
 */
#if((CFG_EVG1_CLK_PER_HEARTBEAT % CFG_EVG1_CLK_PER_BR_AR_ALIGNMENT)!=0)
# error "CFG_RATIO_TXCLK_PER_BR_AR_ALIGNMENT invalid"
#endif

#define SHARED_MEMORY_CAPACITY (XPAR_SHARED_RAM_S_AXI_HIGHADDR + 1 - \
                                                 XPAR_SHARED_RAM_S_AXI_BASEADDR)
static void
checkRecovery(void)
{
    int isRecovery = resetRecoverySwitchPressed();
    if (isRecovery && !sharedMemory->systemParameters.netConfig.isDefault) {
        printf("==== Recovery mode -- Using default network parameters ====\n");
        sharedMemory->systemParameters.netConfig = netDefault;
    }
    displayShowNetworkAddress();
}

int
main(void)
{
    /* Set up infrastructure */
    init_platform();
    st7789vInit();
    if (sizeof *sharedMemory > SHARED_MEMORY_CAPACITY) {
        printf("!!! Shared Memory %d > %d !!!\n", (int)sizeof *sharedMemory,
                                                  SHARED_MEMORY_CAPACITY);
    }
    sharedMemory->udpConsole.rxSize = -1;
    sharedMemory->udpConsole.txSize = -1;
    sharedMemory->udpTFTP.rxSize = -1;
    sharedMemory->udpTFTP.txSize = -1;

    /* Let UART settle down */
    microsecondSpin(20000);

    /* Screen check */
    startupScreen();

    /* Announce our presence */
    printf("\nGit ID (32-bit): 0x%08x\n", GPIO_READ(GPIO_IDX_GITHASH));
    printf("Firmware POSIX seconds: %u\n",
                         (unsigned int)GPIO_READ(GPIO_IDX_FIRMWARE_BUILD_DATE));
    printf("Software POSIX seconds: %u\n", SOFTWARE_BUILD_DATE);

    /*
     * Start things
     */
    bootFlashInit();
    systemParametersReadback();
    mmcMailboxInit();
    iicChunkInit();
    iicProcInit();
    iicFMCinit();
    evioInit();
    mgtClkSwitchInit();
    checkRecovery();
    showNetworkConfig((void *)&sharedMemory->systemParameters.netConfig.np);
    showNTPserver((void *)&sharedMemory->systemParameters.ntpHost);
    tftpReadDefaultSequence();
    eyescanInit();
    evgCoincidenceInit();
    sharedMemory->enableEPICSprocessor = 1;

    /*
     * Main processing loop
     */
    for (;;) {
        checkForReset();
        sysmonUpdate();
        consoleCheck();
        displayUpdate();
        tftpCheck();
        evioCrank();
        evgCoincidenceCrank();
    }

    /* Never reached */
    cleanup_platform();
    return 0;
}
