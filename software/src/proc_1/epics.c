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
 * Accept and act upon commands from the IOC
 */
#include <stdio.h>
#include <string.h>
#include "evg.h"
#include "evgProtocol.h"
#include "bwudp.h"
#include "epics.h"
#include "fmcIO.h"
#include "gpio.h"
#include "injectionCycle.h"
#include "ipc.h"
#include "mgt.h"
#include "softwareBuildDate.h"
#include "swapoutCycle.h"
#include "tod.h"
#include "util.h"

/*
 * Handle a reboot request
 */
static void
crankRebootStateMachine(int value)
{
    static uint16_t match[] = { 1, 100, 10000 };
    static int i;

    if (value == match[i]) {
        i++;
        if (i == (sizeof match / sizeof match[0])) {
            sharedMemory->resetFPGA = 1;
            microsecondSpin(100000);
        }
    }
    else if (value == match[0]) {
        i = 1;
    }
    else {
        i = 0;
    }
}

/*
 * Handle a transceiver realign request
 */
static void
crankRealignStateMachine(int value)
{
    static uint16_t match[] = { 1, 10, 100};
    static int i;

    if (value == match[i]) {
        i++;
        if (i == (sizeof match / sizeof match[0])) {
            mgtTxReset(0x3);
            evgAlign();
        }
    }
    else if (value == match[0]) {
        i = 1;
    }
    else {
        i = 0;
    }
}

/*
 * Read fan tachometers
 * Works for even or odd number of fans
 */
static int
fetchFanSpeeds(uint32_t *ap)
{
    int i, shift = 0, count = 0;
    uint32_t v = 0;
    for (i = 0 ; i < CFG_FAN_COUNT ; i++) {
        if (shift > 16) {
            *ap++ = v;
            count++;
            shift = 0;
        }
        GPIO_WRITE(GPIO_IDX_FAN_TACHOMETERS, i);
        v |= (GPIO_READ(GPIO_IDX_FAN_TACHOMETERS) & 0xFFFF) << shift;
        shift += 16;
    }
    *ap = v;
    return count + 1;
}

/*
 * Fetch system monitors, first from the other processor, then local.
 */
static int
sysmonFetch(uint32_t *args)
{
    int i, proc0count = sharedMemory->sysmonCount;
    uint32_t *ap = args;
    for (i = 0 ;i < proc0count ; i++) {
        *ap++ = sharedMemory->sysmonBuf[i];
    }
    ap += evgFetchCoincidenceStatus(ap);
    ap += injectionCycleFetchStatus(ap);
    *ap++ = todStatus();
    ap += fmcIOfetchStatus(ap);
    *ap++ = GPIO_READ(GPIO_IDX_SWAPOUT_CYCLE_CSR);
    ap += fetchFanSpeeds(ap);
    ap += mgtFetchStatus(ap);
    return ap - args;
}

/*
 * Process command
 */
static int
handleCommand(int commandArgCount, struct evgPacket *cmdp,
                                   struct evgPacket *replyp)
{
    int lo = cmdp->command & EVG_PROTOCOL_CMD_MASK_LO;
    int idx = cmdp->command & EVG_PROTOCOL_CMD_MASK_IDX;
    int replyArgCount = 0;
    static int powerUpStatus = 1;

    switch (cmdp->command & EVG_PROTOCOL_CMD_MASK_HI) {
    case EVG_PROTOCOL_CMD_HI_LONGIN:
        if (commandArgCount != 0) return -1;
        replyArgCount = 1;
        switch (lo) {
        case EVG_PROTOCOL_CMD_LONGIN_LO_GENERIC:
            switch (idx) {
            case EVG_PROTOCOL_CMD_LONGIN_IDX_FIRMWARE_BUILD_DATE:
                replyp->args[0] = GPIO_READ(GPIO_IDX_FIRMWARE_BUILD_DATE);
                break;

            case EVG_PROTOCOL_CMD_LONGIN_IDX_SOFTWARE_BUILD_DATE:
                replyp->args[0] = SOFTWARE_BUILD_DATE;
                break;

            case EVG_PROTOCOL_CMD_LONGIN_IDX_GIT_HASH_ID:
                replyp->args[0] = GPIO_READ(GPIO_IDX_GITHASH);
                break;

            default: return -1;
            }
            break;

        case EVG_PROTOCOL_CMD_LONGIN_LO_PING_LATENCY:
            {
            int l = mgtFetchLatency(idx);
            if (l < 0) return -1;
            replyp->args[0] = l;
            }
            break;

        default: return -1;
        }
        break;

    case EVG_PROTOCOL_CMD_HI_LONGOUT:
        if (commandArgCount != 1) return -1;
        switch (lo) {
        case EVG_PROTOCOL_CMD_LONGOUT_LO_NO_VALUE:
            switch (idx) {
            case EVG_PROTOCOL_CMD_LONGOUT_NV_IDX_CLEAR_POWERUP_STATUS:
                powerUpStatus = 0;
                break;

            case EVG_PROTOCOL_CMD_LONGOUT_NV_IDX_RESYNC_TOD:
                todResync();
                break;

            case EVG_PROTOCOL_CMD_LONGOUT_NV_IDX_TRIGGER_INJECTION:
                injectionCycleManualTrigger();
                break;

            default: return -1;
            }
            break;

        case EVG_PROTOCOL_CMD_LONGOUT_LO_SW_TRIGGER:
            evgSoftwareTrigger(idx, cmdp->args[0]);
            break;

        case EVG_PROTOCOL_CMD_LONGOUT_LO_HW_TRIGGER:
            evgHardwareTrigger(idx, cmdp->args[0]);
            break;

        case EVG_PROTOCOL_CMD_LONGOUT_LO_SEQ_ENABLE:
            evgEnableSequence(idx, cmdp->args[0]);
            break;

        case EVG_PROTOCOL_CMD_LONGOUT_LO_DIAG_OUT:
            fmcIOsetDiagnosticOutputs(idx, cmdp->args[0]);
            break;

        case EVG_PROTOCOL_CMD_LONGOUT_LO_XPT_RBK:
            if ((idx < 0) || (idx > EVG_COUNT)
             || (cmdp->args[0] < 1) || (cmdp->args[0] > 36)) {
                return -1;
            }
            else {
                int shift = 8 * idx;
                unsigned int mask = 0xFF << shift;
                sharedMemory->loopbackRequest =
                                   (sharedMemory->loopbackRequest & ~mask) |
                                       ((cmdp->args[0] << shift) & mask);
            }
            break;

        case EVG_PROTOCOL_CMD_LONGOUT_LO_GENERIC:
            switch (idx) {
            case EVG_PROTOCOL_CMD_LONGOUT_GENERIC_IDX_REBOOT:
                crankRebootStateMachine(cmdp->args[0]);
                break;

            case EVG_PROTOCOL_CMD_LONGOUT_GENERIC_IDX_INJECTION_ENABLE:
                injectionCycleEnable(cmdp->args[0]);
                break;

            case EVG_PROTOCOL_CMD_LONGOUT_GENERIC_IDX_SWAPOUT_ENABLE:
                swapoutCycleEnable(cmdp->args[0]);
                break;

            case EVG_PROTOCOL_CMD_LONGOUT_GENERIC_IDX_REALIGN_MGT_TX:
                crankRealignStateMachine(cmdp->args[0]);
                break;

            case EVG_PROTOCOL_CMD_LONGOUT_GENERIC_IDX_EXTRA_INJ_DELAY:
                injectionCycleExtendInterval(cmdp->args[0]);
                break;

            default: return -1;
            }
            break;

        default: return -1;
        }
        break;

    case EVG_PROTOCOL_CMD_HI_SYSMON:
        if (commandArgCount != 0) return -1;
        replyp->args[0] = powerUpStatus;
        replyArgCount = sysmonFetch(replyp->args + 1) + 1;
        break;

    case EVG_PROTOCOL_CMD_HI_WAVEFORM:
        switch (lo) {
        case EVG_PROTOCOL_CMD_WAVEFORM_LO_SEQUENCE:
            if ((commandArgCount < 2)
             || (!evgStashSequence(idx, cmdp->args[0], commandArgCount - 1,
                                                             cmdp->args + 1))) {
                return -1;
            }
            break;

        default: return -1;
        }
        break;

    default: return -1;
    }
    return replyArgCount;
}

/*
 * Handle commands from IOC
 */
static void
epicsHandler(bwudpHandle replyHandle, char *payload, int length)
{
    struct evgPacket *cmdp = (struct evgPacket *)payload;
    int mustSwap = 0;
    int commandArgCount;
    static struct evgPacket reply;
    static int replySize;
    static uint32_t lastNonce;

    /*
     * Ignore weird-sized packets
     */
    if ((length < EVG_PROTOCOL_ARG_COUNT_TO_SIZE(0))
     || (length > sizeof(struct evgPacket))
     || ((length % sizeof(uint32_t)) != 0)) {
        return;
    }
    commandArgCount = EVG_PROTOCOL_SIZE_TO_ARG_COUNT(length);
    if (cmdp->magic == EVG_PROTOCOL_MAGIC_SWAPPED) {
        mustSwap = 1;
        bswap32(&cmdp->magic, length / sizeof(int32_t));
    }
    if (cmdp->magic == EVG_PROTOCOL_MAGIC) {
        if (debugFlags & DEBUGFLAG_EPICS) {
            printf("Command:%X nonce:%X args:%d 0x%x\n",
                         (unsigned int)cmdp->command, (unsigned int)cmdp->nonce,
                         commandArgCount, (unsigned int)cmdp->args[0]);
        }
        if (cmdp->nonce != lastNonce) {
            int n;
            memcpy(&reply, cmdp, EVG_PROTOCOL_ARG_COUNT_TO_SIZE(0));
            if ((n = handleCommand(commandArgCount, cmdp, &reply)) < 0) {
                return;
            }
            lastNonce = cmdp->nonce;
            replySize = EVG_PROTOCOL_ARG_COUNT_TO_SIZE(n);
            if (mustSwap) {
                bswap32(&reply.magic, replySize / sizeof(int32_t));
            }
        }
        bwudpSend(replyHandle, (const char *)&reply, replySize);
    }
}

/*
 * Minimize update latency and network traffic by publishing
 * sequencer status whenever there is a change.
 */
#define SUBSCRIPTION_US 30000000
#define SHORTEST_US     400
static void
seqStatusHandler(bwudpHandle replyHandle, char *payload, int length)
{
    int i;
    uint32_t now = MICROSECONDS_SINCE_BOOT();
    static bwudpHandle seqStatusPublisher;
    static int mustSend, mustSwap;
    static uint32_t whenSubscribed, whenSent;
    uint32_t currentStatus[EVG_PROTOCOL_EVG_COUNT];
    uint32_t currentStatusSeconds[EVG_PROTOCOL_EVG_COUNT];
    uint32_t currentStatusFraction[EVG_PROTOCOL_EVG_COUNT];
    static uint32_t sentStatus[EVG_PROTOCOL_EVG_COUNT];

    if (replyHandle) {
        if (length != sizeof(uint32_t)) return;
        if (*(uint32_t *)payload == EVG_PROTOCOL_MAGIC) {
            mustSwap = 0;
        }
        else if (*(uint32_t *)payload == EVG_PROTOCOL_MAGIC_SWAPPED) {
            mustSwap = 1;
        }
        else {
            return;
        }
        seqStatusPublisher = replyHandle;
        whenSubscribed = now;
        mustSend = 1;
    }
    else {
        if ((now - whenSubscribed) > SUBSCRIPTION_US) {
            seqStatusPublisher = NULL;
        }
        if (seqStatusPublisher == NULL) return;
    }
    if ((now - whenSent) < SHORTEST_US) return;
    for (i = 0 ; i < EVG_PROTOCOL_EVG_COUNT ; i++) {
        currentStatus[i] = evgSequencerStatus(i, &currentStatusSeconds[i],
                &currentStatusFraction[i]);
        if (currentStatus[i] != sentStatus[i]) {
            mustSend = 1;
        }
    }
    if (mustSend) {
        static struct evgStatusPacket pk;
        static uint32_t pkNumber;
        pk.magic = EVG_PROTOCOL_MAGIC;
        pk.pkNumber = ++pkNumber;
        for (i = 0 ; i < EVG_PROTOCOL_EVG_COUNT ; i++) {
            pk.sequencerStatus[i] = sentStatus[i] = currentStatus[i];
            pk.posixSeconds[i] = currentStatusSeconds[i] - NTP_POSIX_OFFSET;
            pk.ntpFraction[i] = currentStatusFraction[i];
        }
        if (mustSwap) {
            bswap32(&pk.magic, sizeof(pk) / sizeof(int32_t));
        }
        bwudpSend(seqStatusPublisher, (const char *)&pk, sizeof pk);
        whenSent = now;
        mustSend = 0;
    }
}
void
sequencerStatusPublisherCrank(void)
{
    seqStatusHandler(NULL, NULL, 0);
}

void
epicsInit(void)
{
    bwudpRegisterServer(htons(EVG_PROTOCOL_UDP_EPICS_PORT), epicsHandler);
    bwudpRegisterServer(htons(EVG_PROTOCOL_UDP_STATUS_PORT), seqStatusHandler);
}
