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
#include "bwudp.h"
#include "gpio.h"
#include "gps.h"
#include "systemParameters.h"
#include "tod.h"
#include "util.h"

#define STARTUP_PAUSE (4 * 1000000)

#define NTP_PORT 123

/*****************************************************************************
 * NTP packet
 * No endian issues since 8 bit arrays are used rather than 32 bit scalars.
 */
#define NTP_FLAGS_LI_MASK       0xC0
#define NTP_FLAGS_LI_SHIFT      6
#define NTP_FLAGS_VERSION_MASK  0x38
#define NTP_FLAGS_VERSION_SHIFT 3
#define NTP_FLAGS_MODE_MASK     0x07
#define NTP_FLAGS_MODE_SHIFT    0
#define NTP_VERSION_3           (3 << NTP_FLAGS_VERSION_SHIFT)
#define NTP_MODE_CLIENT         (3 << NTP_FLAGS_MODE_SHIFT)
typedef struct ntpTimestamp {
    uint8_t secondsSinceEpoch[4];
    uint8_t fraction[4];
} ntpTimestamp;
struct ntpPacket {
    uint8_t      flags;
    uint8_t      stratum;
    uint8_t      poll;
    int8_t       precision;
    uint8_t      rootDelay[4];
    uint8_t      rootDispersion[4];
    uint8_t      referenceClockIdentifier[4];
    ntpTimestamp referenceTimestamp;
    ntpTimestamp originateTimestamp;
    ntpTimestamp receiveTimestamp;
    ntpTimestamp transmitTimestamp;
};

static bwudpHandle ntpHandle;
struct myTime {
    uint32_t secondsSincePosixEpoch;
    uint32_t fraction;
};
static struct myTime now;
static uint32_t usecWhenQueried;

static void
ntpToMyTime(struct myTime *mt, ntpTimestamp *t)
{
    mt->secondsSincePosixEpoch = ((t->secondsSinceEpoch[0] << 24) |
                                  (t->secondsSinceEpoch[1] << 16) |
                                  (t->secondsSinceEpoch[2] <<  8) |
                                   t->secondsSinceEpoch[3]) - NTP_POSIX_OFFSET;
    mt->fraction = (t->fraction[0] << 24) |
                   (t->fraction[1] << 16) |
                   (t->fraction[2] <<  8) |
                    t->fraction[3];
}
static void
showTimestamp(const char *name, ntpTimestamp *t)
{
    struct myTime mt;
    ntpToMyTime(&mt, t);
    printf("%10s: %u %u\n", name, mt.secondsSincePosixEpoch, mt.fraction);
}

static void
showPacket(struct ntpPacket *ntp)
{
    printf("LI:%d VERS:%d MODE:%d STRATUM:%d POLL:%d PRECISION:%d\n",
                     ntp->flags >> 6, (ntp->flags >> 3) & 0x7, ntp->flags & 0x7,
                     ntp->stratum, ntp->poll, ntp->precision);
    showTimestamp("Reference", &ntp->referenceTimestamp);
    showTimestamp("Originate", &ntp->originateTimestamp);
    showTimestamp("Receive", &ntp->receiveTimestamp);
    showTimestamp("Transmit", &ntp->transmitTimestamp);
}

static void
serverCallback(void *replyHandle, char *payload, int length)
{
    struct ntpPacket *ntp = (struct ntpPacket *)payload;

    if (debugFlags & DEBUGFLAG_TOD) {
        printf("Received NTP:%d\n", length);
    }
    if (length >= sizeof(*ntp)) {
        uint32_t seconds, fraction;
        if (debugFlags & DEBUGFLAG_TOD) {
            showPacket(ntp);
        }
        if (GPIO_READ(GPIO_IDX_NTP_SERVER_STATUS) &
                                              NTP_SERVER_STATUS_SECONDS_VALID) {
            seconds = GPIO_READ(GPIO_IDX_NTP_SERVER_SECONDS);
            for (int i = 0; i < 500; i++) {
                uint32_t s;
                fraction = GPIO_READ(GPIO_IDX_NTP_SERVER_FRACTION);
                s = GPIO_READ(GPIO_IDX_NTP_SERVER_SECONDS);
                if (s == seconds) break;
                seconds = s;
            }
            ntp->flags = (3 << NTP_FLAGS_VERSION_SHIFT) | 4;
            ntp->stratum = 1;
        }
        else {
            seconds = 0;
            fraction = 0;
            ntp->flags = NTP_FLAGS_LI_MASK | (3 << NTP_FLAGS_VERSION_SHIFT) | 4;
            ntp->stratum = 16;
        }
        ntp->poll = 6;
        ntp->precision = -18;
        ntp->rootDelay[0] = 0;
        ntp->rootDelay[1] = 0;
        ntp->rootDelay[2] = 0;
        ntp->rootDelay[3] = 0;
        ntp->rootDispersion[0] = 0;
        ntp->rootDispersion[1] = 0;
        ntp->rootDispersion[2] = 0x00;
        ntp->rootDispersion[3] = 0x20;
        ntp->originateTimestamp = ntp->transmitTimestamp;
        ntp->referenceTimestamp.secondsSinceEpoch[0] = seconds >> 24;
        ntp->referenceTimestamp.secondsSinceEpoch[0] = seconds >> 24;
        ntp->referenceTimestamp.secondsSinceEpoch[1] = seconds >> 16;
        ntp->referenceTimestamp.secondsSinceEpoch[2] = seconds >> 8;
        ntp->referenceTimestamp.secondsSinceEpoch[3] = seconds;
        ntp->referenceTimestamp.fraction[0] = fraction >> 24;
        ntp->referenceTimestamp.fraction[1] = fraction >> 16;
        ntp->referenceTimestamp.fraction[2] = fraction >> 8;
        ntp->referenceTimestamp.fraction[3] = fraction;
        ntp->receiveTimestamp = ntp->referenceTimestamp;
        ntp->transmitTimestamp = ntp->receiveTimestamp;
        ntp->referenceClockIdentifier[0] = 'G';
        ntp->referenceClockIdentifier[1] = 'P';
        ntp->referenceClockIdentifier[2] = 'S';
        ntp->referenceClockIdentifier[3] = ' ';
        bwudpSend(replyHandle, payload, sizeof(*ntp));
        if (debugFlags & DEBUGFLAG_TOD) {
            printf("Transmit NTP\n");
            showPacket(ntp);
        }
    }
}

static void
clientCallback(void *replyHandle, char *payload, int length)
{
    struct ntpPacket *ntp = (struct ntpPacket *)payload;
    uint32_t interval = MICROSECONDS_SINCE_BOOT() - usecWhenQueried;
    if ((length >= sizeof(*ntp))
     && (now.secondsSincePosixEpoch == 0)) {
        if (interval > 100000) {
            warn("NTP round trip %u us", interval);
        }
        ntpToMyTime(&now, &ntp->transmitTimestamp);
    }
    if (debugFlags & DEBUGFLAG_TOD) {
        printf("Received NTP %d, %u us\n", length, interval);
        if (length >= sizeof(*ntp)) {
            showPacket(ntp);
        }
    }
}

static struct ntpPacket query;
void
ntpQuery(void)
{
    uint32_t secondsSinceBoot = GPIO_READ(GPIO_IDX_SECONDS_SINCE_BOOT);

    now.secondsSincePosixEpoch = 0;
    query.flags   = NTP_VERSION_3 | NTP_MODE_CLIENT,
    query.stratum = 16, /* Unsynchronized */
    query.poll    = 3,  /* 8 second interval */
    query.originateTimestamp.secondsSinceEpoch[0] = secondsSinceBoot>>24;
    query.originateTimestamp.secondsSinceEpoch[1] = secondsSinceBoot>>16;
    query.originateTimestamp.secondsSinceEpoch[2] = secondsSinceBoot>>8;
    query.originateTimestamp.secondsSinceEpoch[3] = secondsSinceBoot;
    usecWhenQueried = MICROSECONDS_SINCE_BOOT();
    bwudpSend(ntpHandle, (const char *)&query, sizeof query);
}

/*****************************************************************************
 * Time-of-day state machine
 */
static enum todSource {
    todSourceNTP,
    todSourceGPS } todSource = todSourceNTP;

static enum todState {
    todStart,
    todDelay,
    todAwaitPPS,
    todAwaitTOD,
    todPause,
    todSynced,
    todBeginResync } todState = todStart;

void
todInit(void)
{
    static const ipv4Address null;
    gpsInit();
    if (memcmp((void *)&sharedMemory->systemParameters.ntpHost, &null,
                                                            sizeof null) == 0) {
        todSource = todSourceGPS;
       if (bwudpRegisterServer(htons(NTP_PORT), serverCallback) < 0) {
            warn("Critical -- can't create NTP server");
        }
        return;
    }
    ntpHandle = bwudpCreateClient(
                         (ipv4Address *)&sharedMemory->systemParameters.ntpHost,
                              htons(NTP_PORT), htons(NTP_PORT), clientCallback);
    if (ntpHandle == NULL) {
        warn("Critical -- can't create NTP client");
    }
}

void
todResync(void)
{
    todState = todBeginResync;
}

int
todStatus(void)
{
    return (todState << 16) | (GPIO_READ(GPIO_IDX_NTP_SERVER_STATUS) & 0xFFFF);
}

void
todCrank(void)
{
    uint32_t status = GPIO_READ(GPIO_IDX_NTP_SERVER_STATUS);
    enum todState oldState = todState;
    static uint32_t statusOld;
    static uint32_t then;
    static int beenHere, subsequentPPScheck, reportedMissingPPS;

    if (!beenHere) {
        statusOld = status;
        beenHere = 1;
    }
    if ((debugFlags & DEBUGFLAG_GPS) && (todState != todAwaitTOD)) {
        /* Show GPS input even if using NTP */
        uint32_t t0, t1;
        gpsCheck(&t0, &t1);
    }
    switch (todState) {
    case todStart:
        if (todSource == todSourceNTP) {
            // Issue an an initial query to get the ARP out of the way
            ntpQuery();
        }
        then = MICROSECONDS_SINCE_BOOT();
        todState = todDelay;
        break;

    case todDelay:
        if ((MICROSECONDS_SINCE_BOOT() - then) > STARTUP_PAUSE){
            then = MICROSECONDS_SINCE_BOOT();
            todState = todAwaitPPS;
        }
        break;

    case todBeginResync:
        reportedMissingPPS = 0;
        then = MICROSECONDS_SINCE_BOOT();
        todState = todAwaitPPS;
        break;

    case todAwaitPPS:
        if ((status & NTP_SERVER_STATUS_PPS_VALID)
         && (((status ^ statusOld) & NTP_SERVER_STATUS_PPS_TOGGLE) != 0)) {
            subsequentPPScheck = 1;
            if (reportedMissingPPS) {
                printf("PPS present, continuing with synchronization.\n");
                then = MICROSECONDS_SINCE_BOOT();
                reportedMissingPPS = 0;
                break;
            }
            now.secondsSincePosixEpoch = 0;
            switch (todSource) {
            case todSourceNTP: ntpQuery(); break;
            case todSourceGPS: gpsFlush(); break;
            }
            then = MICROSECONDS_SINCE_BOOT();
            todState = todAwaitTOD;
        }
        else if ((MICROSECONDS_SINCE_BOOT() - then) >
                                     (subsequentPPScheck ? 1000000 : 3100000)) {
            subsequentPPScheck = 1;
            then = MICROSECONDS_SINCE_BOOT();
            if (!reportedMissingPPS++) {
                warn("No PPS");
            }
        }
        break;

    case todAwaitTOD:
        {
        int usecLimit = 100000;
        switch (todSource) {
        case todSourceNTP: break;
        case todSourceGPS:
            usecLimit = 1100000;
            if (GPIO_READ(GPIO_IDX_NTP_SERVER_STATUS) &
                                                  NTP_SERVER_STATUS_PPS_VALID) {
                gpsCheck(&now.secondsSincePosixEpoch, &now.fraction);
            }
            break;
        }
        if (now.secondsSincePosixEpoch) {
            GPIO_WRITE(GPIO_IDX_NTP_SERVER_SECONDS,
                                 now.secondsSincePosixEpoch + NTP_POSIX_OFFSET);
            // Blindly apply the number of seconds to F2 NTP too.
            GPIO_WRITE(GPIO_IDX_NTP_SERVER_F2_SECONDS,
                                 now.secondsSincePosixEpoch + NTP_POSIX_OFFSET);
            printf("TOD %u:%u after %u us\n",
                                      now.secondsSincePosixEpoch, now.fraction,
                                      MICROSECONDS_SINCE_BOOT() - then);
            if (now.fraction > (1UL << 30)) {
                warn("PPS marker to TOD second %d ms.",
                                           ((now.fraction >> 10) * 1000) >> 22);
            }
            todState = todSynced;
            break;
        }
        else if ((MICROSECONDS_SINCE_BOOT() - then) > usecLimit) {
            warn("No response from TOD source.");
            todState = todPause;
        }
        }
        break;

    case todPause:
        if ((MICROSECONDS_SINCE_BOOT() - then) >
                                                ((1 << query.poll) * 1000000)) {
            then = MICROSECONDS_SINCE_BOOT();
            todState = todAwaitPPS;
        }
        break;

    case todSynced:
        if ((status & NTP_SERVER_STATUS_PPS_VALID) == 0) {
            warn("Lost PPS at %u (POSIX) seconds",
                     GPIO_READ(GPIO_IDX_NTP_SERVER_SECONDS) - NTP_POSIX_OFFSET);
            todState = todBeginResync;
        }
        break;
    }
    statusOld = status;
    if ((debugFlags & DEBUGFLAG_TOD) && (todState != oldState)) {
        printf("TOD State %d\n", todState);
    }
}
