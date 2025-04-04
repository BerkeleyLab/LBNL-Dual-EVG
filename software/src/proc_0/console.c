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
 * Simple command interpreter
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <xparameters.h>
#include <xuartlite_l.h>
#include "bwudp.h"
#include "evgCoincidence.h"
#include "evio.h"
#include "eyescan.h"
#include "gpio.h"
#include "iicProc.h"
#include "mgtClkSwitch.h"
#include "st7789v.h"
#include "tod.h"
#include "util.h"
#include "xadc.h"

/*
 * UDP console support
 */
static struct udpConsole {
    int         active;
    int         outIndex;
    uint32_t    usAtFirstOutputCharacter;
    int         inIndex;
} udpConsole;

/*
 * Special modes
 */
static int (*modalHandler)(int argc, char **argv);

/*
 * Mark UDP console inactive while draining in case
 * network code diagnostic messages are enabled.
 */
static void
udpConsoleDrain(void)
{
    udpConsole.active = 0;
    sharedMemory->udpConsole.txSize = udpConsole.outIndex;
    while (sharedMemory->udpConsole.txSize >= 0) continue;
    udpConsole.outIndex = 0;
    udpConsole.active = 1;
}

/*
 * Pulling in real sprintf bloats executable by more than 60 kB so provide this
 * fake version that accepts only a limited number of integer arguments.
 */
static char *outbyteStash;
int
sprintf(char *buf, const char *fmt, ...)
{
    va_list args;
    unsigned int a[6];
    va_start(args, fmt);
    a[0] = va_arg(args, unsigned int);
    a[1] = va_arg(args, unsigned int);
    a[2] = va_arg(args, unsigned int);
    a[3] = va_arg(args, unsigned int);
    a[4] = va_arg(args, unsigned int);
    a[5] = va_arg(args, unsigned int);
    *buf = '\0';
    outbyteStash = buf;
    printf(fmt, a[0], a[1], a[2], a[3], a[4], a[5]);
    outbyteStash = NULL;
    va_end(args);
    return outbyteStash - buf;
}

/*
 * Stash character and return if in 'sprintf'.
 * Convert <newline> to <carriage return><newline> so
 * we can use normal looking printf format strings.
 * Hang on to startup messages.
 * Buffer to limit the number of transmitted packets.
 */
#define STARTBUF_SIZE   5000
static char startBuf[STARTBUF_SIZE];
static int startIdx = 0;
static int isStartup = 1;
static int showingStartup = 0;

void
outbyte(char8 c)
{
    static int wasReturn;

    if (outbyteStash != NULL) {
        *outbyteStash++ = c;
        *outbyteStash = '\0';
        return;
    }
    if ((c == '\n') && !wasReturn) outbyte('\r');
    wasReturn = (c == '\r');
    XUartLite_SendByte(STDOUT_BASEADDRESS, c);
    if (isStartup && (startIdx < STARTBUF_SIZE))
        startBuf[startIdx++] = c;
    if (udpConsole.active) {
        if (udpConsole.outIndex == 0)
            udpConsole.usAtFirstOutputCharacter = MICROSECONDS_SINCE_BOOT();
        sharedMemory->udpConsole.txBuf[udpConsole.outIndex++] = c;
        if (udpConsole.outIndex >= SHARED_RAM_UDP_BUFSIZE) {
            udpConsoleDrain();
        }
    }
}

static int
cmdLOG(int argc, char **argv)
{
    showingStartup = 1;
    return 0;
}

static int
cmdBOOT(int argc, char **argv)
{
    static int bootAlternateImage;
    if (modalHandler) {
        if (argc == 1) {
            if (strcasecmp(argv[0], "Y") == 0) {
                if (udpConsole.active) udpConsoleDrain();
                microsecondSpin(1000);
                resetFPGA(bootAlternateImage);
                modalHandler = NULL;
                return 0;
            }
            if (strcasecmp(argv[0], "N") == 0) {
                modalHandler = NULL;
                return 0;
            }
        }
    }
    else {
        if (argc == 1) {
            bootAlternateImage = 0;
        }
        else if ((argc == 2)
              && (argv[1][0] == '-')
              && ((argv[1][1] == 'b') || (argv[1][1] == 'B'))
              && (argv[1][2] == '\0')) {
            bootAlternateImage = 1;
        }
        else {
            printf("Invalid argument.\n");
            return 0;
        }
        modalHandler = cmdBOOT;
    }
    printf("Reboot FPGA image %c (y or n)? ", 'A' + bootAlternateImage);
    fflush(stdout);
    return 0;
}

static int
cmdDEBUG(int argc, char **argv)
{
    char *endp;
    uint32_t d;
    int sFlag = 0;

    if ((argc > 1) && (strcmp(argv[1], "-s") == 0)) {
        sFlag = 1;
        argc--;
        argv++;
    }
    if (argc > 1) {
        d = strtoul(argv[1], &endp, 16);
        if (*endp == '\0') {
            debugFlags = d;
        }
    }
    printf("Debug flags: 0x%x\n", debugFlags);
    if (debugFlags & DEBUGFLAG_IIC_SCAN) iicProcScan();
    if (debugFlags & DEBUGFLAG_DUMP_SCREEN) st7789vDumpScreen();
    if (debugFlags & DEBUGFLAG_DUMP_MGT_SWITCH) mgtClkSwitchDump();
    if (debugFlags & DEBUGFLAG_DUMP_CROSSPOINT) evioShowCrosspointRegisters();
    if (debugFlags & DEBUGFLAG_SHOW_COINCIDENCE) evgCoincidenceShow(0);
    if (debugFlags & DEBUGFLAG_PLOT_COINCIDENCE) evgCoincidenceShow(1);
    if (sFlag) {
        sharedMemory->systemParameters.startupDebugFlags = debugFlags;
        systemParametersStash();
        printf("Startup debug flags: 0x%x\n", debugFlags);
    }
    return 0;
}

static int
cmdDumpScreen(int argc, char **argv)
{
    printf("dumpscreen\n");
    st7789vDumpScreen();
    return 0;
}

static int
cmdFMON(int argc, char **argv)
{
    int i;
    int ppsValid = ((GPIO_READ(GPIO_IDX_NTP_SERVER_STATUS) &
                                             NTP_SERVER_STATUS_PPS_VALID) != 0);
    static const char *names[] = { "System",
                                   "EVG 1 MGT reference",
                                   "EVG 1 Tx",
                                   "EVG 1 Rx 0",
                                   "EVG 2 MGT reference",
                                   "EVG 2 Tx",
                                   "EVG 2 Rx 0",
                                   "Ethernet Tx",
                                   "Ethernet Rx",
                                   "EVG 1 Rx 1",
                                   "EVG 1 Rx 2",
                                   "EVG 1 Rx 3",
                                   "EVG 2 Rx 1",
                                   "EVG 2 Rx 2",
                                   "EVG 2 Rx 3" };
    if (!ppsValid) {
        printf("Pulse per second signal missing -- Relative accuracy only.\n");
    }
    for (i = 0 ; i < sizeof names / sizeof names[0] ; i++) {
        GPIO_WRITE(GPIO_IDX_FREQ_MONITOR_CSR, i);
        unsigned int rate = GPIO_READ(GPIO_IDX_FREQ_MONITOR_CSR);
        printf("%20s clock: %3d.%06d\n", names[i], rate / 1000000,
                                                   rate % 1000000);
    }
    return 0;
}

static int
cmdNET(int argc, char **argv)
{
    int bad = 0;
    int i;
    char *cp;
    uint32_t netmask;
    unsigned int netLen = 24;
    char *endp;
    static struct sysNetParms np;
    if (modalHandler) {
        if (argc == 1) {
            if (strcasecmp(argv[0], "Y") == 0) {
                sharedMemory->systemParameters.netConfig.np = np;
                systemParametersStash();
                modalHandler = NULL;
                return 0;
            }
            if (strcasecmp(argv[0], "N") == 0) {
                modalHandler = NULL;
                return 0;
            }
        }
        printf("Write to flash (y or n)? ");
        fflush(stdout);
        return 0;
    }
    else {
        if (argc == 1) {
            np = sharedMemory->systemParameters.netConfig.np;
        }
        else if (argc == 2) {
            cp = argv[1];
            i = parseIP(cp, &np.address);
            if (i < 0) {
                bad = 1;
            }
            else if (cp[i] == '/') {
                netLen = strtol(cp + i + 1, &endp, 0);
                if ((*endp != '\0')
                 || (netLen < 8)
                 || (netLen > 24)) {
                    bad = 1;
                    netLen = 24;
                }
            }
            netmask = ~0U << (32 - netLen);
            np.netmask.a[0] = netmask >> 24;
            np.netmask.a[1] = netmask >> 16;
            np.netmask.a[2] = netmask >> 8;
            np.netmask.a[3] = netmask;
            np.gateway.a[0] = np.address.a[0] & np.netmask.a[0];
            np.gateway.a[1] = np.address.a[1] & np.netmask.a[1];
            np.gateway.a[2] = np.address.a[2] & np.netmask.a[2];
            np.gateway.a[3] = (np.address.a[3] & np.netmask.a[3]) | 1;
        }
        else {
            bad = 1;
        }
        if (bad) {
            printf("Command takes single optional argument of the form "
                   "www.xxx.yyy.xxx[/n]\n");
            return 1;
        }
    }
    showNetworkConfig(&np);
    if (!memcmp(&np.address,
                   (void *)&sharedMemory->systemParameters.netConfig.np.address,
                   sizeof(ipv4Address))
     && !memcmp(&np.netmask,
                   (void *)&sharedMemory->systemParameters.netConfig.np.netmask,
                   sizeof(ipv4Address))
     && !memcmp(&np.gateway,
                   (void *)&sharedMemory->systemParameters.netConfig.np.gateway,
                   sizeof(ipv4Address))) {
        return 0;
    }
    printf("Write parameters to flash (y or n)? ");
    fflush(stdout);
    modalHandler = cmdNET;
    return 0;
}

static int
cmdMAC(int argc, char **argv)
{
    int bad = 0;
    int i;
    static ethernetMAC mac;
    if (modalHandler) {
        if (argc == 1) {
            if (strcasecmp(argv[0], "Y") == 0) {
                memcpy(
                  (void *)&sharedMemory->systemParameters.netConfig.ethernetMAC,
                                                               &mac,sizeof mac);
                systemParametersStash();
                modalHandler = NULL;
                return 0;
            }
            if (strcasecmp(argv[0], "N") == 0) {
                modalHandler = NULL;
                return 0;
            }
        }
    }
    else {
        if (argc == 1) {
            memcpy(&mac,
                  (void *)&sharedMemory->systemParameters.netConfig.ethernetMAC,
                  sizeof mac);
        }
        else if (argc == 2) {
            i = parseMAC(argv[1], &mac);
            if ((i < 0) || (argv[1][i] != '\0')) {
                bad = 1;
            }
        }
        else {
            bad = 1;
        }
        if (bad) {
            printf("Command takes single optional argument of the form "
                   "aa:bb:cc:dd:ee:ff\n");
            return 1;
        }
    }
    printf("   ETHERNET ADDRESS: %s\n", formatMAC(&mac));
    if (!(memcmp((void *)&sharedMemory->systemParameters.netConfig.ethernetMAC,
                                                           &mac, sizeof mac))) {
        return 0;
    }
    modalHandler = cmdMAC;
    printf("Write to flash (y or n)? ");
    fflush(stdout);
    return 0;
}

static int
cmdNTP(int argc, char **argv)
{
    int bad = 0;
    int i;
    char *cp;
    static ipv4Address ntpHost;
    if (modalHandler) {
        if (argc == 1) {
            if (strcasecmp(argv[0], "Y") == 0) {
                sharedMemory->systemParameters.ntpHost = ntpHost;
                systemParametersStash();
                modalHandler = NULL;
                return 0;
            }
            if (strcasecmp(argv[0], "N") == 0) {
                modalHandler = NULL;
                return 0;
            }
        }
    }
    else {
        if (argc == 1) {
            ntpHost = sharedMemory->systemParameters.ntpHost;
        }
        else if (argc == 2) {
            cp = argv[1];
            i = parseIP(cp, &ntpHost);
            if ((i < 0) || (cp[i] != '\0')) {
                bad = 1;
            }
        }
        else {
            bad = 1;
        }
        if (bad) {
            printf("Command takes single optional argument of the form "
                   "www.xxx.yyy.xxx\n");
            return 1;
        }
    }
    showNTPserver(&ntpHost);
    if (memcmp(&ntpHost, (void *)&sharedMemory->systemParameters.ntpHost,
                                                         sizeof(ipv4Address))) {
        printf("Write to flash (y or n)? ");
        fflush(stdout);
        modalHandler = cmdNTP;
    }
    return 0;
}

/*
 * PLL phase alignment targets
 */
static int
cmdPLL(int argc, char **argv)
{
    int i;
    for (i = 0 ; i < (sizeof sharedMemory->systemParameters.pllPhaseShift /
                      sizeof sharedMemory->systemParameters.pllPhaseShift[0]) ;
                                                                          i++) {
        if (argc == 1) {
            printf("EVG %d phase shift target: %d\n", i + 1,
                               sharedMemory->systemParameters.pllPhaseShift[i]);
        }
        else {
            char *endp;
            int v;
            if ((i + 1) >= argc) {
                printf("Missing value\n");
                return 1;
            }
            v = strtol(argv[i+1], &endp, 0);
            if (*endp != '\0') {
                printf("Bad value\n");
                return 1;
            }
            sharedMemory->systemParameters.pllPhaseShift[i] = v;
        }
    }
    if (argc > 1) {
        systemParametersStash();
    }
    return 0;
}

/*
 * Set coincide couint manually (mainly for testing)
 */
static int
cmdCoinc(int argc, char **argv, int evgNumber)
{
    char *endp;
    int a = 0;
    uint32_t csrIdx = 0;

    switch (evgNumber) {
    case 0:
        csrIdx = GPIO_IDX_EVG_1_COINC_CSR;
        break;
    case 1:
        csrIdx = GPIO_IDX_EVG_2_COINC_CSR;
        break;
    default:
        warn("Invalid EVG number");
        return 0;
    }

    if (argc == 1) {
        printf("Coincidence commands expects an argument\n");
    }
    else {
        a = strtol(argv[1], &endp, 0);
        if (*endp != '\0') {
            printf("Bad value\n");
            return 1;
        }

        GPIO_WRITE(csrIdx, CSR_W_SET_COINCIDENCE | ADDRESS_WR_W(a));
        printf("EVG %d coincidence count: %d\n", evgNumber + 1, a);
    }

    return 0;
}

static int
cmdCoincEvg1(int argc, char **argv)
{
    return cmdCoinc(argc, argv, 0);
}

static int
cmdCoincEvg2(int argc, char **argv)
{
    return cmdCoinc(argc, argv, 1);
}

/*
 * Get LOL state (mainly for testing)
 */
static int
cmdLOL(int argc, char **argv, int evgNumber)
{
    char *endp;
    int state = 0;
    uint16_t lane = 0;
    uint16_t evgBitmap = 0;

    switch (evgNumber) {
    case 0:
        evgBitmap = 0x1;
        break;
    case 1:
        evgBitmap = 0x2;
        break;
    default:
        warn("Invalid EVG number");
        return 0;
    }

    if (argc > 1) {
        lane = strtoul(argv[1], &endp, 0);
        if (*endp != '\0') {
            return 1;
        }
    }

    /* Set EVG and lane */
    sharedMemory->lolStateMGTBitmap = evgBitmap;
    sharedMemory->lolStateMGTLane = lane;
    microsecondSpin(1000);

    state = sharedMemory->lolState;
    if (state < 0) {
        return 1;
    }

    printf("EVG %d LOL state: %d\n", evgNumber + 1, state);

    return 0;
}

static int
cmdLOLEvg1(int argc, char **argv)
{
    return cmdLOL(argc, argv, 0);
}

static int
cmdLOLEvg2(int argc, char **argv)
{
    return cmdLOL(argc, argv, 1);
}

static int
cmdREG(int argc, char **argv)
{
    char *endp;
    int i;
    int first;
    int n = 1;

    if (argc > 1) {
        first = strtol(argv[1], &endp, 0);
        if (*endp != '\0')
            return 1;
        if (argc > 2) {
            n = strtol(argv[2], &endp, 0);
            if (*endp != '\0')
                return 1;
        }
        if ((first < 0) || (first >= GPIO_IDX_COUNT) || (n <= 0))
            return 1;
        if ((first + n) > GPIO_IDX_COUNT)
            n = GPIO_IDX_COUNT - first;
        for (i = first ; i < first + n ; i++) {
            showReg(i);
        }
    }
    return 0;
}

static int
cmdTLOG(int argc, char **argv, int evgNumber)
{
    uint32_t csr;
    static int isActive[2], isFirstHB[2], todBitCount[2];
    static int rAddr[2];
    static int addrMask[2];
    uint32_t gpioIdxEventLogCsr, gpioIdxEventLogTicks;
    int pass = 0;

    /* GPIO_IDX selection corresponding to EVG 1 or 2 */
    switch (evgNumber) {
    case 0:
        gpioIdxEventLogCsr = GPIO_IDX_EVG_1_TLOG_CSR;
        gpioIdxEventLogTicks = GPIO_IDX_EVG_1_TLOG_TICKS;
        break;
    case 1:
        gpioIdxEventLogCsr = GPIO_IDX_EVG_2_TLOG_CSR;
        gpioIdxEventLogTicks = GPIO_IDX_EVG_2_TLOG_TICKS;
        break;
    default:
        warn("Invalid EVG number");
        return 0;
    }
    /* argc determinates the operation to perform */
    if (argc < 0) {
        if (isActive[evgNumber]) {
            GPIO_WRITE(gpioIdxEventLogCsr, 0);
            isActive[evgNumber] = 0;
        }
        return 0;
    }
    if (argc > 0) {
        // Disable all the evFifo
        GPIO_WRITE(GPIO_IDX_EVG_1_TLOG_CSR, 0);
        GPIO_WRITE(GPIO_IDX_EVG_2_TLOG_CSR, 0);
        // Enable only the target one
        csr = GPIO_READ(gpioIdxEventLogCsr);
        addrMask[evgNumber] = ~(~0UL << ((csr >> 24) & 0xF));
        GPIO_WRITE(gpioIdxEventLogCsr, 0x80000000);
        rAddr[evgNumber] = 0;
        isActive[evgNumber] = 1;
        isFirstHB[evgNumber] = 1;
        todBitCount[evgNumber] = 0;
        return 0;
    }
    if (isActive[evgNumber]) {
        int wAddr, wAddrOld;
        static uint32_t lastHbTicks, lastEvTicks, todShift;
        csr = GPIO_READ(gpioIdxEventLogCsr);
        wAddrOld = csr & addrMask[evgNumber];
        for (;;) {
            csr = GPIO_READ(gpioIdxEventLogCsr);
            wAddr = csr & addrMask[evgNumber];
            if (wAddr == wAddrOld) break;
            if (++pass > 10) {
                printf("Event logger unstable!\n");
                isActive[evgNumber] = 0;
                return 0;
            }
            wAddrOld = wAddr;
        }
        for (pass = 0 ; rAddr[evgNumber] != wAddr ; ) {
            int event;
            GPIO_WRITE(gpioIdxEventLogCsr, 0x80000000 | rAddr[evgNumber]);
            rAddr[evgNumber] = (rAddr[evgNumber] + 1) & addrMask[evgNumber];
            event = (GPIO_READ(gpioIdxEventLogCsr) >> 16) & 0xFF;
            if (event == 112) {
                todBitCount[evgNumber]++;
                todShift = (todShift << 1) | 0;
            }
            else if (event == 113) {
                todBitCount[evgNumber]++;
                todShift = (todShift << 1) | 1;
            }
            else {
                uint32_t ticks = GPIO_READ(gpioIdxEventLogTicks);
                printf("EVG%1d - ", evgNumber+1);
                switch(event) {
                case 122:
                    if (isFirstHB[evgNumber]) {
                        printf("HB\n");
                        isFirstHB[evgNumber] = 0;
                    }
                    else {
                        printf("HB %d\n", ticks - lastHbTicks);
                    }
                    lastHbTicks = ticks;
                    break;

                case 125:
                    if (todBitCount[evgNumber] == 32) {
                        printf("PPS %d\n", todShift);
                    }
                    else {
                        printf("PPS\n");
                    }
                    todBitCount[evgNumber] = 0;
                    break;

                default:
                    printf("%d %d\n", event, ticks - lastEvTicks);
                    lastEvTicks = ticks;
                    break;
                }
            }
            if (++pass >= addrMask[evgNumber]) {
                printf("Event logger can't keep up.\n");
                isActive[evgNumber] = 0;
                return 0;
            }
        }
        return 1;
    }
    return 0;
}

static int
cmdTLOGevg1(int argc, char **argv)
{
    return cmdTLOG(argc, argv, 0);
}

static int
cmdTLOGevg2(int argc, char **argv)
{
    return cmdTLOG(argc, argv, 1);
}

static void
commandHandler(int argc, char **argv)
{
    int i;
    int len;
    int matched = -1;
    struct commandInfo {
        const char *name;
        int       (*handler)(int argc, char **argv);
        const char *description;
    };
    static struct commandInfo commandTable[] = {
      { "boot",       cmdBOOT,        "Reboot FPGA"                        },
      { "debug",      cmdDEBUG,       "Set debug flags"                    },
      { "dumpscreen", cmdDumpScreen,  "Perform screen dump via console"    },
      { "eyescan",    eyescanCommand, "Perform transceiver eye scan"       },
      { "fmon",       cmdFMON,        "Show clock frequencies"             },
      { "log",        cmdLOG,         "Replay startup console output"      },
      { "mac",        cmdMAC,         "Set Ethernet MAC address"           },
      { "net",        cmdNET,         "Set network parameters"             },
      { "pll",        cmdPLL,         "Set PLL phase alignment targets"    },
      { "reg",        cmdREG,         "Show GPIO register(s)"              },
      { "tlog1",      cmdTLOGevg1,    "Timing system event logger (EVG1)"  },
      { "tlog2",      cmdTLOGevg2,    "Timing system event logger (EVG2)"  },
      { "tod",        cmdNTP,         "Set time-of-day (NTP) host address" },
      { "cmdC1",      cmdCoincEvg1,   "Set coincidence value (EVG1)"       },
      { "cmdC2",      cmdCoincEvg2,   "Set coincidence value (EVG2)"       },
      { "lol1",       cmdLOLEvg1,     "Get Loss of lock stats (EVG1)"      },
      { "lol2",       cmdLOLEvg2,     "Get Loss of lock stats (EVG2)"      },
    };

    if (argc <= 0)
        return;
    len = strlen(argv[0]);
    for (i = 0 ; i < sizeof commandTable / sizeof commandTable[0] ; i++) {
        if (strncasecmp(argv[0], commandTable[i].name, len) == 0) {
            if (matched >= 0) {
                printf("Not unique.\n");
                return;
            }
            matched = i;
        }
    }
    if (matched >= 0) {
        (*commandTable[matched].handler)(argc, argv);
        return;
    }
    if ((strncasecmp(argv[0], "help", len) == 0) || (argv[0][0] == '?')) {
        printf("Commands:\n");
        for (i = 0 ; i < sizeof commandTable / sizeof commandTable[0] ; i++) {
            printf("%8s -- %s\n", commandTable[i].name,
                                  commandTable[i].description);
        }
    }
    else {
        printf("Invalid command\n");
    }
}

static void
handleLine(char *line)
{
    char *argv[10];
    int argc;
    char *tokArg, *tokSave;

    argc = 0;
    tokArg = line;
    while ((argc < (sizeof argv / sizeof argv[0]) - 1)) {
        char *cp = strtok_r(tokArg, " ,", &tokSave);
        if (cp == NULL)
            break;
        argv[argc++] = cp;
        tokArg = NULL;
    }
    argv[argc] = NULL;
    if (modalHandler) {
        (*modalHandler)(argc, argv);
    }
    else {
        commandHandler(argc, argv);
    }
}
/*
 * Check for and act upon character from console
 */
void
consoleCheck(void)
{
    int c;
    static char line[200];
    static int idx = 0;

    /*
     * See if other processor has console output pending
     * Used to be a for(;;) loop but that resulted in this processor
     * remaining stuck here when the other processor started spewing.
     *
     * Emitting characters from other processor could mess up an eye scan
     * but messages from the other processor are important enough that
     * this is acceptable.
     */
    for (int n = 0 ; n < 500 ; n++) {
        static int warnIdx = -1;
        int tail = sharedMemory->stdoutBufTail;
        if (tail == sharedMemory->stdoutBufHead) break;
        c = sharedMemory->stdoutBuf[tail];
        sharedMemory->stdoutBufTail = (tail == (SHARED_RAM_STDOUT_BUFSIZE-1)) ?
                                                                 0 : (tail + 1);
        if (c == ASCII_SO) {
            warnIdx = 0;
        }
        else if (warnIdx >= 0) {
            static char warnBuf[120];
            if (c == '\n') {
                warnBuf[warnIdx] = '\0';
                warn(warnBuf);
                warnIdx = -1;
            }
            else if (warnIdx < ((sizeof warnBuf) - 1)) {
                warnBuf[warnIdx++] = c;
            }
        }
        else {
            outbyte(c);
        }
    }
    if (udpConsole.outIndex != 0){
        if ((MICROSECONDS_SINCE_BOOT() - udpConsole.usAtFirstOutputCharacter) >
                                                                       100000) {
            udpConsoleDrain();
        }
    }

    /*
     * Startup log display in progress?
     */
    if (showingStartup) {
        static int i;
        if (i < startIdx) {
            outbyte(startBuf[i++]);
        }
        else {
            i = 0;
            showingStartup = 0;
        }
    }

    /*
     * Eye scan in progress?
     */
    if (eyescanCrank()) return;

    /*
     * See if UART or other processor has console input pending
     */
    if (!XUartLite_IsReceiveEmpty(STDIN_BASEADDRESS)) {
        c = XUartLite_RecvByte(STDIN_BASEADDRESS) & 0xFF;
        udpConsole.active = 0;
        udpConsole.outIndex = 0;
        udpConsole.inIndex = 0;
        sharedMemory->udpConsole.rxSize = -1;
    }
    else if (sharedMemory->udpConsole.rxSize >= 0) {
        udpConsole.active = 1;
        if (udpConsole.inIndex < sharedMemory->udpConsole.rxSize) {
            c = sharedMemory->udpConsole.rxBuf[udpConsole.inIndex++] & 0xFF;
        }
        else {
            udpConsole.inIndex = 0;
            sharedMemory->udpConsole.rxSize = -1;
            return;
        }
    }
    else {
        cmdTLOGevg1(0, NULL);
        cmdTLOGevg2(0, NULL);
        return;
    }
    cmdTLOGevg1(-1, NULL);
    cmdTLOGevg2(-1, NULL);
    if ((c == '\001') || (c > '\177')) return;
    if (c == '\t') c = ' ';
    else if (c == '\177') c = '\b';
    else if (c == '\r') c = '\n';
    if (c == '\n') {
        isStartup = 0;
        printf("\n");
        line[idx] = '\0';
        idx = 0;
        handleLine(line);
        return;
    }
    if (c == '\b') {
        if (idx) {
            printf("\b \b");
            fflush(stdout);
            idx--;
        }
        return;
    }
    if (c < ' ')
        return;
    if (idx < ((sizeof line) - 1)) {
        printf("%c", c);
        fflush(stdout);
        line[idx++] = c;
    }
}
