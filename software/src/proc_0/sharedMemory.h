/*
 * Interprocessor communication -- shared memory block
 * Shared memory access is serialized so there is no
 * risk of a read/write or write/write conflict.
 */

#ifndef _SHARED_RAM_H_
#define _SHARED_RAM_H_

#include <stdint.h>
#include "evg.h"
#include "systemParameters.h"

#define SHARED_RAM_COINCIDENCE_COUNT (EVG_COUNT * EVG_COINCIDENCE_COUNT)

#define SHARED_RAM_SYSMON_SIZE    40
#define SHARED_RAM_UDP_BUFSIZE    1460
#define SHARED_RAM_STDOUT_BUFSIZE 4000

#define ASCII_SO '\016'

struct sharedRAM_UDP {
    char    rxBuf[SHARED_RAM_UDP_BUFSIZE];
    int     rxSize;
    char    txBuf[SHARED_RAM_UDP_BUFSIZE];
    int     txSize;
};

struct sharedRAM {
    int     enableEPICSprocessor;
    int     resetFPGA;
    int     debug;
    int     loopbackRequest;
    int     loopbackActual;

    struct systemParameters systemParameters;

    int      requestCoincidenceMeasurement;
    int      requestAlignment;
    int      isAligned;
    uint32_t coincidence[SHARED_RAM_COINCIDENCE_COUNT];
    int16_t  pllPhaseOffset[EVG_COUNT];
    int16_t  pllPhaseError[EVG_COUNT];
    int      referenceFrequency[EVG_COUNT];

    struct sharedRAM_UDP udpTFTP;
    struct sharedRAM_UDP udpConsole;

    uint32_t    sysmonBuf[SHARED_RAM_SYSMON_SIZE];
    int         sysmonCount;

    char    stdoutBuf[SHARED_RAM_STDOUT_BUFSIZE];
    int     stdoutBufHead;
    int     stdoutBufTail;
};
    
#define sharedMemory ((volatile struct sharedRAM *)(XPAR_SHARED_RAM_S_AXI_BASEADDR))
#define debugFlags (sharedMemory->debug)

#endif /* _SHARED_RAM_H_ */
