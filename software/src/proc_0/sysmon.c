/*
 * System monitoring -- slow readbacks.
 * Fast readbacks handled directly by processor 1.
 */
#include <stdint.h>
#include "evio.h"
#include "gpio.h"
#include "iicChunk.h"
#include "mmcMailbox.h"
#include "sysmon.h"
#include "util.h"
#include "xadc.h"

void
sysmonUpdate(void)
{
    int i;
    uint32_t *base, *ap;
    uint32_t now = MICROSECONDS_SINCE_BOOT();
    static uint32_t then;

    if ((now - then) < 1000000) return;
    then = now;
    ap = base = (uint32_t *)sharedMemory->sysmonBuf;
    ap = xadcUpdate(ap);
    ap = iicChunkReadback(ap);
    ap = evioFetchSysmon(ap);
    ap = mmcMailboxFetchSysmon(ap);
    sharedMemory->sysmonCount = ap - base;
    for (i = 0 ; i < EVG_COUNT ; i++) {
        /*
         * Channel order set by freq_multi_count instantiation in
         * DualEventGeneratorTop.v.
         */
        GPIO_WRITE(GPIO_IDX_FREQ_MONITOR_CSR, (3 * i) + 1);
        sharedMemory->referenceFrequency[i] =
                              GPIO_READ(GPIO_IDX_FREQ_MONITOR_CSR) & 0x3FFFFFFF;
    }
}
