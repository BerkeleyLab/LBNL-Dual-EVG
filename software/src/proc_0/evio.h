/*
 * Support for EVIO FMC card components:
 *  ADN4605 40x40 crosspoint switch
 *  Firefly modules
 */

#ifndef EVIO_H
#define EVIO_H

#include <stdint.h>

void evioInit(void);
void evioCrank(void);
void evioShowCrosspointRegisters(void);
uint32_t *evioFetchSysmon(uint32_t *ap);
int evioWarmestTransmitter(int fmcIndex);
int evioWarmestReceiver(int fmcIndex);

#endif /* EVIO_H */

