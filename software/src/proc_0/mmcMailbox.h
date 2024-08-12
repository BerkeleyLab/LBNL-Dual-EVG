/*
 * Communicate with microcontroller
 */

#ifndef _MMC_MAILBOX_H_
#define _MMC_MAILBOX_H_

#include <stdint.h>

int mmcMailboxInit(void);
void mmcMailboxWrite(unsigned int address, int value);
int mmcMailboxRead(unsigned int address);
uint32_t *mmcMailboxFetchSysmon(uint32_t *ap);
int getU28temperature(void);
int getU29temperature(void);
uint16_t getMMCPG3Count(void);
uint16_t getMMCPG4Count(void);
int mmcMailboxIsInit(void);

#endif /* _MMC_MAILBOX_H_ */
