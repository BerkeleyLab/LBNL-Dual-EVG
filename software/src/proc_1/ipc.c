/*
 * Interprocessor communication
 */

#include <string.h>
#include <xparameters.h>
#include "ipc.h"
#include "util.h"

/*
 * Send console output to other processor for handling
 */
void
outbyte(char c)
{
    int head = sharedMemory->stdoutBufHead;
    int nextHead = (head == (SHARED_RAM_STDOUT_BUFSIZE-1)) ? 0 : (head + 1);
    sharedMemory->stdoutBuf[head] = c;
    sharedMemory->stdoutBufHead = nextHead;
}

/*
 * Handle an incoming packet on the console port
 */
static bwudpHandle consoleHandle;
void
callbackConsole(bwudpHandle replyHandle, char *payload, int length)
{
    volatile struct sharedRAM_UDP * udp = &sharedMemory->udpConsole;
    consoleHandle = replyHandle;
    if (udp->rxSize < 0) {
        memcpy((void *)udp->rxBuf, payload, length);
        udp->rxSize = length;
    }
}

/*
 * Handle an incoming packet on the TFTP port
 */
static bwudpHandle tftpHandle;
void
callbackTFTP(bwudpHandle replyHandle, char *payload, int length)
{
    volatile struct sharedRAM_UDP * udp = &sharedMemory->udpTFTP;
    tftpHandle = replyHandle;
    if (udp->rxSize < 0) {
        memcpy((void *)udp->rxBuf, payload, length);
        udp->rxSize = length;
    }
}

/*
 * See if other processor has console or TFTP output pending
 */
void
ipcCrank(void)
{
    if (consoleHandle && (sharedMemory->udpConsole.txSize >= 0)) {
        bwudpSend(consoleHandle, (char *)sharedMemory->udpConsole.txBuf,
                               sharedMemory->udpConsole.txSize);
        sharedMemory->udpConsole.txSize = -1;
    }
    if (tftpHandle && (sharedMemory->udpTFTP.txSize >= 0)) {
        bwudpSend(tftpHandle, (char *)sharedMemory->udpTFTP.txBuf,
                            sharedMemory->udpTFTP.txSize);
        sharedMemory->udpTFTP.txSize = -1;
    }
}

/*
 * Set up UDP ports handled by other processor
 */
#define TFTP_UDP_PORT    69
#define CONSOLE_UDP_PORT 50004
void
ipcInit(void)
{
    bwudpRegisterServer(htons(CONSOLE_UDP_PORT), callbackConsole);
    bwudpRegisterServer(htons(TFTP_UDP_PORT), callbackTFTP);
}
