#ifndef WINDOWSCOMM_H
#define WINDOWSCOMM_H

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define usleep(x) Sleep(x/1000)
#else
#include <unistd.h>
#endif // _WIN32

#include "OrionPublicPacketShim.h"

#define UDP_OUT_PORT        8745
#define UDP_IN_PORT         8746
#define TCP_PORT            8747

#ifdef __cplusplus
extern "C"
{
#endif

// Macro to maintain backwards compatibility
#define OrionCommOpenNetwork(void) OrionCommOpenNetworkIp("255.255.255.255")

BOOL OrionCommOpen(int *pArgc, char ***pArgv);
BOOL OrionCommOpenSerial(const char *pPath);
BOOL OrionCommOpenNetworkIp(const char *pAddress);
BOOL OrionCommIpStringValid(const char *pAddress);
void OrionCommClose(void);
BOOL OrionCommSend(const OrionPkt_t *pPkt);
BOOL OrionCommReceive(OrionPkt_t *pPkt);

#ifdef __cplusplus
}
#endif

#endif // WINDOWSCOMM_H
