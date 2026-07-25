/*
See the README.md for details.
*/

#ifndef INCLUDED_gsoapWinInet2_h
#define INCLUDED_gsoapWinInet2_h

#include "stdsoap2.h"
#include <wininet.h>

#ifdef __cplusplus
extern "C" {
#endif 

typedef enum { rseFalse = 0, rseTrue, rseDisplayDlg } wininet_rseReturn;

typedef wininet_rseReturn(*wininet_rse_callback)(HINTERNET a_hHttpRequest, DWORD a_dwErrorCode);

extern void wininet_set_rse_callback(struct soap *a_pSoap, wininet_rse_callback a_pRseCallback);

extern int wininet_plugin(struct soap *a_pSoap, struct soap_plugin *a_pPluginData, void *a_pUnused);

#ifdef __cplusplus
}
#endif 

#endif
