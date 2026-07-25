/** Interface of the ISAPI_Server class 
  * @file ISAPI_SoapServerFactory.h
  * @author Christian Aberger 
  * Copyright (C) 2001 Genivia Inc and WebWare
  */
#ifndef ISAPI_H_F3A8FE11_2C12_11d5_8BEB
#define ISAPI_H_F3A8FE11_2C12_11d5_8BEB

#include <httpext.h>

/** Internet Server Application Programming Interface server.
  *
  */
class ISAPI_Server {
public:
    static DWORD HttpExtensionProc(EXTENSION_CONTROL_BLOCK *pECB);
    static BOOL GetExtensionVersion(OUT HSE_VERSION_INFO *pVer);
    static BOOL TerminateExtension(DWORD dwFlags);
};

#endif // ISAPI_H_F3A8FE11_2C12_11d5_8BEB
