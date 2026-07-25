/** ISAPI extension entry points.
  * @file isapi.cpp
  */
#include "ISAPI_Server.h"

/** called for every request to this dll */
DWORD WINAPI HttpExtensionProc(EXTENSION_CONTROL_BLOCK *pECB) {
    return ISAPI_Server::HttpExtensionProc(pECB);
}
/** The required ISAPI Extension DLL entry point.
    @pVer points to extension version info structure 
    @return always returns TRUE
  */
BOOL WINAPI GetExtensionVersion(HSE_VERSION_INFO *pVer) {
    // set version to httpext.h version constants
    pVer->dwExtensionVersion = MAKELONG(HSE_VERSION_MINOR, HSE_VERSION_MAJOR);
    return ISAPI_Server::GetExtensionVersion(pVer);
}       
/** Last call from IIS before the extension is terminated.
  */
BOOL WINAPI TerminateExtension(DWORD dwFlags) {
    return ISAPI_Server::TerminateExtension(dwFlags);
}

