// calc.cpp : Defines the entry point for the DLL application.
//

#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
#include <windows.h>
#include <winsock2.h>

#if _MSC_VER > 1000
#pragma comment(lib, "wsock32")
#endif

static struct WSAInit { \
        WSADATA WsaData; \
        WSAInit() {WSAStartup(0x0101, &WsaData);} \
        ~WSAInit() {WSACleanup();} \
} WsaInit;


BOOL APIENTRY DllMain(HANDLE hModule, DWORD  ul_reason_for_call, LPVOID lpReserved) {
    return TRUE;
}

