// winstuff.cpp : Defines the entry point for the DLL application.
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


/** Main entry point when dll is loaded or a thread is attached. 
	Well, its windoze. So don't rely on DllMain to be called at every new thread using your server.
	IIS has a thread pool and if you get used in a pooled thread for the first time you will miss the thread attach notification.
*/
BOOL APIENTRY DllMain(HANDLE hModule, DWORD  ul_reason_for_call, LPVOID lpReserved) {
    return TRUE;
}

