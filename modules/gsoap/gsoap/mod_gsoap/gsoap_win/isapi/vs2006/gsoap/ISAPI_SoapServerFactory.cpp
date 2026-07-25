/** Implementation of the ISAPI_SoapServerFactory class
  * @file SoapServerFactory.h
  * @author Christian Aberger 
  * Copyright (C) 2001 Genivia Inc and WebWare
  */
#include <cassert>
#include <cstring> // for casecmp
#include <map>
#include <string>
#include "ISAPI_SoapServerFactory.h"
#include "casecmpless.h"
using namespace std;

class CriticalSectionLock {
public:
	CriticalSectionLock(CRITICAL_SECTION *pcs) : m_pcs(pcs) {
		assert(NULL != pcs);
		::EnterCriticalSection(pcs);
	}
	~CriticalSectionLock() {
		::LeaveCriticalSection(m_pcs);
	}
protected:
	CRITICAL_SECTION *m_pcs;
};


ISAPI_SoapServerFactory *ISAPI_SoapServerFactory::m_pInstance = NULL;

mod_gsoap_interface::mod_gsoap_interface() 
: fsoap_init(NULL)
, fsoap_serve(NULL)
, fsoap_delete(NULL)
, fsoap_done(NULL)
, fsoap_end(NULL)
, fsoap_register_plugin_arg(NULL)
, fsoap_lookup_plugin(NULL)
, fmod_gsoap_init(NULL)
, fmod_gsoap_terminate(NULL)
, reserved(NULL)
{
}
mod_gsoap_interface::mod_gsoap_interface(const mod_gsoap_interface& intf) {
	*this = intf;
}
mod_gsoap_interface& mod_gsoap_interface::operator=(const mod_gsoap_interface& intf) {
	fsoap_init = intf.fsoap_init;
	fsoap_serve = intf.fsoap_serve;
	fsoap_delete = intf.fsoap_delete;
	fsoap_done = intf.fsoap_done;
	fsoap_end = intf.fsoap_end;
	fsoap_register_plugin_arg = intf.fsoap_register_plugin_arg;
	fsoap_lookup_plugin = intf.fsoap_lookup_plugin;
	fmod_gsoap_init = intf.fmod_gsoap_init;
	fmod_gsoap_terminate = intf.fmod_gsoap_terminate;
	return *this;
}
/** Have all entry points been found in the gosap server library ?
	@return true if all entry points have been assigned successfully. 
  */
bool mod_gsoap_interface::linked() const {
	return 
		NULL != fsoap_init &&
		NULL != fsoap_serve &&
		NULL != fsoap_done &&
		NULL != fsoap_end &&
		NULL != fsoap_register_plugin_arg &&
		NULL != fsoap_lookup_plugin;
	// don't check for 	fsoap_delete, only servers with classes have it.
}

SoapDll::SoapDll() 
: m_hDll(NULL)
{
}
SoapDll::SoapDll(const SoapDll& dll) {
    *this = dll;
}
SoapDll& SoapDll::operator=(const SoapDll& dll) {
    m_hDll = dll.m_hDll;
    m_strLastError = dll.m_strLastError;
	m_interface = dll.m_interface;
    return *this;
}
bool SoapDll::unload() {
    bool bRet = true;
    if (NULL != m_hDll) {
		if (NULL != m_interface.fmod_gsoap_terminate) {
			int nRet = (*m_interface.fmod_gsoap_terminate)();
		}
        BOOL bFree = ::FreeLibrary(m_hDll);
        bRet = bFree ? true : false;
    }
    return bRet;
}
SoapDll::~SoapDll() {
}
/**
    @param pszPath the name of the dll to load.
  */
bool SoapDll::load(const char *pszPath) {
	bool bRet = true;
    if (NULL == m_hDll) {
		m_strLastError.erase();
		m_hDll = LoadLibrary(pszPath);
		if (NULL != m_hDll) {
			bRet = GetEntryPoints(pszPath);
		} else {
			LPVOID lpMsgBuf = NULL;
			FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
				NULL, GetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // Default language
				(LPTSTR) &lpMsgBuf, 0, NULL);
			m_strLastError = "Attempt to load '" + string(pszPath) + "' failed. ";
			m_strLastError += (LPCTSTR)lpMsgBuf;
			LocalFree(lpMsgBuf);
			bRet = false;
		}
	}
    assert(false == bRet || (NULL != m_hDll));
    return bRet;
}
const char *SoapDll::getLastError() const {
	return m_strLastError.c_str();
}
static isapi_gsoap_server_fn GetFunction(
	HMODULE hDll, 
	const char *pszFunctionName, 
	string& strErrorMessage, 
	const char *pszPath) 
{
	assert(NULL != pszFunctionName);

	isapi_gsoap_server_fn fn = NULL;
	if (NULL != hDll) {
		strErrorMessage.erase();
		FARPROC DllEntryPoint = GetProcAddress(hDll, pszFunctionName); // Find entry point.
		if (NULL != DllEntryPoint) {
			fn = reinterpret_cast<isapi_gsoap_server_fn>(DllEntryPoint);
		} else {
			strErrorMessage = "Entry Point '";
			strErrorMessage += pszFunctionName;
			strErrorMessage += "' not found in dll '" + string(pszPath) + "'.";
		}
	}
	return fn;
}
/** Find the address of all entry points that we must use. */
bool SoapDll::GetEntryPoints(const char *pszPath) {
	assert(NULL != m_hDll);
	
	string strTmp;
	m_interface.fsoap_init    = GetFunction(m_hDll, "soap_initialize", m_strLastError, pszPath);
	m_interface.fsoap_serve   = GetFunction(m_hDll, "soap_serve", m_strLastError, pszPath);
	m_interface.fsoap_delete  = (isapi_soap_delete_fn)GetFunction(m_hDll, "soap_delete", strTmp, pszPath); // soap_delete may be missing for non - cpp
	m_interface.fsoap_done    = GetFunction(m_hDll, "soap_done", m_strLastError, pszPath);
	m_interface.fsoap_end     = GetFunction(m_hDll, "soap_end", m_strLastError, pszPath);
	m_interface.fsoap_register_plugin_arg = (isapi_soap_register_plugin_fn)GetFunction(m_hDll, "soap_register_plugin_arg", m_strLastError, pszPath);
	m_interface.fsoap_lookup_plugin = (isapi_soap_lookup_plugin_fn)GetFunction(m_hDll, "soap_lookup_plugin", m_strLastError, pszPath);
	m_interface.fmod_gsoap_init = (mod_gsoap_init_fn)GetFunction(m_hDll, "mod_gsoap_init", strTmp, pszPath);
	m_interface.fmod_gsoap_terminate = (mod_gsoap_terminate_fn)GetFunction(m_hDll, "mod_gsoap_terminate", strTmp, pszPath);
	bool bRet = m_interface.linked();

	assert(!bRet || m_interface.linked());
	return bRet;
}
const mod_gsoap_interface *SoapDll::gsoap_interface() const {
	return m_interface.linked() ? &m_interface : NULL;
}
/** a map of dll names to the dll handles */
typedef map<string, SoapDll, casecmpless> MapString2Dll; 

class DllMap : public MapString2Dll {
public:
    DllMap() {};
    virtual ~DllMap();
};
DllMap::~DllMap() {
    for (iterator it = begin(); it != end(); ++it) {
        bool bRet = it->second.unload();
        assert(bRet);
    }
}
const char *ISAPI_SoapServerFactory::getLastError() {
    return m_strError.c_str();
}

ISAPI_SoapServerFactory *ISAPI_SoapServerFactory::instance() {
	if (NULL == m_pInstance) {
		m_pInstance = new ISAPI_SoapServerFactory;
	}

	assert(NULL != m_pInstance);
	return m_pInstance;
}
void ISAPI_SoapServerFactory::shutdown() {
	assert(this == m_pInstance);

	m_pInstance = NULL;
	delete this;

	assert(NULL == m_pInstance);
}
ISAPI_SoapServerFactory::ISAPI_SoapServerFactory() 
: m_pDlls(NULL)
{
	::InitializeCriticalSection(&m_cs);
	m_pDlls = new DllMap;
}
ISAPI_SoapServerFactory::~ISAPI_SoapServerFactory() {
	delete m_pDlls;
	::DeleteCriticalSection(&m_cs);
}
/** @return the function pointers of this dll if they could be loaded and found, else NULL.
  */
const mod_gsoap_interface *ISAPI_SoapServerFactory::getInterface(const char *pszDll) {
	const mod_gsoap_interface *pIntf = NULL;
	CriticalSectionLock lock(&m_cs);
	m_strError.erase();
	if (NULL != pszDll) {
		DllMap::iterator it = m_pDlls->find(pszDll);
		if (it == m_pDlls->end()) {
			SoapDll dll;
			bool bLoad = dll.load(pszDll);
			if (bLoad) {
				const mod_gsoap_interface *pInterface = dll.gsoap_interface();
				int nRet = 0;
				if (NULL != pInterface && NULL != pInterface->fmod_gsoap_init) {
					nRet = (*pInterface->fmod_gsoap_init)();
					if (0 != nRet) {
						char szBuf[256];
						_snprintf_s(szBuf, sizeof(szBuf), _TRUNCATE, "failed to initialize %s: mod_gsoap_init returned %d", (const char *)pszDll, nRet);
						m_strError = szBuf;
					}
				}
				if (0 == nRet) {
					it = m_pDlls->insert(DllMap::value_type(pszDll, dll)).first;
				}
			} else {
				m_strError = dll.getLastError();
			}
		}
		if (it != m_pDlls->end()) {
			const SoapDll& dll = it->second;
			pIntf = dll.gsoap_interface();
			if (NULL == pIntf) {
				m_strError = dll.getLastError();
			}
		}
	}
    return pIntf;
}
