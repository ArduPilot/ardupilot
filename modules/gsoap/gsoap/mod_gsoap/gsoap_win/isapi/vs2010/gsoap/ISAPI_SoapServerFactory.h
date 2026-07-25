/** Interface for the ISAPI_SoapServerFactory class
  * @file ISAPI_SoapServerFactory.h
  * @author Christian Aberger 
  * Copyright (C) 2001 Genivia Inc and WebWare
  */
#ifndef SoapServerFactory_H
#define SoapServerFactory_H
#include <string>
#include "stdsoap2.h"
#include <httpext.h>

typedef SOAP_FMAC1 void (SOAP_FMAC2 *isapi_gsoap_server_fn)(struct soap *); ///< calls soap_serve inside shared library 
typedef SOAP_FMAC1 void (SOAP_FMAC2 *isapi_soap_delete_fn)(struct soap*, void*);
typedef SOAP_FMAC1 int (SOAP_FMAC2 *isapi_soap_register_plugin_fn)(struct soap*, int (*fcreate)(struct soap *, struct soap_plugin *, void*), void *arg);
typedef SOAP_FMAC1 void* SOAP_FMAC2 (*isapi_soap_lookup_plugin_fn)(struct soap*, const char*);

extern "C" {
	typedef int (*mod_gsoap_init_fn)();
	typedef int (*mod_gsoap_terminate_fn)();
}
class mod_gsoap_interface {
public:
	mod_gsoap_interface();
	mod_gsoap_interface(const mod_gsoap_interface&); ///< copy constructor.
	mod_gsoap_interface& operator=(const mod_gsoap_interface&); ///< assignment operator
	bool linked() const; 
public:
        isapi_gsoap_server_fn fsoap_init;
        isapi_gsoap_server_fn fsoap_serve;
        isapi_soap_delete_fn  fsoap_delete;
        isapi_gsoap_server_fn fsoap_done;
        isapi_gsoap_server_fn fsoap_end;
	isapi_soap_register_plugin_fn fsoap_register_plugin_arg;
	isapi_soap_lookup_plugin_fn fsoap_lookup_plugin;
	mod_gsoap_init_fn fmod_gsoap_init;
	mod_gsoap_terminate_fn fmod_gsoap_terminate;

    void *reserved;
};

/** a dynamic link library containing the gsoap entry points soap_init, soap_server etc. */
class SoapDll {
public:
    SoapDll(); ///< constructor
    SoapDll(const SoapDll&); ///< copy constructor
    virtual ~SoapDll(); ///< virtual destructor.

    SoapDll& operator=(const SoapDll&); ///< assignment operator
    bool load(const char *pszPath); ///< dynamically load this dll.
    bool unload(); ///< dynamically load this dll.
	const mod_gsoap_interface *gsoap_interface() const; ///< @return the function pointers for calling into this dll. 
	const char *getLastError() const; ///< @return the latest error message that occurred.
protected:
	bool GetEntryPoints(const char *pszPath);
protected:
    HMODULE m_hDll; ///< the handle if the dll is already loaded.
	std::string m_strLastError; ///< last error message for error reporting
	mod_gsoap_interface m_interface;
};

/** Factory object that creates the appropriate server for a given QueryString.
    See documentation of the createServer function for details of how the 
    QueryString is evaluated.
  */
class ISAPI_SoapServerFactory {
public:
	static ISAPI_SoapServerFactory *instance(); ///< it is a singleton
	void shutdown();

	const mod_gsoap_interface *getInterface(const char *pszDll);
    const char *getLastError(); ///< return the error message if the last operation failed.
protected:
	ISAPI_SoapServerFactory();
	~ISAPI_SoapServerFactory();
	bool GetEntryPoints();
protected:
	class DllMap *m_pDlls;
    std::string m_strError;
	static ISAPI_SoapServerFactory *m_pInstance;
	CRITICAL_SECTION m_cs;
};

#endif // SoapServerFactory_H
