/** Console Application to test the dynamic SOAP server dlls in a simple commandline environment.
  * @file ConsoleMain.cpp
    It tests the same SOAP server shared libraries that can be later deployed in the Apache server.
    to test it do the following: Start this program, open a commandline prompt window,
    change to the build output directory and enter.
  */
#include <unistd.h>
#include <signal.h>

#include <iostream>
#include <cassert>
#include <ltdl.h>
#include <stdio.h>
#include <memory.h>
#include "stdsoap2.h"
#include "apache_gsoap.h"

/** the parameter passed to the dll for constructing a server. */
static const char szQueryParameter[] = "HelloWorld";

/** Run a single threaded loop to accept client requests.
  */

void main(const int /*argc*/, const char *argv[]) {
    char szSearchPath[1024];
    struct soap soap;
    int nRet = SOAP_OK;
    struct apache_soap_interface Intf;
    memset(&Intf, 0, sizeof Intf);
    
    memset(szSearchPath, 0, sizeof szSearchPath);
    getcwd(szSearchPath, sizeof szSearchPath);
    
    strcat(szSearchPath, "/../example/calculator");
    cout << "path is: '"<< szSearchPath << "'"<< endl;
    static const char szLibraryName[] = "libCalculator";

    int nOK = lt_dlinit();
    if (0 != nOK) {
        cerr << "failed to lt_dlinit()" << endl;
        exit(1);
    } 
    //set the path where the library can be found:
    nOK = lt_dladdsearchdir(szSearchPath);
    if (0 != nOK) {
        cerr << "failed to set search path for loading library" << endl;
    }
    cout << "The the search directory for shared libraries is: '" << lt_dlgetsearchpath() << "'" << endl;
    lt_dlhandle soaplib = NULL;
    void *pfun  = NULL; // our entry point 
    if (0 == nOK) {
        soaplib = lt_dlopenext(szLibraryName);
        if (NULL == soaplib) {
	    cerr << "failed to open " << szSearchPath << ", lib "
		 << szLibraryName << " last error: " << lt_dlerror() << endl;
			nOK = 1;
        } else {
            cout << "the library " << szLibraryName << " was loaded" << endl;
		}
    }
    if (0 == nOK) {
        pfun = (void *)lt_dlsym(soaplib, APACHE_HTTPSERVER_ENTRY_POINT);
        if (NULL == pfun) {
			cout << "entry point " << APACHE_HTTPSERVER_ENTRY_POINT << " not found" << endl;
			nOK = 1;
		} else {
			cout << "entry point " << APACHE_HTTPSERVER_ENTRY_POINT << " found" << endl;
        }
    }
    if (0 == nOK) {
		apache_init_soap_interface_fn pfn = (apache_init_soap_interface_fn)(pfun);
		(*pfn)(&Intf);
		if (NULL != Intf.fsoap_init) {
			(*Intf.fsoap_init)(&soap);
			if (NULL != Intf.fsoap_serve) {
				int nRet = (*Intf.fsoap_serve)(&soap);
			}
			//TODO: call cleanup functions.
       }
    }
    if (NULL != soaplib) {
        int nClose = lt_dlclose(soaplib);
	if (0 != nClose) {
            cerr << "failed to unload the soap server" << endl;
        }
    }
    lt_dlexit();
}

