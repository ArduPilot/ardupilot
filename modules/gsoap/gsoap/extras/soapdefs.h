//	soapdefs.h
//	Place this file in the same directory as stdsoap2.h
//	This file will be included in stdsoap2.h when compiling with
//	-DWITH_SOAPDEFS_H (see stdsoap2.h line 16)
//	See extras/logging.cpp for custom logging
//	
//	Runtime/Custom logging by Mike Helmick
//	Copyright (c) 2002 - Mike Helmick. Convergys IMG. All Rights Reserved.
//	This contributed code is covered under the MPL 1.1 license

#ifndef SOAPDEFS_H
#define SOAPDEFS_H

#ifdef DEBUG_CALLBACKS
#ifndef DEBUG
#define DEBUG
#endif
#define SOAP_MESSAGE sprintf
#define DBGLOG(DBGFILE, CMD) \
{ char fdebug[SOAP_BUFLEN+1];\
  CMD;\
  soap_dispatch_callback(soap, SOAP_INDEX_##DBGFILE, fdebug, strlen(fdebug));\
}
#define DBGMSG(DBGFILE, MSG, LEN) soap_dispatch_callback(soap, SOAP_INDEX_##DBGFILE, MSG, LEN);
void soap_dispatch_callback(struct soap*, int, const char*, size_t);
#endif

#endif
