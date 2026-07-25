//	logging.cpp
//
//	Place this file in the same direcory as stdsoap2.h
//	Requires soapdefs.h (compile stdsoap2.cpp with -DWITH_SOAPDEFS_H
//	and -DDEBUG_CALLBACKS)
//
//	Runtime/Custom logging by Mike Helmick
//	Copyright (c) 2002 - Mike Helmick. Convergys IMG. All Rights Reserved.
//	This contributed code is covered under the MPL 1.1 license

#include "stdsoap2.h"	// includes "soapdefs.h" when compiled with -DWITH_SOAPDEFS_H

void soap_recv_callback(struct soap*, const char*, size_t len);
void soap_sent_callback(struct soap*, const char*, size_t len);
void soap_test_callback(struct soap*, const char*, size_t len);

void
soap_dispatch_callback(struct soap *soap, int idx, const char *msg, size_t len)
{	if (!soap->user)
	{	// you can set stuff up here, streams etc.
		// soap->user is used to pass user-defined data
		// soap->user is never set nor cleared by gSOAP
		soap->user = (void*)&cout;
		// don't forget to clean up the streams in the main code
		// (before discarding the soap runtime environment)
	}
	switch (idx)
	{	case SOAP_INDEX_RECV:
			soap_recv_callback(soap, msg, len);
			break;
	 	case SOAP_INDEX_SENT:
			soap_sent_callback(soap, msg, len);
			break;
	 	case SOAP_INDEX_TEST:
			soap_test_callback(soap, msg, len);
			break;
	}
}

// Note: 'msg' is not 0-terminated!
void
soap_recv_callback(struct soap *soap, const char *msg, size_t len)
{ ostream& os = *(ostream*)soap->user;
  os << endl
	<< "Received:" << endl
	<< "----------------------------------------" << endl;
  os.write(msg, len);
  os << "----------------------------------------" << endl;
}

void
soap_sent_callback(struct soap *soap, const char *msg, size_t len)
{ ostream& os = *(ostream*)soap->user;
  os << endl
	<< "Sent:" << endl
	<< "----------------------------------------" << endl;
  os.write(msg, len);
  os << "----------------------------------------" << endl;
}

void
soap_test_callback(struct soap *soap, const char *msg, size_t len)
{ (*(ostream*)soap->user << "Trace: ").write(msg, len);
}
