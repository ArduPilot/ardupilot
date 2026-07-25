/*
	ckdb.c

	HTTP cookie database manager.

	The contents of this file are subject to the gSOAP Public License
	Version 1.0 (the "License"); you may not use this file except in
	compliance with the License. You may obtain a copy of the License at
	http://www.cs.fsu.edu/~engelen/soaplicense.html Software distributed
	under the License is distributed on an "AS IS" basis, WITHOUT WARRANTY
	OF ANY KIND, either express or implied. See the License for the
	specific language governing rights and limitations under the License.

	The Initial Developer of the Original Code is Robert A. van Engelen.
	Copyright (C) 2000-2002 Robert A. van Engelen. All Rights Reserved.

        1. Compile ckdb.h:
           soapcpp2 -cpckdb ckdb.h
        2. Compile ckdb.c:
           cc -DWITH_COOKIES -DWITH_NOGLOBAL -c ckdb.c
        3. Compile and link with main program, e.g. ckdbtest.c:
           soapcpp2 -c ckdbtest.h
           cc -DWITH_COOKIES ckdbtest.c ckdb.o stdsoap2.c soapC.c soapClient.c

*/

#include <sys/stat.h>
#include "stdsoap2.h"
#define WITH_NOGLOBAL
#undef SOAP_FMAC3
#define SOAP_FMAC3 static
#include "ckdbC.c"

int soap_save_cookies(struct soap *soap, const char *pathname)
{
  SOAP_SOCKET socket = soap->socket;
  SOAP_SOCKET sendfd = soap->sendfd;
  soap_begin(soap);
  soap->socket = SOAP_INVALID_SOCKET;	/* make sure plain I/O is used */
  soap->sendfd = open(pathname, O_CREAT|O_TRUNC|O_WRONLY, S_IREAD|S_IWRITE);
  if (soap->sendfd >= 0)
  {
    soap_serialize_cookie(soap, (struct cookie*)soap->cookies);
    soap_begin_send(soap);
    soap_put_cookie(soap, (struct cookie*)soap->cookies, "jar", NULL);
    soap_end_send(soap);
    close(soap->sendfd);
    soap->socket = socket;
    soap->sendfd = sendfd;
    return SOAP_OK;
  }
  soap->socket = socket;
  soap->sendfd = sendfd;
  return SOAP_EOF;
}

int soap_load_cookies(struct soap *soap, const char *pathname)
{
  SOAP_SOCKET socket = soap->socket;
  SOAP_SOCKET recvfd = soap->recvfd;
  soap_begin(soap);
  soap->socket = SOAP_INVALID_SOCKET;	/* make sure plain I/O is used */
  soap->recvfd = open(pathname, O_RDONLY);
  if (soap->recvfd >= 0)
  {
    if (soap_begin_recv(soap))
    {
      close(soap->recvfd);
      soap->socket = socket;
      soap->recvfd = recvfd;
      return soap->error;
    }
    soap->cookies = (struct soap_cookie*)soap_get_cookie(soap, NULL, "jar", NULL);
    if (!soap->cookies && soap->error)
    {
      close(soap->recvfd);
      soap->socket = socket;
      soap->recvfd = recvfd;
      return soap->error;
    }
    if (soap_end_recv(soap))
    {
      close(soap->recvfd);
      soap->socket = socket;
      soap->recvfd = recvfd;
      return soap->error;
    }
    close(soap->recvfd);
    soap->socket = socket;
    soap->recvfd = recvfd;
    return SOAP_OK;
  }
  soap->socket = socket;
  soap->recvfd = recvfd;
  return SOAP_EOF;
}
