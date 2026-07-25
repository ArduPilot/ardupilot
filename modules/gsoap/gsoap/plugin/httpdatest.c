/*

httpdatest.c

gSOAP HTTP Digest Authentication example application.

gSOAP XML Web services tools
Copyright (C) 2000-2015, Robert van Engelen, Genivia, Inc., All Rights Reserved.

--------------------------------------------------------------------------------
gSOAP public license.

The contents of this file are subject to the gSOAP Public License Version 1.3
(the "License"); you may not use this file except in compliance with the
License. You may obtain a copy of the License at
http://www.cs.fsu.edu/~engelen/soaplicense.html
Software distributed under the License is distributed on an "AS IS" basis,
WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
for the specific language governing rights and limitations under the License.

The Initial Developer of the Original Code is Robert A. van Engelen.
Copyright (C) 2000-2015, Robert van Engelen, Genivia, Inc., All Rights Reserved.
--------------------------------------------------------------------------------
GPL license.

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place, Suite 330, Boston, MA 02111-1307 USA

Author contact information:
engelen@genivia.com / engelen@acm.org

This program is released under the GPL with the additional exemption that
compiling, linking, and/or using OpenSSL is allowed.
--------------------------------------------------------------------------------

Requires OpenSSL

Compile:

soapcpp2 -c httpdatest.h
cc -DWITH_OPENSSL -o httpdatest httpdatest.c soapC.c soapClient.c soapServer.c httpda.c smdevp.c threads.c stdsoap2.c -lssl -lcrypto -lz

Run server:

./httpdatest 8080

Run client test:

./httpdatest http://127.0.0.1:8080 options

with an option string consisting of:
  c   chunking
  z   compression

*/

#include "httpda.h"
#include "soapH.h"
#include "ns.nsmap"

static char authrealm[] = "gSOAP Authentication Test";

int run_serve(int port);
int run_tests(int,char**);

int main(int argc, char **argv)
{
  int port;
  if (argc < 2)
    return soap_serve(soap_new1(SOAP_XML_INDENT));
  else if (argc < 3 && (port = atoi(argv[1])))
    return run_serve(port);
  else
    return run_tests(argc, argv);
}

int run_serve(int port)
{
  struct soap *soap = soap_new1(SOAP_XML_INDENT);
  int ret;

  if (soap_register_plugin(soap, http_da))
    exit(1);

  if (!soap_valid_socket(soap_bind(soap, NULL, port, 100)))
    soap_print_fault(soap, stderr);
  else
  {
    fprintf(stderr, "Bind to port %d successful\n", port);
    soap->accept_timeout = 3600; /* let server time out after one hour */
    for (;;)
    {
      SOAP_SOCKET sock = soap_accept(soap);
      if (!soap_valid_socket(sock))
      {
        if (soap->errnum)
          soap_print_fault(soap, stderr);
        else
        {
	  fprintf(stderr, "Server timed out\n");
          break;
        }
      }
      fprintf(stderr, "Accepting socket %d connection from IP %d.%d.%d.%d\n", sock, (int)(soap->ip>>24)&0xFF, (int)(soap->ip>>16)&0xFF, (int)(soap->ip>>8)&0xFF, (int)soap->ip&0xFF);
      if (soap_serve(soap))
        soap_print_fault(soap, stderr);
      fprintf(stderr, "Served\n\n");
      soap_destroy(soap);
      soap_end(soap);
    } 
  }
  ret = soap->error;
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  return ret;
}

int run_tests(int argc, char **argv)
{
  struct soap *soap = soap_new1(SOAP_XML_INDENT);
  struct ns__echoString r;
  char *endpoint, *arg;
  int ret;

  if (soap_register_plugin(soap, http_da))
    exit(1);

  endpoint = argv[1];
  if (argc >= 3)
  {
    arg = argv[2];

    if (strchr(arg, 'c'))
      soap_set_omode(soap, SOAP_IO_CHUNK); /* use HTTP chunking */

    if (strchr(arg, 'z'))
      soap_set_omode(soap, SOAP_ENC_ZLIB); /* use HTTP compression */
  }
  else
  {
    arg = "Hello world!";
  }

  if (soap_call_ns__echoString(soap, endpoint, NULL, arg, &r))
  {
    if (soap->error == 401)
    {
      if (!strcmp(soap->authrealm, authrealm))
      {
	/* save userid and passwd for basic or digest authentication */
	struct http_da_info info;
	http_da_save(soap, &info, authrealm, "Mufasa", "Circle Of Life");
        if (!soap_call_ns__echoString(soap, endpoint, NULL, arg, &r))
	{
          /* clean up */
          soap_destroy(soap);
	  soap_end(soap);
	  /* need to restore digest auth info for authentication */
	  http_da_restore(soap, &info);
	  if (!soap_call_ns__echoString(soap, endpoint, NULL, arg, &r))
          {
	    if (!strcmp(arg, r.arg))
              printf("EchoString test OK\n");
            else
              printf("Transmission error\n");
	  } 
        }
        /* release and free digest auth data */
	http_da_release(soap, &info);
	/* regular calls may follow */
      }
    }
  }
  if (soap->error)
    soap_print_fault(soap, stderr);
  ret = soap->error;
  /* clean up and free the context and plugins */
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  return ret;
}

int ns__echoString(struct soap *soap, char *arg, struct ns__echoString *response)
{
  if (soap->userid && soap->passwd)	/* Basic authentication: we may want to reject this since the password was sent in the clear */
  { if (!strcmp(soap->userid, "Mufasa")
     && !strcmp(soap->passwd, "Circle Of Life"))
    {
      response->arg = arg;
      return SOAP_OK;
    }
  }
  else if (soap->authrealm && soap->userid)
  {
    /* simulate database lookup on userid to find passwd */
    if (!strcmp(soap->authrealm, authrealm) && !strcmp(soap->userid, "Mufasa"))
    {
      char *passwd = "Circle Of Life";

      if (!http_da_verify_post(soap, passwd))
      {
        response->arg = arg;
        return SOAP_OK;
      }
    }
  }
  soap->authrealm = authrealm;
  return 401; /* Not authorized, challenge digest authentication with httpda plugin */
}
