/*
	httpgettest.c

	gSOAP HTTP GET plugin client example application.

	For server example usage of the HTTP GET plugin, see samples/webserver

gSOAP XML Web services tools
Copyright (C) 2000-2006, Robert van Engelen, Genivia, Inc., All Rights Reserved.

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
Copyright (C) 2000-2004, Robert van Engelen, Genivia, Inc., All Rights Reserved.
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

Compile:

soapcpp2 -CS -c httpgettest.h
cc -DWITH_NONAMESPACES -Iplugin -o httpgettest httpgettest.c soapC.c soapClient.c httpget.c stdsoap2.c

To support https and compression, compile with:

cc -DWITH_NONAMESPACES -DWITH_OPENSSL -DWITH_GZIP -Iplugin -o httpgettest httpgettest.c soapC.c httpget.c stdsoap2.c -lssl -lcrypto -lz

*/

#include "soapH.h"
#include "httpget.h"

int main(int argc, char **argv)
{ struct soap soap;
  size_t len = 0;
  char *body;
  if (argc < 2)
  { fprintf(stderr, "Usage: httpgettest URL\n");
    exit(0);
  }
  soap_init(&soap);
  soap_register_plugin(&soap, http_get); /* register plugin */
  if (soap_http_get_connect(&soap, argv[1], NULL)
   || soap_begin_recv(&soap))
  { soap_print_fault(&soap, stderr);
    exit(1);
  }
  if (soap.http_content)
    printf("HTTP Content Type: %s\n", soap.http_content);
  body = soap_http_get_body(&soap, &len);
  soap_end_recv(&soap);
  if (body)
    printf("HTTP Body (%zu bytes) :\n%s\n", len, body);
  soap_end(&soap);
  soap_done(&soap);
  return 0;
}

SOAP_NMAC struct Namespace namespaces[] =
{
	{NULL, NULL, NULL, NULL}
};
