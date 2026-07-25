/*
	json-currentTime.c

	JSON currenTime with the CURL plugin for gSOAP

	Prints current time.

	Requires the following files from samples/xml-rpc-json:
	  xml-rpc.h
	  xml-rpc.c
	  json.h
	  json.c

	Compile:
	soapcpp2 -c -CSL xml-rpc.h
	cc curl-json-currentTime.c xml-rpc.c json.c stdsoap2.c soapC.c plugin/curlapi.c -lcurl

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2017, Robert van Engelen, Genivia, Inc. All Rights Reserved.
This software is released under one of the following two licenses:
GPL.
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
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

#include "json.h"
#include "curlapi.h"

int main()
{
  struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
  struct value *request = new_value(ctx);
  struct value response;

  curl_global_init(CURL_GLOBAL_ALL);
  soap_register_plugin(ctx, soap_curl);
  ctx->connect_timeout = 60; /* 60 sec, stop if server is not connecting */
  ctx->send_timeout = 10; /* 10 sec, stop if server is not accepting msg */
  ctx->recv_timeout = 10; /* 10 sec, stop if server does not respond in time */

  /* make the JSON REST POST request and get response */
  *string_of(request) = "getCurrentTime";
  if (json_call(ctx, "http://www.cs.fsu.edu/~engelen/currentTimeJSON.cgi", request, &response))
  {
    soap_curl_reset(ctx);
    soap_print_fault(ctx, stderr);
  }
  else if (is_string(&response)) /* JSON does not support a dateTime value: this is a string */
  {
    printf("Time = %s\n", *string_of(&response));
  }
  else /* error? */
  {
    printf("Error: ");
    json_write(ctx, &response);
  }

  /* clean up */
  soap_destroy(ctx);
  soap_end(ctx);
  soap_free(ctx);

  curl_global_cleanup();

  return 0;
}

/* Don't need a namespace table. We put an empty one here to avoid link errors */
struct Namespace namespaces[] = { {NULL, NULL} };
