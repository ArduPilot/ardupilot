/*
	json-currentTime.c

	JSON currenTime (C version)

	Prints current time.

	Compile:
	soapcpp2 -c -CSL xml-rpc.h
	cc json-currentTime.c xml-rpc.c json.c stdsoap2.c soapC.c

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2012, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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

int main()
{
  struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
  struct value *request = new_value(ctx);
  struct value response;

  ctx->send_timeout = 5; /* 5 sec max socket send idle time */
  ctx->recv_timeout = 5; /* 5 sec max socket recv idle time */
  ctx->transfer_timeout = 30; /* 30 sec message transfer timeout */

  /* make the JSON REST POST request and get response */
  *string_of(request) = "getCurrentTime";
  if (json_call(ctx, "http://www.cs.fsu.edu/~engelen/currentTimeJSON.cgi", request, &response))
    soap_print_fault(ctx, stderr);
  else if (is_string(&response)) /* JSON does not support a dateTime value: this is a string */
    printf("Time = %s\n", *string_of(&response));
  else /* error? */
  {
    printf("Error: ");
    json_write(ctx, &response);
  }

  /* clean up */
  soap_destroy(ctx);
  soap_end(ctx);
  soap_free(ctx);
  return 0;
}

/* Don't need a namespace table. We put an empty one here to avoid link errors */
struct Namespace namespaces[] = { {NULL, NULL} };
