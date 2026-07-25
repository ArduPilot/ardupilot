/*
	curlrest.c

	Example CURL-based HTTP REST client in C

	This example shows an XML REST PUT, GET, POST, DELETE client

	Uses the custom/struct_tm_date.c xsd:date serializer

	To run the client, first build and start the gSOAP webserver on port 8080
	and make sure person.xml (in samples/webserver) is in the webserver's folder
	for the GET and POST operations to work.
	See samples/webserver.

	Compilation:
	$ soapcpp2 -c -0 person.h
	$ cc -Iplugin -Icustom -I. -o curlrest curlrest.c stdsoap2.c soapC.c custom/struct_tm_date.c plugin/curlapi.c -lcurl
	where stdsoap2.c is in the 'gsoap' directory, or use libgsoap:
	$ cc -Iplugin -Icustom -I. -o curlrest curlrest.c soapC.c custom/struct_tm_date.c plugin/curlapi.c -lcurl -lgsoap

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

#include "soapH.h"
#include "soap.nsmap"
#include "curlapi.h"

int main(int argc, char **argv)
{
  struct soap *ctx = soap_new1(SOAP_XML_INDENT | SOAP_XML_STRICT);
  CURL *curl;
  struct Person p;

  /* should init curl once - this call is not thread safe */
  curl_global_init(CURL_GLOBAL_ALL);
  /* create curl handle and set options */
  curl = curl_easy_init();
  /* optionally use chunking (SOAP_IO_CHUNK) and compression (SOAP_ENC_ZLIB)
     though this will write_Person chunked and compressed too, so not enabled here */
  /* soap_set_mode(ctx, SOAP_IO_CHUNK); */
  /* register the soap_curl plugin */
  soap_register_plugin_arg(ctx, soap_curl, curl);
  /* or you can simply call w/o setting up a curl handle (the plugin uses a temporary handle):
     soap_register_plugin(ctx, soap_curl);
  */
  /* set timeouts, also used by the plugin */
  ctx->connect_timeout = 60;
  ctx->send_timeout = 10;
  ctx->recv_timeout = 10;
  soap_set_version(ctx, 0);

  p.name = "John Doe";
  p.dob.tm_year = 66; // 1966 (year since 1900)
  p.dob.tm_mon = 0;   // month 0..11
  p.dob.tm_mday = 31 ;
  p.gender = MALE;

  printf("\nPUT\n");
  if (soap_PUT_Person(ctx, "http://localhost:8080/person.xml", &p))
    soap_print_fault(ctx, stderr);

  printf("\nGET\n");
  if (soap_GET_Person(ctx, "http://localhost:8080/person.xml", &p))
    soap_print_fault(ctx, stderr);
  else
    soap_write_Person(ctx, &p); // default stdout

  printf("\nPOST\n");
  if (soap_POST_send_Person(ctx, "http://localhost:8080/person.xml", &p)
   || soap_POST_recv_Person(ctx, &p))
    soap_print_fault(ctx, stderr);
  else
    soap_write_Person(ctx, &p); // default stdout

  printf("\nDELETE\n");
  if (soap_DELETE(ctx, "http://localhost:8080/person.xml"))
    soap_print_fault(ctx, stderr);

  soap_destroy(ctx);
  soap_end(ctx);
  soap_free(ctx);

  /* cleanup handle */
  curl_easy_cleanup(curl);
  /* we're done so clean up curl */
  curl_global_cleanup();

  return 0;
}

