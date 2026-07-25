/*
	curlclient.c

	Example CURL-based calculator service client in C

	Compilation:
	$ soapcpp2 -c calc.h
	$ cc -Iplugin -I. -o curlclient curlclient.c stdsoap2.c soapC.c soapClient.c plugin/curlapi.c -lcurl
	where stdsoap2.c is in the 'gsoap' directory, or use libgsoap:
	$ cc -Iplugin -I. -o curlclient curlclient.c soapC.c soapClient.c plugin/curlapi.c -lcurl -lgsoap

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
#include "calc.nsmap"
#include "curlapi.h"

const char server[] = "http://websrv.cs.fsu.edu/~engelen/calcserver.cgi";
/* = "http://localhost:8080"; to test against samples/webserver */

int main(int argc, char **argv)
{ struct soap soap;
  CURL *curl;
  double a, b, result;
  if (argc < 4)
  { fprintf(stderr, "Usage: [add|sub|mul|div|pow] num num\n");
    exit(0);
  }
  /* should init curl once - this call is not thread safe */
  curl_global_init(CURL_GLOBAL_ALL);
  /* create curl handle and set options */
  curl = curl_easy_init();
  /* init soap context (with mode parameter) */
  soap_init1(&soap, SOAP_IO_CHUNK | SOAP_XML_INDENT);
  /* register the soap_curl plugin */
  soap_register_plugin_arg(&soap, soap_curl, curl);
  /* or you can simply call w/o setting up a curl handle (the plugin uses a temporary handle):
     soap_register_plugin(&soap, soap_curl);
  */
  /* set timeouts, also used by the plugin */
  soap.connect_timeout = 60;
  soap.send_timeout = 10;
  soap.recv_timeout = 10;
  a = strtod(argv[2], NULL);
  b = strtod(argv[3], NULL);
  switch (*argv[1])
  { case 'a':
      soap_call_ns__add(&soap, server, "", a, b, &result);
      break;
    case 's':
      soap_call_ns__sub(&soap, server, "", a, b, &result);
      break;
    case 'm':
      soap_call_ns__mul(&soap, server, "", a, b, &result);
      break;
    case 'd':
      soap_call_ns__div(&soap, server, "", a, b, &result);
      break;
    case 'p':
      soap_call_ns__pow(&soap, server, "", a, b, &result);
      break;
    default:
      fprintf(stderr, "Unknown command\n");
      exit(0);
  }
  if (soap.error)
  {
    soap_curl_reset(&soap);
    soap_print_fault(&soap, stderr);
  }
  else
    printf("result = %g\n", result);
  soap_destroy(&soap);
  soap_end(&soap);
  soap_done(&soap);
  /* cleanup handle */
  curl_easy_cleanup(curl);
  /* we're done so clean up curl */
  curl_global_cleanup();
  return 0;
}
