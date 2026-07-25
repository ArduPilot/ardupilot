/*
	chaining.c

	Example chaining of C services into one service application,

	To generate non-client-server header and fault handlers:
	$ soapcpp2 -c -CS -penv env.h

	The quote service:
	$ soapcpp2 -c -S -np q quote.h

	The calc service:
	$ soapcpp2 -c -S -np c calc.h

	cc -o chaining.cgi chaining.c stdsoap2.c envC.c qServiceLib.c cServiceLib.c

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2011, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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

/* prevent duplicate declarations for global SOAP Header and Fault */
#define WITH_NOGLOBAL

#include "envH.h" /* include only if this file is needed */
#include "cH.h"
#include "qH.h"

#include "q.nsmap"
#include "c.nsmap"

/* A dummy global table, to keep the linker happy */
struct Namespace namespaces[] = { {NULL} };

int main()
{
  struct soap *soap = soap_new();

  soap_set_namespaces(soap, q_namespaces);
  /* serve over stdin/out, CGI style, see the user manual for a port-based version */
  if (soap_begin_serve(soap))
    soap_print_fault(soap, stderr);
  else if (q_serve_request(soap) == SOAP_NO_METHOD)
  {
    soap_set_namespaces(soap, c_namespaces);
    if (c_serve_request(soap))
      soap_send_fault(soap);
  }
  else if (soap->error)
    soap_send_fault(soap);
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  return 0;
}

int ns__getQuote(struct soap *soap, char *s, float *r)
{ *r = 123; /* a dummy service, stocks don't move */
  return SOAP_OK;
}

int ns__add(struct soap *soap, double a, double b, double *r)
{ *r = a + b;
  return SOAP_OK;
}

int ns__sub(struct soap *soap, double a, double b, double *r)
{ *r = a - b;
  return SOAP_OK;
}

int ns__mul(struct soap *soap, double a, double b, double *r)
{ *r = a * b;
  return SOAP_OK;
}

int ns__div(struct soap *soap, double a, double b, double *r)
{ *r = a / b;
  return SOAP_OK;
}

int ns__pow(struct soap *soap, double a, double b, double *r)
{ *r = pow(a, b);
  return SOAP_OK;
}

