/*
	calcserver.c

	Example calculator service in C

	Compilation in C (see samples/calc/calc.h):
	> soapcpp2 -c calc.h
	> cc -o calcserver calcserver.c stdsoap2.c soapC.c soapServer.c

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2010, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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

#include "tandem.h"

int main(int argc, char **argv)
{ SOAP_SOCKET m, s; /* master and slave sockets */
  struct soap soap;
  if (argc < 2)
  { fprintf(stderr, "Missing tcpip process and port\n");
    fprintf(stderr, "Example: RUN CALCS $ZTC0 3456\n");
    exit(-1);
  }
  soap_init(&soap);
  tandem_init(&soap, argv[1]);
  socket_set_inet_name(argv[1]); /* ? See Tandem TCP/IP programming */
  soap.send_timeout = 10; /* 10 sec */
  soap.recv_timeout = 10; /* 10 sec */
  m = soap_bind(&soap, NULL, atoi(argv[2]), 100);
  if (m == -1)
  { soap_print_fault(&soap, stderr);
    exit(-1);
  }
  fprintf(stderr, "Socket connection successful: master socket = %d\n", m);
  for ( ; ; )
  { s = soap_accept(&soap);
    fprintf(stderr, "Socket connection successful: slave socket = %d\n", s);
    if (s == -1)
    { soap_print_fault(&soap, stderr);
      break;
    } 
    soap_serve(&soap);
    soap_end(&soap);
  }
  tandem_done(&soap);
  soap_done(&soap);
  return 0;
} 

int ns__add(struct soap *soap, double a, double b, double *result)
{ *result = a + b;
  return SOAP_OK;
} 

int ns__sub(struct soap *soap, double a, double b, double *result)
{ *result = a - b;
  return SOAP_OK;
} 

int ns__mul(struct soap *soap, double a, double b, double *result)
{ *result = a * b;
  return SOAP_OK;
} 

int ns__div(struct soap *soap, double a, double b, double *result)
{ if (b)
    *result = a / b;
  else
  { char *s = (char*)soap_malloc(soap, 1024);
    sprintf(s, "<error xmlns=\"http://tempuri.org/\">Can't divide %f by %f</error>", a, b);
    return soap_sender_fault(soap, "Division by zero", s);
  }
  return SOAP_OK;
} 

int ns__pow(struct soap *soap, double a, double b, double *result)
{ *result = pow(a, b);
  if (soap_errno == EDOM)	/* soap_errno is like errno, but compatible with Win32 */
  { char *s = (char*)soap_malloc(soap, 1024);
    sprintf(s, "Can't take the power of %f to %f", a, b);
    sprintf(s, "<error xmlns=\"http://tempuri.org/\">Can't take power of %f to %f</error>", a, b);
    return soap_sender_fault(soap, "Power function domain error", s);
  }
  return SOAP_OK;
} 
