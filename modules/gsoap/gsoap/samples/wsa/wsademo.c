/*
	wsademo.c

	WS-Addressing demo service. See the wsademo.h header file for details.

gSOAP XML Web services tools
Copyright (C) 2000-2008, Robert van Engelen, Genivia Inc., All Rights Reserved.
This part of the software is released under one of the following licenses:
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
#include "wsademo.nsmap"

#include "wsaapi.h" /* from plugin/wsaapi.h */

/******************************************************************************\
 *
 *	Common Constants
 *
\******************************************************************************/

const char *FromAddress    = "http://localhost:11000";
const char *ToAddress      = "http://localhost:11001";
const char *ReplyToAddress = "http://localhost:11002";
const char *FaultToAddress = "http://localhost:11003";

/* the SOAP and WSA actions, as defined in wsdldemo.h service definitions */
const char *RequestAction  = "urn:wsademo/wsademoPort/wsademo";
const char *ResponseAction = "urn:wsademo/wsademoPort/wsademoResponse";

/******************************************************************************\
 *
 *	Main
 *
\******************************************************************************/

int main(int argc, char **argv)
{
  struct soap *soap = soap_new1(SOAP_XML_INDENT);

  soap_register_plugin(soap, soap_wsa);

  if (argc < 2)
  { /* no args: act as CGI service over stdin/out */
    if (soap_serve(soap))
    { soap_print_fault(soap, stderr);
      soap_print_fault_location(soap, stderr);
    }
  }
  else
  {
    int port = atoi(argv[1]);

    if (port)
    { /* stand-alone server serving messages over port */
      soap->bind_flags = SO_REUSEADDR;
      if (!soap_valid_socket(soap_bind(soap, NULL, port, 100)))
      { soap_print_fault(soap, stderr);
        exit(1);
      }
      printf("Server is running\n");
      while (soap_valid_socket(soap_accept(soap)))
      { if (soap_serve(soap))
        { soap_print_fault(soap, stderr);
          soap_print_fault_location(soap, stderr);
        }
        printf("Request served\n");
        soap_destroy(soap);
        soap_end(soap);
      }
    }
    else
    { /* client */
      struct ns__wsademoResult res;
      const char *RequestMessageID;

      RequestMessageID = soap_wsa_rand_uuid(soap);
      soap_wsa_request(soap, RequestMessageID, ToAddress, RequestAction);
      if (argc >= 3)
      { if (strchr(argv[2], 'f'))
          soap_wsa_add_From(soap, FromAddress);
        if (strchr(argv[2], 'r'))
          soap_wsa_add_ReplyTo(soap, ReplyToAddress);
        if (strchr(argv[2], 'n'))
        {
#ifdef SOAP_WSA_2005
          soap_wsa_add_NoReply(soap);
#else
          printf("'NoReply' feature not available with WS-Addressing 2003/2004\n");
#endif
        }
        if (strchr(argv[2], 'e'))
          soap_wsa_add_FaultTo(soap, FaultToAddress);
      }
      if (soap_call_ns__wsademo(soap, ToAddress, NULL, argv[1], &res))
      {
#ifdef SOAP_WSA_2005
        wsa5__FaultCodesType fault;
	const char *info;

        if (soap->error == 202)	/* HTTP ACCEPTED */
          printf("Request was accepted\n");
        else if (soap_wsa_check_fault(soap, &fault, &info))
	{ switch (fault)
	  { case wsa5__EndpointUnavailable:
	      fprintf(stderr, "Server %s is currently not available.\n", info?info:"");
	      break;
            default:
	      fprintf(stderr, "A wsa fault %d occurred:\n", (int)fault);
	      soap_print_fault(soap, stderr);
	      break;
	  }
	}
        else
	  soap_print_fault(soap, stderr);
#else
        if (soap->error == 202)	/* HTTP ACCEPTED */
          printf("Request was accepted\n");
        else
	  soap_print_fault(soap, stderr);
#endif
      }
      else if (res.out)
        printf("Result = %s\n", res.out);
    }
  }
  soap_destroy(soap);
  soap_end(soap);
  soap_done(soap);
  free(soap);
  return 0;
}

/******************************************************************************\
 *
 *	Service Operation of Main Server
 *
\******************************************************************************/

int ns__wsademo(struct soap *soap, char *in, struct ns__wsademoResult *result)
{
  const char *ResponseMessageID;
  if (soap_wsa_check(soap))
    return soap->error;
  printf("Received '%s'\n", in?in:"(null)");
  /* simulate a wsa Fault */
  if (in && !strcmp(in, "error"))
  { printf("Simulating WS-Addressing Fault\n");
#if defined(SOAP_WSA_2005)
    return soap_wsa_error(soap, SOAP_WSA(EndpointUnavailable), ToAddress);
#elif defined(SOAP_WSA_2003)
    return soap_wsa_error(soap, "Endpoint is not available");
#else
    return soap_wsa_error(soap, SOAP_WSA(EndpointUnavailable));
#endif
  }
  /* simulate a user-defined SOAP Fault */
  if (in && !strcmp(in, "fault"))
  { printf("Simulating Server Operation Fault\n");
    return soap_wsa_sender_fault(soap, "The demo service wsademo() operation returned a fault", NULL);
  }
  result->out = in;
  ResponseMessageID = soap_wsa_rand_uuid(soap);
  return soap_wsa_reply(soap, ResponseMessageID, ResponseAction);
}

/******************************************************************************\
 *
 *	Relayed Response Handler for ReplyTo Server
 *
\******************************************************************************/

int ns__wsademoResult(struct soap *soap, char *out)
{
  if (soap_wsa_check(soap))
    return soap_send_empty_response(soap, 500); /* HTTP 500 error */
  printf("Received Result = %s\n", out?out:"");
  return soap_send_empty_response(soap, SOAP_OK); /* HTTP 202 */
}

/******************************************************************************\
 *
 *	Relayed SOAP-ENV:Fault Handler for FaultTo Server
 *
\******************************************************************************/

int SOAP_ENV__Fault(struct soap *soap, char *faultcode, char *faultstring, char *faultactor, struct SOAP_ENV__Detail *detail, struct SOAP_ENV__Code *SOAP_ENV__Code, struct SOAP_ENV__Reason *SOAP_ENV__Reason, char *SOAP_ENV__Node, char *SOAP_ENV__Role, struct SOAP_ENV__Detail *SOAP_ENV__Detail)
{
  printf("Received Fault:\n");
  /* populate the fault struct from the operation arguments to print it */
  soap_fault(soap);
  /* SOAP 1.1 */
  soap->fault->faultcode = faultcode;
  soap->fault->faultstring = faultstring;
  soap->fault->faultactor = faultactor;
  soap->fault->detail = detail;
  /* SOAP 1.2 */
  soap->fault->SOAP_ENV__Code = SOAP_ENV__Code;
  soap->fault->SOAP_ENV__Reason = SOAP_ENV__Reason;
  soap->fault->SOAP_ENV__Node = SOAP_ENV__Node;
  soap->fault->SOAP_ENV__Role = SOAP_ENV__Role;
  soap->fault->SOAP_ENV__Detail = SOAP_ENV__Detail;
  /* set error */
  soap->error = SOAP_FAULT;
  soap_print_fault(soap, stdout);
  return soap_send_empty_response(soap, SOAP_OK); /* HTTP 202 Accepted */
}
