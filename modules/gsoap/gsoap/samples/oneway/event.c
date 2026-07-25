/*
	event.c

	C-style eventing client

	Events are based on one-way SOAP messaging using HTTP keep-alive for
	persistent connections.

	The 'synchronous' global flag illustrates SOAP one-way messaging,
	which requires an HTTP OK or HTTP Accept response with an empty body to
	be returned by the server.

	Copyright (C) 2000-2002 Robert A. van Engelen. All Rights Reserved.

	Compile:
	soapcpp2 -c event.h
	cc -o event event.c stdsoap2.c soapC.c soapClient.c

	Run (first start the event handler on localhost port 18000):
	event

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2008, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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
#include "Event.nsmap"

int synchronous = 1;
/* synchronous=0: asynchronous one-way messaging over HTTP (no HTTP response) */
/* synchronous=1: SOAP interoperable synchronous one-way messaging over HTTP */

/* Service details (copied from event.h): */
const char *event_handler_endpoint = "http://localhost:18000";
const char *event_handler_action = "event";

int main()
{ struct soap soap;
  soap_init2(&soap, SOAP_IO_KEEPALIVE, SOAP_IO_KEEPALIVE | SOAP_XML_INDENT);
  /* Events A to C do not generate a response from the server */
  fprintf(stderr, "Client Sends Event: A\n");
  if (soap_send_ns__handle(&soap, event_handler_endpoint, event_handler_action, EVENT_A))
    soap_print_fault(&soap, stderr);
  if (synchronous && soap_recv_empty_response(&soap))
    soap_print_fault(&soap, stderr);
  fprintf(stderr, "Client Sends Event: B\n");
  if (soap_send_ns__handle(&soap, event_handler_endpoint, event_handler_action, EVENT_B))
    soap_print_fault(&soap, stderr);
  if (synchronous && soap_recv_empty_response(&soap))
    soap_print_fault(&soap, stderr);
  /* reset keep-alive when client needs to inform the server that it will close the connection. It may reconnect later */
  soap_clr_omode(&soap, SOAP_IO_KEEPALIVE);
  fprintf(stderr, "Client Sends Event: C\n");
  if (soap_send_ns__handle(&soap, event_handler_endpoint, event_handler_action, EVENT_C))
    soap_print_fault(&soap, stderr);
  if (synchronous && soap_recv_empty_response(&soap))
    soap_print_fault(&soap, stderr);
  /* close the socket */
  soap_closesock(&soap);
  /* re-enable keep-alive which is required to accept and execute multiple receives */
  soap_set_omode(&soap, SOAP_IO_KEEPALIVE);
  /* Events Z generates a series of response from the server */
  fprintf(stderr, "Client Sends Event: Z\n");
  if (soap_send_ns__handle(&soap, event_handler_endpoint, event_handler_action, EVENT_Z))
    soap_print_fault(&soap, stderr);
  else
  { struct ns__handle response;
    for (;;)
    { if (!soap_valid_socket(soap.socket))
      { fprintf(stderr, "Connection was terminated (keep alive disabled?)\n");
        break;
      }
      if (soap_recv_ns__handle(&soap, &response))
      { if (soap.error == SOAP_EOF)
          fprintf(stderr, "Connection was gracefully closed by server\n");
        else
	  soap_print_fault(&soap, stderr);
	break;
      }
      else
      { switch (response.event)
        { case EVENT_A: fprintf(stderr, "Client Received Event: A\n"); break;
          case EVENT_B: fprintf(stderr, "Client Received Event: B\n"); break;
          case EVENT_C: fprintf(stderr, "Client Received Event: C\n"); break;
          case EVENT_Z: fprintf(stderr, "Client Received Event: Z\n"); break;
        }
      }
    }
  }
  soap_closesock(&soap); /* EVENT_Z has no HTTP response (fire and forget), so close the socket */
  soap_destroy(&soap);
  soap_end(&soap);
  soap_done(&soap); /* detach environment (also closes sockets even with keep-alive) */
  return 0;
}
