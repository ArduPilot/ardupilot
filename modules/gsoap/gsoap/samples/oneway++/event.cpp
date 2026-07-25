/*
	event.cpp

	C++-style client

	Events based on one-way SOAP messaging using HTTP keep-alive for
	persistent connections. C++ style with Proxy object.

	The 'synchronous' global flag illustrates SOAP one-way messaging,
	which requires an HTTP OK or HTTP Accept response with an empty body to
	be returned by the server.

	Compile:
	soapcpp2 -i event.h
	c++ -o event event.cpp stdsoap2.cpp soapC.cpp soapEventProxy.cpp

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

#include "soapEventProxy.h"
#include "Event.nsmap"

int synchronous = 1;
/* synchronous=0: asynchronous one-way messaging over HTTP (no HTTP response) */
/* synchronous=1: SOAP interoperable synchronous one-way messaging over HTTP */

/* Service details (copied from event.h): */
const char *event_handler_endpoint = "http://localhost:18000";
const char *event_handler_action = "event";

int main()
{ EventProxy e(SOAP_IO_KEEPALIVE | SOAP_XML_INDENT);
  // Events A to C do not generate a response from the server
  fprintf(stderr, "Client Sends Event: A\n");
  if (e.send_handle(EVENT_A))
    e.soap_print_fault(stderr);
  else if (synchronous && e.recv_handle_empty_response())
    e.soap_print_fault(stderr);
  fprintf(stderr, "Client Sends Event: B\n");
  if (e.send_handle(EVENT_B))
    e.soap_print_fault(stderr);
  else if (synchronous && e.recv_handle_empty_response())
    e.soap_print_fault(stderr);
  // connection should not be kept alive after the last call: be nice to the server and tell it that we close the connection after this call
  soap_clr_omode(&e, SOAP_IO_KEEPALIVE);
  fprintf(stderr, "Client Sends Event: C\n");
  if (e.send_handle(EVENT_C))
    e.soap_print_fault(stderr);
  else if (synchronous && e.recv_handle_empty_response())
    e.soap_print_fault(stderr);
  e.soap_close_socket();
  // Re-enable keep-alive
  soap_set_omode(&e, SOAP_IO_KEEPALIVE);
  // Events Z generates a series of responses from the server
  fprintf(stderr, "Client Sends Event: Z\n");
  if (e.send_handle(EVENT_Z))
    e.soap_print_fault(stderr);
  else
  { struct ns__handle response;
    for (;;)
    { if (!soap_valid_socket(e.socket))
      { fprintf(stderr, "Connection was terminated (keep alive disabled?)\n");
        break;
      }
      if (e.recv_handle(response))
      { if (e.error == SOAP_EOF)
          fprintf(stderr, "Connection was gracefully closed by server\n");
        else
	  e.soap_print_fault(stderr);
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
  return 0;
}
