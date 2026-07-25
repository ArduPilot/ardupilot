/*
	handler.c

	Multi-threaded stand-alone event handler service

	Events are based on one-way SOAP messaging using HTTP keep-alive for
	persistent connections

	The 'synchronous' global flag illustrates SOAP one-way messaging,
	which requires an HTTP OK response with an empty body to be returned
	by the server.

	Copyright (C) 2000-2007 Robert A. van Engelen. All Rights Reserved.

	Compile:
	soapcpp2 -c event.h
	cc -o handler handler.c stdsoap2.c soapC.c soapService.c

	Run:
	handler 18000 &

	Server will time out after 24hr of inactivity

	This code enables keep-alive support which can cause "broken pipes"
	when the client prematurely closes the connection while indicating
	it wants the connection to stay alive. Broken pipes (SIGPIPE) can be
	fixed using MSG_NOSIGNAL, SO_NOSIGPIPE, or with a signal handler,
	but this is not very portable (see code below)

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

#include <unistd.h>
#if defined(_POSIX_THREADS) || defined(_SC_THREADS)
#include <pthread.h>    // use Pthreads
#endif

#define BACKLOG (100)
#define TIMEOUT (24*60*60) /* timeout after 24hrs of inactivity */

void *process_request(void*);

int synchronous = 1;
/* synchronous=0: asynchronous one-way messaging over HTTP (no HTTP response) */
/* synchronous=1: SOAP interoperable synchronous one-way messaging over HTTP */

int main(int argc, char **argv)
{
#if defined(_POSIX_THREADS) || defined(_SC_THREADS)
  pthread_t tid;
  struct soap *tsoap;
#endif
  struct soap soap;
  int port;
  SOAP_SOCKET m, s;
  int i;
  if (argc < 2)
  { fprintf(stderr, "Usage: handler <port>\n");
    exit(1);
  }
  port = atoi(argv[1]);
  /* keep I/O alive and server timeout settings */
  soap_init2(&soap, SOAP_IO_KEEPALIVE, SOAP_IO_KEEPALIVE | SOAP_XML_INDENT);
  soap.accept_timeout = TIMEOUT;
  soap.bind_flags |= SO_REUSEADDR;	/* don't use this in unsecured environments */
  /* soap.socket_flags = MSG_NOSIGNAL; */	/* use this to disable SIGPIPE */
  /* soap.bind_flags |= SO_NOSIGPIPE; */	/* or use this to disable SIGPIPE */
  m = soap_bind(&soap, NULL, port, BACKLOG);
  if (!soap_valid_socket(m))
  { soap_print_fault(&soap, stderr);
    exit(1);
  }
  fprintf(stderr, "Socket connection successful %d\n", m);
  for (i = 1; ; i++)
  { s = soap_accept(&soap);
    if (!soap_valid_socket(s))
    { if (soap.errnum)
        soap_print_fault(&soap, stderr);
      else
        fprintf(stderr, "%s timed out\n", argv[0]);	/* should really wait for threads to terminate, but 24hr timeout should be enough ... */
      break;
    }
    fprintf(stderr, "Thread %d accepts socket %d connection from IP %d.%d.%d.%d\n", i, s, (int)(soap.ip>>24)&0xFF, (int)(soap.ip>>16)&0xFF, (int)(soap.ip>>8)&0xFF, (int)soap.ip&0xFF);
#if defined(_POSIX_THREADS) || defined(_SC_THREADS)
    tsoap = soap_copy(&soap);
    pthread_create(&tid, NULL, (void*(*)(void*))process_request, (void*)tsoap);
#else
    soap_serve(&soap);
    soap_destroy(&soap);
    soap_end(&soap);
#endif
  }
  return 0;
}

void *process_request(void *soap)
{ struct soap *tsoap = (struct soap*)soap;
#if defined(_POSIX_THREADS) || defined(_SC_THREADS)
  pthread_detach(pthread_self());
#endif
  soap_serve(tsoap);
  soap_destroy(tsoap);
  soap_end(tsoap);
  soap_free(tsoap);
  return NULL;
}

int ns__handle(struct soap *soap, enum ns__event event)
{ switch (event)
  { /* each event is just consumed without server response */
    case EVENT_A: fprintf(stderr, "Server Received Event: A\n"); break;
    case EVENT_B: fprintf(stderr, "Server Received Event: B\n"); break;
    case EVENT_C: fprintf(stderr, "Server Received Event: C\n"); break;
    /* after receiving event Z, we echo events A to C back to the client */
    case EVENT_Z: fprintf(stderr, "Server Received Event: Z\n");
    { struct soap *resp = soap_copy(soap);
      /* these multiple sends assume that the client enabled keep-alive */
      fprintf(stderr, "Server Sends Event: A\n");
      soap_send_ns__handle(resp, "http://", NULL, EVENT_A);
      fprintf(stderr, "Server Sends Event: B\n");
      soap_send_ns__handle(resp, "http://", NULL, EVENT_B);
      fprintf(stderr, "Server Sends Event: C\n");
      soap_send_ns__handle(resp, "http://", NULL, EVENT_C);
      soap_end(resp);
      soap_free(resp);
    }
  }
  if (event != EVENT_Z && synchronous)
    return soap_send_empty_response(soap, SOAP_OK); /* HTTP 202 Accepted */
  return SOAP_OK;
}
