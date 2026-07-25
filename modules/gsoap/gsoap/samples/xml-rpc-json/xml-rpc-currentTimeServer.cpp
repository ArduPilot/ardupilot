/*
	xml-rpc-currentTimeServer.cpp

	XML-RPC currenTime server (C++ version)
	CGI or stand-alone multi-threaded server

	Returns XML-RPC message with current time to client.

	Compile:
	soapcpp2 xml-rpc.h
	c++ -o xml-rpc-currentTimeServer xml-rpc-currentTimeServer.cpp xml-rpc.cpp xml-rpc-io.cpp stdsoap2.cpp soapC.cpp
	Install as CGI on Web server
	Or run as stand-alone server (e.g. on port 18000):
	./xml-rpc-currentTimeServer 18000

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

#include <unistd.h>
#ifdef _POSIX_THREADS
#include <pthread.h>    // use Pthreads
#endif

#define BACKLOG (100)	// Max. request backlog
#define MAX_THR (8)	// Max. threads to serve requests

using namespace std;

int serve_request(soap*);

int main(int argc, char **argv)
{
  soap *ctx = soap_new1(SOAP_IO_KEEPALIVE | SOAP_XML_INDENT);
  ctx->send_timeout = 10; // 10 sec
  ctx->recv_timeout = 10; // 10 sec

  if (argc < 2)
    return serve_request(ctx);

  int port = atoi(argv[1]);

#ifdef _POSIX_THREADS
  pthread_t tid;
  void *arg;
#endif

  if (!soap_valid_socket(soap_bind(ctx, NULL, port, BACKLOG)))
  {
    soap_print_fault(ctx, stderr);
    exit(1);
  }

  for (;;)
  {
    if (!soap_valid_socket(soap_accept(ctx)))
    {
      soap_print_fault(ctx, stderr);
    }
    else
    {
#ifdef _POSIX_THREADS
      arg = (void*)soap_copy(ctx);
      while (pthread_create(&tid, NULL, (void*(*)(void*))serve_request, arg))
	sleep(1);
#else
      serve_request(ctx);
#endif
    }
  }
  soap_free(ctx);

  return 0;
}

int serve_request(soap* ctx)
{
  methodCall m(ctx);

#ifdef _POSIX_THREADS
  pthread_detach(pthread_self());
#endif

  // HTTP keep-alive max number of iterations
  unsigned int k = ctx->max_keep_alive;

  do
  {
    if (ctx->max_keep_alive > 0 && !--k)
      ctx->keep_alive = 0;

    if (m.recv() != SOAP_OK)
      soap_print_fault(ctx, stderr);
    else
    {
      methodResponse r(ctx);
  
      if (!strcmp(m.name(), "currentTime.getCurrentTime"))
        // method name matches: first parameter of response is time
        r[0] = (ULONG64)time(0);
      else
        // otherwise, set fault
        r.set_fault("Wrong method");

      if (r.send() != SOAP_OK)
        soap_print_fault(ctx, stderr);
    }
    // close (keep-alive may keep socket open when client supports it)
    soap_closesock(ctx);

  } while (ctx->keep_alive);

  int err = ctx->error;

  // clean up
  soap_destroy(ctx);
  soap_end(ctx);

#ifdef _POSIX_THREADS
  // free the ctx copy for this thread
  soap_free(ctx);
#endif

  return err;
}

/* Don't need a namespace table. We put an empty one here to avoid link errors */
struct Namespace namespaces[] = { {NULL, NULL} };
