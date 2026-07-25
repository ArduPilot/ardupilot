/*
        json-currentTimeServer.cpp

        JSON currenTime server (C++ version)
        CGI or stand-alone multi-threaded server

        Returns JSON message with current time to client.

        Compile:
        soapcpp2 -CSL xml-rpc.h
        c++ -o json-currentTimeServer json-currentTimeServer.cpp json.cpp xml-rpc.cpp stdsoap2.cpp soapC.cpp
        Install as CGI on Web server
        Or run as stand-alone server (e.g. on port 18000):
        ./json-currentTimeServer 18000

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

#include "json.h"

#include "plugin/threads.h"

using namespace std;

int serve_request(soap*);

int main(int argc, char **argv)
{
  soap *ctx = soap_new1(SOAP_C_UTFSTRING);

  if (argc < 2)
  {
    // process CGI request
    value request(ctx);

    if (soap_begin_recv(ctx)
     || json_recv(ctx, request)
     || soap_end_recv(ctx))
    {
      json_send_fault(ctx);
    }
    else
    {
      // if the name matches: set response to time, else error
      if (request.is_string() && !strcmp(request, "getCurrentTime"))
      {
        value response(ctx);
        response = (ULONG64)time(0); // ULONG64 stores a dateTime value in seconds
        // set the http content type
        ctx->http_content = "application/json; charset=utf-8";
        // send the response
        if (soap_response(ctx, SOAP_FILE)
         || json_send(ctx, response)
         || soap_end_send(ctx))
          soap_print_fault(ctx, stderr);
      }
      else
      {
        // JSON error as per Google JSON Style Guide
        json_send_error(ctx, 400, "Wrong method", request);
      }
    }
    soap_destroy(ctx);
    soap_end(ctx);
    soap_free(ctx);
    return 0;
  }

  int port = atoi(argv[1]);

  if (!soap_valid_socket(soap_bind(ctx, NULL, port, 100)))
  {
    soap_print_fault(ctx, stderr);
    exit(1);
  }

  soap_set_mode(ctx, SOAP_IO_KEEPALIVE); // enable HTTP keep-alive, which is optional

  ctx->send_timeout = 10; // 10 sec max socket idle time
  ctx->recv_timeout = 10; // 10 sec max socket idle time
  ctx->transfer_timeout = 30; // 30 sec message transfer timeout

  while (1)
  {
    THREAD_TYPE tid;
    if (soap_valid_socket(soap_accept(ctx)))
    {
      struct soap *cpy = soap_copy(ctx);
      if (!cpy)
        soap_closesock(ctx);
      else
        while (THREAD_CREATE(&tid, (void*(*)(void*))&serve_request, (void*)cpy))
          sleep(1);
    }
    else if (ctx->errnum == 0) // accept timed out, quit looping
    {
      break;
    }
    else // accept failed, try again after 5 seconds
    {
      soap_print_fault(ctx, stderr);
      sleep(5);
    }
  }

  soap_destroy(ctx);
  soap_end(ctx);
  soap_free(ctx);

  return 0;
}

int serve_request(soap* ctx)
{
  THREAD_DETACH(THREAD_ID);

  // HTTP keep-alive max number of iterations
  unsigned int k = ctx->max_keep_alive;
  value request(ctx);

  do
  {
    if (ctx->max_keep_alive > 0 && !--k)
      ctx->keep_alive = 0;

    // receive JSON request
    if (soap_begin_recv(ctx)
     || json_recv(ctx, request)
     || soap_end_recv(ctx))
    {
      json_send_fault(ctx);
    }
    else
    {
      if (request.is_string() && !strcmp(request, "getCurrentTime"))
      {
        value response(ctx);
        response = (ULONG64)time(0); // ULONG64 stores a dateTime value in seconds
        ctx->http_content = "application/json; charset=utf-8";
        if (soap_response(ctx, SOAP_FILE)
         || json_send(ctx, response)
         || soap_end_send(ctx))
          soap_print_fault(ctx, stderr);
      }
      else
      {
        // JSON error as per Google JSON Style Guide
        json_send_error(ctx, 400, "Wrong method", request);
      }
    }
    // close (keep-alive may keep socket open when client supports it)
    soap_closesock(ctx);

  } while (ctx->keep_alive);

  int err = ctx->error;

  // clean up
  soap_destroy(ctx);
  soap_end(ctx);
  soap_free(ctx);

  return err;
}

/* Don't need a namespace table. We put an empty one here to avoid link errors */
struct Namespace namespaces[] = { {NULL, NULL} };

