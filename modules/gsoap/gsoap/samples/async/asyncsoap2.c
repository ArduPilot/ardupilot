/*
        asyncsoap2.c

        Example synchronous versus asynchronous SOAP messaging with threads

        Compilation:
        $ soapcpp2 -c -CL -r -wx async.h
        $ cc -o asyncsoap2 asyncsoap2.c stdsoap2.c soapC.c soapClient.c

        Run by starting the webserver with HTTP pipeline and keep-alive
        enabled at port 8080, then run asyncsoap2:
        $ ../webserver/webserver -ek 8080 &
        $ ./asyncsoap2

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2019, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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

#include "plugin/threads.h"
#include "soapH.h"
#include "async.nsmap"

#define ENDPOINT "http://localhost:8080"
#define SOAPACTION NULL

#define CHECK(op) if (op) exit(EXIT_FAILURE)

void if_error_then_die(struct soap *soap);

void *async_receiver(void *arg);

MUTEX_TYPE start_lock;
MUTEX_TYPE ready_lock;
COND_TYPE start;
COND_TYPE ready;

int main()
{
  THREAD_TYPE tid;
  struct soap *soap = soap_new();
  double a, b, r;

  MUTEX_SETUP(start_lock);
  MUTEX_SETUP(ready_lock);
  COND_SETUP(start);
  COND_SETUP(ready);

  CHECK(THREAD_CREATEX(&tid, async_receiver, soap));

  soap->connect_timeout = 10;  /* 10 sec connect timeout */
  soap->transfer_timeout = 10; /* 10 second max message transfer time */
  soap->send_timeout = 5;      /* 5 second max socket recv idle time */
  soap->recv_timeout = 5;      /* 5 second max socket send idle time */

  a = 2.0;
  b = 3.0;

  printf("Synchronous SOAP call:\n");
  if (soap_call_ns__add(soap, ENDPOINT, SOAPACTION, a, b, &r))
    if_error_then_die(soap);
  printf("%g + %g = %g\n\n", a, b, r);

  printf("Synchronous SOAP call:\n");
  if (soap_call_ns__mul(soap, ENDPOINT, SOAPACTION, a, b, &r))
    if_error_then_die(soap);
  printf("%g * %g = %g\n\n", a, b, r);

  soap_set_mode(soap, SOAP_IO_KEEPALIVE); /* try to keep the connection alive */

  printf("Asynchronous SOAP send & recv:\n");
  if (soap_send_ns__add(soap, ENDPOINT, SOAPACTION, a, b))
    if_error_then_die(soap);

  CHECK(COND_SIGNAL(start));              /* connection established, start async_receiver */

  printf("Doing some work for one second...\n");
  printf("%g + %g = ", a, b);
  sleep(1);
  CHECK(COND_SIGNAL(start));
  CHECK(MUTEX_LOCK(ready_lock));
  CHECK(COND_WAIT(ready, ready_lock));    /* we may want to use a non-blocking wait instead of blocking */
  CHECK(MUTEX_UNLOCK(ready_lock));

  soap_clr_mode(soap, SOAP_IO_KEEPALIVE); /* optional, to inform the server the next message is the last */

  printf("Asynchronous SOAP send & recv:\n");
  if (soap_send_ns__mul(soap, ENDPOINT, SOAPACTION, a, b))
    if_error_then_die(soap);

  printf("Doing some work for one second...\n");
  printf("%g * %g = ", a, b);
  sleep(1);
  CHECK(COND_SIGNAL(start));
  CHECK(MUTEX_LOCK(ready_lock));
  CHECK(COND_WAIT(ready, ready_lock));  /* we may want to use a non-blocking wait instead of blocking */
  CHECK(MUTEX_UNLOCK(ready_lock));

  THREAD_JOIN(tid);

  soap_force_closesock(soap);           /* optional, soap_free() or a new connection will close the old anyway */

  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);

  MUTEX_CLEANUP(start_lock);
  MUTEX_CLEANUP(ready_lock);
  COND_CLEANUP(start);
  COND_CLEANUP(ready);

  return 0;
}

void if_error_then_die(struct soap *soap)
{
  if (soap->error)
  {
    soap_print_fault(soap, stderr);
    exit(EXIT_FAILURE);
  }
}

void *async_receiver(void *arg)
{
  struct soap *soap;
  double r;

  CHECK(MUTEX_LOCK(start_lock));
  CHECK(COND_WAIT(start, start_lock));
  soap = soap_copy((struct soap*)arg);
  CHECK(MUTEX_UNLOCK(start_lock));

  if (soap_recv_ns__add(soap, &r))
    if_error_then_die(soap);
  printf("%g\n\n", r);

  CHECK(MUTEX_LOCK(start_lock));
  CHECK(COND_WAIT(start, start_lock));
  CHECK(MUTEX_UNLOCK(start_lock));
  CHECK(COND_SIGNAL(ready));

  if (soap_valid_socket(soap->socket))
  {
    if (soap_recv_ns__mul(soap, &r))
      if_error_then_die(soap);
    printf("%g\n\n", r);
  }
  else
  {
    printf("Connection closed, server rejected keep-alive!\n");
  }

  CHECK(MUTEX_LOCK(start_lock));
  CHECK(COND_WAIT(start, start_lock));
  CHECK(MUTEX_UNLOCK(start_lock));
  CHECK(COND_SIGNAL(ready));

  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);

  return NULL;
}
