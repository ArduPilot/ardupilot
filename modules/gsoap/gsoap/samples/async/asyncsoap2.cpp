/*
        asyncsoap2.cpp

        Example synchronous versus asynchronous SOAP messaging without threads

        Compilation:
        $ soapcpp2 -j -C -r -wx async.h
        $ c++ -o asyncsoap2 asyncsoap2.cpp stdsoap2.cpp soapC.cpp soapasyncProxy.cpp

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
#include "soapasyncProxy.h"
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
  asyncProxy async; /* optionally use SOAP_IO_KEEPALIVE here to improve performance */
  double a, b, r;

  MUTEX_SETUP(start_lock);
  MUTEX_SETUP(ready_lock);
  COND_SETUP(start);
  COND_SETUP(ready);

  CHECK(THREAD_CREATEX(&tid, async_receiver, &async));

  async.soap->connect_timeout = 10;  /* 10 sec connect timeout */
  async.soap->transfer_timeout = 10; /* 10 second max message transfer time */
  async.soap->send_timeout = 5;      /* 5 second max socket recv idle time */
  async.soap->recv_timeout = 5;      /* 5 second max socket send idle time */

  a = 2.0;
  b = 3.0;

  printf("Synchronous SOAP call:\n");
  if (async.add(ENDPOINT, SOAPACTION, a, b, &r))
    if_error_then_die(async.soap);
  printf("%g + %g = %g\n\n", a, b, r);

  printf("Synchronous SOAP call:\n");
  if (async.mul(ENDPOINT, SOAPACTION, a, b, &r))
    if_error_then_die(async.soap);
  printf("%g * %g = %g\n\n", a, b, r);

  soap_set_mode(async.soap, SOAP_IO_KEEPALIVE); /* try to keep the connection alive */

  printf("Asynchronous SOAP send & recv:\n");
  if (async.send_add(ENDPOINT, SOAPACTION, a, b))
    if_error_then_die(async.soap);

  CHECK(COND_SIGNAL(start));              /* connection established, start async_receiver */

  printf("Doing some work for one second...\n");
  printf("%g + %g = ", a, b);
  sleep(1);
  CHECK(COND_SIGNAL(start));
  CHECK(MUTEX_LOCK(ready_lock));
  CHECK(COND_WAIT(ready, ready_lock));    /* we may want to use a non-blocking wait instead of blocking */
  CHECK(MUTEX_UNLOCK(ready_lock));

  soap_clr_mode(async.soap, SOAP_IO_KEEPALIVE); /* optional, to inform the server the next message is the last */

  printf("Asynchronous SOAP send & recv:\n");
  if (async.send_mul(ENDPOINT, SOAPACTION, a, b))
    if_error_then_die(async.soap);

  printf("Doing some work for one second...\n");
  printf("%g * %g = ", a, b);
  sleep(1);
  CHECK(COND_SIGNAL(start));
  CHECK(MUTEX_LOCK(ready_lock));
  CHECK(COND_WAIT(ready, ready_lock));  /* we may want to use a non-blocking wait instead of blocking */
  CHECK(MUTEX_UNLOCK(ready_lock));

  THREAD_JOIN(tid);

  async.soap_force_close_socket();      /* optional, destructor or a new connection will close the old anyway */

  async.destroy();

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
  asyncProxy *async;
  double r;

  CHECK(MUTEX_LOCK(start_lock));
  CHECK(COND_WAIT(start, start_lock));
  async = ((asyncProxy*)arg)->copy();
  CHECK(MUTEX_UNLOCK(start_lock));

  if (async->recv_add(&r))
    if_error_then_die(async->soap);
  printf("%g\n\n", r);

  CHECK(MUTEX_LOCK(start_lock));
  CHECK(COND_WAIT(start, start_lock));
  CHECK(MUTEX_UNLOCK(start_lock));
  CHECK(COND_SIGNAL(ready));

  if (soap_valid_socket(async->soap->socket))
  {
    if (async->recv_mul(&r))
      if_error_then_die(async->soap);
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

  async->destroy();

  delete async;

  return NULL;
}
