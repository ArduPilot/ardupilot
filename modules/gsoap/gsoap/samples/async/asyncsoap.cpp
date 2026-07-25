/*
        asyncsoap.cpp

        Example synchronous versus asynchronous SOAP messaging without threads

        Compilation:
        $ soapcpp2 -j -C -r -wx async.h
        $ c++ -o asyncsoap asyncsoap.cpp stdsoap2.cpp soapC.cpp soapasyncProxy.cpp

        Run by starting the webserver at port 8080, then run asyncsoap:
        $ ../webserver/webserver 8080 &
        $ ./asyncsoap

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

#include "soapasyncProxy.h"
#include "async.nsmap"

#define ENDPOINT "http://localhost:8080"
#define SOAPACTION NULL

void if_error_then_die(struct soap *soap);

int main()
{
  asyncProxy async; /* optionally use SOAP_IO_KEEPALIVE here to improve performance */
  double a, b, r;

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

  printf("Asynchronous SOAP send & recv:\n");
  if (async.send_add(ENDPOINT, SOAPACTION, a, b))
    if_error_then_die(async.soap);
  while (soap_ready(async.soap) == SOAP_EOF)
  {
    printf("Doing some work until server is ready...\n");
    usleep(10000); /* sleep 10ms */
  }
  if_error_then_die(async.soap);
  if (async.recv_add(&r))
    if_error_then_die(async.soap);
  printf("%g + %g = %g\n\n", a, b, r);

  printf("Asynchronous SOAP send & recv:\n");
  if (async.send_mul(ENDPOINT, SOAPACTION, a, b))
    if_error_then_die(async.soap);
  while (soap_ready(async.soap) == SOAP_EOF)
  {
    printf("Doing some work until server is ready...\n");
    usleep(10000); /* sleep 10ms */
  }
  if_error_then_die(async.soap);
  if (async.recv_mul(&r))
    if_error_then_die(async.soap);
  printf("%g * %g = %g\n\n", a, b, r);

  async.destroy();

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
