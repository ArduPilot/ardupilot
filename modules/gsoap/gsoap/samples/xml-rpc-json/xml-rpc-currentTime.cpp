/*
	xml-rpc-currentTime.cpp

	XML-RPC currenTime (C++ version)

	Prints current time.

	Compile:
	soapcpp2 xml-rpc.h
	c++ xml-rpc-currentTime.cpp xml-rpc.cpp xml-rpc-io.cpp stdsoap2.cpp soapC.cpp

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

using namespace std;

int main()
{
  soap *ctx = soap_new1(SOAP_IO_KEEPALIVE | SOAP_XML_INDENT | SOAP_C_UTFSTRING);
  ctx->send_timeout = 10; // 10 sec, stop if server is not accepting msg
  ctx->recv_timeout = 10; // 10 sec, stop if server does not respond in time

  // set up the method call
  methodCall m(ctx, "http://www.cs.fsu.edu/~engelen/currentTime.cgi", "currentTime.getCurrentTime");
  // make the call and get response params
  params r = m();
  // error?
  if (m.error())
    soap_print_fault(ctx, stderr);
  // empty response means fault in JSON format
  else if (r.empty())
    cout << m.fault() << endl;
  // print time in JSON format (just a string as per json.h)
  else
    cout << "Time = " << r[0] << endl;

  // clean up
  soap_destroy(ctx);
  soap_end(ctx);
  soap_free(ctx);
  return 0;
}

/* Don't need a namespace table. We put an empty one here to avoid link errors */
struct Namespace namespaces[] = { {NULL, NULL} };
