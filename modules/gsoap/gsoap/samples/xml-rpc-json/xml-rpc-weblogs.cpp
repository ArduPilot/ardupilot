/*
	xml-rpc-weblogs.cpp

	XML-RPC weblogUpdates.ping example in C++

	See http://xmlrpc.scripting.com/weblogsCom for more details.

	Requires xml-rpc.h and xml-rpc.cpp

	Compile:
	soapcpp2 xml-rpc.h
	cc xml-rpc-weblogs.cpp xml-rpc.cpp xml-rpc-io.cpp stdsoap2.cpp soapC.cpp

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
#include "xml-rpc-io.h"

using namespace std;

#define ENDPOINT "http://rpc.weblogs.com/RPC2"

int main()
{
  soap *ctx = soap_new1(SOAP_C_UTFSTRING);
  // define the weblogsUpdates.ping method
  methodCall ping(ctx, ENDPOINT, "weblogUpdates.ping");
  // set the two parameters of the method
  ping[0] = "Scripting News";
  ping[1] = "http://www.scripting.com/";
  // make the call
  params output = ping();
  // error?
  if (ping.error())
    soap_print_fault(ctx, stderr);
  else if (output.empty())
    cout << ping.fault() << endl;
  else
  { if (output[0].is_struct())
    { if (output[0]["flerror"].is_false())
        cout << "Success:" << endl;
      else
        cout << "Failure:" << endl;
      cout << output[0]["message"] << endl;
    }
    else
      cout << "Message format error:" << endl << output[0] << endl;
  }
  // delete all data
  soap_destroy(ctx);
  soap_end(ctx);
  // free context
  soap_free(ctx);
  return 0;
}

/* Don't need a namespace table. We put an empty one here to avoid link errors */
struct Namespace namespaces[] = { {NULL, NULL} };
