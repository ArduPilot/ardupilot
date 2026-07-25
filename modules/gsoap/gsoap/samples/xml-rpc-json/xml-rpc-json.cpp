/*
        xml-rpc-json.cpp

        XML-RPC <=> JSON serialization example

        Requires xml-rpc.h, xml-rpc.cpp, json.h, and json.cpp

        Compile:
        soapcpp2 -CSL xml-rpc.h
        c++ -o xml-rpc-json xml-rpc-json.cpp xml-rpc.cpp json.cpp stdsoap2.cpp soapC.cpp

        To put JSON types and operations in a C++ namespace, compile:
        soapcpp2 -qjson -CSL xml-rpc.h
        soapcpp2 -CSL -penv env.h
        c++ -DJSON_NAMESPACE -o xml-rpc-json xml-rpc-json.cpp xml-rpc.cpp json.cpp stdsoap2.cpp jsonC.cpp envC.cpp

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2015, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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
#include <sstream>

using namespace std;

#ifdef JSON_NAMESPACE
using namespace json;
#endif

int main()
{
  // set up context
  soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);

  // create a value
  value v(ctx);

  // create an input stream from a given string with JSON content
  stringstream in("[ [1,\"2\",3.14, true], {\"name\": \"gsoap\", \"major\": 2.8, \"©\": 2015} ]");

  // parse JSON content
  in >> v;
  if (v.soap->error)
    cerr << "Error reading JSON\n";

  // write v in XML-RPC format to cout (soap_write_value is soapcpp2-generated)
  ctx->os = &cout;
  soap_write_value(ctx, &v);
  if (v.soap->error)
    cerr << "Error writing XML-RPC\n"; // assumes writes can fail if deemed critical

  // let's change some of v's values:
  v[0][0] = (char*)v[0][0];   // convert int 1 to string "1"
  v[0][1] = (int)v[0][1];     // convert string "2" to 32 bit int = 2
  v[0][2] = (LONG64)v[0][2];  // truncate 3.14 to 64 bit int = 3
  v[0].size(3);               // reset size to 3 to remove last entry
  v[1]["name"] = L"gSOAP © Genivia";
  v[1]["major"] = 2.9;
  v[1][L"©"] = 2016;
  v[1]["released"] = false;
  // v[2] = deliberately skipped, which will show up as null
  v[3] = (ULONG64)time(0);
  v[4] = 123;          // see below
  v[5] = "123";        // see below
  v[6] = 456;          // see below
  v[7] = v[0] + v[4];  // array concat: copy array v[0] to v[7] extended with one element value of v[4]

  // find all values 123 in v[] and change to 456, increment all other ints,
  // string "123" is also changed to 456, since we deliberately do not guard
  // the change by a type check
  for (value::iterator i = v.begin(); i != v.end(); ++i)
  {
    if ((int)*i == 123)
      *i = 456;
    else if (i->is_int())
      *i = (int)*i + 1;
  }

  // check without changing values
  const value x = v;
  const value y = x[7];
  if (!y.is_array() || !y[0].is_string() || strcmp(y[0], "1"))
    exit(1);
  const value z = v[1]["name"];
  std::wstring w = z;
  std::cout << soap_wchar2s(ctx, w.c_str()) << std::endl;

  // print index, name, and value of structure v[1]
  for (value::const_iterator i = v[1].begin(); i != v[1].end(); ++i)
    cout << "[" << i.index() << "] " << i.name() << ": " << *i << endl;

  // display in JSON format using stream ops defined in json.h/.cpp API
  cout << endl << "JSON output of modified value:" << endl << v << endl;

  // display in XML-RPC format using soap_write_value()
  cout << endl << "XML-RPC output of modified value:" << endl;
  ctx->os = &cout;
  soap_write_value(ctx, &v);
  cout << endl;

  // clean up
  soap_destroy(ctx);
  soap_end(ctx);
  soap_free(ctx);

  return 0;
}

/* Don't need a namespace table. We put an empty one here to avoid link errors */
struct Namespace namespaces[] = { {NULL, NULL} };
