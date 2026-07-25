/*
	dom2calc.cpp

	Example DOM processing with a calculator service
	This is version 2 based on the improved DOM API v5 gSOAP 2.8.28

	This example is based on the calculator service (samples/calc) and
	implements a DOM-message-based calculator client.

	See gsoap/doc/dom/html/index.html for the DOM API documentation,
	including the domcpp tool that was used to generate the XPath code in
	this exampple.

	soapcpp2 -Iimport dom2calc.h
	cc -o dom2calc dom2calc.cpp stdsoap2.cpp soapC.cpp dom.cpp

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2016, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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

const char server[] = "http://websrv.cs.fsu.edu/~engelen/calcserver.cgi";

// copied and modified from:
// domcpp -M -n -rrequest calc.add.req.xml
struct Namespace namespaces[] = {
  {"SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/", "http://www.w3.org/*/soap-envelope",      NULL},
  {"SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/", "http://www.w3.org/*/soap-encoding",      NULL},
  {"xsi",      "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*/XMLSchema-instance", NULL},
  {"xsd",      "http://www.w3.org/2001/XMLSchema",          "http://www.w3.org/*/XMLSchema",          NULL},
  {"ns", "urn:calc", NULL, NULL },
  {NULL, NULL, NULL, NULL}
};

int main(int argc, char **argv)
{
  struct soap *ctx = soap_new1(SOAP_DOM_TREE | SOAP_XML_INDENT);

  if (argc <= 3)
  {
    std::cerr << "Usage: dom2calc [add|sub|mul|div|pow] <num> <num>" << std::endl;
    exit(1);
  }

  // create command tag ns:add, ns:sub, ns:mul, ns:div, or ns:pow
  std::string command = std::string("ns:").append(argv[1]);

  // copied and modified from:
  // domcpp -M -n -rrequest calc.add.req.xml
  xsd__anyType request(ctx, "SOAP-ENV:Envelope");
  request["SOAP-ENV:Body"].att("SOAP-ENV:encodingStyle") = "http://schemas.xmlsoap.org/soap/encoding/";
  request["SOAP-ENV:Body"][command.c_str()]["a"] = strtod(argv[2], NULL);
  request["SOAP-ENV:Body"][command.c_str()]["b"] = strtod(argv[3], NULL);

  std::cout << "** Request message: " << std::endl << request << std::endl << std::endl;

  // create response
  xsd__anyType response(ctx);

  // invoke server: make POST XML request and receive XML response
  if (soap_dom_call(ctx, server, "", request, response))
  {
    soap_stream_fault(ctx, std::cerr);
  }
  else
  {
    std::cout << "** Response message:" << std::endl << response << std::endl << std::endl;

    // copied from:
    // domcpp -p'/SOAP-ENV:Envelope/SOAP-ENV:Body/ns:addResponse/result' -rresponse -x'std::cout << "Result = " << v << std::endl;'
    if (response.match("SOAP-ENV:Envelope"))
    {
      size_t pos = 1;
      for (xsd__anyType *it = response.elt_get("SOAP-ENV:Body"); it; it = it->get_next(), ++pos)
      {
	xsd__anyType& v = *it;
	size_t pos = 1;
	for (xsd__anyType *it = v.elt_get("ns:addResponse"); it; it = it->get_next(), ++pos)
	{
	  xsd__anyType& v = *it;
	  size_t pos = 1;
	  for (xsd__anyType *it = v.elt_get("result"); it; it = it->get_next(), ++pos)
	  {
	    xsd__anyType& v = *it;
	    std::cout << "Result = " << v.get_double() << std::endl;
	  }
	}
      }
    }

    // Here is another way to search DOM for <result> element(s):
    // for (xsd__anyType::iterator it = response.find("result"); it != response.end(); ++it)
      // std::cout << std::endl << "Result = " << it->get_double() << std::endl;
  }

  soap_destroy(ctx); // delete objects
  soap_end(ctx);     // delete temp data
  soap_free(ctx);    // free context

  return 0;
}
