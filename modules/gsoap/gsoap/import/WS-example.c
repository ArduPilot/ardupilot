/*

	WS-example.c

	Example to demonstrate WS-Header.h

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2004-2008, Robert van Engelen, Genivia Inc. All Rights Reserved.
This software is released under one of the following two licenses:
GPL or Genivia's license for commercial use.
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

This program is released under the GPL with the additional exemption that
compiling, linking, and/or using OpenSSL is allowed.
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

#include "soapH.h"
#include "calc.nsmap"

int main()
{
  struct soap soap;
  struct SOAP_ENV__Header header;
  struct wsa__EndpointReferenceType replyTo;
  double n;

  soap_init(&soap);

  soap_default_SOAP_ENV__Header(&soap, &header);

  soap_default_wsa__EndpointReferenceType(&soap, &replyTo);
  replyTo.Address = "http://websrv.cs.fsu.edu/~engelen/calcclient.cgi";

  header.wsa__MessageID = "uuid:aaaabbbb-cccc-dddd-eeee-ffffffffffff";
  header.wsa__ReplyTo = &replyTo;
  header.wsa__To = "http://websrv.cs.fsu.edu/~engelen/calcserver.cgi";
  header.wsa__Action = "http://websrv.cs.fsu.edu/~engelen/result";
  soap.header = &header;

  printf("Testing: the WS-Addressing request send to the calc server is not understood:\n");

  if (soap_call_ns__add(&soap, NULL, NULL, 1, 2, &n))
    soap_print_fault(&soap, stderr);
  else
    printf("1 + 2 = %g\n", n);

  soap_end(&soap);
  soap_done(&soap);

  return 0;
}
