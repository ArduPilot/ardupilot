/*
	udpclient.c

	SOAP-over-UDP demo client with zlib compression and WS-Addressing

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
#include "udp.nsmap"

#define SERVER "soap.udp://localhost:10000"

int main(int argc, char **argv)
{ struct soap soap;
  struct SOAP_ENV__Header header;
  struct wsa__EndpointReferenceType replyTo;
  char *res;
  char *mid1 = "id1";
  char *mid2 = "id2";

  if (argc > 1)
    mid1 = argv[1];
  if (argc > 2)
    mid2 = argv[2];

  soap_init(&soap);

  soap.send_timeout = 1;
  soap.recv_timeout = 1;

  /* To compress the request message (compile with -DWITH_GZIP): */
#ifdef WITH_GZIP
  soap_set_omode(&soap, SOAP_ENC_ZLIB);
#endif

  /* To transmit MIME attachments:
  soap_set_mime(&soap, NULL, NULL);
  soap_set_mime_attachment(&soap, "abc", 3, SOAP_MIME_7BIT, "text/xml", "cid:abc", NULL, NULL);
  */

  /* To transmit DIME attachments:
  soap_set_dime(&soap);
  soap_set_dime_attachment(&soap, "abc", 3, "cid:abc", "text/xml", 0, NULL);
  */

  /* Prepare SOAP Header */
  soap_default_SOAP_ENV__Header(&soap, &header);
  soap.header = &header;

  soap_default_wsa__EndpointReferenceType(&soap, &replyTo);
  replyTo.Address = "http://schemas.xmlsoap.org/ws/2004/08/addressing/role/anonymous";

  /* Set WS-Addressing elements for request-response unicast */
  header.wsa__MessageID = mid1; /* or use soap_wsa_rand_uuid(soap) for UUID */
  header.wsa__To = "http://genivia.com/udp/server";
  header.wsa__Action = "http://genivia.com/udp/echoString";
  header.wsa__ReplyTo = &replyTo;

  /* Make request-response call */
  if (soap_call_ns__echoString(&soap, SERVER, NULL, "hello world!", &res))
  { if (soap.error == SOAP_EOF && soap.errnum == 0)
      printf("Timeout: message probably already delivered\n");
    else
      soap_print_fault(&soap, stderr);
  }
  else
    printf("UDP server response: %s\n", res);

  /* Reset the SOAP Header */
  soap_default_SOAP_ENV__Header(&soap, &header);
  soap.header = &header;

  /* Set WS-Addressing elements for one-way unicast */
  header.wsa__MessageID = mid2; /* or use soap_wsa_rand_uuid(soap) for UUID */
  header.wsa__To = "http://genivia.com/udp/server";
  header.wsa__Action = "http://genivia.com/udp/sendString";

  /* Send one-way message */
  if (soap_send_ns__sendString(&soap, NULL, NULL, "hello world!"))
    soap_print_fault(&soap, stderr);

  soap_destroy(&soap);
  soap_end(&soap);
  soap_done(&soap);

  return 0;
}
