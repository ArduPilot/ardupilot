/*
	wsademo.h

	WS-Addressing demo service. See usage comments below.

gSOAP XML Web services tools
Copyright (C) 2000-2008, Robert van Engelen, Genivia Inc., All Rights Reserved.
This part of the software is released under one of the following licenses:
GPL or the gSOAP public license.
--------------------------------------------------------------------------------
gSOAP public license.

The contents of this file are subject to the gSOAP Public License Version 1.3
(the "License"); you may not use this file except in compliance with the
License. You may obtain a copy of the License at
http://www.cs.fsu.edu/~engelen/soaplicense.html
Software distributed under the License is distributed on an "AS IS" basis,
WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
for the specific language governing rights and limitations under the License.

The Initial Developer of the Original Code is Robert A. van Engelen.
Copyright (C) 2000-2008, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

	This example application demonstrates server-side and client-side logic
	for services based on WS-Addressing. At the server side, WS-Addressing
	enables forwarding/relaying of service responses and faults to other
	services. At the client side, a relayed response or fault will not be
	received and an HTTP ACCEPTED (code 202) is delivered instead, assuming
	that the relay was successful.

	This header file illustrates two gSOAP soapcpp2 tooling tricks to
	enable services to accept SOAP Fault messages and to create a one-way
	service operation to handle responses.

	Compile with (note the use of soapcpp2 -a to handle http action header):

	soapcpp2 -a -c -Iimport wsademo.h
	cc -Iplugin -o wsademo wsademo.c stdsoap2.c soapC.c soapClient.c soapServer.c plugin/wsaapi.c

	Usage:

	After compilation, start the main server at port 11001:
	> ./wsademo 11001

	In a new window, start a return service at port 11002:
	> ./wsademo 11002
	This service handles response messages from the main service.

	In a new window, start a fault service at port 11003:
	> ./wsademo 11003
	This service handles faults from the main service.

	In a new window, run the client:
	> ./wsademo hello
	This example shows the server returning "hello" to the client.

	> ./wsademo fault
	This example shows the server returning a SOAP fault to the client:
	"The demo service wsademo() operation returned a fault".

	> ./wsademo hello r
	This example shows the server returning "hello" to the return service.

	> ./wsademo hello n
	This example shows the server accepting the message without reply.

	> ./wsademo error e
	This example shows the server returning a wsa fault to fault service:
	"The endpoint is unable to process the message at this time".

	> ./wsademo fault e
	This example shows the server returning a SOAP fault to fault service.
	"The demo service wsademo() operation returned a fault".

	Note: when the response service is down, the response cannot be relayed
	and the client (or fault service) will be informed about the failure.
*/

#import "soap12.h"
#import "wsa5.h"

//gsoap ns service name:	wsademo demonstrates WS-Addressing capabilities
//gsoap ns service port:	http://localhost:11001
//gsoap ns service type:	wsademoPort
//gsoap ns service namespace:	urn:wsademo

/* To generate SOAP-ENV:Fault struct via a one-way service operation.
 * This allows us to implement a one-way service operation that accepts Faults.
 * Because a service operation input parameters has a corresponding struct, we
 * automatically generate the (original) SOAP_ENV__Fault struct on the fly!
 * Note: it is important to associate the wsa fault action with this operation
 * as defined below. The action is version-dependent, here we use 2005/08.
 * This declaration is now part of the wsa5.h imported specification and hereby
 * removed from this code.

//gsoap SOAP_ENV service method-action: Fault http://www.w3.org/2005/08/addressing/soap/fault
int SOAP_ENV__Fault
(       _QName			 faultcode,		// SOAP 1.1
        char			*faultstring,		// SOAP 1.1
        char			*faultactor,		// SOAP 1.1
        struct SOAP_ENV__Detail	*detail,		// SOAP 1.1
        struct SOAP_ENV__Code	*SOAP_ENV__Code,	// SOAP 1.2
        struct SOAP_ENV__Reason	*SOAP_ENV__Reason,	// SOAP 1.2
        char			*SOAP_ENV__Node,	// SOAP 1.2
        char			*SOAP_ENV__Role,	// SOAP 1.2
        struct SOAP_ENV__Detail	*SOAP_ENV__Detail,	// SOAP 1.2
	void
);
*/

/* For the server side we need to generate a response struct for each
 * operation to implement one-way service response operations that can be
 * relayed. Because the service operation has a corresponding struct, we can
 * use that struct as a response parameter for the second two-way service
 * operation. This step is required to implement a wsa-capable server.
 */

//gsoap ns service method-header-part:     wsademoResult wsa5__MessageID
//gsoap ns service method-header-part:     wsademoResult wsa5__RelatesTo
//gsoap ns service method-header-part:     wsademoResult wsa5__From
//gsoap ns service method-header-part:     wsademoResult wsa5__ReplyTo
//gsoap ns service method-header-part:     wsademoResult wsa5__FaultTo
//gsoap ns service method-header-part:     wsademoResult wsa5__To
//gsoap ns service method-header-part:     wsademoResult wsa5__Action
//gsoap ns service method-action:          wsademoResult urn:wsademo/wsademoPort/wsademoResponse
//gsoap ns service method-documentation:   wsademoResult accepts a string value from a relayed response
int ns__wsademoResult(char *out, void);

//gsoap ns service method-header-part:     wsademo wsa5__MessageID
//gsoap ns service method-header-part:     wsademo wsa5__RelatesTo
//gsoap ns service method-header-part:     wsademo wsa5__From
//gsoap ns service method-header-part:     wsademo wsa5__ReplyTo
//gsoap ns service method-header-part:     wsademo wsa5__FaultTo
//gsoap ns service method-header-part:     wsademo wsa5__To
//gsoap ns service method-header-part:     wsademo wsa5__Action
//gsoap ns service method-action:          wsademo urn:wsademo/wsademoPort/wsademo
//gsoap ns service method-output-action: wsademo urn:wsademo/wsademoPort/wsademoResponse
//gsoap ns service method-documentation:   wsademo echos a string value and relays the response to the wsa replyTo address (if present)
int ns__wsademo(char *in, struct ns__wsademoResult *result);
