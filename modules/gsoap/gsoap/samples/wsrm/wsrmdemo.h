/*
	wsrmdemo.h

	WS-ReliableMessaging and WS-Addressing demo service and client.
	This file is to be passed to soapcpp2 only.

	See usage comments below.

gSOAP XML Web services tools
Copyright (C) 2000-2012 Robert van Engelen, Genivia Inc., All Rights Reserved.
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
Copyright (C) 2000-2012, Robert van Engelen, Genivia Inc., All Rights Reserved.
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
	for services based on WS-ReliableMessaging (which uses WS-Addressing).

	WS-ReliableMessaging provides a reliable message sequence mechanism. A
	message sequence is created by the WS-RM source (e.g. client), the
	messages follow with possible acknowledgements and resends of
	unacknowledged messages in between, then the sequence is closed and
	finally terminated. Sequences can expire, to allow local resources to
	be reclaimed.

	WS-ReliableMessaging may not be useful to improve the reliability of
	request-response message exchanges between two parties over HTTP, since
	receiving a response obviously indicates successful delivery of the
	request.

	WS-ReliableMessaging is useful to improve the reliability of one-way
	asynchronous messaging, for unreliable data gram messaging
	(SOAP-over-UDP), or when responses are relayed to other destinations,
	such as response messages that are relayed to destinations indicated by
	the WS-Addressing ReplyTo header. It is also useful when multiple
	sources are sending messages that arrive in different order or must be
	flagged as an incomplete message assemble when messages are missing as
	defined by the notion of a sequence of related messages.

	WS-ReliableMessaging ensures that all messages must have been received
	for the message sequence to terminate normally (indicated by the
	DiscardEntireSequence behavior). Messages in a sequence are enumerated
	and message acknowledgements are message ranges received.
	Acknowledgements are normally sent to the source to help identify which
	messages should be resend. Acknowledgements can also be send to a
	destination, the AcksTo destination server.

	This header file illustrates two gSOAP soapcpp2 tooling tricks to
	enable services to accept SOAP Fault messages (the SOAP_ENV__Fault
	message definition below) and to create a one-way service operation to
	handle WS-Addressing relayed responses (the ns__wsrmdemoResponse
	one-way message definition below) to accept a response as a service
	request.

	Compile with (note the use of soapcpp2 -a to handle http action header):

	soapcpp2 -a -c -Iimport wsrmdemo.h
	cc -Iplugin -o wsrmdemo wsrmdemo.c stdsoap2.c soapC.c soapClient.c soapServer.c plugin/wsaapi.c plugin/wsrmapi.c custom/duration.c

	Alternative compile step, to use SOAP-over-UDP testing (no HTTP!):
	cc -Iplugin -DWITH_UDP -o wsrmdemo_udp wsrmdemo.c stdsoap2.c soapC.c soapClient.c soapServer.c plugin/wsaapi.c plugin/wsrmapi.c custom/duration.c

	Alternative compile step, to use OpenSSL for HTTPS:
	cc -Iplugin -DWITH_OPENSSL -o wsrmdemo_ssl wsrmdemo.c stdsoap2.c soapC.c soapClient.c soapServer.c plugin/wsaapi.c plugin/wsrmapi.c custom/duration.c -lssl -l crypto
	With SSL enabled, please make sure that server.pem, cacert.pem, and
	client.pem are copied with your executable.

	Usage:

	After compilation, start the main server:
	> ./wsrmdemo

	In a new window, run the client:

	> ./wsrmdemo hello
	This example shows the main server returning "hello" to the client.
	WS-RM message acknowledgements are also returned to the client as
	piggy-backed headers, so the client keeps track of ack'ed messages that
	do not need to be resend.

	> ./wsrmdemo hello d
	This example shows the main server returning "hello" to the client in
	duplex mode. The client uses a callback polling service to accept
	response messages on a port.

	> ./wsrmdemo hello ds
	This example shows the main server returning "hello" to the client in
	duplex mode. The client uses a stand-alone (non-polling) callback
	server to accept response messages on a port.

	> ./wsrmdemo fault
	Simulates a fault generated by the server. The fault is non-fatal and
	processing resumes until the sequence is terminated.

	> ./wsrmdemo fault d
	Same, in duplex mode with callback polling.

	> ./wsrmdemo fault ds
	Same, in duplex mode with callback server.

	> ./wsrmdemo error
	Simulates an error generated by the server. The error is fatal and
	further processing is refused by the server.

	> ./wsrmdemo error d
	Same, in duplex mode with callback polling.

	> ./wsrmdemo error ds
	Same, in duplex mode with callback server.

	Note 1: when the ReplyTo service is down, the response cannot be
	relayed and the client (or the fault service when FaultTo is set) will
	be informed about the failure.

	Note 2: HTTP Basic authentication can be enabled by setting the
	following values in wsrmdemo.c:
		const char *userid = "...";
		const char *passwd = "...";
	The client operations set HTTP Basic Auth while server operation check
	it. The WS-RM CreateSequence, CloseSequence, and TerminateSequence do
	NOT check the credentials of the client (only the sequence messages).
	Of course, and incomplete sequence abnormally terminates when the
	behavior is set to DiscardEntireSequence.  To upgrade to HTTP Digest
        authentication, register the HTTP digest authentication plugin at the
        client and server sides.

	Note 3: HTTP compression can be enabled by compiling the sources with
	-DWITH_ZLIB. Then use SOAP_ENC_ZLIB flag to send compressed messages.

	Note 4: HTTPS can be enabled by compiling with -DWITH_OPENSSL.

	Note 5: The client is set-up for two-way messaging, so UDP one-way
	messaging is not demonstrated. This also leads to timeouts at the
	client side when no response is received (and no acks or faults). To
	implement one-way messaging, please refer to the documentation.
*/

#import "soap12.h"
#import "wsrm.h"
#import "wsa5.h"

//gsoap ns service name:	wsrmdemo demonstrates WS-ReliableMessaging capabilities
//gsoap ns service port:	http://localhost:11001
//gsoap ns service type:	wsrmdemoPort
//gsoap ns service namespace:	urn:wsrmdemo

/* We need to generate a response struct for each operation to implement
 * one-way service response operations that can be relayed. Because the service
 * operation has a corresponding struct, we can use that struct as a response
 * parameter for the second two-way service operation. This step is required to
 * implement a wsa-capable server. Future changes to wsdl2h will make manual
 * addition of these one-way operations unnecessary and automated.
 */

//gsoap ns service method-header-part:     wsrmdemoResponse wsa5__MessageID
//gsoap ns service method-header-part:     wsrmdemoResponse wsa5__RelatesTo
//gsoap ns service method-header-part:     wsrmdemoResponse wsa5__From
//gsoap ns service method-header-part:     wsrmdemoResponse wsa5__ReplyTo
//gsoap ns service method-header-part:     wsrmdemoResponse wsa5__FaultTo
//gsoap ns service method-header-part:     wsrmdemoResponse wsa5__To
//gsoap ns service method-header-part:     wsrmdemoResponse wsa5__Action
//gsoap ns service method-header-part:     wsrmdemoResponse wsrm__Sequence
//gsoap ns service method-header-part:     wsrmdemoResponse wsrm__AckRequested
//gsoap ns service method-header-part:     wsrmdemoResponse wsrm__SequenceAcknowledgement
//gsoap ns service method-action:          wsademoResponse urn:wsrmdemo/wsrmdemoPort/wsrmdemoResponse
//gsoap ns service method-documentation:   wsrmdemoResponse accepts a string value from a relayed response
int ns__wsrmdemoResponse(char *out, void);

//gsoap ns service method-header-part:     wsrmdemo wsa5__MessageID
//gsoap ns service method-header-part:     wsrmdemo wsa5__RelatesTo
//gsoap ns service method-header-part:     wsrmdemo wsa5__From
//gsoap ns service method-header-part:     wsrmdemo wsa5__ReplyTo
//gsoap ns service method-header-part:     wsrmdemo wsa5__FaultTo
//gsoap ns service method-header-part:     wsrmdemo wsa5__To
//gsoap ns service method-header-part:     wsrmdemo wsa5__Action
//gsoap ns service method-header-part:     wsrmdemo wsrm__Sequence
//gsoap ns service method-header-part:     wsrmdemo wsrm__AckRequested
//gsoap ns service method-header-part:     wsrmdemo wsrm__SequenceAcknowledgement
//gsoap ns service method-action:          wsrmdemo urn:wsrmdemo/wsrmdemoPort/wsrmdemo
//gsoap ns service method-output-action:   wsrmdemo urn:wsrmdemo/wsrmdemoPort/wsrmdemoResponse
//gsoap ns service method-documentation:   wsrmdemo echos a string value and relays the response to the wsa replyTo address (if present)
int ns__wsrmdemo(char *in, struct ns__wsrmdemoResponse *result);
