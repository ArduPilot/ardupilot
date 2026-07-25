
SOAP-over-UDP

Conformance
-----------

The SOAP-over-UDP specification relies on WS-Addressing. The WS-Addressing.h
file defines the WS-Addressing elements for client and server applications.

The implementation conforms to the SOAP-over-UDP requirements:

* SOAP-over-UDP server endpoint URL format: soap.udp://host:port/path
* Support one-way message-exchange pattern (MEP) where a SOAP envelope is
  carried in a user datagram.
* Support request-response message-exchange pattern (MEP) where SOAP envelopes
  are carried in user datagrams.
* Support multicast transmission of SOAP envelopes carried in user datagrams.
* Support both SOAP 1.1 and SOAP 1.2 envelopes.

The SOAP-over-UDP specification can be found at:

http://msdn.microsoft.com/library/default.asp?url=/library/en-us/dnglobspec/html/soap-over-udp.asp

Features
--------

The following additional features are also available:

* Zlib/gzip message compression (compile -DWITH_GZIP and link with -lgsoapssl).
* SOAP with DIME attachments over UDP.
* SOAP with MIME attachments (SwA) over UDP.
* Support for IPv6 (compile -DWITH_IPV6)

Message retransmission
----------------------

Retransmission is implemented using a retry and exponential back-off algorithm
as per SOAP-over-UDP retransmission requirements, see SOAP-over-UDP Appendix I.

Duplicate message detection
---------------------------

The user is responsible for implementing a duplicate message detection
algorithm, see SOAP-over-UDP Appendix II. An example algorithm is given in
udpserver.c

Closing the socket
------------------
The socket is not automatically closed. At the server side the socket should
never be closed (its bound). At the client side the socket is closed with
soap_done() or soap_free(). Or close the socket explicitly with
soap_closesocket(soap.socket).

Usage
-----

* Header file:

//gsoap wsa schema import: http://schemas.xmlsoap.org/ws/2004/08/addressing
#import "was.h"
struct SOAP_ENV__Header
{ mustUnderstand wsa__AttributedURI                     wsa__MessageID  0;
  mustUnderstand struct wsa__Relationship               *wsa__RelatesTo 0;
  mustUnderstand struct wsa__EndpointReferenceType      *wsa__From      0;
  mustUnderstand struct wsa__EndpointReferenceType      *wsa__ReplyTo   0;
  mustUnderstand struct wsa__EndpointReferenceType      *wsa__FaultTo   0;
  mustUnderstand struct wsa__EndpointReferenceType      *wsa__Recipient 0;
  mustUnderstand wsa__AttributedURI                     wsa__To         0;
  mustUnderstand wsa__AttributedURI                     wsa__Action     0;
};
... your declarations go here (request-response and one-way) ...
... for example:
    int ns__echoString(char *str, char **res);
    int ns__sendString(char *str, void);
    int ns__sendStringResponse(char *res, void);

* Client-side one-way unicast/multicast:
  struct soap soap;
  struct SOAP_ENV__Header header;
  soap_init(&soap);
  if (multicast)
    soap.connect_flags = SO_BROADCAST;
  soap.send_timeout = 1; // 1s timeout
  soap_default_SOAP_ENV__Header(&soap, &header);
  soap.header = &header;
  header.wsa__MessageID = "<message ID>";
  header.wsa__To = "<server URL>";
  header.wsa__Action = "<server action>";
  if (soap_send_ns__echoString(&soap, "soap.udp://...", NULL, "hello world!"))
    soap_print_fault(&soap, stderr);
  soap_destroy(&soap); // cleanup
  soap_end(&soap); // cleanup
  soap_done(&soap); // close connection (should not use soap struct after this)

* Client-side request-response unicast:
  struct soap soap;
  struct SOAP_ENV__Header header;
  struct wsa__EndpointReferenceType replyTo;
  char *res;
  soap_init(&soap);
  soap.send_timeout = 1; // 1s timeout
  soap.recv_timeout = 1; // 1s timeout
  soap_default_SOAP_ENV__Header(&soap, &header);
  soap.header = &header;
  soap_default_wsa__EndpointReferenceType(&soap, &replyTo);
  replyTo.Address = "http://schemas.xmlsoap.org/ws/2004/08/addressing/role/anonymous";
  header.wsa__MessageID = "<message ID>";
  header.wsa__To = "<server URL>";
  header.wsa__Action = "<server action>";
  header.wsa__ReplyTo = &replyTo;
  if (soap_call_ns__echoString(&soap, "soap.udp://...", NULL, "hello world!", &res))
  { if (soap.error == SOAP_EOF && soap.errnum == 0)
      printf("Timeout: message probably already delivered\n");
    else
      soap_print_fault(&soap, stderr);
  }
  else
    printf("UDP server response: %s\n", res);
  soap_destroy(&soap); // cleanup
  soap_end(&soap); // cleanup
  soap_done(&soap); // close connection (should not use soap struct after this)

* Client-side request-response multicast:
  struct soap soap;
  struct SOAP_ENV__Header header;
  struct wsa__EndpointReferenceType replyTo;
  char *res;
  soap_init(&soap);
  soap.connect_flags = SO_BROADCAST;
  soap.send_timeout = 1; // 1s timeout
  soap.recv_timeout = 1; // 1s timeout
  soap_default_SOAP_ENV__Header(&soap, &header);
  soap.header = &header;
  soap_default_wsa__EndpointReferenceType(&soap, &replyTo);
  replyTo.Address = "http://schemas.xmlsoap.org/ws/2004/08/addressing/role/anonymous";
  header.wsa__MessageID = "<message ID>";
  header.wsa__To = "<server URL>";
  header.wsa__Action = "<server action>";
  header.wsa__ReplyTo = &replyTo;
  if (soap_send_ns__echoString(&soap, "soap.udp://...", NULL, "hello world!"))
    soap_print_fault(&soap, stderr);
  else
  { for (;;)
    { if (soap_recv_ns__sendStringResponse(&soap, &res))
        break;
      printf("Multicast response: %s\n", res);
    }
    if (soap.error == SOAP_EOF && soap.errnum == 0)
      printf("Timeout: no more messages received\n");
    else
      soap_print_fault(&soap, stderr);
  }
  soap_destroy(&soap); // cleanup
  soap_end(&soap); // cleanup
  soap_done(&soap); // close connection (should not use soap struct after this)


* Server-side
  struct soap soap;
  soap_init1(&soap, SOAP_IO_UDP); // must set UDP flag
  if (!soap_valid_socket(soap_bind(&soap, host, port, 100)))
  { soap_print_fault(&soap, stderr);
    exit(1);
  }
  for (;;)
  { if (soap_serve(&soap))
      soap_print_fault(&soap, stderr); // just report the problem
    soap_destroy(&soap);
    soap_end(&soap);
  }
  soap_done(&soap);
  ...
  int ns__echoString(struct soap *soap, char *str, char **res)
  { if (!soap->header)
      return soap_sender_fault(soap, "No SOAP header", NULL);
    if (!soap->header->wsa__MessageID)
      return soap_sender_fault(soap, "No WS-Addressing MessageID", NULL);
    soap->header->wsa__RelatesTo = (struct wsa__Relationship*)soap_malloc(soap, sizeof(struct wsa__Relationship));
    soap_default_wsa__Relationship(soap, soap->header->wsa__RelatesTo);
    soap->header->wsa__RelatesTo->__item = soap->header->wsa__MessageID;
    // must check for duplicate messages
    if (check_received(soap->header->wsa__MessageID))
    { printf("Request message %s already received\n", soap->header->wsa__MessageID);
      return SOAP_STOP; // don't return response
    }
    if (!soap->header->wsa__ReplyTo || !soap->header->wsa__ReplyTo->Address)
      return soap_sender_fault(soap, "No WS-Addressing ReplyTo address", NULL);
    soap->header->wsa__To = soap->header->wsa__ReplyTo->Address;
    soap->header->wsa__MessageID = soap_strdup(soap, soap_int2s(soap,
id_count++)) ;
    soap->header->wsa__Action = "http://genivia.com/udp/echoStringResponse";
    *res = str;
    return SOAP_OK;
  }
  ...
  int ns__sendString(struct soap *soap, char *str)
  { if (!soap->header)
      return SOAP_STOP;
    if (!soap->header->wsa__MessageID)
      return SOAP_STOP;
    // must check for duplicate messages
    if (check_received(soap->header->wsa__MessageID))
      return SOAP_STOP;
    return SOAP_OK;
  }
  ...
  int ns__sendStringResponse(struct soap *soap, char *res)
  { return SOAP_NO_METHOD; } // we don't expect to serve this message

Built steps
-----------

A Makefile is included to build client and server applications.

To build the example client:
make udpclient

To build the example server:
make udpserver

To generate the wsa.h file:

wsdl2h -c -t ../../WS/WS-typemap.dat -o wsa.h http://schemas.xmlsoap.org/ws/2004/08/addressing

Then manually replace in WS-Addressing.h
//gsoapopt cw
with
//gsoapopt c
to ensure .wsdl and .xsd files are generated for your Web service
applications.

Example run
-----------

Start udpserver in a shell window (window 1):
$ ./udpserver

Execute udpclient in another shell window (window 2):
$ ./udpclient

Window 1 displays (server):

Accepting requests...
Request message id1 accepted
Response message 1 returned
Accepting requests...
One-way message id2 accepted and serviced
Accepting requests...

Window 2 displays (client):

UDP server response: hello world!

Re-execute the udpclient (window 2):
$ ./udpclient

Window 1 displays (server):

Request message id1 already received
SOAP FAULT: SOAP-ENV:Client
"Stopped: no response sent"
Accepting requests...
One-way message id2 already received
Accepting requests...

Window 2 displays (client):

Timeout: message probably already delivered

