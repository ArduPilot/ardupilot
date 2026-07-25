/*
        wsddapi.c

        WS-Discovery 1.0 and 1.1 (WSDD) plugin API

        See gsoap/doc/wsdd for the WSDD plugin user guide.

gSOAP XML Web services tools
Copyright (C) 2000-2011, Robert van Engelen, Genivia Inc., All Rights Reserved.
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
Copyright (C) 2000-2011, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

/**

@mainpage

- @ref wsdd_0 documents the wsdd library for WS-Discovery support.

*/

/**

@page wsdd_0 The WS-Discovery plugin

[TOC]

@section wsdd_1 WS-Discovery Setup

The material in this section relates to the WS-Discovery specification.

To use the wsdd library:

-# Define WS-Discovery event handlers in your code, see Section @ref wsdd_2.
-# Use the wsdd API functions as described below.
-# (Re-)compile and link stdsoap2.c/pp or libgsoap, (dom.c/.cpp when needed),
   wsddapi.c, wsaapi.c and the soapcpp2-generated source files. Compile and link
   with plugin/threads.c when needed.

The material in this document pertains to the WS-Discovery protocol and model
and assumes that the reader is familiar with the WS-Discovery protocol, its
terms and definitions, and the WS-Discovery model. This document describes the
WS-Discovery interface to invoke and handle WS-Discovery events, while the
higher-level logic remains application-specific. Especially the mode of
operation, ad-hoc or managed with a Discovery Proxy, depends on the application
deployment and WS-Discovery support requirements.

The following assumptions are made. A Client (C) is an endpoint that searches for
Target Service(s). A Target Service (TS) is a service endpoint that makes itself
available for discovery. A Discovery Proxy (DP) is an endpoint that facilitates
discovery of Target Services by Clients. The interfaces defined in the wsdd
library can be used to implement Client, Target Service, and Discovery Proxy
operations.

WS-Discovery ad-hoc and managed modes are supported by the wsdd library. In an
ad-hoc mode discovery messages are sent multicast and response messages are
sent unicast.  In a managed mode discovery messages are sent unicast to a
Discovery Proxy.

@note The wsdl2h tool uses typemap.dat to auto-generate a gSOAP service
definitions header file that imports wsdd.h to support WS-Discovery 1.1 with
WS-Addressing 2005/08. The tool imports wsdd10.h to support WS-Discovery 1.0
with WS-Addressing 2004/08. To use WS-Discovery 1.0 with WS-Addressing 2005/08,
you will need to change typemap.dat to let wsdl2h import wsdd5.h (see comments
in typemap.dat).

@section wsdd_2 WS-Discovery Event Handlers

The following event handlers MUST be defined by the user to handle inbound
WS-Discovery messages. The event handlers receive notifications (Hello, Bye,
ProbeMatches, and ResolveMatches) or receive requests to provide data (Probe
and Resolve).

The event handlers to define are:
- @ref wsdd_event_Hello
- @ref wsdd_event_Bye
- @ref wsdd_event_Probe
- @ref wsdd_event_ProbeMatches
- @ref wsdd_event_Resolve
- @ref wsdd_event_ResolveMatches

See the documentation provided with each of these functions in wsddapi.h.

@section wsdd_3 WS-Discovery Event Listener

Inbound WS-Discovery multicast messages are handled via a listener on a port.
The user-defined event handlers are invoked when WS-Discovery messages arrive
on the port.

The `soap_wsdd_listen` function listens on the current port opened with
`soap_bind` for WS-Discovery messages for a brief time period as specified by a
timeout value in seconds (negative for micro seconds). The function allows for
periodically polling the port as shown:

@code
    #include "wsddapi.h"
    int port = 8080;
    struct soap *soap = soap_new();
    soap->user = (void*)&my_state;
    if (!soap_valid_socket(soap_bind(soap, port, 100)))
    {
      soap_print_fault(soap, stderr);
      exit(0);
    }
    soap_wsdd_listen(soap, -1000); // listen for messages for 1 ms
    ...
@endcode

WS-Discovery messages are relayed to the event handlers. The soap->user pointer
can be used to point to a state object that is updated by the event handlers as
per desired behavior at the Client side, the Target Service, or the Discovery
Proxy implementation.

@section wsdd_4 Invoking WS-Discovery Operations

A Client may invoke the following WS-Discovery operations:
- `soap_wsdd_Probe`
- `soap_wsdd_Resolve`

A Target Service may invoke the following WS-Discovery operations:

- `soap_wsdd_Hello`
- `soap_wsdd_Bye`
- `soap_wsdd_ProbeMatches` (e.g. via `soap_wsdd_listen`)
- `soap_wsdd_ResolveMatches` (e.g. via `soap_wsdd_listen`)

A Discovery Proxy can perform all operations listed above, and should use
"wsdd:DiscoveryProxy" as the Type with the Hello, Bye, and ProbeMatches.

To send a Hello message to join a network:

@code
    soap_wsdd_Hello(soap,
      SOAP_WSDD_MANAGED,    // or SOAP_WSDD_ADHOC for ad-hoc mode
      "to address",         // "http(s):" URL, or "soap.udp:" UDP, or TCP/IP address
      soap_wsa_rand_uuid(soap), // a unique message ID
      NULL,
      "my address",         // where they can find me for WS-Discovery
      "wsdd:DiscoveryProxy",// Types: I'm a DP
      NULL,                 // Scope
      NULL,                 // MatchBy
      NULL,                 // XAddrs
      75965);               // MDVersion
@endcode

Note that `Types` above is a string with namespace-qualified names (QNames). These
should be qualified as in "namespace":name or you can use a namespace prefix
that is part of your namespace table (in the .nsmap). So you can use
"wsdd:DiscoveryPRoxy" as a QName in Types because wsdd is a namespace prefix
with a defined binding in the namespace table.

For UDP multicast, use

@code
    soap.connect_flags = SO_BROADCAST;
@endcode

and optionally set the interface and TTL settings:

@code
    in_addr_t addr = inet_addr("1.2.3.4");
    soap.ipv4_multicast_if = &addr; // see setsockopt IPPROTO_IP IP_MULTICAST_IF
    soap.ipv6_multicast_if = addr; // multicast sin6_scope_id
    soap.ipv4_multicast_ttl = 1; // see setsockopt IPPROTO_IP, IP_MULTICAST_TTL
@endcode

Please refer to the socket options for `IPPROTO_IP` `IP_MULTICAST_IF` to specify
the default interface for multicast datagrams to be sent from. Otherwise,
the default interface set by the system administrator will be used (if any).

Please refer to the socket options for `IPPROTO_IP` `IP_MULTICAST_TTL` to limit
the lifetime of the packet. Multicast datagrams are sent with a default value
of 1, to prevent them to be forwarded beyond the local network. This parameter
can be set between 1 to 255.

To send a Bye message to leave a network:

@code
    soap_wsdd_Bye(soap,
      SOAP_WSDD_MANAGED,    // or SOAP_WSDD_ADHOC for ad-hoc mode
      "to address",         // "http(s):" URL, or "soap.udp:" UDP, or TCP/IP address
      soap_wsa_rand_uuid(soap), // a unique message ID
      NULL,
      "my address",         // where they can find me for WS-Discovery
      "wsdd:DiscoveryProxy",// Types: I'm a DP
      NULL,                 // Scope
      NULL,                 // MatchBy
      NULL,                 // XAddrs
      75965);               // MDVersion
@endcode

To send a Probe message (see WS-Discovery 1.1 Section 1.7) and then listen to
ProbeMatches:

@code
    struct soap soap = soap_new(); // to invoke messages
    struct soap serv = soap_new(); // for the listener and event handlers

    soap_bind(serv, port, 100);

    const char *id = soap_wsa_rand_uuid(soap);
    serv->user = (void*)&my_state;
    my_state.probe_id = id;

    soap_wsdd_Probe(soap,
      SOAP_WSDD_ADHOC,      // ad-hoc mode
      SOAP_WSDD_TO_TS,      // to a TS
      "to address",         // address of TS
      id,                   // message ID
      NULL,                 // ReplyTo
      "\"http://printer.example.org/2003/imaging\":PrintBasic",
      "ldap:///ou=engineering,o=examplecom,c=us",
      "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01/ldap");

    soap_wsdd_listen(serv, -1000);
@endcode

The `id` above is the WS-Addressing message ID that will be included in the
ProbeMatches RelatesTo WS-Addressing header. As an example, `my_state` is set to
this `id` so that when the `wsdd_event_ProbeMatches` event handler is invoked
it can find the `id` in the current state that is pointed to by `serv->user`
(`soap->user` in the handler).

To send a Resolve message and then listen to ResolveMatches:

@code
    struct soap soap = soap_new(); // to invoke messages
    struct soap serv = soap_new(); // for the listener and event handlers

    soap_bind(serv, port, 100);

    const char *id = soap_wsa_rand_uuid(soap);
    serv->user = (void*)&my_state;
    my_state.resolve_id = id;

    soap_wsdd_Resolve(soap,
      SOAP_WSDD_ADHOC,      // ad-hoc mode
      SOAP_WSDD_TO_TS,      // to a TS
      "to address",         // address to send to
      id,                   // message ID
      NULL,                 // ReplyTo
      "endpoint");          // EndpointReference of TS
     
    soap_wsdd_listen(serv, -1000);
@endcode

Again, the `id` and `my_state` are used to associate the asynchronously received
ResolveMatches response that is handled by the @ref wsdd_event_ResolveMatches
for the original request.

In managed mode with unicast messages (request-response messages), the
`soap_wsdd_Probe` and `soap_wsdd_Resolve` are sufficient to invoke without
setting up a listener. The event handlers are invoked when the unicast response
message arrives.

In managed mode, the ProbeMatches and ResolveMatches are automatically sent via
`soap_wsdd_listen` and the event `wsdd_event_Probe` and
`wsdd_event_Resolve` handlers. These event handlers should set the matches to be
returned.

In ad-hoc mode, ProbeMatches or ResolveMatches responses are NOT sent
automatically.  In ad-hoc mode the responses can be returned by adding code to
the event handler or from anywhere in the main program, for example after @ref
soap_wsdd_listen.  When responses are to be returned from the event handler or
from the main program, you should invoke @ref soap_wsdd_ProbeMatches and
@ref soap_wsdd_ResolveMatches to explicitly send unicast messages with the
match(es) back to the clients.  The WS-Addressing ReplyTo address can be used as
the return address (when not anonymous), or by using the peer's host
information that is accessible in the soap->peer and soap->peerlen members. For
example:

@code
    char host[1024], port[16];
    getnameinfo((struct sockaddr*)&soap->peer, soap->peerlen, host, sizeof(host), port, 16, NI_DGRAM | NI_NAMEREQD | NI_NUMERICSERV);
@endcode

The @ref soap_wsdd_ProbeMatches function takes an array of
@ref wsdd__ProbeMatchesType matches to transmit.  This array is created by
calling functions @ref soap_wsdd_init_ProbeMatches and then @ref
soap_wsdd_add_ProbeMatch multiple times.  Each call adds an element to the
matches:

@code
    const char *endpoint, *types, *scopes, *matchby, *xaddrs;
    unsigned int version;
    const char *relatesto, *to;
    struct wsdd__ProbeMatchesType matches;
    soap_wsdd_init_ProbeMatches(soap, &matches);
    ...
    // repeat this to add multiple matches:
    if (soap_wsdd_add_ProbeMatch(soap, &matches, endpoint, types, scopes, matchby, xaddrs, version))
      ... // out of memory
    ...
    // send the ProbeMatches message
    if (soap_wsdd_ProbeMatches(soap, endpoint, soap_wsa_rand_uuid(soap), relatesto, to, &matches))
      ... // an error occurred
@endcode

After calling @ref soap_wsdd_add_ProbeMatch it is possible to add additional
WS-Addressing header values to this matches array element.  For example the
WS-Addressing reference parameters channel instance:

@code
    if (soap_wsdd_add_ProbeMatch(soap, &matches, endpoint, types, scopes, matchby, xaddrs, version))
      ... // out of memory
    // now allocate and add the WS-Addresssing reference parameters
    struct wsa5__EndpointReference *ref = &matches.ProbeMatch[matches.__sizeProbeMatch - 1].wsa5__EndpointReference;
    ref->ReferenceParameters = (struct wsa5__ReferenceParametersType*)soap_malloc(soap, sizeof(struct wsa5__ReferenceParametersType));
    if (ref == NULL)
      ... // out of memory
    soap_default_wsa5__ReferenceParametersType(soap, ref->ReferenceParameters);
    ref->ReferenceParameters->chan__ChannelInstance = ...
@endcode

See also the [Data binding](../../databinding/html/index.html) documentation on
allocating and initializing C/C++ data in managed memory, managed by the `soap`
context.

@section wsdd_5 Generating C++ Server Objects

The WSDD plugin is developed to support C and C++. To support C++ server
objects generated with soapcpp2 option `-j` (or `-i`), run soapcpp2 again:

    soapcpp2 -a -j -Iimport import/wsdd.h

You should define in your C++ code the following wrappers (use `this` instead
of `this->soap` below with soapcpp2 option `-i`):

@code
    int wsddService::Hello(struct wsdd__HelloType *hello)
    {
      return __wsdd__Hello(this->soap, hello);
    }
    int wsddService::Bye(struct wsdd__ByeType *bye)
    {
      return __wsdd__Bye(this->soap, bye);
    }
    int wsddService::Probe(struct wsdd__ProbeType *probe)
    {
      return __wsdd__Probe(this->soap, probe);
    }
    int wsddService::ProbeMatches(struct wsdd__ProbeMatchesType *matches)
    {
      return __wsdd__ProbeMatches(this->soap, matches);
    }
    int wsddService::Resolve(struct wsdd__ResolveType *resolve)
    {
      return __wsdd__Resolve(this->soap, resolve);
    }
    int wsddService::ResolveMatches(struct wsdd__ResolveMatchesType *matches)
    {
      return __wsdd__ResolveMatches(this->soap, matches);
    }
@endcode

Note that soapcpp2 option `-a` may be needed to enable automatic service
dispatching of WS-Addressing services based on the SOAP Action value instead of
the SOAP/XML request operation.

Another approach to generate the WSDD service operations is to run soapcpp2
separately on wsdd.h (or wsdd5.h or wsdd10.h for WS-Discovery 1.0) by:

    soapcpp2 -a -L -pwsdd -Iimport import/wsdd.h
    
Now with this approach you must chain the service operations at the server side
as follows:

@code
    if (soap_begin_serve(service.soap) == SOAP_OK)
      if (service.dispatch() == SOAP_NO_METHOD)
        soap_serve_request(service.soap);
@endcode

where the `service` object is an instance of the application services generated
by soapcpp2 `-j`.

Then compile the generated wsddClient.cpp file with the macro
`-DSOAP_H_FILE=wsddH.h` to specify that `wsddH.h` should be used instead of
`soapH.h.

To combine WS-Security with WS-Discovery, please see the next section.

@section wsdd_6 Miscellaneous

You must generate client-side operations that the WSDD library expects to be
linked with, by executing:

    soapcpp2 -a -L -pwsdd -Iimport import/wsdd.h

Then compile the generated wsddClient.cpp file with the macro
`-DSOAP_H_FILE=wsddH.h` to specify that `wsddH.h` should be used instead of
`soapH.h.

If WS-Security is used with WS-Discovery, then create a file `imports.h` with
the following two lines:

@code
    // file: imports.h
    #import "wsdd.h" // or wsdd10.h, wsdd5.h
    #import "wsse.h"
@endcode

Then execute:

    soapcpp2 -a -L -pwsdd -Iimport imports.h

This generates wsddC.cpp and wsddClient.cpp, which should be compiled together
with plugin/wsddapi.c, plugin/wsseapi.c, plugin/mecevp.c, and plugin/smdevp.c.
All files should be compiled with `-DSOAP_H_FILE=wsddH.h`, i.e. macro
`SOAP_H_FILE` set to `wsddH.h`.  WS-Security requires OpenSSL and linkage with
libssl and libcrypto.

For server-side projects, also compile and link the generated wsddServer.cpp
code.  You will also need to implement the @ref wsdd_2.

Because WS-Addressing may relay faults to a FaultTo service, when implementing
a service you will also have to define a SOAP Fault service operation to accept
and handle these:

@code
    int SOAP_ENV__Fault(struct soap *soap, char *faultcode, char *faultstring, char *faultactor, struct SOAP_ENV__Detail *detail, struct SOAP_ENV__Code *SOAP_ENV__Code, struct SOAP_ENV__Reason *SOAP_ENV__Reason, char *SOAP_ENV__Node, char *SOAP_ENV__Role, struct SOAP_ENV__Detail *SOAP_ENV__Detail)
    { 
      ... = faultcode; // SOAP 1.1 fault code string (QName)
      ... = faultstring; // SOAP 1.1 fault string
      ... = faultactor; // SOAP 1.1 fault actor string
      ... = detail; // SOAP 1.1 fault detail struct
      ... = SOAP_ENV__Code; // SOAP 1.2 fault code struct
      ... = SOAP_ENV__Reason; // SOAP 1.2 reason struct
      ... = SOAP_ENV__Node; // SOAP 1.2 node string
      ... = SOAP_ENV__Role; // SOAP 1.2 role string
      ... = SOAP_ENV__Detail; // SOAP 1.2 detail struct
      return SOAP_OK;
    }
@endcode

When implementing a WS-Discovery client and/or server without any other XML Web
services, the above suffices to generate the required code.

*/

#include "wsddapi.h"

#ifdef SOAP_WSDD_2005
/* WS-Discovery 1.0 2005 */
const char *to_ts_URL = "urn:schemas-xmlsoap-org:ws:2005:04:discovery";
#else
/* WS-Discovery 1.1 2009 */
const char *to_ts_URL = "urn:docs-oasis-open-org:ws-dd:ns:discovery:2009:01";
#endif

/******************************************************************************\
 *
 *      WS-Discovery AppSequence State
 *
\******************************************************************************/

static MUTEX_TYPE soap_wsdd_state = MUTEX_INITIALIZER;

static unsigned int soap_wsdd_InstanceId = 0;
static const char *soap_wsdd_SequenceId = NULL;
static unsigned int soap_wsdd_MessageNumber = 1;

static int soap_wsdd_serve___wsdd__Hello(struct soap *soap);
static int soap_wsdd_serve___wsdd__Bye(struct soap *soap);
static int soap_wsdd_serve___wsdd__Probe(struct soap *soap);
static int soap_wsdd_serve___wsdd__ProbeMatches(struct soap *soap);
static int soap_wsdd_serve___wsdd__Resolve(struct soap *soap);
static int soap_wsdd_serve___wsdd__ResolveMatches(struct soap *soap);

static void soap_wsdd_set_AppSequence(struct soap*);
static void soap_wsdd_reset_AppSequence(struct soap*);
static void soap_wsdd_delay(struct soap*);

/******************************************************************************\
 *
 *      WS-Discovery Outbound Messages
 *
\******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/**
@fn int soap_wsdd_Hello(struct soap *soap, soap_wsdd_mode mode, const char *endpoint, const char *MessageID, const char *RelatesTo, const char *EndpointReference, const char *Types, const char *Scopes, const char *MatchBy, const char *XAddrs, unsigned int MetadataVersion)
@brief TS or DP Hello message to join a network.
@param soap context
@param[in] mode SOAP_WSDD_MANAGED or SOAP_WSDD_ADHOC
@param[in] endpoint to send Hello message to (unicast or multicast)
@param[in] MessageID unique WS-Addressing message ID (@ref soap_wsa_rand_uuid)
@param[in] RelatesTo WS-Addressing message ID
@param[in] EndpointReference of this Target Server or Discovery Proxy
@param[in] Types an unordered string of QNames or NULL, a Discovery Proxy MUST include "wsdd:DiscoveryProxy"
@param[in] Scopes an unordered set of scopes or NULL
@param[in] MatchBy NULL (unused, reserved)
@param[in] XAddrs contains the transport address(es) that MAY be used to communicate with the Target Service or Discovery Proxy
@param[in] MetadataVersion incremented by a positive value (>= 1) whenever there is a change in the metadata of the Target Service
@return SOAP_OK or error code

Hello is a one-way message sent by a Target Service to announce its
availability when it joins the network. It is also sent by a Discovery Proxy to
reduce multicast traffic on an ad hoc network.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsdd_Hello(struct soap *soap, soap_wsdd_mode mode, const char *endpoint, const char *MessageID, const char *RelatesTo, const char *EndpointReference, const char *Types, const char *Scopes, const char *MatchBy, const char *XAddrs, unsigned int MetadataVersion)
{
  struct wsdd__HelloType req;
  struct wsdd__ScopesType scopes;
  const char *Action = SOAP_NAMESPACE_OF_wsdd"/Hello";
  const char *To = endpoint;

  /* SOAP Header */
  if (mode == SOAP_WSDD_ADHOC)
    To = to_ts_URL;
  soap_wsa_request(soap, MessageID, To, Action);
  soap_wsa_add_RelatesTo(soap, RelatesTo);
#ifdef SOAP_WSDD_2005
  soap_wsdd_set_AppSequence(soap);
#else
  if (mode == SOAP_WSDD_ADHOC)
    soap_wsdd_set_AppSequence(soap);
  else
    soap_wsdd_reset_AppSequence(soap);
#endif

  /* Hello */
  soap_default_wsdd__HelloType(soap, &req);
#ifdef SOAP_WSA_2005
  req.wsa5__EndpointReference.Address = (char*)EndpointReference;
#else
  req.wsa__EndpointReference.Address = (char*)EndpointReference;
#endif
  req.Types = (char*)Types;
  if (Scopes)
  {
    soap_default_wsdd__ScopesType(soap, &scopes);
    scopes.__item = (char*)Scopes;
    scopes.MatchBy = (char*)MatchBy;
    req.Scopes = &scopes;
  }
  req.XAddrs = (char*)XAddrs;
  req.MetadataVersion = MetadataVersion;
  if (soap_send___wsdd__Hello(soap, endpoint, Action, &req))
    return soap->error;

  /* HTTP(S) POST expects an HTTP OK response */
  if (endpoint && !strncmp(endpoint, "http", 4))
    return soap_recv_empty_response(soap);
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsdd_Bye(struct soap *soap, soap_wsdd_mode mode, const char *endpoint, const char *MessageID, const char *EndpointReference, const char *Types, const char *Scopes, const char *MatchBy, const char *XAddrs, unsigned int MetadataVersion)
@brief TS or DP Bye message to leave a network.
@param soap context
@param[in] mode SOAP_WSDD_MANAGED or SOAP_WSDD_ADHOC
@param[in] endpoint to send Bye message to (unicast or multicast)
@param[in] MessageID unique WS-Addressing message ID (@ref soap_wsa_rand_uuid)
@param[in] EndpointReference of this Target Server or Discovery Proxy
@param[in] Types an unordered string of QNames or NULL, a Discovery Proxy MUST include "wsdd:DiscoveryProxy"
@param[in] Scopes an unordered set of scopes or NULL
@param[in] MatchBy NULL (unused, reserved)
@param[in] XAddrs contains the transport address(es) that MAY be used to communicate with the Target Service or Discovery Proxy
@param[in] MetadataVersion incremented by a positive value (>= 1) whenever there is a change in the metadata of the Target Service
@return SOAP_OK or error code

Bye is a one-way message sent by a Target Service to announce its
unavailability as a best effort when it leaves the network.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsdd_Bye(struct soap *soap, soap_wsdd_mode mode, const char *endpoint, const char *MessageID, const char *EndpointReference, const char *Types, const char *Scopes, const char *MatchBy, const char *XAddrs, unsigned int MetadataVersion)
{
  struct wsdd__ByeType req;
  struct wsdd__ScopesType scopes;
  const char *Action = SOAP_NAMESPACE_OF_wsdd"/Bye";
  const char *To = endpoint;

  /* SOAP Header */
  if (mode == SOAP_WSDD_ADHOC)
    To = to_ts_URL;
  soap_wsa_request(soap, MessageID, To, Action);
#ifdef SOAP_WSDD_2005
  soap_wsdd_set_AppSequence(soap);
#else
  if (mode == SOAP_WSDD_ADHOC)
    soap_wsdd_set_AppSequence(soap);
  else
    soap_wsdd_reset_AppSequence(soap);
#endif

  /* Bye */
  soap_default_wsdd__ByeType(soap, &req);
#ifdef SOAP_WSA_2005
  req.wsa5__EndpointReference.Address = (char*)EndpointReference;
#else
  req.wsa__EndpointReference.Address = (char*)EndpointReference;
#endif
  req.Types = (char*)Types;
  if (Scopes)
  {
    soap_default_wsdd__ScopesType(soap, &scopes);
    scopes.__item = (char*)Scopes;
    scopes.MatchBy = (char*)MatchBy;
    req.Scopes = &scopes;
  }
  req.XAddrs = (char*)XAddrs;
  req.MetadataVersion = &MetadataVersion;
  if (soap_send___wsdd__Bye(soap, endpoint, Action, &req))
    return soap->error;

  /* HTTP(S) POST expects an HTTP OK response */
  if (endpoint && !strncmp(endpoint, "http", 4))
    return soap_recv_empty_response(soap);
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsdd_Probe(struct soap *soap, soap_wsdd_mode mode, soap_wsdd_to to, const char *endpoint, const char *MessageID, const char *ReplyTo, const char *Types, const char *Scopes, const char *MatchBy)
@brief Client Probe message to a TS or DP.
@param soap context
@param[in] mode SOAP_WSDD_MANAGED or SOAP_WSDD_ADHOC
@param[in] to SOAP_WSDD_TO_TS or SOAP_WSDD_TO_DP
@param[in] endpoint to send Probe to (unicast or multicast)
@param[in] MessageID WS-Addressing message ID of the message
@param[in] ReplyTo WS-Addressing ReplyTo message ID of the message
@param[in] Types an unordered string of QNames to probe
@param[in] Scopes an unordered set of scopes to probe
@param[in] MatchBy matching rule to apply for this probe
@return SOAP_OK or error code

A Client sends a probe to find Target Services by the Type of the Target
Service, a Scope in which the Target Service resides, both, or simply all
Target Services. The matches will be delivered to @ref wsdd_event_ProbeMatches
when @ref soap_wsdd_listen receives a ProbeMatch response. The RelatesTo of the
ProbeMatches is the MessageID of the Probe.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsdd_Probe(struct soap *soap, soap_wsdd_mode mode, soap_wsdd_to to, const char *endpoint, const char *MessageID, const char *ReplyTo, const char *Types, const char *Scopes, const char *MatchBy)
{
  unsigned int InstanceId = 0;
  const char *SequenceId = NULL;
  unsigned int MessageNumber = 0;
  struct wsdd__ProbeType req;
  struct __wsdd__ProbeMatches res;
  struct wsdd__ScopesType scopes;
  const char *Action = SOAP_NAMESPACE_OF_wsdd"/Probe";
  const char *To = endpoint;

  /* SOAP Header */
  if (to == SOAP_WSDD_TO_TS)
    To = to_ts_URL;
  soap_wsa_request(soap, MessageID, To, Action);
  soap_wsa_add_ReplyTo(soap, ReplyTo);
  soap_wsdd_reset_AppSequence(soap);

  /* Probe */
  soap_default_wsdd__ProbeType(soap, &req);
  req.Types = (char*)Types;
  if (Scopes)
  {
    soap_default_wsdd__ScopesType(soap, &scopes);
    scopes.__item = (char*)Scopes;
    scopes.MatchBy = (char*)MatchBy;
    req.Scopes = &scopes;
  }
  if (soap_send___wsdd__Probe(soap, endpoint, Action, &req))
    return soap->error;

  /* ad-hoc mode: we're done */
  if (mode == SOAP_WSDD_ADHOC)
  {
    /* HTTP(S) POST expects an HTTP OK response */
    if (endpoint && !strncmp(endpoint, "http", 4))
      return soap_recv_empty_response(soap);
  }
  else
  { 
    /* managed mode: receive the matches */
    if (soap_recv___wsdd__ProbeMatches(soap, &res))
      return soap->error;

    /* check Header WSDD */
    if (!soap->header)
      return soap_wsa_sender_fault(soap, "WSDD ProbeMatches incomplete", NULL);

    if (soap->header->wsdd__AppSequence)
    {
      wsdd__AppSequenceType *seq = soap->header->wsdd__AppSequence;
      InstanceId = seq->InstanceId;
      SequenceId = seq->SequenceId;
      MessageNumber = seq->MessageNumber;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "AppSeq id=%u seq=%s msg=%u\n", InstanceId, SequenceId ? SequenceId : "(null)", MessageNumber));
    }

    /* pass probe matches on to user-defined event handler */
    wsdd_event_ProbeMatches(soap,
      InstanceId,
      SequenceId,
      MessageNumber, 
#ifdef SOAP_WSA_2005
      soap->header->wsa5__MessageID,
      soap->header->wsa5__RelatesTo ? soap->header->wsa5__RelatesTo->__item : NULL,
#else
      soap->header->wsa__MessageID,
      soap->header->wsa__RelatesTo ? soap->header->wsa__RelatesTo->__item : NULL,
#endif
      res.wsdd__ProbeMatches);
  }
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsdd_Resolve(struct soap *soap, soap_wsdd_mode mode, soap_wsdd_to to, const char *endpoint, const char *MessageID, const char *ReplyTo, const char *EndpointReference)
@brief Client Resolve message to a TS or DP.
@param soap context
@param[in] mode SOAP_WSDD_MANAGED or SOAP_WSDD_ADHOC
@param[in] to SOAP_WSDD_TO_TS or SOAP_WSDD_TO_DP
@param[in] endpoint to send Resolve to (unicast or multicast)
@param[in] MessageID WS-Addressing message ID of the message
@param[in] ReplyTo WS-Addressing ReplyTo message ID of the message
@param[in] EndpointReference of the Target Service or Discovery Proxy
@return SOAP_OK or error code

A Client sends a resolve to locate a Target Service, i.e., to retrieve its
transport address(es). This server-side event handler returns the match(es).
The matches will be delivered to @ref wsdd_event_ResolveMatches when @ref
soap_wsdd_listen receives a ResolveMatch response. The RelatesTo of the
ResolveMatches is the MessageID of the Resolve.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsdd_Resolve(struct soap *soap, soap_wsdd_mode mode, soap_wsdd_to to, const char *endpoint, const char *MessageID, const char *ReplyTo, const char *EndpointReference)
{
  unsigned int InstanceId = 0;
  const char *SequenceId = NULL;
  unsigned int MessageNumber = 0;
  struct wsdd__ResolveType req;
  struct __wsdd__ResolveMatches res;
  const char *Action = SOAP_NAMESPACE_OF_wsdd"/Resolve";
  const char *To = endpoint;

  /* SOAP Header */
  if (to == SOAP_WSDD_TO_TS)
    To = to_ts_URL;
  soap_wsa_request(soap, MessageID, To, Action);
  soap_wsa_add_ReplyTo(soap, ReplyTo);
  soap_wsdd_reset_AppSequence(soap);

  /* Resolve */
  soap_default_wsdd__ResolveType(soap, &req);
#ifdef SOAP_WSA_2005
  req.wsa5__EndpointReference.Address = (char*)EndpointReference;
#else
  req.wsa__EndpointReference.Address = (char*)EndpointReference;
#endif
  if (soap_send___wsdd__Resolve(soap, endpoint, Action, &req))
    return soap->error;

  /* ad-hoc mode: we're done */
  if (mode == SOAP_WSDD_ADHOC)
  {
    /* HTTP(S) POST expects an HTTP OK response */
    if (endpoint && !strncmp(endpoint, "http", 4))
      return soap_recv_empty_response(soap);
  }
  else
  { 
    /* managed mode: receive the matches */
    if (soap_recv___wsdd__ResolveMatches(soap, &res))
      return soap->error;

    /* check Header WSDD */
    if (!soap->header)
      return soap_wsa_sender_fault(soap, "WSDD ProbeMatches incomplete", NULL);

    if (soap->header->wsdd__AppSequence)
    {
      wsdd__AppSequenceType *seq = soap->header->wsdd__AppSequence;
      InstanceId = seq->InstanceId;
      SequenceId = seq->SequenceId;
      MessageNumber = seq->MessageNumber;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "AppSeq id=%u seq=%s msg=%u\n", InstanceId, SequenceId ? SequenceId : "(null)", MessageNumber));
    }

    /* pass probe matches on to user-defined event handler */
    if (res.wsdd__ResolveMatches && res.wsdd__ResolveMatches->ResolveMatch)
      wsdd_event_ResolveMatches(soap,
        InstanceId,
        SequenceId,
        MessageNumber, 
#ifdef SOAP_WSA_2005
        soap->header->wsa5__MessageID,
        soap->header->wsa5__RelatesTo ? soap->header->wsa5__RelatesTo->__item : NULL,
#else
        soap->header->wsa__MessageID,
        soap->header->wsa__RelatesTo ? soap->header->wsa__RelatesTo->__item : NULL,
#endif
        res.wsdd__ResolveMatches->ResolveMatch);
  }
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn void soap_wsdd_init_ProbeMatches(struct soap *soap, struct wsdd__ProbeMatchesType *matches)
@brief Initalize the probe matches container.
@param soap context
@param matches the container with matches to initialize
*/
SOAP_FMAC1
void
SOAP_FMAC2
soap_wsdd_init_ProbeMatches(struct soap *soap, struct wsdd__ProbeMatchesType *matches)
{
  soap_default_wsdd__ProbeMatchesType(soap, matches);
}

/******************************************************************************/

/**
@fn int soap_wsdd_add_ProbeMatch(struct soap *soap, struct wsdd__ProbeMatchesType *matches, const char *EndpointReference, const char *Types, const char *Scopes, const char *MatchBy, const char *XAddrs, unsigned int MetadataVersion)
@brief Add a match to the probe matches container.
@param soap context
@param matches the container to update with a new match
@param[in] EndpointReference of this Target Server or Discovery Proxy
@param[in] Types an unordered string of QNames or NULL, a Discovery Proxy MUST include "wsdd:DiscoveryProxy"
@param[in] Scopes an unordered set of scopes or NULL
@param[in] MatchBy NULL (unused, reserved)
@param[in] XAddrs contains the transport address(es) that MAY be used to communicate with the Target Service or Discovery Proxy
@param[in] MetadataVersion incremented by a positive value (>= 1) whenever there is a change in the metadata of the Target Service
@return SOAP_OK or error code

To populate a Probe matches container, first initialize with @ref
soap_wsdd_init_ProbeMatches, then use this function to add each match. The
container is deallocated with soap_end(soap) and can be initialized again
(without leaks).
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsdd_add_ProbeMatch(struct soap *soap, struct wsdd__ProbeMatchesType *matches, const char *EndpointReference, const char *Types, const char *Scopes, const char *MatchBy, const char *XAddrs, unsigned int MetadataVersion)
{
  int n = 0, k = matches->__sizeProbeMatch;
  struct wsdd__ProbeMatchType *m;

  /* need to increase space? */
  if (k == 0)
    n = 4;
  else if (k >= 4 && (k & (k - 1)) == 0)
    n = 2 * k;
  /* yes we do */
  if (n)
  {
    struct wsdd__ProbeMatchType *tmp = (struct wsdd__ProbeMatchType*)soap_malloc(soap, n * sizeof(struct wsdd__ProbeMatchType));
    int i;

    if (!tmp)
      return soap->error = SOAP_EOM;
    for (i = 0; i < k; i++)
      tmp[i] = matches->ProbeMatch[i];
    matches->ProbeMatch = tmp;
  }

  /* add entry */
  m = &matches->ProbeMatch[k];
  soap_default_wsdd__ProbeMatchType(soap, m);
#ifdef SOAP_WSA_2005
  m->wsa5__EndpointReference.Address = (char*)EndpointReference;
#else
  m->wsa__EndpointReference.Address = (char*)EndpointReference;
#endif
  m->Types = (char*)Types;
  if (Scopes)
  {
    m->Scopes = (struct wsdd__ScopesType*)soap_malloc(soap, sizeof(struct wsdd__ScopesType));
    soap_default_wsdd__ScopesType(soap, m->Scopes);
    m->Scopes->__item = (char*)Scopes;
    m->Scopes->MatchBy = (char*)MatchBy;
  }
  m->XAddrs = (char*)XAddrs;
  m->MetadataVersion = MetadataVersion;
  matches->__sizeProbeMatch++;
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsdd_ProbeMatches(struct soap *soap, const char *endpoint, const char *MessageID, const char *RelatesTo, const char *To, struct wsdd__ProbeMatchesType *matches)
@brief TS or DP ProbeMatches message.
@param soap context
@param[in] endpoint to send Probe to (unicast or multicast)
@param[in] MessageID WS-Addressing message ID of the message
@param[in] RelatesTo WS-Addressing RelatesTo message ID of the message
@param[in] To WS-Addressing endpoint IP and port or NULL for anonymous
@param[in] matches contains the probe matches, use @ref soap_wsdd_add_ProbeMatch to populate this container
@return SOAP_OK or error code

A Client sends a probe to find Target Services by the Type of the Target
Service, a Scope in which the Target Service resides, both, or simply all
Target Services. The Target Server(s) or Discovery Proxy responds by sending
ProbeMatches to the Client.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsdd_ProbeMatches(struct soap *soap, const char *endpoint, const char *MessageID, const char *RelatesTo, const char *To, struct wsdd__ProbeMatchesType *matches)
{
  const char *Action = SOAP_NAMESPACE_OF_wsdd"/ProbeMatches";

  /* 0..APP_MAX_DELAY ms delay */
  soap_wsdd_delay(soap);

  /* SOAP Header */
  soap_wsa_request(soap, MessageID, To, Action);
  soap_wsa_add_RelatesTo(soap, RelatesTo);
  soap_wsdd_set_AppSequence(soap);

  /* ProbeMatches */
  if (soap_send___wsdd__ProbeMatches(soap, endpoint, Action, matches))
    return soap->error;

  /* HTTP(S) POST expects an HTTP OK response */
  if (endpoint && !strncmp(endpoint, "http", 4))
    return soap_recv_empty_response(soap);
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsdd_ResolveMatches(struct soap *soap, const char *endpoint, const char *MessageID, const char *RelatesTo, const char *To, const char *EndpointReference, const char *Types, const char *Scopes, const char *MatchBy, const char *XAddrs, unsigned int MetadataVersion)
@brief TS or DP ResolveMatches message.
@param soap context
@param[in] endpoint to send Probe to (unicast or multicast)
@param[in] MessageID WS-Addressing message ID of the message
@param[in] RelatesTo WS-Addressing RelatesTo message ID of the message
@param[in] To WS-Addressing endpoint IP and port or NULL for anonymous
@param[in] EndpointReference of the Target Service
@param[in] Types an unordered string of QNames or NULL, a Discovery Proxy MUST include "wsdd:DiscoveryProxy"
@param[in] Scopes an unordered set of scopes or NULL
@param[in] MatchBy NULL (unused, reserved)
@param[in] XAddrs contains the transport address(es) that MAY be used to communicate with the Target Service or Discovery Proxy
@param[in] MetadataVersion incremented by a positive value (>= 1) whenever there is a change in the metadata of the Target Service
@return SOAP_OK or error code

A Client sends a resolve to locate a Target Service, i.e., to retrieve its
transport address(es). The Target Server(s) or Discovery Proxy responds by
sending ResolveMatches to the Client.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsdd_ResolveMatches(struct soap *soap, const char *endpoint, const char *MessageID, const char *RelatesTo, const char *To, const char *EndpointReference, const char *Types, const char *Scopes, const char *MatchBy, const char *XAddrs, unsigned int MetadataVersion)
{
  struct wsdd__ResolveMatchesType res;
  struct wsdd__ResolveMatchType match;
  struct wsdd__ScopesType scopes;
  const char *Action = SOAP_NAMESPACE_OF_wsdd"/ResolveMatches";

  /* 0..APP_MAX_DELAY ms delay */
  soap_wsdd_delay(soap);

  /* SOAP Header */
  soap_wsa_request(soap, MessageID, To, Action);
  soap_wsa_add_RelatesTo(soap, RelatesTo);
  soap_wsdd_set_AppSequence(soap);

  /* ResolveMatches */
  soap_default_wsdd__ResolveMatchType(soap, &match);
#ifdef SOAP_WSA_2005
  match.wsa5__EndpointReference.Address = (char*)EndpointReference;
#else
  match.wsa__EndpointReference.Address = (char*)EndpointReference;
#endif
  match.Types = (char*)Types;
  if (Scopes)
  {
    soap_default_wsdd__ScopesType(soap, &scopes);
    scopes.__item = (char*)Scopes;
    scopes.MatchBy = (char*)MatchBy;
    match.Scopes = &scopes;
  }
  match.XAddrs = (char*)XAddrs;
  match.MetadataVersion = MetadataVersion;
  soap_default_wsdd__ResolveMatchesType(soap, &res);
  res.ResolveMatch = &match;
  if (soap_send___wsdd__ResolveMatches(soap, endpoint, Action, &res))
    return soap->error;

  /* HTTP(S) POST expects an HTTP OK response */
  if (endpoint && !strncmp(endpoint, "http", 4))
    return soap_recv_empty_response(soap);
  return SOAP_OK;
}

/******************************************************************************\
 *
 *      WS-Discovery Serving Inbound Messages
 *
\******************************************************************************/

/**
@fn int soap_wsdd_listen(struct soap *soap, int timeout)
@brief Listen on a port for inbound WS-Discovery messages, also cleans up memory with `soap_destroy` and `soap_end`.
@param soap context
@param[in] timeout seconds to listen (use negative value for micro seconds)
@return SOAP_OK or error code

First you need to open a port with soap_bind(soap). Then invoke this function
to listen to WS-Discovery messages on that port. Inbound messages are accepted
and the information is relayed to the event handlers, see Section @ref wsdd_2.
Use soap->user to pass a pointer to a state object that the event hanlders can
use. The timeout allows a Client to periodically poll the port for messages. A
value of zero will loop the listener forever or until an error occurs.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsdd_listen(struct soap *soap, int timeout)
{
  soap->accept_timeout = soap->recv_timeout = soap->send_timeout = timeout;

  /* event-serve loop (exits upon timeout) */
  for (;;)
  {
    if (!soap_valid_socket(soap_accept(soap))
     || soap_begin_serve(soap))
    {
      /* timeout? */
      if (!soap->errnum)
        break;
      return soap->error;
    }

    /* always close HTTP afterwards */
    soap->keep_alive = 0;

    /* receive event and serve */
    if (soap_wsdd_serve_request(soap))
    {
      /* timeout? */
      if (!soap->errnum)
	break;
#ifdef WITH_FASTCGI
      soap_send_fault(soap);
#else
      return soap_send_fault(soap);
#endif
    }
    /* clean up for next iteration */
    soap_destroy(soap);
    soap_end(soap);
  } 

  /* timed out */
  soap_destroy(soap);
  soap_end(soap);
  return soap->error = SOAP_OK;
}

/******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
soap_wsdd_serve_request(struct soap *soap)
{
  soap_peek_element(soap);
  if (!soap_match_tag(soap, soap->tag, "wsdd:Hello"))
    return soap_wsdd_serve___wsdd__Hello(soap);
  if (!soap_match_tag(soap, soap->tag, "wsdd:Bye"))
    return soap_wsdd_serve___wsdd__Bye(soap);
  if (!soap_match_tag(soap, soap->tag, "wsdd:Probe"))
    return soap_wsdd_serve___wsdd__Probe(soap);
  if (!soap_match_tag(soap, soap->tag, "wsdd:ProbeMatches"))
    return soap_wsdd_serve___wsdd__ProbeMatches(soap);
  if (!soap_match_tag(soap, soap->tag, "wsdd:Resolve"))
    return soap_wsdd_serve___wsdd__Resolve(soap);
  if (!soap_match_tag(soap, soap->tag, "wsdd:ResolveMatches"))
    return soap_wsdd_serve___wsdd__ResolveMatches(soap);
  soap->error = SOAP_NO_METHOD;
  /* close socket unless soap->keep_alive == 1 */
  return soap_closesock(soap);
}

/******************************************************************************/

#ifdef __cplusplus
}
#endif

/******************************************************************************/

static int
soap_wsdd_serve___wsdd__Hello(struct soap *soap)
{
  struct __wsdd__Hello soap_tmp___wsdd__Hello;
  soap_default___wsdd__Hello(soap, &soap_tmp___wsdd__Hello);
  soap->encodingStyle = NULL;
  if (!soap_get___wsdd__Hello(soap, &soap_tmp___wsdd__Hello, "-wsdd:Hello", NULL))
    return soap->error;
  if (soap_body_end_in(soap)
   || soap_envelope_end_in(soap)
   || soap_end_recv(soap))
    return soap->error;
  soap->error = __wsdd__Hello(soap, soap_tmp___wsdd__Hello.wsdd__Hello);
  if (soap->error)
    return soap->error;
  return soap_closesock(soap);
}

/******************************************************************************/

static int
soap_wsdd_serve___wsdd__Bye(struct soap *soap)
{
  struct __wsdd__Bye soap_tmp___wsdd__Bye;
  soap_default___wsdd__Bye(soap, &soap_tmp___wsdd__Bye);
  soap->encodingStyle = NULL;
  if (!soap_get___wsdd__Bye(soap, &soap_tmp___wsdd__Bye, "-wsdd:Bye", NULL))
    return soap->error;
  if (soap_body_end_in(soap)
   || soap_envelope_end_in(soap)
   || soap_end_recv(soap))
    return soap->error;
  soap->error = __wsdd__Bye(soap, soap_tmp___wsdd__Bye.wsdd__Bye);
  if (soap->error)
    return soap->error;
  return soap_closesock(soap);
}

/******************************************************************************/

static int
soap_wsdd_serve___wsdd__Probe(struct soap *soap)
{
  struct __wsdd__Probe soap_tmp___wsdd__Probe;
  soap_default___wsdd__Probe(soap, &soap_tmp___wsdd__Probe);
  soap->encodingStyle = NULL;
  if (!soap_get___wsdd__Probe(soap, &soap_tmp___wsdd__Probe, "-wsdd:Probe", NULL))
    return soap->error;
  if (soap_body_end_in(soap)
   || soap_envelope_end_in(soap)
   || soap_end_recv(soap))
    return soap->error;
  soap->error = __wsdd__Probe(soap, soap_tmp___wsdd__Probe.wsdd__Probe);
  if (soap->error)
    return soap->error;
  return soap_closesock(soap);
}

/******************************************************************************/

static int
soap_wsdd_serve___wsdd__ProbeMatches(struct soap *soap)
{
  struct __wsdd__ProbeMatches soap_tmp___wsdd__ProbeMatches;
  soap_default___wsdd__ProbeMatches(soap, &soap_tmp___wsdd__ProbeMatches);
  soap->encodingStyle = NULL;
  if (!soap_get___wsdd__ProbeMatches(soap, &soap_tmp___wsdd__ProbeMatches, "-wsdd:ProbeMatches", NULL))
    return soap->error;
  if (soap_body_end_in(soap)
   || soap_envelope_end_in(soap)
   || soap_end_recv(soap))
    return soap->error;
  soap->error = __wsdd__ProbeMatches(soap, soap_tmp___wsdd__ProbeMatches.wsdd__ProbeMatches);
  if (soap->error)
    return soap->error;
  return soap_closesock(soap);
}

/******************************************************************************/

static int
soap_wsdd_serve___wsdd__Resolve(struct soap *soap)
{
  struct __wsdd__Resolve soap_tmp___wsdd__Resolve;
  soap_default___wsdd__Resolve(soap, &soap_tmp___wsdd__Resolve);
  soap->encodingStyle = NULL;
  if (!soap_get___wsdd__Resolve(soap, &soap_tmp___wsdd__Resolve, "-wsdd:Resolve", NULL))
    return soap->error;
  if (soap_body_end_in(soap)
   || soap_envelope_end_in(soap)
   || soap_end_recv(soap))
    return soap->error;
  soap->error = __wsdd__Resolve(soap, soap_tmp___wsdd__Resolve.wsdd__Resolve);
  if (soap->error)
    return soap->error;
  return soap_closesock(soap);
}

/******************************************************************************/

static int
soap_wsdd_serve___wsdd__ResolveMatches(struct soap *soap)
{
  struct __wsdd__ResolveMatches soap_tmp___wsdd__ResolveMatches;
  soap_default___wsdd__ResolveMatches(soap, &soap_tmp___wsdd__ResolveMatches);
  soap->encodingStyle = NULL;
  if (!soap_get___wsdd__ResolveMatches(soap, &soap_tmp___wsdd__ResolveMatches, "-wsdd:ResolveMatches", NULL))
    return soap->error;
  if (soap_body_end_in(soap)
   || soap_envelope_end_in(soap)
   || soap_end_recv(soap))
    return soap->error;
  soap->error = __wsdd__ResolveMatches(soap, soap_tmp___wsdd__ResolveMatches.wsdd__ResolveMatches);
  if (soap->error)
    return soap->error;
  return soap_closesock(soap);
}

/******************************************************************************/

/**
@fn int __wsdd__Hello(struct soap *soap, struct wsdd__HelloType *Hello)
@brief Internal WS-Discovery service operation.
*/
SOAP_FMAC5
int
SOAP_FMAC6
__wsdd__Hello(struct soap *soap, struct wsdd__HelloType *Hello)
{
  unsigned int InstanceId = 0;
  const char *SequenceId = NULL;
  unsigned int MessageNumber = 0;

  DBGFUN("__wsdd__Hello");

  /* check for WSA */
  if (soap_wsa_check(soap))
    return soap->error;

  /* check WSDD */
  if (!Hello
#ifdef SOAP_WSA_2005
   || !Hello->wsa5__EndpointReference.Address
#else
   || !Hello->wsa__EndpointReference.Address
#endif
    )
    return soap_wsa_sender_fault(soap, "WSDD Hello incomplete", NULL);

#ifdef SOAP_WSA_2005
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EndpointReference=%s\n", Hello->wsa5__EndpointReference.Address));
#else
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EndpointReference=%s\n", Hello->wsa__EndpointReference.Address));
#endif

  if (soap->header->wsdd__AppSequence)
  {
    wsdd__AppSequenceType *seq = soap->header->wsdd__AppSequence;
    InstanceId = seq->InstanceId;
    SequenceId = seq->SequenceId;
    MessageNumber = seq->MessageNumber;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "AppSeq id=%u seq=%s msg=%u\n", InstanceId, SequenceId ? SequenceId : "(null)", MessageNumber));
  }

  /* pass on to user-defined event handler */
  wsdd_event_Hello(soap,
    InstanceId,
    SequenceId,
    MessageNumber, 
#ifdef SOAP_WSA_2005
    soap->header->wsa5__MessageID,
    soap->header->wsa5__RelatesTo ? soap->header->wsa5__RelatesTo->__item : NULL,
    Hello->wsa5__EndpointReference.Address,
#else
    soap->header->wsa__MessageID,
    soap->header->wsa__RelatesTo ? soap->header->wsa__RelatesTo->__item : NULL,
    Hello->wsa__EndpointReference.Address,
#endif
    Hello->Types,
    Hello->Scopes ? Hello->Scopes->__item : NULL,
    Hello->Scopes ? Hello->Scopes->MatchBy : NULL,
    Hello->XAddrs,
    Hello->MetadataVersion);

  /* one-way HTTP(S) POST message requires an HTTP OK response message */
  if (!strncmp(soap->endpoint, "http", 4))
    return soap_send_empty_response(soap, SOAP_OK);
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int __wsdd__Bye(struct soap *soap, struct wsdd__ByeType *Bye)
@brief Internal WS-Discovery service operation.
*/
SOAP_FMAC5
int
SOAP_FMAC6
__wsdd__Bye(struct soap *soap, struct wsdd__ByeType *Bye)
{
  unsigned int InstanceId = 0;
  const char *SequenceId = NULL;
  unsigned int MessageNumber = 0;

  DBGFUN("__wsdd__Bye");

  /* check for WSA */
  if (soap_wsa_check(soap))
    return soap->error;

  /* check WSDD */
  if (!Bye
#ifdef SOAP_WSA_2005
   || !Bye->wsa5__EndpointReference.Address
#else
   || !Bye->wsa__EndpointReference.Address
#endif
    )
    return soap_wsa_sender_fault(soap, "WSDD Bye incomplete", NULL);

#ifdef SOAP_WSA_2005
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EndpointReference=%s\n", Bye->wsa5__EndpointReference.Address));
#else
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EndpointReference=%s\n", Bye->wsa__EndpointReference.Address));
#endif

  if (soap->header->wsdd__AppSequence)
  {
    wsdd__AppSequenceType *seq = soap->header->wsdd__AppSequence;
    InstanceId = seq->InstanceId;
    SequenceId = seq->SequenceId;
    MessageNumber = seq->MessageNumber;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "AppSeq id=%u seq=%s msg=%u\n", InstanceId, SequenceId ? SequenceId : "(null)", MessageNumber));
  }

  /* pass on to user-defined event handler */
  wsdd_event_Bye(soap,
    InstanceId,
    SequenceId,
    MessageNumber, 
#ifdef SOAP_WSA_2005
    soap->header->wsa5__MessageID,
    soap->header->wsa5__RelatesTo ? soap->header->wsa5__RelatesTo->__item : NULL,
    Bye->wsa5__EndpointReference.Address,
#else
    soap->header->wsa__MessageID,
    soap->header->wsa__RelatesTo ? soap->header->wsa__RelatesTo->__item : NULL,
    Bye->wsa__EndpointReference.Address,
#endif
    Bye->Types,
    Bye->Scopes ? Bye->Scopes->__item : NULL,
    Bye->Scopes ? Bye->Scopes->MatchBy : NULL,
    Bye->XAddrs,
    Bye->MetadataVersion);

  /* one-way HTTP(S) POST message requires an HTTP OK response message */
  if (!strncmp(soap->endpoint, "http", 4))
    return soap_send_empty_response(soap, SOAP_OK);
  return SOAP_OK;
}

/******************************************************************************/

static int
soap_wsdd_http(struct soap *soap, const char *endpoint, const char *host, int port, const char *path, const char *action, ULONG64 count)
{
  (void)endpoint; (void)host; (void)port; (void)path; (void)action;
  return soap->fresponse(soap, SOAP_OK, count);
}

/******************************************************************************/

/**
@fn int __wsdd__Probe(struct soap *soap, struct wsdd__ProbeType *Probe)
@brief Internal WS-Discovery service operation.
*/
SOAP_FMAC5
int
SOAP_FMAC6
__wsdd__Probe(struct soap *soap, struct wsdd__ProbeType *Probe)
{
  struct wsdd__ProbeMatchesType ProbeMatches;
  soap_wsdd_mode mode;

  DBGFUN("__wsdd__Probe");

  /* check for WSA */
  if (soap_wsa_check(soap))
    return soap->error;

  /* check WSDD */
  if (!Probe)
    return soap_wsa_sender_fault(soap, "WSDD Probe incomplete", NULL);

  soap_default_wsdd__ProbeMatchesType(soap, &ProbeMatches);

  /* pass probe request on to user-defined event handler to get matches */
  mode = wsdd_event_Probe(soap,
#ifdef SOAP_WSA_2005
    soap->header->wsa5__MessageID,
    soap->header->wsa5__ReplyTo ? soap->header->wsa5__ReplyTo->Address : NULL,
#else
    soap->header->wsa__MessageID,
    soap->header->wsa__ReplyTo ? soap->header->wsa__ReplyTo->Address : NULL,
#endif
    Probe->Types,
    Probe->Scopes ? Probe->Scopes->__item : NULL,
    Probe->Scopes ? Probe->Scopes->MatchBy : NULL,
    &ProbeMatches);

  if (mode == SOAP_WSDD_ADHOC)
  {
    /* event handler or main application MUST return matches independently */

    /* one-way HTTP(S) POST message requires an HTTP OK response message */
    if (!strncmp(soap->endpoint, "http", 4))
      return soap_send_empty_response(soap, SOAP_OK);
  }
  else
  {
    int err;
    int (*fpost)(struct soap*, const char*, const char*, int, const char*, const char*, ULONG64);
    const char *MessageID = soap_wsa_rand_uuid(soap);
    const char *Action = SOAP_NAMESPACE_OF_wsdd"/ProbeMatches";

    /* SOAP Header */
    soap_wsdd_set_AppSequence(soap);
    soap_wsa_reply(soap, MessageID, Action);

    /* send over current socket as HTTP OK response: */
    fpost = soap->fpost;
    soap->fpost = soap_wsdd_http;
    err = soap_send___wsdd__ProbeMatches(soap, "http://", Action, &ProbeMatches);
    soap->fpost = fpost;
    return err;
  }
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int __wsdd__ProbeMatches(struct soap *soap, struct wsdd__ProbeMatchesType *ProbeMatches)
@brief Internal WS-Discovery service operation.
*/
SOAP_FMAC5
int
SOAP_FMAC6
__wsdd__ProbeMatches(struct soap *soap, struct wsdd__ProbeMatchesType *ProbeMatches)
{
  unsigned int InstanceId = 0;
  const char *SequenceId = NULL;
  unsigned int MessageNumber = 0;

  DBGFUN("__wsdd__ProbeMatches");

  /* check for WSA */
  if (soap_wsa_check(soap))
    return soap->error;

  /* check WSDD */
  if (!ProbeMatches)
    return soap_wsa_sender_fault(soap, "WSDD ProbeMatches incomplete", NULL);

  if (soap->header->wsdd__AppSequence)
  {
    wsdd__AppSequenceType *seq = soap->header->wsdd__AppSequence;
    InstanceId = seq->InstanceId;
    SequenceId = seq->SequenceId;
    MessageNumber = seq->MessageNumber;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "AppSeq id=%u seq=%s msg=%u\n", InstanceId, SequenceId ? SequenceId : "(null)", MessageNumber));
  }

  /* pass probe matches on to user-defined event handler */
  wsdd_event_ProbeMatches(soap,
    InstanceId,
    SequenceId,
    MessageNumber, 
#ifdef SOAP_WSA_2005
    soap->header->wsa5__MessageID,
    soap->header->wsa5__RelatesTo ? soap->header->wsa5__RelatesTo->__item : NULL,
#else
    soap->header->wsa__MessageID,
    soap->header->wsa__RelatesTo ? soap->header->wsa__RelatesTo->__item : NULL,
#endif
    ProbeMatches);

  /* one-way HTTP(S) POST message requires an HTTP OK response message */
  if (!strncmp(soap->endpoint, "http", 4))
    return soap_send_empty_response(soap, SOAP_OK);
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int __wsdd__Resolve(struct soap *soap, struct wsdd__ResolveType *Resolve)
@brief Internal WS-Discovery service operation.
*/
SOAP_FMAC5
int
SOAP_FMAC6
__wsdd__Resolve(struct soap *soap, struct wsdd__ResolveType *Resolve)
{
  struct wsdd__ResolveMatchesType ResolveMatches;
  struct wsdd__ResolveMatchType match;
  soap_wsdd_mode mode;

  DBGFUN("__wsdd__Resolve");

  /* check for WSA */
  if (soap_wsa_check(soap))
    return soap->error;

  /* check WSDD */
  if (!Resolve
#ifdef SOAP_WSA_2005
   || !Resolve->wsa5__EndpointReference.Address
#else
   || !Resolve->wsa__EndpointReference.Address
#endif
    )
    return soap_wsa_sender_fault(soap, "WSDD Resolve incomplete", NULL);

  soap_default_wsdd__ResolveMatchesType(soap, &ResolveMatches);
  soap_default_wsdd__ResolveMatchType(soap, &match);
  ResolveMatches.ResolveMatch = &match;

  /* pass resolve request on to user-defined event handler */
  mode = wsdd_event_Resolve(soap,
#ifdef SOAP_WSA_2005
    soap->header->wsa5__MessageID,
    soap->header->wsa5__ReplyTo ? soap->header->wsa5__ReplyTo->Address : NULL,
    Resolve->wsa5__EndpointReference.Address,
#else
    soap->header->wsa__MessageID,
    soap->header->wsa__ReplyTo ? soap->header->wsa__ReplyTo->Address : NULL,
    Resolve->wsa__EndpointReference.Address,
#endif
    &match);

  if (mode == SOAP_WSDD_ADHOC)
  {
    /* event handler or main application MUST return match independently */

    /* one-way HTTP(S) POST message requires an HTTP OK response message */
    if (!strncmp(soap->endpoint, "http", 4))
      return soap_send_empty_response(soap, SOAP_OK);
  }
  else
  {
    int err;
    int (*fpost)(struct soap*, const char*, const char*, int, const char*, const char*, ULONG64);
    const char *MessageID = soap_wsa_rand_uuid(soap);
    const char *Action = SOAP_NAMESPACE_OF_wsdd"/ResolveMatches";

    /* SOAP Header */
    soap_wsdd_set_AppSequence(soap);
    soap_wsa_reply(soap, MessageID, Action);

    /* send over current socket as HTTP OK response: */
    fpost = soap->fpost;
    soap->fpost = soap_wsdd_http;
    err = soap_send___wsdd__ResolveMatches(soap, NULL, Action, &ResolveMatches);
    soap->fpost = fpost;
    return err;
  }
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int __wsdd__ResolveMatches(struct soap *soap, struct wsdd__ResolveMatchesType *ResolveMatches)
@brief Internal WS-Discovery service operation.
*/
SOAP_FMAC5
int
SOAP_FMAC6
__wsdd__ResolveMatches(struct soap *soap, struct wsdd__ResolveMatchesType *ResolveMatches)
{
  unsigned int InstanceId = 0;
  const char *SequenceId = NULL;
  unsigned int MessageNumber = 0;

  DBGFUN("__wsdd__ResolveMatches");

  /* check for WSA */
  if (soap_wsa_check(soap))
    return soap->error;

  /* check WSDD */
  if (!ResolveMatches
   || !ResolveMatches->ResolveMatch)
    return soap_wsa_sender_fault(soap, "WSDD ResolveMatch incomplete", NULL);

  if (soap->header->wsdd__AppSequence)
  {
    wsdd__AppSequenceType *seq = soap->header->wsdd__AppSequence;
    InstanceId = seq->InstanceId;
    SequenceId = seq->SequenceId;
    MessageNumber = seq->MessageNumber;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "AppSeq id=%u seq=%s msg=%u\n", InstanceId, SequenceId ? SequenceId : "(null)", MessageNumber));
  }

  /* pass resolve matches on to user-defined event handler */
  wsdd_event_ResolveMatches(soap,
    InstanceId,
    SequenceId,
    MessageNumber, 
#ifdef SOAP_WSA_2005
    soap->header->wsa5__MessageID,
    soap->header->wsa5__RelatesTo ? soap->header->wsa5__RelatesTo->__item : NULL,
#else
    soap->header->wsa__MessageID,
    soap->header->wsa__RelatesTo ? soap->header->wsa__RelatesTo->__item : NULL,
#endif
    ResolveMatches->ResolveMatch);

  /* one-way HTTP(S) POST message requires an HTTP OK response message */
  if (!strncmp(soap->endpoint, "http", 4))
    return soap_send_empty_response(soap, SOAP_OK);
  return SOAP_OK;
}

/******************************************************************************\
 *
 *      WS-Discovery AppSequence State
 *
\******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/**
@fn void soap_wsdd_set_InstanceId(unsigned int InstanceId)
@brief Set the global AppSequence InstanceId that is used to populate messages
@param[in] InstanceId
*/
SOAP_FMAC1
void
SOAP_FMAC2
soap_wsdd_set_InstanceId(unsigned int InstanceId)
{
  MUTEX_LOCK(soap_wsdd_state);
  soap_wsdd_InstanceId = InstanceId;
  soap_wsdd_MessageNumber = 1;
  MUTEX_UNLOCK(soap_wsdd_state);
}

/******************************************************************************/

/**
@fn void soap_wsdd_set_SequenceId(const char *SequenceId)
@brief Set the global AppSequence SequenceId that is used to populate messages
@param[in] SequenceId
*/
SOAP_FMAC1
void
SOAP_FMAC2
soap_wsdd_set_SequenceId(const char *SequenceId)
{
  MUTEX_LOCK(soap_wsdd_state);
  if (soap_wsdd_SequenceId)
    free((void*)soap_wsdd_SequenceId);
  soap_wsdd_SequenceId = strdup(SequenceId);
  soap_wsdd_MessageNumber = 1;
  MUTEX_UNLOCK(soap_wsdd_state);
}

/******************************************************************************/

#ifdef __cplusplus
}
#endif

/******************************************************************************/

static void
soap_wsdd_set_AppSequence(struct soap *soap)
{
  soap_header(soap);
  if (!soap->header->wsdd__AppSequence)
  {
    soap->header->wsdd__AppSequence = (wsdd__AppSequenceType*)soap_malloc(soap, sizeof(wsdd__AppSequenceType));
    soap_default_wsdd__AppSequenceType(soap, soap->header->wsdd__AppSequence);
  }

  MUTEX_LOCK(soap_wsdd_state);
  soap->header->wsdd__AppSequence->InstanceId = soap_wsdd_InstanceId;
  soap->header->wsdd__AppSequence->SequenceId = (char*)soap_wsdd_SequenceId;
  soap->header->wsdd__AppSequence->MessageNumber = soap_wsdd_MessageNumber++;
  MUTEX_UNLOCK(soap_wsdd_state);
}

/******************************************************************************/

static void
soap_wsdd_reset_AppSequence(struct soap *soap)
{
  if (soap->header)
    soap->header->wsdd__AppSequence = NULL;
}

/******************************************************************************/

static void
soap_wsdd_delay(struct soap *soap)
{
  unsigned int r = (unsigned int)soap_random % SOAP_WSDD_APP_MAX_DELAY;
#if defined(_WRS_KERNEL)
  int delay = r * sysClkRateGet() / 1000;
  taskDelay(delay); /* VxWorks compatible sleep API, delay is specified in number of ticks, which depends on the System Clock Rate */
#elif defined(WIN32)
  DWORD delay = (DWORD)r;
  Sleep(delay);
#else
  useconds_t delay = 1000 * r;
  usleep(delay);
#endif
  (void)soap;
}

/******************************************************************************/
