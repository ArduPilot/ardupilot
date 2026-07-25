/*
        wsddapi.h

        WS-Discovery 1.1 and 1.0 (WSDD) plugin API

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

#ifndef WSDDAPI_H
#define WSDDAPI_H

#include "wsaapi.h"     /* also includes soapH.h, see wsaapi.h if you are using a different fileH.h */
#include "threads.h"    /* threads and locks from plugin/threads.h */

/******************************************************************************\
 *
 *      Common Constants
 *
\******************************************************************************/

/**
A Target Service MUST wait t (ms) to elapse before sending the message, where
0 < t < APP_MAX_DELAY is randomly choosen. The default APP_MAX_DELAY is 500 ms.
*/
#ifndef SOAP_WSDD_APP_MAX_DELAY
# define SOAP_WSDD_APP_MAX_DELAY        (500)   /* ms delay */
#endif

/**
If the DP is unresponsive after DP_MAX_TIMEOUT, or if the Client finds the
responses from the DP unsatisfactory, the Client reverts to using the multicast
messages.
*/
#ifdef SOAP_WSDD_DP_MAX_TIMEOUT
# define SOAP_WSDD_DP_MAX_TIMEOUT       (5000)  /* ms timeout */
#endif

/**
Managed or ad-hoc mode
*/
typedef enum soap_wsdd_mode {SOAP_WSDD_MANAGED, SOAP_WSDD_ADHOC} soap_wsdd_mode;

/**
Send message to Target Service (TS) or Discovery Proxy (DP)
*/
typedef enum soap_wsdd_to { SOAP_WSDD_TO_DP, SOAP_WSDD_TO_TS } soap_wsdd_to;

/******************************************************************************\
 *
 *      WS-Discovery operations (copied here from soapClient.cpp)
 *
\******************************************************************************/

SOAP_FMAC5 int SOAP_FMAC6 soap_send___wsdd__Hello(struct soap *soap, const char *soap_endpoint, const char *soap_action, struct wsdd__HelloType *wsdd__Hello);

SOAP_FMAC5 int SOAP_FMAC6 soap_recv___wsdd__Hello(struct soap *soap, struct __wsdd__Hello *);

SOAP_FMAC5 int SOAP_FMAC6 soap_send___wsdd__Bye(struct soap *soap, const char *soap_endpoint, const char *soap_action, struct wsdd__ByeType *wsdd__Bye);

SOAP_FMAC5 int SOAP_FMAC6 soap_recv___wsdd__Bye(struct soap *soap, struct __wsdd__Bye *);

SOAP_FMAC5 int SOAP_FMAC6 soap_send___wsdd__Probe(struct soap *soap, const char *soap_endpoint, const char *soap_action, struct wsdd__ProbeType *wsdd__Probe);

SOAP_FMAC5 int SOAP_FMAC6 soap_recv___wsdd__Probe(struct soap *soap, struct __wsdd__Probe *);

SOAP_FMAC5 int SOAP_FMAC6 soap_send___wsdd__ProbeMatches(struct soap *soap, const char *soap_endpoint, const char *soap_action, struct wsdd__ProbeMatchesType *wsdd__ProbeMatches);

SOAP_FMAC5 int SOAP_FMAC6 soap_recv___wsdd__ProbeMatches(struct soap *soap, struct __wsdd__ProbeMatches *);


SOAP_FMAC5 int SOAP_FMAC6 soap_send___wsdd__Resolve(struct soap *soap, const char *soap_endpoint, const char *soap_action, struct wsdd__ResolveType *wsdd__Resolve);

SOAP_FMAC5 int SOAP_FMAC6 soap_recv___wsdd__Resolve(struct soap *soap, struct __wsdd__Resolve *);

SOAP_FMAC5 int SOAP_FMAC6 soap_send___wsdd__ResolveMatches(struct soap *soap, const char *soap_endpoint, const char *soap_action, struct wsdd__ResolveMatchesType *wsdd__ResolveMatches);

SOAP_FMAC5 int SOAP_FMAC6 soap_recv___wsdd__ResolveMatches(struct soap *soap, struct __wsdd__ResolveMatches *);

/******************************************************************************\
 *
 *      WS-Discovery Operations
 *
\******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

SOAP_FMAC1 void SOAP_FMAC2 soap_wsdd_set_InstanceId(unsigned int InstanceId);

SOAP_FMAC1 void SOAP_FMAC2 soap_wsdd_set_SequenceId(const char *SequenceId);

SOAP_FMAC1 int SOAP_FMAC2 soap_wsdd_Hello(struct soap *soap, soap_wsdd_mode mode, const char *endpoint, const char *MessageID, const char *RelatesTo, const char *EndpointReference, const char *Types, const char *Scopes, const char *MatchBy, const char *XAddrs, unsigned int MetadataVersion);

SOAP_FMAC1 int SOAP_FMAC2 soap_wsdd_Bye(struct soap *soap, soap_wsdd_mode mode, const char *endpoint, const char *MessageID, const char *EndpointReference, const char *Types, const char *Scopes, const char *MatchBy, const char *XAddrs, unsigned int MetadataVersion);

SOAP_FMAC1 int SOAP_FMAC2 soap_wsdd_Probe(struct soap *soap, soap_wsdd_mode mode, soap_wsdd_to to, const char *endpoint, const char *MessageID, const char *ReplyTo, const char *Types, const char *Scopes, const char *MatchBy);

SOAP_FMAC1 int SOAP_FMAC2 soap_wsdd_Resolve(struct soap *soap, soap_wsdd_mode mode, soap_wsdd_to to, const char *endpoint, const char *MessageID, const char *ReplyTo, const char *EndpointReference);

SOAP_FMAC1 void SOAP_FMAC2 soap_wsdd_init_ProbeMatches(struct soap *soap, struct wsdd__ProbeMatchesType *matches);

SOAP_FMAC1 int SOAP_FMAC2 soap_wsdd_add_ProbeMatch(struct soap *soap, struct wsdd__ProbeMatchesType *matches, const char *EndpointReference, const char *Types, const char *Scopes, const char *MatchBy, const char *XAddrs, unsigned int MetadataVersion);

SOAP_FMAC1 int SOAP_FMAC2 soap_wsdd_ProbeMatches(struct soap *soap, const char *endpoint, const char *MessageID, const char *RelatesTo, const char *To, struct wsdd__ProbeMatchesType *matches);

SOAP_FMAC1 int SOAP_FMAC2 soap_wsdd_ResolveMatches(struct soap *soap, const char *endpoint, const char *MessageID, const char *RelatesTo, const char *To, const char *EndpointReference, const char *Types, const char *Scopes, const char *MatchBy, const char *XAddrs, unsigned int MetadataVersion);

SOAP_FMAC1 int SOAP_FMAC2 soap_wsdd_listen(struct soap *soap, int timeout);
SOAP_FMAC1 int SOAP_FMAC2 soap_wsdd_serve_request(struct soap *soap);

#ifdef __cplusplus
}
#endif

SOAP_FMAC5 int SOAP_FMAC6 __wsdd__Hello(struct soap *soap, struct wsdd__HelloType *Hello);

SOAP_FMAC5 int SOAP_FMAC6 __wsdd__Bye(struct soap *soap, struct wsdd__ByeType *Bye);

SOAP_FMAC5 int SOAP_FMAC6 __wsdd__Probe(struct soap *soap, struct wsdd__ProbeType *Probe);

SOAP_FMAC5 int SOAP_FMAC6 __wsdd__ProbeMatches(struct soap *soap, struct wsdd__ProbeMatchesType *ProbeMatches);

SOAP_FMAC5 int SOAP_FMAC6 __wsdd__Resolve(struct soap *soap, struct wsdd__ResolveType *Resolve);

SOAP_FMAC5 int SOAP_FMAC6 __wsdd__ResolveMatches(struct soap *soap, struct wsdd__ResolveMatchesType *ResolveMatches);

/******************************************************************************\
 *
 *      User-defined WS-Discovery event handlers
 *
\******************************************************************************/

/**
@fn void wsdd_event_Hello(struct soap *soap, unsigned int InstanceId, const char *SequenceId, unsigned int MessageNumber, const char *MessageID, const char *RelatesTo, const char *EndpointReference, const char *Types, const char *Scopes, const char *MatchBy, const char *XAddrs, unsigned int MetadataVersion)
@brief Handles and registers a Hello event from a TS or DP joining the network.
@param soap context (use soap->user as a pointer to a global state if needed)
@param[in] InstanceId (see WS-Discovery 1.1 Section 7 Application Sequencing)
@param[in] SequenceId (see WS-Discovery 1.1 Section 7 Application Sequencing)
@param[in] MessageNumber (see WS-Discovery 1.1 Section 7 Application Sequencing)
@param[in] MessageID WS-Addressing message ID of the Hello message
@param[in] RelatesTo WS-Addressing RelatesTo message ID of the Hello message
@param[in] EndpointReference of the Target Service or Discovery Proxy that joins
@param[in] Types an unordered string of QNames of services provided by the Target Service or Discovery Proxy where a Discovery Proxy MUST include "wsdd:DiscoveryProxy"
@param[in] Scopes an unordered set of Scopes the Target Service or Discovery Proxy is in
@param[in] MatchBy unused (reserved)
@param[in] XAddrs contains the transport address(es) that MAY be used to communicate with the Target Service or Discovery Proxy
@param[in] MetadataVersion incremented by a positive value (>= 1) whenever there is a change in the metadata of the Target Service

Hello is a one-way message sent by a Target Service to announce its
availability when it joins the network. It is also sent by a Discovery Proxy to
reduce multicast traffic on an ad hoc network.

To maintain a global state between events, for example to internally register
Target Services, Discovery Proxies, and update the status of these, use
void *soap->user to point to a global state (that you need to define).
*/
void wsdd_event_Hello(struct soap *soap, unsigned int InstanceId, const char *SequenceId, unsigned int MessageNumber, const char *MessageID, const char *RelatesTo, const char *EndpointReference, const char *Types, const char *Scopes, const char *MatchBy, const char *XAddrs, unsigned int MetadataVersion);

/**
@fn void wsdd_event_Bye(struct soap *soap, unsigned int InstanceId, const char *SequenceId, unsigned int MessageNumber, const char *MessageID, const char *RelatesTo, const char *EndpointReference, const char *Types, const char *Scopes, const char *MatchBy, const char *XAddrs, unsigned int *MetadataVersion)
@brief Handles and registers a Bye event from a TS or DP leaving the network.
@param soap context (use soap->user as a pointer to a global state if needed)
@param[in] InstanceId (see WS-Discovery 1.1 Section 7 Application Sequencing)
@param[in] SequenceId (see WS-Discovery 1.1 Section 7 Application Sequencing)
@param[in] MessageNumber (see WS-Discovery 1.1 Section 7 Application Sequencing)
@param[in] MessageID WS-Addressing message ID of the Bye message
@param[in] RelatesTo WS-Addressing RelatesTo message ID of the Bye message
@param[in] EndpointReference of the Target Service or Discovery Proxy
@param[in] Types an unordered string of QNames of services provided by the Target Service or Discovery Proxy where a Discovery Proxy MUST include "wsdd:DiscoveryProxy"
@param[in] Scopes an unordered set of Scopes the Target Service or Discovery
Proxy is in
@param[in] MatchBy unused (reserved)
@param[in] XAddrs contains the transport address(es) that MAY be used to communicate with the Target Service or Discovery Proxy
@param[in] MetadataVersion incremented by a positive value (>= 1) whenever there is a change in the metadata of the Target Service

Bye is a one-way message sent by a Target Service to announce its
unavailability as a best effort when it leaves the network.

To maintain a global state between events, for example to internally register
Target Services, Discovery Proxies, and update the status of these, use
void *soap->user to point to a global state (that you need to define).
*/
void wsdd_event_Bye(struct soap *soap, unsigned int InstanceId, const char *SequenceId, unsigned int MessageNumber, const char *MessageID, const char *RelatesTo, const char *EndpointReference, const char *Types, const char *Scopes, const char *MatchBy, const char *XAddrs, unsigned int *MetadataVersion);

/**
@fn soap_wsdd_mode wsdd_event_Probe(struct soap *soap, const char *MessageID, const char *ReplyTo, const char *Types, const char *Scopes, const char *MatchBy, struct wsdd__ProbeMatchesType *matches)
@brief Handles a Probe event from a Client.
@param soap context (use soap->user as a pointer to a global state if needed)
@param[in] MessageID WS-Addressing message ID of the message
@param[in] ReplyTo WS-Addressing ReplyTo message ID of the message
@param[in] Types an unordered string of QNames to probe
@param[in] Scopes an unordered set of scopes to probe
@param[in] MatchBy matching rule to apply for this probe
@param[out] matches contains probe matches returned by event handler, use @ref soap_wsdd_add_ProbeMatch to populate the matches in the handler
@return managed (SOAP_WSDD_MANAGED) or ad-hoc (SOAP_WSDD_ADHOC) mode to use to return the matches

A Client sends a probe to find Target Services by the Type of the Target
Service, a Scope in which the Target Service resides, both, or simply all
Target Services. The matches are returned by this server-side event handler
that match the Client's probe.

To maintain a global state between events, for example to internally register
Target Services, Discovery Proxies, and update the status of these, use
void *soap->user to point to a global state (that you need to define).
*/
soap_wsdd_mode wsdd_event_Probe(struct soap *soap, const char *MessageID, const char *ReplyTo, const char *Types, const char *Scopes, const char *MatchBy, struct wsdd__ProbeMatchesType *matches);

/**
@fn void wsdd_event_ProbeMatches(struct soap *soap, unsigned int InstanceId, const char *SequenceId, unsigned int MessageNumber, const char *MessageID, const char *RelatesTo, struct wsdd__ProbeMatchesType *matches)
@brief Handles a Probe event from a Client.
@param soap context (use soap->user as a pointer to a global state if needed)
@param[in] InstanceId (see WS-Discovery 1.1 Section 7 Application Sequencing)
@param[in] SequenceId (see WS-Discovery 1.1 Section 7 Application Sequencing)
@param[in] MessageNumber (see WS-Discovery 1.1 Section 7 Application Sequencing)
@param[in] MessageID WS-Addressing message ID of the message
@param[in] RelatesTo WS-Addressing RelatesTo message ID of the message
@param[in] matches contains the probe matches

A Client sends a probe to find Target Services by the Type of the Target
Service, a Scope in which the Target Service resides, both, or simply all
Target Services. The matches are provided to this client-side event handler.

To maintain a global state between events, for example to internally register
Target Services, Discovery Proxies, and update the status of these, use
void *soap->user to point to a global state (that you need to define).
*/
void wsdd_event_ProbeMatches(struct soap *soap, unsigned int InstanceId, const char *SequenceId, unsigned int MessageNumber, const char *MessageID, const char *RelatesTo, struct wsdd__ProbeMatchesType *matches);

/**
@fn soap_wsdd_mode wsdd_event_Resolve(struct soap *soap, const char *MessageID, const char *ReplyTo, const char *EndpointReference, struct wsdd__ResolveMatchType *match);
@brief Handles a Resolve event from a Client.
@param soap context (use soap->user as a pointer to a global state if needed)
@param[in] MessageID WS-Addressing message ID of the message
@param[in] ReplyTo WS-Addressing ReplyTo message ID of the message
@param[in] EndpointReference of the Target Service or Discovery Proxy
@param[out] match contains the match returned by the event handler
@return managed (SOAP_WSDD_MANAGED) or ad-hoc (SOAP_WSDD_ADHOC) mode to use to return the matches

A Client sends a resolve to locate a Target Service, i.e., to retrieve its
transport address(es). This server-side event handler returns the match(es).

To maintain a global state between events, for example to internally register
Target Services, Discovery Proxies, and update the status of these, use
void *soap->user to point to a global state (that you need to define).
*/
soap_wsdd_mode wsdd_event_Resolve(struct soap *soap, const char *MessageID, const char *ReplyTo, const char *EndpointReference, struct wsdd__ResolveMatchType *match);

/**
@fn void wsdd_event_ResolveMatches(struct soap *soap, unsigned int InstanceId, const char *SequenceId, unsigned int MessageNumber, const char *MessageID, const char *RelatesTo, struct wsdd__ResolveMatchType *match)
@brief Handles a Probe event from a Client.
@param soap context (use soap->user as a pointer to a global state if needed)
@param[in] InstanceId (see WS-Discovery 1.1 Section 7 Application Sequencing)
@param[in] SequenceId (see WS-Discovery 1.1 Section 7 Application Sequencing)
@param[in] MessageNumber (see WS-Discovery 1.1 Section 7 Application Sequencing)
@param[in] MessageID WS-Addressing message ID of the message
@param[in] RelatesTo WS-Addressing RelatesTo message ID of the message
@param[in] match contains the resolve match

A Client sends a resolve to locate a Target Service, i.e., to retrieve its
transport address(es). This client-side event handler receives the match.

To maintain a global state between events, for example to internally register
Target Services, Discovery Proxies, and update the status of these, use
void *soap->user to point to a global state (that you need to define).
*/
void wsdd_event_ResolveMatches(struct soap *soap, unsigned int InstanceId, const char *SequenceId, unsigned int MessageNumber, const char *MessageID, const char *RelatesTo, struct wsdd__ResolveMatchType *match);

#endif
