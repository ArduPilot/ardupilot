/*
	wsdx.h

	WS-Discovery 1.0/1.1 operation definitions:
	SOAP Header definitions for WS-Discovery
	WSDD operations Hello, Bye, Probe, ProbeMatches, Resolve, ResolveMatches

	Imported by import/wsdd.h, import/wsdd5.h, import/wsdd10.h

gSOAP XML Web services tools
Copyright (C) 2000-2011, Robert van Engelen, Genivia Inc., All Rights Reserved.
This part of the software is released under ONE of the following licenses:
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

mutable struct SOAP_ENV__Header
{
  struct wsdd__AppSequenceType           *wsdd__AppSequence             0;
};

//gsoap wsdd service name: wsdd

//gsoap wsdd service method-header-part:     Hello wsa5__RelatesTo
//gsoap wsdd service method-header-part:     Hello wsa5__From
//gsoap wsdd service method-header-part:     Hello wsa5__ReplyTo
//gsoap wsdd service method-header-part:     Hello wsa5__FaultTo
//gsoap wsdd service method-header-part:     Hello wsa5__To
//gsoap wsdd service method-header-part:     Hello wsa5__Action
//gsoap wsdd service method-action:          Hello http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01/Hello
int __wsdd__Hello(struct wsdd__HelloType *wsdd__Hello, void);

//gsoap wsdd service method-header-part:     Bye wsa5__RelatesTo
//gsoap wsdd service method-header-part:     Bye wsa5__From
//gsoap wsdd service method-header-part:     Bye wsa5__ReplyTo
//gsoap wsdd service method-header-part:     Bye wsa5__FaultTo
//gsoap wsdd service method-header-part:     Bye wsa5__To
//gsoap wsdd service method-header-part:     Bye wsa5__Action
//gsoap wsdd service method-action:          Bye http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01/Bye
int __wsdd__Bye(struct wsdd__ByeType *wsdd__Bye, void);

//gsoap wsdd service method-header-part:     Probe wsa5__RelatesTo
//gsoap wsdd service method-header-part:     Probe wsa5__From
//gsoap wsdd service method-header-part:     Probe wsa5__ReplyTo
//gsoap wsdd service method-header-part:     Probe wsa5__FaultTo
//gsoap wsdd service method-header-part:     Probe wsa5__To
//gsoap wsdd service method-header-part:     Probe wsa5__Action
//gsoap wsdd service method-action:          Probe http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01/Probe
int __wsdd__Probe(struct wsdd__ProbeType *wsdd__Probe, void);

//gsoap wsdd service method-header-part:     ProbeMatches wsa5__RelatesTo
//gsoap wsdd service method-header-part:     ProbeMatches wsa5__From
//gsoap wsdd service method-header-part:     ProbeMatches wsa5__ReplyTo
//gsoap wsdd service method-header-part:     ProbeMatches wsa5__FaultTo
//gsoap wsdd service method-header-part:     ProbeMatches wsa5__To
//gsoap wsdd service method-header-part:     ProbeMatches wsa5__Action
//gsoap wsdd service method-action:          ProbeMatches http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01/ProbeMatches
int __wsdd__ProbeMatches(struct wsdd__ProbeMatchesType *wsdd__ProbeMatches, void);

//gsoap wsdd service method-header-part:     Resolve wsa5__RelatesTo
//gsoap wsdd service method-header-part:     Resolve wsa5__From
//gsoap wsdd service method-header-part:     Resolve wsa5__ReplyTo
//gsoap wsdd service method-header-part:     Resolve wsa5__FaultTo
//gsoap wsdd service method-header-part:     Resolve wsa5__To
//gsoap wsdd service method-header-part:     Resolve wsa5__Action
//gsoap wsdd service method-action:          Resolve http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01/Resolve
int __wsdd__Resolve(struct wsdd__ResolveType *wsdd__Resolve, void);

//gsoap wsdd service method-header-part:     ResolveMatches wsa5__RelatesTo
//gsoap wsdd service method-header-part:     ResolveMatches wsa5__From
//gsoap wsdd service method-header-part:     ResolveMatches wsa5__ReplyTo
//gsoap wsdd service method-header-part:     ResolveMatches wsa5__FaultTo
//gsoap wsdd service method-header-part:     ResolveMatches wsa5__To
//gsoap wsdd service method-header-part:     ResolveMatches wsa5__Action
//gsoap wsdd service method-action:          ResolveMatches http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01/ResolveMatches
int __wsdd__ResolveMatches(struct wsdd__ResolveMatchesType *wsdd__ResolveMatches, void);

