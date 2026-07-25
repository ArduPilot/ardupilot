/*
	wsrmp.h

	WS-ReliableMessaging Policy

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2010, Robert van Engelen, Genivia Inc. All Rights Reserved.
This software is released under one of the following licenses:
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

//gsoap wsrmp schema documentation:	WS-ReliableMessaging Policy binding
//gsoap wsrmp schema namespace:		http://docs.oasis-open.org/ws-rx/wsrm/200702
//gsoap wsrmp schema elementForm:	qualified             
//gsoap wsrmp schema attributeForm:	unqualified           

//gsoap wsrmp5 schema documentation:	WS-ReliableMessaging Policy binding
//gsoap wsrmp5 schema namespace:	http://schemas.xmlsoap.org/ws/2005/02/rm/policy
//gsoap wsrmp5 schema elementForm:	qualified             
//gsoap wsrmp5 schema attributeForm:	unqualified           

#import "imports.h"

class wsrmp__Timeout
{ public:
  @char *Milliseconds;
};

class wsrmp__RMAssertion : public wsp__Assertion
{ public:
  wsrmp__Timeout *InactivityTimeout;
  wsrmp__Timeout *BaseRetransmissionInterval;
  wsrmp__Timeout *AcknowledgementInterval;
  char           *ExponentialBackoff;
// TODO: WCF netrmp extension elements go here, as necessary
};

class wsrmp5__Timeout
{ public:
  @char *Milliseconds;
};

class wsrmp5__RMAssertion : public wsp__Assertion
{ public:
  wsrmp5__Timeout *InactivityTimeout;
  wsrmp5__Timeout *BaseRetransmissionInterval;
  wsrmp5__Timeout *AcknowledgementInterval;
  char           *ExponentialBackoff;
// TODO: WCF netrmp extension elements go here, as necessary
};

