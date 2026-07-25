/*
	sp.h

	WS-SecurityPolicy 1.2 binding schemas

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

//gsoap sp schema documentation:	WS-SecurityPolicy binding
//gsoap sp schema namespace:		http://docs.oasis-open.org/ws-sx/ws-securitypolicy/200702
// 1.1 //gsoap sp schema namespace:	http://schemas.xmlsoap.org/ws/2005/07/securitypolicy
//gsoap sp schema elementForm:		qualified             
//gsoap sp schema attributeForm:	unqualified           

#import "imports.h"
#import "wsam.h"
#import "wst.h"

class sp__Header
{ public:
	@xsd__NCName			Name;
	@xsd__anyURI			Namespace;
};

class sp__Parts
{ public:
	xsd__string			Body;
	std::vector<sp__Header>		Header;
	xsd__string			Attachments;
};

class sp__Elements
{ public:
	@xsd__anyURI			XPathVersion;
	std::vector<xsd__string>	XPath;
};

class sp__Token : public wsp__Assertion
{ public:
	@xsd__anyURI			IncludeToken;
	wsa__EndpointReferenceType	*Issuer;
	xsd__anyURI			IssuerName;
	wst__Claims			*wst__Claims_;
};
