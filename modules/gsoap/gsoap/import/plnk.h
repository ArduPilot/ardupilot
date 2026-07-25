/*
	plnk.h

	BPEL 2.0 binding schema

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2014, Robert van Engelen, Genivia Inc. All Rights Reserved.
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

#import "stlvector.h"

typedef char    *xsd__NCName,
                *xsd__QName,
                *xsd__string;

//gsoap plnk  schema documentation:	Partner Link Type Schema for WS-BPEL 2.0
//gsoap plnk  schema namespace:		http://docs.oasis-open.org/wsbpel/2.0/plnktype
//gsoap plnk  schema elementForm:	qualified
//gsoap plnk  schema attributeForm:	unqualified

class plnk__tRole
{ public:
	@xsd__NCName			name;
	@xsd__QName			portType;
	xsd__string			documentation;
};

class plnk__tPartnerLinkType
{ public:
	@xsd__NCName			name;
	std::vector<plnk__tRole> 	role;
	xsd__string			documentation;
};
