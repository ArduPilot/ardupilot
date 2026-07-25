/*
	wsam.h

	WS-Addressing and WS-Addressing Metadata

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

//gsoap wsa schema documentation:	WS-Addressing
//gsoap wsa schema namespace:		http://www.w3.org/2005/08/addressing
//gsoap wsa schema elementForm:		qualified
//gsoap wsa schema attributeForm:	unqualified

//gsoap wsam schema documentation:	WS-Addressing Metadata
//gsoap wsam schema namespace:		http://www.w3.org/2007/05/addressing/metadata
//gsoap wsam schema elementForm:	qualified
//gsoap wsam schema attributeForm:	unqualified

//gsoap wsaw schema documentation:	WS-Addressing WSDL
//gsoap wsaw schema namespace:		http://www.w3.org/2006/05/addressing/wsdl
//gsoap wsaw schema elementForm:	qualified
//gsoap wsaw schema attributeForm:	unqualified

class wsa__EndpointReferenceType
{ public:
	xsd__anyURI			Address;
	_XML				__any;
};

