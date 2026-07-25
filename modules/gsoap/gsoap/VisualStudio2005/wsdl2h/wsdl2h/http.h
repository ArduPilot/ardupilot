/*
	http.h

	WSDL/HTTP binding schema interface

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2012, Robert van Engelen, Genivia Inc. All Rights Reserved.
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

//gsoap http schema documentation:	WSDL 1.1 HTTP binding schema
//gsoap http schema namespace:		http://schemas.xmlsoap.org/wsdl/http/
//gsoap http schema elementForm:	qualified
//gsoap http schema attributeForm:	unqualified

//gsoap whttp schema documentation:	WSDL 2.0 HTTP binding schema
//gsoap whttp schema namespace:		http://www.w3.org/ns/wsdl/http
//gsoap whttp schema elementForm:	qualified
//gsoap whttp schema attributeForm:	unqualified

#import "imports.h"

class http__address
{ public:
	@xsd__anyURI		location;
};

class http__binding
{ public:
	@xsd__NMTOKEN		verb;
};

class http__operation
{ public:
	@xsd__anyURI		location;
};

class whttp__header
{ public:
	@xsd__string		name;
	@xsd__QName		type;
	@xsd__boolean		required = false;
};
