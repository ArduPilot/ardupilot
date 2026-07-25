/*
	gwsdl.h

	OGSI GWSDL binding schema interface

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2008, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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

//gsoap gwsdl schema documentation:	OGSI GWSDL binding schema
//gsoap gwsdl schema namespace:		http://www.gridforum.org/namespaces/2003/03/gridWSDLExtensions
//gsoap sd schema namespace:		http://www.gridforum.org/namespaces/2003/03/serviceData

#import "schema.h"

class wsdl__operation;

enum sd__mutability { static_, constant, extendable, mutable_ };

class sd__serviceData
{ public:
	@xsd__NMTOKEN			name;
	@xsd__QName			type;
	@xsd__boolean			nillable		= false;
	@xsd__string			minOccurs;		// xsd:nonNegativeInteger
	@xsd__string			maxOccurs;		// xsd:nonNegativeInteger|unbounded
	@enum sd__mutability		mutability		= extendable;
	@xsd__boolean			modifiable		= false;
	/* has any content */
  public:
};

class sd__staticServiceDataValues
{ public:
	int				__type; /* any content, probably should use DOM */
	void*				_any;
};

class gwsdl__portType
{ public:
	@xsd__NMTOKEN			name;
	@xsd__QName			extends;		// a list of QNames
	xsd__string			documentation;		// <wsdl:documentation>?
	std::vector<wsdl__operation*>	operation;		// <wsdl:operation>*
	std::vector<sd__serviceData>	sd__serviceData_;
	sd__staticServiceDataValues	*sd__staticServiceDataValues_;
  public:
};
