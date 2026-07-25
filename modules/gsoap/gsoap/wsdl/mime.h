/*
	mime.h

	mime and xmime binding schema

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2008, Robert van Engelen, Genivia Inc. All Rights Reserved.
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

//gsoap mime schema documentation:	WSDL/MIME binding schema
//gsoap mime schema namespace:		http://schemas.xmlsoap.org/wsdl/mime/
//gsoap mime schema elementForm:	qualified
//gsoap mime schema attributeForm:	unqualified

//gsoap xmime schema documentation:	xmime binding schema
//gsoap xmime schema namespace:		http://www.w3.org/2005/05/xmlmime

#import "imports.h"
#import "soap.h"

class mime__content
{ public:
 	@xsd__NMTOKEN			part;
	@xsd__string			type;
};

class mime__part
{ public:
	soap__body			*soap__body_;
	std::vector<soap__header>	soap__header_;
	std::vector<mime__content>	content;
  public:
  	int				traverse(wsdl__definitions&);
};

class mime__multipartRelated
{ public:
	std::vector<mime__part>		part;
  public:
  	int				traverse(wsdl__definitions&);
};

class mime__mimeXml
{ public:
	@xsd__NMTOKEN			part;
};
