/*
	soap.h

	WSDL/SOAP binding schema

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

//gsoap soap schema documentation:	WSDL 1.1 SOAP binding schema
//gsoap soap schema namespace:		http://schemas.xmlsoap.org/wsdl/soap/
//gsoap soap schema elementForm:        qualified
//gsoap soap schema attributeForm:      unqualified

//gsoap wsoap schema documentation:	WSDL 2.0 SOAP binding schema
//gsoap wsoap schema namespace:		http://www.w3.org/ns/wsdl/soap
//gsoap wsoap schema elementForm:       qualified
//gsoap wsoap schema attributeForm:     unqualified

#import "imports.h"

class wsdl__definitions;		// forward declaration
class wsdl__message;			// forward declaration
class wsdl__part;			// forward declaration

enum soap__styleChoice { rpc, document };

class soap__binding
{ public:
	@xsd__anyURI			transport;
 	@enum soap__styleChoice		*style;
};

class soap__operation
{ public:
	@xsd__anyURI			soapAction;
	@xsd__boolean			soapActionRequired	= true;
	@enum soap__styleChoice		*style;
};

enum soap__useChoice { literal, encoded };

class soap__body
{ public:
	@xsd__anyURI			encodingStyle;
 	@xsd__NMTOKENS			parts;
	@enum soap__useChoice		use;
	@xsd__anyURI			namespace_;
};

class soap__fault
{ public:
	@xsd__NMTOKEN			name;
	@xsd__anyURI			encodingStyle;
	@enum soap__useChoice		use;
	@xsd__anyURI			namespace_;
};

class soap__headerfault
{ public:
	@xsd__QName			message;
 	@xsd__NMTOKEN			part;
	@enum soap__useChoice		use;
	@xsd__anyURI			encodingStyle;
	@xsd__anyURI			namespace_;
  private:
  	wsdl__message			*messageRef;
  	wsdl__part			*partRef;
  public:
  	int				traverse(wsdl__definitions&);
	void				messagePtr(wsdl__message*);
	void				partPtr(wsdl__part*);
	wsdl__message			*messagePtr() const;
	wsdl__part			*partPtr() const;
};

class soap__header
{ public:
	@xsd__QName			message;
 	@xsd__NMTOKEN			part;
	@enum soap__useChoice		use;
	@xsd__anyURI			encodingStyle;
	@xsd__anyURI			namespace_;
	std::vector<soap__headerfault>	headerfault;		// <soap:headerfault>*
  private:
  	wsdl__message			*messageRef;
  	wsdl__part			*partRef;
  public:
  	int				traverse(wsdl__definitions&);
	void				messagePtr(wsdl__message*);
	void				partPtr(wsdl__part*);
	wsdl__message			*messagePtr() const;
	wsdl__part			*partPtr() const;
};

class soap__address
{ public:
	@xsd__anyURI			location;
};

class wsoap__module
{ public:
	@xsd__anyURI			ref;
	@xsd__boolean			required = false;
};

class wsoap__header
{ public:
	@xsd__QName			element;
	@xsd__boolean			mustUnderstand_ = false;
	@xsd__boolean			required = false;
  private:
  	xs__element			*elementRef;
  public:
  	int				traverse(wsdl__definitions&);
	void				elementPtr(xs__element*);
	xs__element			*elementPtr() const;
        void                            mark();
};


