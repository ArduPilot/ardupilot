/*
	xsd.h

	Use #import "xsd.h" in a gSOAP header file to import predefined XSD
	types.

	Use wsdl2h option -m to automatically import these xsd.h definitions in
	the generated .h file.

gSOAP XML Web services tools
Copyright (C) 2000-2008, Robert van Engelen, Genivia Inc., All Rights Reserved.
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
Copyright (C) 2000-2008 Robert A. van Engelen, Genivia inc. All Rights Reserved.
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

typedef char *	_xml__lang;

typedef char *	_xsd__schema;

class xsd__anyType
{	_XML __item;
	struct soap *soap;
};

typedef char *	xsd__anyURI;

class xsd__base64Binary
{       unsigned char *__ptr;
        int __size;
        char *id, *type, *options;
        struct soap *soap;
};

class xsd__hexBinary
{       unsigned char *__ptr;
        int __size;
        struct soap *soap;                                    
};

typedef char		xsd__byte;
typedef char *		xsd__date;
typedef double  	xsd__decimal;
typedef double		xsd__double;
typedef char *		xsd__duration;
typedef float		xsd__float;
typedef char *		xsd__ID;
typedef LONG64		xsd__integer;
typedef char *		xsd__language;
typedef LONG64		xsd__long;
typedef char *		xsd__Name;
typedef char *		xsd__NCName;
typedef char *		xsd__NMTOKEN;
typedef LONG64		xsd__negativeInteger;
typedef unsigned LONG64	xsd__nonNegativeInteger;
typedef LONG64		xsd__nonPositiveInteger;
typedef char *		xsd__normalizedString;
typedef ULONG64		xsd__positiveInteger;
typedef char *		xsd__QName;
typedef char *		xsd__time;
typedef char *		xsd__token;
typedef short		xsd__short;
typedef unsigned char	xsd__unsignedByte;
typedef unsigned int	xsd__unsignedInt;
typedef unsigned LONG64	xsd__unsignedLong;
typedef unsigned short	xsd__unsignedShort;

typedef char *		_SOAP_ENC__actor;
typedef int		_SOAP_ENC__mustUnderstand;
