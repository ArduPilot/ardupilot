/*
	imports.h

	Common XSD types and externs for gSOAP header file import

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

template <class T> class std::vector;

typedef char	*xsd__anyURI,
		*xsd__ID,
		*xsd__NCName,
		*xsd__NMTOKEN,
		*xsd__NMTOKENS,
		*xsd__QName,
		*xsd__token,
		*xsd__string;
typedef bool	xsd__boolean;

extern class std::ostream;
extern class std::istream;

#include "includes.h"

extern class MapOfStringToString;
extern class SetOfString;
