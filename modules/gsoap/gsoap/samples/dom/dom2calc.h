/*
	dom2calc.h

	A gSOAP header file for the DOM example

	You can define C and C++ types to be serialized and deserialized
	automatically in a DOM object. To do this, enable the SOAP_DOM_NODE
	flag and make sure that either xsi:types attribute values or XML
	element/attribute tag names match the name of the type definition.

	The deserialization only works when xsi:type attributes are present in
	messages or when XML element or attribute tag names match the type
	name. A deserializer is chosen based on these tag names xsi:type
	attribute value. For example, when an element has an
	xsi:type="xsd:double" attribute and SOAP_DOM_NODE is set, then the
	xsd__double type is deserialized into the void *soap_dom_element::node
	(points to a double) and soap_dom_element::type==SOAP_TYPE_xsd__double.
	Likewise, when an element tag name is <result>, then the 'result'
	deserializer is invoked to populate void *soap_dom_element::node
	(points to double) and soap_dom_element::type==SOAP_TYPE_result.

	When declaring types for elements and attributes, it is advised to use
	a leading '_' to prevent them from being (de)serialized as types. The
	leading '_' suppresses the rendering of xsi:type. See the '_result'
	example below.

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2008, Robert van Engelen, Genivia, Inc. All Rights Reserved.
This software is released under one of the following two licenses:
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

#import "dom.h"

typedef double xsd__double;	/* matches xsi:type="xsd:double" */

typedef double _result;		/* matches the unqualified <result> tag */
