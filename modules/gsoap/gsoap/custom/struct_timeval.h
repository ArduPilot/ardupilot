/*
	struct_timeval.h

	Custom serializer for struct timeval as xsd:dateTime

	Because time_t (binds to xsd:dateTime) lacks fractional seconds, struct
	timeval can be used to represent microseconds since 1970-01-01.

	#import this file into your gSOAP .h file to enable struct timeval
	serialization and use the serializable xsd__dateTime type.

	The struct timeval represents dates since 1970-01-01 with microsecond
	precision:

	struct timeval {
		time_t       tv_sec;   // seconds since Jan. 1, 1970
		suseconds_t  tv_usec;  // and microseconds
	};

	To automate the wsdl2h-mapping of xsd:dateTime to struct timeval, add
	this line to the typemap.dat file:

	xsd__dateTime = #import "custom/struct_timeval.h" | xsd__dateTime

	The typemap.dat file is used by wsdl2h to map types (wsdl2h option -t).

	When using soapcpp2 option -q<name> or -p<name>, you must change
	struct_timeval.c as follows:

		#include "soapH.h"  ->  #include "nameH.h"

	Compiler and link your code with struct_timeval.c

gSOAP XML Web services tools
Copyright (C) 2000-2007, Robert van Engelen, Genivia Inc., All Rights Reserved.
This part of the software is released under ONE of the following licenses:
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
Copyright (C) 2000-2007, Robert van Engelen, Genivia, Inc., All Rights Reserved.
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

extern typedef struct timeval xsd__dateTime;
