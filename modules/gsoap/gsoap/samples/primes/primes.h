/*
	primes.h

	Declarations for the soapcpp2 compiler to define a primes class derived
	from the 'soap' context and to define a simple_vector template.

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2010, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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

// declare the simple_vector<> template:
template <class T> class simple_vector;

// declare a primes class that inherits a context for SOAP/XML serialization:
class primes: struct soap
{ public:
    simple_vector<int> prime; // container of ints
    void sieve(int n);
    void write();
    virtual ~primes();
};

// #include is deferred to the generated code, which will then include the defs:
#include "simple_vector.h" // defines simple_vector<>
