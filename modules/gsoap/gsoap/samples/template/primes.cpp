/*
	primes.cpp

	Prime sieve example that demsontrates the use of a user-defined
	simple_vector container and auto-generated code to display the
	container contents in XML.

	Build:
	> soapcpp2 -CS primes.h
	> c++ -o primes primes.cpp soapC.cpp stdsoap2.cpp

	Usage:
	> ./primes

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2011, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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

// include all generated header files:
#include "soapH.h"
#include <iostream>

int main()
{
  primes p;     // also instantiates the 'soap' context
  p.sieve(100); // sieve primes

  primes q;
  q = p;	// to show that copy constructor and assignment are OK
  q.write();    // write primes in XML

  return 0;
}

// sieving primes in the simple_vector<> container:
void primes::sieve(int n)
{
  prime.clear();
  prime.insert(prime.end(), 1);
  prime.insert(prime.end(), 2);
  for (int i = 3; i <= n; i += 2)
  {
    bool composite = false;

    for (simple_vector<int>::const_iterator j = prime.begin(); j != prime.end(); ++j)
    {
      if (*j != 1 && i % *j == 0)
      { composite = true;
	break;
      }
    }
    if (!composite)
      prime.insert(prime.end(), i);
  }
}

// the writer uses the fact that the primes class inherits the context:
void primes::write()
{
  soap_set_omode(this, SOAP_XML_INDENT); // show with indentation please
  soap_write_primes(this, this); // soap_write_prime is generated
}

// the destructor cleans up the 'soap' context
primes::~primes()
{
  soap_destroy(this);
  soap_end(this);
}

// we need a dummy namespace table, even though we don't use XML namespaces:
SOAP_NMAC struct Namespace namespaces[] = { {NULL, NULL, NULL, NULL} };
