/*
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

//gsoap ns1 service name:	magic Magic squares
//gsoap ns1 service style:	rpc
//gsoap ns1 service encoding:	encoded
//gsoap ns1 service namespace:	http://websrv.cs.fsu.edu/~engelen/magic.wsdl
//gsoap ns1 service location:	http://websrv.cs.fsu.edu/~engelen/magicserver.cgi

//gsoap ns1 schema namespace: urn:MagicSquare

typedef int xsd__int;

// SOAP encoded array of ints
class vector
{ public:
  xsd__int *__ptr;
  int __size;
  struct soap *soap;
  vector();
  vector(int);
  virtual ~vector();
  void resize(int);
  int& operator[](int) const;
};

// SOAP encoded array of arrays of ints
class matrix
{ public:
  vector *__ptr;
  int __size;
  struct soap *soap;
  matrix();
  matrix(int, int);
  virtual ~matrix();
  void resize(int, int);
  vector& operator[](int) const;
};

//gsoap ns1 service method: magic Compute magic square of given rank
//gsoap ns1 service method: magic::rank magic square matrix rank
int ns1__magic(xsd__int rank, matrix *result);
