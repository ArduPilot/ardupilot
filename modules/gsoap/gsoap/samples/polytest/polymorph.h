/*
	polymorph.h

	Polymorphic object exchange example.

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

//gsoap ns service name:	polymorph
//gsoap ns service style:	rpc
//gsoap ns service encoding:	encoded
//gsoap ns service namespace:	http://websrv.cs.fsu.edu/~engelen/polymorph.wsdl
//gsoap ns service location:	http://websrv.cs.fsu.edu/~engelen/polytest.cgi

//gsoap ns schema namespace: urn:copy
class ns__Object
{ public:
  char *name;
  ns__Object();
  ns__Object(const char *name);
  virtual ~ns__Object();
  virtual void print();
};
class ns__Shape: public ns__Object
{ public:
  char *name; // test overriding, both the ns__Object:name and ns__Shape:name are encoded
  int sides;
  ns__Shape();
  ns__Shape(const char *name, int sides);
  virtual ~ns__Shape();
  virtual void print();
};
class ns__Square: public ns__Shape
{ public:
  char *name; // test overriding
  static const int sides = 4; // will not be endoded and decoded
  int size;
  ns__Square();
  ns__Square(const char *name, int size);
  virtual ~ns__Square();
  virtual void print();
};
class ns__List: public ns__Object // ns__List is a dynamic array
{ public:
  ns__Object **__ptr; // array of pointers to objects
  int __size;
  ns__List();
  ns__List(int size);
  virtual ~ns__List();
  virtual ns__Object*& operator[](int i);
  virtual void print();
};
ns__polytest(ns__Object *in, struct ns__polytestResponse { ns__Object *out; } &result);
