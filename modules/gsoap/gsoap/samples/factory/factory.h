/*
	factory.h

	Server-side remote object factory definitions

	Server code: factory.cpp

	This header file contains all the class declarations of the remote
	objects to support serialization of these objects for server-side
	state management (simple save/load operations are implemented).

	The remote object factory uses a lease-based system. Remote objects
	are purged from the pool when the lease expires (see LEASETERM in
	factory.cpp). Supports inheritance.

	Compile:
	soapcpp2 factory.h
	c++ -o factory factory.cpp stdsoap2.cpp soapC.cpp soapServer.cpp

	Run (e.g. in the background)
	factory <port>
	where <port> is a available port number, e.g. 18085

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

//gsoap ns service name:	factory
//gsoap ns service style:	rpc
//gsoap ns service encoding:	encoded
//gsoap ns service namespace:	http://websrv.cs.fsu.edu/~engelen/factory.wsdl
//gsoap ns service location:	http://localhost:18085

//gsoap ns schema namespace: urn:factoryService
//gsoap t schema namespace: urn:factoryTypes
//gsoap h schema namespace: urn:factoryHandles

////////////////////////////////////////////////////////////////////////////////
//
//  SOAP Header: used to exchange stateful object handles
//
////////////////////////////////////////////////////////////////////////////////

struct SOAP_ENV__Header
{ mustUnderstand unsigned int h__handle;
};

////////////////////////////////////////////////////////////////////////////////
//
//  Server-side root class
//
////////////////////////////////////////////////////////////////////////////////

enum t__object				// object types:
{ ROOT,					// t__root object
  ADDER,				// t__adder object
  COUNTER				// t__counter object
};

enum t__status				// remote object status:
{ FACTORY_OK,					// ok
  FACTORY_INVALID,				// invalid handle (wrong type of object or lease expired)
  FACTORY_NOTFOUND,				// lookup operation not successful
  FACTORY_RETRY					// cannot create new object: try later
};

class t__root
{ public:
  enum t__object object;		// object type
  char *name;				// object name for lookup operation (optional)
  unsigned int handle;			// internal handle
  time_t lease;				// lease expiration time
  t__root();
  virtual ~t__root();
  virtual void renew();			// to renew lease
};

////////////////////////////////////////////////////////////////////////////////
//
//  Server-side adder class derived from root
//
////////////////////////////////////////////////////////////////////////////////

class t__adder: public t__root
{ public:
  double val;				// current value of the adder
  void set(double val);			// to set the adder
  double get();				// to get value of the adder
  void add(double val);			// to add a value to the adder
};

////////////////////////////////////////////////////////////////////////////////
//
//  Server-side counter class derived from adder
//
////////////////////////////////////////////////////////////////////////////////

class t__counter: public t__adder
{ public:
  void inc();				// to increment the counter
};

////////////////////////////////////////////////////////////////////////////////
//
//  Remote factory method interfaces
//
////////////////////////////////////////////////////////////////////////////////

//gsoap ns service method-header-part: create h__handle
int ns__create(enum t__object object, char *name, enum t__status &status);

//gsoap ns service method-header-part: lookup h__handle
int ns__lookup(enum t__object object, char *name, enum t__status &status);

//gsoap ns service method-header-part: rename h__handle
int ns__rename(char *name, enum t__status &status);

//gsoap ns service method-header-part: release h__handle
int ns__release(enum t__status &status);

////////////////////////////////////////////////////////////////////////////////
//
//  Rewote adder method interfaces
//
////////////////////////////////////////////////////////////////////////////////

//gsoap ns service method-header-part: set h__handle
int ns__set(double val, enum t__status &status);

//gsoap ns service method-header-part: get h__handle
int ns__get(double &val);

//gsoap ns service method-header-part: add h__handle
int ns__add(double val, enum t__status &status);

////////////////////////////////////////////////////////////////////////////////
//
//  Remote counter method interfaces
//
////////////////////////////////////////////////////////////////////////////////

//gsoap ns service method-header-part: inc h__handle
int ns__inc(enum t__status &status);

