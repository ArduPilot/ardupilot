/*
	factorytest.h

	Client-side remote object factory definitions

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

//gsoap ns service name:	factorytest
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
//  Remote factory objects
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

////////////////////////////////////////////////////////////////////////////////
//
//  Remote factory method interfaces
//
////////////////////////////////////////////////////////////////////////////////

int ns__create(enum t__object object, char *name, enum t__status &status);

int ns__lookup(enum t__object object, char *name, enum t__status &status);

int ns__rename(char *name, enum t__status &status);

int ns__release(enum t__status &status);

////////////////////////////////////////////////////////////////////////////////
//
//  Rewote adder method interfaces
//
////////////////////////////////////////////////////////////////////////////////

int ns__set(double val, enum t__status &status);

int ns__get(double &val);

int ns__add(double val, enum t__status &status);

////////////////////////////////////////////////////////////////////////////////
//
//  Remote counter method interfaces
//
////////////////////////////////////////////////////////////////////////////////

int ns__inc(enum t__status &status);

