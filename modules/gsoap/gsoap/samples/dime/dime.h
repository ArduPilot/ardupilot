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

//gsoap ns service name:	dime
//gsoap ns service style:	rpc
//gsoap ns service encoding:	encoded
//gsoap ns service namespace:	http://websrv.cs.fsu.edu/~engelen/dime.wsdl
//gsoap ns service location:	http://websrv.cs.fsu.edu/~engelen/dimesrv.cgi

//gsoap ns schema  namespace:	urn:dime

//gsoap ns schema type-documentation: Data contains DIME attachment data
class ns__Data
{ unsigned char *__ptr; /* points to data */
  int __size;		/* size of data */
  char *id;		/* dime attachment ID (set to NULL to obtain unique cid) */
  char *type;		/* dime attachment content type */
  char *options;	/* dime attachment options (optional) */
  ns__Data();
  struct soap *soap;	/* soap context that created this instance */
};

class arrayOfData	/* SOAP array of data */
{ ns__Data *__ptr;
  int __size;
  arrayOfData();
  arrayOfData(struct soap*, int);
  virtual ~arrayOfData();
  int size();
  void resize(int);
  ns__Data& operator[](int) const;
  struct soap *soap;
};

class arrayOfName	/* SOAP array of strings */
{ char **__ptr;
  int __size;
  arrayOfName();
  arrayOfName(struct soap*, int);
  virtual ~arrayOfName();
  int size();
  void resize(int);
  char*& operator[](int) const;
  struct soap *soap;
};
int ns__putData(arrayOfData *data, arrayOfName *names);
int ns__getData(arrayOfName *names, arrayOfData *data);
int ns__getImage(char *name, ns__Data &image);
