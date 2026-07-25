/*
	varparam.cpp

	Example use of variable parameter lists with the full XML
	schema type hierarchy implemented as a C++ class hierarchy.

	Demonstrates the use of variable number of parameters and polymorphic
	parameters. The 'trick' is to use __size parameters. The __size fields
	can be used in structs/classes to embed arrays. Because a request
	message of a remote method is essentially a struct, the use of __size
	in parameters of a method has the effect of sending a variable number
	of parameters.

	Run the executable as client from the command line with one argument to
	test the polymorphic parameter exchange. Run with more arguments to
	send the arguments as a variable parameter list to the server.

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

#include "soapH.h"
#include "varparam.nsmap"

const char *endpoint = "http://websrv.cs.fsu.edu/~engelen/varparam.cgi";

#define N 100 // max number of parameters

int main(int argc, char **argv)
{
  struct soap soap;
  int n;
  soap_init(&soap);
  if (argc < 2)
  {
    soap_serve(&soap);
    soap_destroy(&soap);
    soap_end(&soap);
    return 0;
  }
  if (argc < 3)
  {
    struct ns__varPolyParamTestResponse r;
    xsd__anyType *p[N];	// array of polymorphic parameters
    p[0] = new xsd__anyURI((char*)endpoint);
    p[1] = new xsd__string(argv[1]);
    p[2] = new xsd__boolean(true);
    p[3] = new xsd__dateTime(time(NULL));
    p[4] = new xsd__double(1234567.89);
    p[5] = new xsd__base64Binary((char*)"encoded in base64");
    p[6] = new xsd__hexBinary((char*)"encoded in hex");
    p[7] = new array(4);
    (*p[7])[0] = new xsd__int(7);
    (*p[7])[1] = NULL;
    (*p[7])[2] = new xsd__token((char*)"x");
    (*p[7])[3] = p[1];
    p[8] = p[1];
    n = 9; // actual number of parameters
    if (soap_call_ns__varPolyParamTest(&soap, endpoint, NULL, n, p, r))
    {
      soap_print_fault(&soap, stderr);
    }
    else
    {
      std::cout << "Server has echoed:" << std::endl;
      for (int i = 0; i < r.__size; i++)
        r.param[i]->print(std::cout);
      std::cout << std::endl;
    }
    delete (*p[7])[0];
    delete (*p[7])[2];
    for (int i = 0; i <= 7; i++)
      delete p[i];
  }
  else
  {
    if (soap_call_ns__varStringParamTest(&soap, endpoint, "", argc, argv, n))
      soap_print_fault(&soap, stderr);
    else
      printf("Server has responded to %d strings\n", n);
  }
  soap_destroy(&soap);
  soap_end(&soap);
  soap_done(&soap);
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Remote method implementations
//
////////////////////////////////////////////////////////////////////////////////

int ns__varStringParamTest(struct soap *soap, int __size, char **param, int &return_)
{
  return_ = __size;
  return SOAP_OK;
}

int ns__varPolyParamTest(struct soap *soap, int __size, xsd__anyType **param, struct ns__varPolyParamTestResponse &out)
{
  out.__size = __size;
  out.param = param;
  return SOAP_OK;
}

////////////////////////////////////////////////////////////////////////////////
//
//  XSD schema class hierarchy
//
////////////////////////////////////////////////////////////////////////////////

xsd__anyType::xsd__anyType()
{
  soap = NULL;			// This will be set by gSOAP environment later (upon deserialization or calling soap_new_X()
}
xsd__anyType::~xsd__anyType()
{
  soap_unlink(soap, this);	// Let gSOAP not deallocate this object again if this object was explicitly removed
}
xsd__anyType*& xsd__anyType::operator[](int i)
{
  static xsd__anyType *p = this;	// trick: we don't expect the operator to be applied, but we need a method
  return p;
}
void xsd__anyType::print(std::ostream &s) const
{
  s << "<anyType>";
}
xsd__anySimpleType::xsd__anySimpleType()
{ }
xsd__anySimpleType::~xsd__anySimpleType()
{ }
void xsd__anySimpleType::print(std::ostream &s) const
{
  s << "<anySimpleType>";
}
xsd__anyURI::xsd__anyURI()
{ }
xsd__anyURI::~xsd__anyURI()
{ }
xsd__anyURI::xsd__anyURI(char *s)
{
  __item = s;
}
void xsd__anyURI::print(std::ostream &s) const
{
  s << "<anyURI=" << (__item ? __item : "(null)") << ">";
}
xsd__boolean::xsd__boolean()
{ }
xsd__boolean::~xsd__boolean()
{ }
xsd__boolean::xsd__boolean(bool b)
{
  __item = b;
}
void xsd__boolean::print(std::ostream &s) const
{
  s << "<boolean=" << __item << ">";
}
xsd__date::xsd__date()
{ }
xsd__date::~xsd__date()
{ }
xsd__date::xsd__date(char *s)
{
  __item = s;
}
void xsd__date::print(std::ostream &s) const
{
  s << "<date=" << (__item ? __item : "(null)") << ">";
}
xsd__dateTime::xsd__dateTime()
{ }
xsd__dateTime::~xsd__dateTime()
{ }
xsd__dateTime::xsd__dateTime(time_t t)
{
  __item = t;
}
void xsd__dateTime::print(std::ostream &s) const
{
  s << "<dateTime=" << __item << ">";
}
xsd__double::xsd__double()
{ }
xsd__double::~xsd__double()
{ }
xsd__double::xsd__double(double d)
{
  __item = d;
}
void xsd__double::print(std::ostream &s) const
{
  s << "<double=" << __item << ">";
}
xsd__duration::xsd__duration()
{ }
xsd__duration::~xsd__duration()
{ }
xsd__duration::xsd__duration(char *s)
{
  __item = s;
}
void xsd__duration::print(std::ostream &s) const
{
  s << "<duration=" << (__item ? __item : "(null)") << ">";
}
xsd__float::xsd__float()
{ }
xsd__float::~xsd__float()
{ }
xsd__float::xsd__float(float f)
{
  __item = f;
}
void xsd__float::print(std::ostream &s) const
{
  s << "<float=" << __item << ">";
}
xsd__time::xsd__time()
{ }
xsd__time::~xsd__time()
{ }
xsd__time::xsd__time(char *s)
{
  __item = s;
}
void xsd__time::print(std::ostream &s) const
{
  s << "<time=" << (__item ? __item : "(null)") << ">";
}
xsd__string::xsd__string()
{ }
xsd__string::~xsd__string()
{ }
xsd__string::xsd__string(char *s)
{
  __item = s;
}
void xsd__string::print(std::ostream &s) const
{
  s << "<string=" << (__item ? __item : "(null)") << ">";
}
xsd__normalizedString::xsd__normalizedString()
{ }
xsd__normalizedString::~xsd__normalizedString()
{ }
xsd__normalizedString::xsd__normalizedString(char *s)
{
  __item = s;
}
void xsd__normalizedString::print(std::ostream &s) const
{
  s << "<normalizedString=" << (__item ? __item : "(null)") << ">";
}
xsd__token::xsd__token()
{ }
xsd__token::~xsd__token()
{ }
xsd__token::xsd__token(char *s)
{
  __item = s;
}
void xsd__token::print(std::ostream &s) const
{
  s << "<token=" << (__item ? __item : "(null)") << ">";
}
xsd__decimal::xsd__decimal()
{ }
xsd__decimal::~xsd__decimal()
{ }
xsd__decimal::xsd__decimal(char *s)
{
  __item = s;
}
void xsd__decimal::print(std::ostream &s) const
{
  s << "<decimal=" << (__item ? __item : "(null)") << ">";
}
xsd__integer::xsd__integer()
{ }
xsd__integer::~xsd__integer()
{ }
xsd__integer::xsd__integer(char *s)
{
  __item = s;
}
void xsd__integer::print(std::ostream &s) const
{
  s << "<integer=" << (__item ? __item : "(null)") << ">";
}
xsd__nonPositiveInteger::xsd__nonPositiveInteger()
{ }
xsd__nonPositiveInteger::~xsd__nonPositiveInteger()
{ }
xsd__nonPositiveInteger::xsd__nonPositiveInteger(char *s)
{
  __item = s;
}
void xsd__nonPositiveInteger::print(std::ostream &s) const
{
  s << "<nonPositiveInteger=" << (__item ? __item : "(null)") << ">";
}
xsd__negativeInteger::xsd__negativeInteger()
{ }
xsd__negativeInteger::~xsd__negativeInteger()
{ }
xsd__negativeInteger::xsd__negativeInteger(char *s)
{
  __item = s;
}
void xsd__negativeInteger::print(std::ostream &s) const
{
  s << "<negativeInteger=" << (__item ? __item : "(null)") << ">";
}
xsd__nonNegativeInteger::xsd__nonNegativeInteger()
{ }
xsd__nonNegativeInteger::~xsd__nonNegativeInteger()
{ }
xsd__nonNegativeInteger::xsd__nonNegativeInteger(char *s)
{
  __item = s;
}
void xsd__nonNegativeInteger::print(std::ostream &s) const
{
  s << "<nonNegativeInteger=" << (__item ? __item : "(null)") << ">";
}
xsd__positiveInteger::xsd__positiveInteger()
{ }
xsd__positiveInteger::~xsd__positiveInteger()
{ }
xsd__positiveInteger::xsd__positiveInteger(char *s)
{
  __item = s;
}
void xsd__positiveInteger::print(std::ostream &s) const
{
  s << "<positiveInteger=" << (__item ? __item : "(null)") << ">";
}
xsd__long::xsd__long()
{ }
xsd__long::~xsd__long()
{ }
xsd__long::xsd__long(LONG64 ll)
{
  __item = ll;
}
void xsd__long::print(std::ostream &s) const
{
  s << "<long=" << __item << ">";
}
xsd__int::xsd__int()
{ }
xsd__int::~xsd__int()
{ }
xsd__int::xsd__int(int i)
{
  __item = i;
}
void xsd__int::print(std::ostream &s) const
{
  s << "<int=" << __item << ">";
}
xsd__short::xsd__short()
{ }
xsd__short::~xsd__short()
{ }
xsd__short::xsd__short(short h)
{
  __item = h;
}
void xsd__short::print(std::ostream &s) const
{
  s << "<short=" << __item << ">";
}
xsd__byte::xsd__byte()
{ }
xsd__byte::~xsd__byte()
{ }
xsd__byte::xsd__byte(char c)
{
  __item = c;
}
void xsd__byte::print(std::ostream &s) const
{
  s << "<byte=" << __item << ">";
}
xsd__unsignedLong::xsd__unsignedLong()
{ }
xsd__unsignedLong::~xsd__unsignedLong()
{ }
xsd__unsignedLong::xsd__unsignedLong(ULONG64 ull)
{
  __item = ull;
}
void xsd__unsignedLong::print(std::ostream &s) const
{
  s << "<unsignedLong=" << __item << ">";
}
xsd__unsignedInt::xsd__unsignedInt()
{ }
xsd__unsignedInt::~xsd__unsignedInt()
{ }
xsd__unsignedInt::xsd__unsignedInt(unsigned int ui)
{
  __item = ui;
}
void xsd__unsignedInt::print(std::ostream &s) const
{
  s << "<unsignedInt=" << __item << ">";
}
xsd__unsignedShort::xsd__unsignedShort()
{ }
xsd__unsignedShort::~xsd__unsignedShort()
{ }
xsd__unsignedShort::xsd__unsignedShort(unsigned short uh)
{
  __item = uh;
}
void xsd__unsignedShort::print(std::ostream &s) const
{
  s << "<unsignedShort=" << __item << ">";
}
xsd__unsignedByte::xsd__unsignedByte()
{ }
xsd__unsignedByte::~xsd__unsignedByte()
{ }
xsd__unsignedByte::xsd__unsignedByte(unsigned char uc)
{
  __item = uc;
}
void xsd__unsignedByte::print(std::ostream &s) const
{
  s << "<unsignedByte=" << __item << ">";
}
xsd__base64Binary::xsd__base64Binary()
{ }
xsd__base64Binary::~xsd__base64Binary()
{ }
xsd__base64Binary::xsd__base64Binary(char *s)
{
  __size = strlen(s)+1;
  __ptr = (unsigned char*)s;
}
void xsd__base64Binary::print(std::ostream &s) const
{
  s << "<base64Binary=" << __size << " bytes>";
}
xsd__hexBinary::xsd__hexBinary()
{ }
xsd__hexBinary::~xsd__hexBinary()
{ }
xsd__hexBinary::xsd__hexBinary(char *s)
{
  __size = strlen(s)+1;
  __ptr = (unsigned char*)s;
}
void xsd__hexBinary::print(std::ostream &s) const
{
  s << "<hexBinary=" << __size << " bytes>";
}
array::array()
{ }
array::~array()
{ }
array::array(int n)
{
  __size = n;
  __ptr = (xsd__anyType**)soap_malloc(soap, n*sizeof(xsd__anyType*));
}
xsd__anyType*& array::operator[](int i)
{
  return __ptr[i];
}
void array::print(std::ostream &s) const
{
  s << "<array=";
  for (int i = 0; i < __size; i++)
    if (__ptr[i])
      __ptr[i]->print(s);
    else
      s << "<[none]>";
  s << ">";
}
