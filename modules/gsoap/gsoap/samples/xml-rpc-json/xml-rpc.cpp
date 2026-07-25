/*
        xml-rpc.cpp

        C++ API for XML-RPC/JSON data management

        Note: 

        XML-RPC declarations are located in the gSOAP header file xml-rpc.h,
        which is used to generate XML-RPC serializers (soapH.h, soapStub.h, and
        soapC.cpp) with:

        $ soapcpp2 -CSL xml-rpc.h

        To generate the soap_dup_value deep copy function use -Ec:

        $ soapcpp2 -Ec -CSL xml-rpc.h

        Iterators to walk the tree data are declared in xml-rpc-iters.h

        IO stream operations for XML-RPC data are declared in xml-rpc-io.h

        For more information please visit:

        http://www.genivia.com/doc/xml-rpc-json/html/

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2015, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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

#ifdef JSON_NAMESPACE
# include "jsonH.h"
# define SOAP_TYPE__boolean             SOAP_TYPE_json__boolean
# define SOAP_TYPE__i4                  SOAP_TYPE_json__i4
# define SOAP_TYPE__int                 SOAP_TYPE_json__int
# define SOAP_TYPE__double              SOAP_TYPE_json__double
# define SOAP_TYPE__dateTime_DOTiso8601 SOAP_TYPE_json__dateTime_DOTiso8601
# define SOAP_TYPE__string              SOAP_TYPE_json__string
# define SOAP_TYPE__array               SOAP_TYPE_json__array
# define SOAP_TYPE__struct              SOAP_TYPE_json__struct
# define SOAP_TYPE__base64              SOAP_TYPE_json__base64
# define SOAP_TYPE__rawdata             SOAP_TYPE_json__rawdata
#else
# include "soapH.h"
#endif

#ifdef JSON_NAMESPACE
namespace json {
#endif

static char * s_copy_l(struct soap *soap, const char *s);
static char * s_copy_l(struct soap *soap, const wchar_t *s);
static int size2k(int n);

value::value()
{
  soap_default_value(NULL, this);
}

value::value(const value& v)
{
  __type = v.__type;
  ref = v.ref;
  __any = v.__any;
  soap = v.soap;
}

value::value(struct soap *soap)
{
  soap_default_value(soap, this);
}

value::value(struct soap *soap, bool b)
{
  soap_default_value(soap, this);
  *this = b;
}

value::value(struct soap *soap, _double d)
{
  soap_default_value(soap, this);
  *this = d;
}

value::value(struct soap *soap, _i4 n)
{
  soap_default_value(soap, this);
  *this = n;
}

value::value(struct soap *soap, _int n)
{
  soap_default_value(soap, this);
  *this = n;
}

value::value(struct soap *soap, ULONG64 t)
{
  soap_default_value(soap, this);
  *this = t;
}

value::value(struct soap *soap, const char *s)
{
  soap_default_value(soap, this);
  *this = s;
}

value::value(struct soap *soap, const std::string& s)
{
  soap_default_value(soap, this);
  *this = s;
}

value::value(struct soap *soap, const wchar_t *s)
{
  soap_default_value(soap, this);
  *this = s;
}

value::value(struct soap *soap, const std::wstring& s)
{
  soap_default_value(soap, this);
  *this = s;
}

value::value(struct soap *soap, const _array& a)
{
  soap_default_value(soap, this);
  *this = a;
}

value::value(struct soap *soap, const _struct& r)
{
  soap_default_value(soap, this);
  *this = r;
}

value::value(struct soap *soap, const _base64& b)
{
  soap_default_value(soap, this);
  *this = b;
}

value::value(struct soap* soap, const _rawdata& b)
{
    soap_default_value(soap, this);
    *this = b;
}

value::operator bool() const
{
  return this->is_true();
}

value::operator _i4() const
{
  if (__type == SOAP_TYPE__boolean)
    return (_i4)*(_boolean*)ref;
  if (__type == SOAP_TYPE__i4)
    return (_i4)*(_i4*)ref;
  if (__type == SOAP_TYPE__int)
    return (_i4)*(_int*)ref;
  if (__type == SOAP_TYPE__double)
    return (_i4)(*(_double*)ref);
  int r = 0;
  const char *s = *this;
  soap_s2int(soap, s, &r);
  soap->error = SOAP_OK;
  return r;
}

value::operator _int() const
{
  if (__type == SOAP_TYPE__boolean)
    return (_int)*(_boolean*)ref;
  if (__type == SOAP_TYPE__i4)
    return (_int)*(_i4*)ref;
  if (__type == SOAP_TYPE__int)
    return (_int)*(_int*)ref;
  if (__type == SOAP_TYPE__double)
    return (_int)(*(_double*)ref);
  LONG64 r = 0;
  const char *s = *this;
  soap_s2LONG64(soap, s, &r);
  soap->error = SOAP_OK;
  return r;
}

value::operator _double() const
{
  if (__type == SOAP_TYPE__boolean)
    return (_double)*(_boolean*)ref;
  if (__type == SOAP_TYPE__i4)
    return (_double)*(_i4*)ref;
  if (__type == SOAP_TYPE__int)
    return (_double)*(_int*)ref;
  if (__type == SOAP_TYPE__double)
    return (_double)*(_double*)ref;
  double r = 0.0;
  const char *s = *this;
  soap_s2double(soap, s, &r);
  soap->error = SOAP_OK;
  return r;
}

value::operator char*() const
{
  if (__type == SOAP_TYPE__string || __type == SOAP_TYPE__dateTime_DOTiso8601 || __type == SOAP_TYPE__rawdata)
    return (char*)ref;
  if (__type == SOAP_TYPE__boolean)
    return (char*)(*(_boolean*)ref ? "true" : "false");
  if (__type == SOAP_TYPE__i4)
    return (char*)soap_strdup(soap, soap_int2s(soap, (int)*(_i4*)ref));
  if (__type == SOAP_TYPE__int)
    return (char*)soap_strdup(soap, soap_LONG642s(soap, (LONG64)*(_int*)ref));
  if (__type == SOAP_TYPE__double)
    return (char*)soap_strdup(soap, soap_double2s(soap, (double)*(_double*)ref));
  if (__type == SOAP_TYPE__base64)
    return (char*)soap_s2base64(soap, (unsigned char*)((_base64*)ref)->ptr(), NULL, ((_base64*)ref)->size());
  if (__type == SOAP_TYPE__rawdata)
    return (char*)soap_strdup(soap, (const char*)((_rawdata*)ref)->ptr());
  if (__any)
    return (char*)__any;
  if (!__type)
    return (char*)"null";
  return (char*)"";
}

value::operator std::string() const
{
  const char *s = *this;
  return std::string(s);
}

value::operator wchar_t*() const
{
  const char *s = *this;
  wchar_t *t = NULL;
  soap_s2wchar(soap, s, &t, 1, 0, -1, NULL);
  soap->error = SOAP_OK;
  return t;
}

value::operator std::wstring() const
{
  const char *s = *this;
  wchar_t *t = NULL;
  soap_s2wchar(soap, s, &t, 1, 0, -1, NULL);
  soap->error = SOAP_OK;
  return std::wstring(t);
}

value::operator ULONG64() const
{
  time_t t = 0;
  if (__type == SOAP_TYPE__string || __type == SOAP_TYPE__dateTime_DOTiso8601 || __type == SOAP_TYPE__rawdata)
  {
    if (soap_s2dateTime(soap, (const char*)ref, &t))
    {
      soap->error = SOAP_OK;
      return 0;
    }
  }
  return (ULONG64)t;
}

value::operator _array&()
{
  if (__type == SOAP_TYPE__array)
    return *(_array*)ref;
  _array *a = soap_new__array(soap);
  if (!a)
    throw std::bad_alloc();
  soap_default__array(soap, a);
  (*a)[0] = *this;
  return *a;
}

value::operator const _array&() const
{
  if (__type == SOAP_TYPE__array)
    return *(const _array*)ref;
  _array *a = soap_new__array(soap);
  if (!a)
    throw std::bad_alloc();
  soap_default__array(soap, a);
  (*a)[0] = *this;
  return *a;
}

value::operator _struct&()
{
  if (__type == SOAP_TYPE__struct)
    return *(_struct*)ref;
  _struct *r = soap_new__struct(soap);
  if (!r)
    throw std::bad_alloc();
  soap_default__struct(soap, r);
  return *r;
}

value::operator const _struct&() const
{
  if (__type == SOAP_TYPE__struct)
    return *(const _struct*)ref;
  _struct *r = soap_new__struct(soap);
  if (!r)
    throw std::bad_alloc();
  soap_default__struct(soap, r);
  return *r;
}

value::operator _base64&()
{
  if (__type == SOAP_TYPE__base64)
    return *(_base64*)ref;
  _base64 *base64 = soap_new__base64(soap);
  if (base64)
  {
    soap_default__base64(soap, base64);
    char *s = *this;
    base64->__ptr = (unsigned char*)s;
    base64->__size = (int)strlen(s);
  }
  return *base64;
}

value::operator const _base64&() const
{
  if (__type == SOAP_TYPE__base64)
    return *(const _base64*)ref;
  _base64 *base64 = soap_new__base64(soap);
  if (base64)
  {
    soap_default__base64(soap, base64);
    const char *s = *this;
    base64->__ptr = (unsigned char*)s;
    base64->__size = (int)strlen(s);
  }
  return *base64;
}

value::operator _rawdata&()
{
  if (__type == SOAP_TYPE__rawdata)
    return *(_rawdata*)ref;
  _rawdata *rawdata = soap_new__rawdata(soap);
  if (rawdata)
  {
    soap_default__rawdata(soap, rawdata);
    char *s = *this;
    rawdata->__ptr = (unsigned char*)s;
    rawdata->__size = (int)strlen(s);
  }
  return *rawdata;
}

value::operator const _rawdata&() const
{
  if (__type == SOAP_TYPE__rawdata)
    return *(const _rawdata*)ref;
  _rawdata *rawdata = soap_new__rawdata(soap);
  if (rawdata)
  {
    soap_default__rawdata(soap, rawdata);
    char *s = *this;
    rawdata->__ptr = (unsigned char*)s;
    rawdata->__size = (int)strlen(s);
  }
  return *rawdata;
}

value& value::operator[](int n)
{
  if (__type == SOAP_TYPE__struct)
    return ((_struct*)ref)->operator[](n);
  _array& a = *this;
  __type = SOAP_TYPE__array;
  __any = NULL;
  ref = &a;
  return a[n];
}

value& value::operator[](const char *s)
{
  _struct& r = *this;
  __type = SOAP_TYPE__struct;
  __any = NULL;
  ref = &r;
  return r[s];
}

value& value::operator[](const std::string& s)
{
  _struct& r = *this;
  __type = SOAP_TYPE__struct;
  __any = NULL;
  ref = &r;
  return r[s.c_str()];
}

value& value::operator[](const wchar_t *s)
{
  _struct& r = *this;
  __type = SOAP_TYPE__struct;
  __any = NULL;
  ref = &r;
  return r[s];
}

value& value::operator[](const std::wstring& s)
{
  _struct& r = *this;
  __type = SOAP_TYPE__struct;
  __any = NULL;
  ref = &r;
  return r[s.c_str()];
}

const value& value::operator[](int n) const
{
  if (__type == SOAP_TYPE__struct)
    return ((const _struct*)ref)->operator[](n);
  if (__type == SOAP_TYPE__array)
    return ((const _array*)ref)->operator[](n);
  return *new_value(soap);
}

const value& value::operator[](const char *s) const
{
  if (__type == SOAP_TYPE__struct)
    return ((const _struct*)ref)->operator[](s);
  return *new_value(soap);
}

const value& value::operator[](const std::string& s) const
{
  if (__type == SOAP_TYPE__struct)
    return ((const _struct*)ref)->operator[](s.c_str());
  return *new_value(soap);
}

const value& value::operator[](const wchar_t *s) const
{
  if (__type == SOAP_TYPE__struct)
    return ((const _struct*)ref)->operator[](s);
  return *new_value(soap);
}

const value& value::operator[](const std::wstring& s) const
{
  if (__type == SOAP_TYPE__struct)
    return ((const _struct*)ref)->operator[](s.c_str());
  return *new_value(soap);
}

bool value::operator=(bool b)
{
  __type = SOAP_TYPE__boolean;
  __any = NULL;
  ref = soap_malloc(soap, sizeof(_boolean));
  if (ref)
    *(_boolean*)ref = (_boolean)b;
  return b;
}

_i4 value::operator=(_i4 n)
{
  __type = SOAP_TYPE__i4;
  __any = NULL;
  ref = soap_malloc(soap, sizeof(_i4));
  if (ref)
    *(_i4*)ref = n;
  return n;
}

_int value::operator=(_int n)
{
  __type = SOAP_TYPE__int;
  __any = NULL;
  ref = soap_malloc(soap, sizeof(_int));
  if (ref)
    *(_int*)ref = n;
  return n;
}

_double value::operator=(_double d)
{
  __type = SOAP_TYPE__double;
  __any = NULL;
  ref = soap_malloc(soap, sizeof(_double));
  if (ref)
    *(_double*)ref = (_double)d;
  return d;
}

const char * value::operator=(const char *s)
{
  __type = SOAP_TYPE__string;
  __any = NULL;
  ref = s_copy_l(soap, s);
  return s;
}

char * value::operator=(char *s)
{
  __type = SOAP_TYPE__string;
  __any = NULL;
  ref = s_copy_l(soap, s);
  return s;
}

char * value::operator=(const std::string& s)
{
  __type = SOAP_TYPE__string;
  __any = NULL;
  ref = s_copy_l(soap, s.c_str());
  return (char*)ref;
}

const char * value::operator=(const wchar_t *s)
{
  __type = SOAP_TYPE__string;
  __any = NULL;
  ref = s_copy_l(soap, s);
  return (char*)ref;
}

char * value::operator=(wchar_t *s)
{
  __type = SOAP_TYPE__string;
  __any = NULL;
  ref = s_copy_l(soap, s);
  return (char*)ref;
}

char * value::operator=(const std::wstring& s)
{
  __type = SOAP_TYPE__string;
  __any = NULL;
  ref = s_copy_l(soap, s.c_str());
  return (char*)ref;
}

ULONG64 value::operator=(ULONG64 t)
{
  __type = SOAP_TYPE__dateTime_DOTiso8601;
  __any = NULL;
  ref = soap_strdup(soap, soap_dateTime2s(soap, (time_t)t));
  return t;
}

_array& value::operator=(const _array& a)
{
  __type = SOAP_TYPE__array;
  __any = NULL;
  ref = soap_new__array(soap);
  if (!ref)
    throw std::bad_alloc();
  soap_default__array(soap, (_array*)ref);
  *(_array*)ref = a;
  return *(_array*)ref;
}

_struct& value::operator=(const _struct& r)
{
  __type = SOAP_TYPE__struct;
  __any = NULL;
  ref = soap_new__struct(soap);
  if (!ref)
    throw std::bad_alloc();
  soap_default__struct(soap, (_struct*)ref);
  *(_struct*)ref = r;
  return *(_struct*)ref;
}

_base64& value::operator=(const _base64& b)
{
  __type = SOAP_TYPE__base64;
  __any = NULL;
  ref = soap_new__base64(soap);
  if (!ref)
    throw std::bad_alloc();
  soap_default__base64(soap, (_base64*)ref);
  *(_base64*)ref = b;
  return *(_base64*)ref;
}

_rawdata& value::operator=(const _rawdata& b)
{
  __type = SOAP_TYPE__rawdata;
  __any = NULL;
  ref = soap_new__rawdata(soap);
  if (!ref)
    throw std::bad_alloc();
  soap_default__rawdata(soap, (_rawdata*)ref);
  *(_rawdata*)ref = b;
  return *(_rawdata*)ref;
}

value& value::operator=(const value& v)
{
  __type = v.__type;
  ref = v.ref;
  __any = v.__any;
  soap = v.soap;
  return *this;
}

void value::size(int n)
{
  if (__type == SOAP_TYPE__array)
  {
    ((_array*)ref)->size(n);
  }
  else
  {
    __type = SOAP_TYPE__array;
    __any = NULL;
    ref = soap_new__array(soap);
    if (!ref)
      throw std::bad_alloc();
    soap_default__array(soap, (_array*)ref);
    ((_array*)ref)->size(n);
  }
}

int value::size() const
{
  if (__type == SOAP_TYPE__array)
    return ((_array*)ref)->size();
  if (__type == SOAP_TYPE__struct)
    return ((_struct*)ref)->size();
  return 0;
}

bool value::empty() const
{
  if (__type == SOAP_TYPE__array)
    return ((_array*)ref)->empty();
  if (__type == SOAP_TYPE__struct)
    return ((_struct*)ref)->empty();
  return true;
}

int value::nth(int n) const
{
  if (__type == SOAP_TYPE__array)
  {
    if (n < 0)
      return n + size();
    if (n < size())
      return n;
  }
  return -1;
}

int value::nth(const char *s) const
{
  if (s != NULL && __type == SOAP_TYPE__struct)
  {
    const _struct *r = (const _struct*)ref;
    for (int i = 0; i < r->__size; i++)
      if (!strcmp(r->member[i].name, s))
        return i;
  }
  return -1;
}

int value::nth(const wchar_t *s) const
{
  if (s == NULL)
    return -1;
  const char *t = soap_wchar2s(NULL, s);
  int i = nth(t);
  free((void*)t);
  return i;
}

bool value::has(int n) const
{
  if (__type == SOAP_TYPE__array)
    return n < 0 ? n + size() >= 0 : n < size();
  return false;
}

bool value::has(const char *s) const
{
  if (s != NULL && __type == SOAP_TYPE__struct)
  {
    const _struct *r = (const _struct*)ref;
    for (int i = 0; i < r->__size; i++)
      if (!strcmp(r->member[i].name, s))
        return true;
  }
  return false;
}

bool value::has(const wchar_t *s) const
{
  if (s == NULL)
    return false;
  const char *t = soap_wchar2s(NULL, s);
  bool b = has(t);
  free((void*)t);
  return b;
}

bool value::is_null() const
{
  return __type == 0 && !(__any && *__any);
}

bool value::is_bool() const
{
  return __type == SOAP_TYPE__boolean;
}

bool value::is_true() const
{
  return __type == SOAP_TYPE__boolean && (bool)*(_boolean*)ref == true;
}

bool value::is_false() const
{
  return __type == SOAP_TYPE__boolean && (bool)*(_boolean*)ref == false;
}

bool value::is_int() const
{
  return __type == SOAP_TYPE__i4 || __type == SOAP_TYPE__int;
}

bool value::is_double() const
{
  return __type == SOAP_TYPE__double;
}

bool value::is_number() const
{
  return __type == SOAP_TYPE__i4 || __type == SOAP_TYPE__int || __type == SOAP_TYPE__double;
}

bool value::is_string() const
{
  return __type == SOAP_TYPE__string || (__any && *__any);
}

bool value::is_dateTime() const
{
  return __type == SOAP_TYPE__dateTime_DOTiso8601;
}

bool value::is_array() const
{
  return __type == SOAP_TYPE__array;
}

bool value::is_struct() const
{
  return __type == SOAP_TYPE__struct;
}

bool value::is_base64() const
{
  return __type == SOAP_TYPE__base64;
}

bool value::is_rawdata() const
{
  return __type == SOAP_TYPE__rawdata;
}

value_iterator value::begin()
{
  if (__type == SOAP_TYPE__struct)
    return value_iterator(this, ((_struct*)ref)->member);
  return value_iterator(this);
}

value_iterator value::end()
{
  if (__type == SOAP_TYPE__struct)
    return value_iterator(this, ((_struct*)ref)->member + ((_struct*)ref)->__size);
  if (__type == SOAP_TYPE__array)
    return value_iterator(this, ((_array*)ref)->data.__size);
  return value_iterator();
}

bool value_const_iterator::operator==(const value_iterator& that) const
{
  if (_value == NULL)
    return that._value == NULL;
  if (that._value == NULL)
    return _value == NULL;
  if (_value->__type != that._value->__type)
    return false;
  if (_value->__type == SOAP_TYPE__struct)
    return _member == that._member;
  if (_value->__type == SOAP_TYPE__array)
    return (((_array*)_value->ref)->data.value + _index) == (((_array*)that._value->ref)->data.value + that._index);
  return _value == that._value;
}

static char * s_copy_l(struct soap *soap, const char *s)
{
  return soap_strdup(soap, s);
}

static char * s_copy_l(struct soap *soap, const wchar_t *s)
{
  return (char*)soap_wchar2s(soap, s);
}

_struct::_struct()
{
  soap_default__struct(NULL, this);
}

_struct::_struct(const _struct& r)
{
  __size = r.__size;
  member = r.member;
  soap = r.soap;
}

_struct::_struct(struct soap *soap)
{
  soap_default__struct(soap, this);
}

_struct::_struct(struct soap *soap, int size)
{
  soap_default__struct(soap, this);
  if (size > 0)
  {
    __size = size;
    member = soap_new_member(soap, __size);
    if (!member)
      throw std::bad_alloc();
    for (int i = 0; i < __size; i++)
      soap_default_member(soap, &member[i]);
  }
}

_struct& _struct::operator=(const struct _struct& r)
{
  __size = r.__size;
  member = r.member;
  soap = r.soap;
  return *this;
}

bool _struct::empty() const
{
  return __size == 0;
}

int _struct::size() const
{
  return __size;
}

value& _struct::operator[](int n) const
{
  if (member == NULL || __size == 0)
    return *new_value(soap);
  if (n < 0)
  {
    n += __size;
    if (n < 0)
      n = 0;
  }
  if (n >= __size)
    n = __size - 1;
  return member[n].value;
}

value& _struct::operator[](const char *s)
{
  int i = 0;
  if (s == NULL)
    s = "";
  if (member == NULL)
  {
    int newsize = size2k(__size = 1);
    member = soap_new_member(soap, newsize);
    if (!member)
      throw std::bad_alloc();
    for (i = 0; i < newsize; i++)
      soap_default_member(soap, &member[i]);
  }
  else
  {
    for (i = 0; i < __size; i++)
      if (!strcmp(member[i].name, s))
        return member[i].value;
    int oldsize = size2k(__size);
    int newsize = size2k(++__size);
    if (oldsize < newsize)
    {
      struct member *newmember = soap_new_member(soap, newsize);
      if (!newmember)
        throw std::bad_alloc();
      for (i = 0; i < oldsize; i++)
        newmember[i] = member[i];
      for (; i < newsize; i++)
        soap_default_member(soap, &newmember[i]);
      (void)soap_unlink(soap, member);
      delete[] member;
      member = newmember;
    }
  }
  i = __size - 1;
  member[i].name = soap_strdup(soap, s);
  soap_default_value(soap, &member[i].value);
  return member[i].value;
}

value& _struct::operator[](const wchar_t *s)
{
  const char *t = soap_wchar2s(NULL, s);
  value& r = operator[](t);
  free((void*)t);
  return r;
}

const value& _struct::operator[](const char *s) const
{
  if (s == NULL)
    s = "";
  if (member != NULL)
    for (int i = 0; i < __size; i++)
      if (!strcmp(member[i].name, s))
        return member[i].value;
  return *new_value(soap);
}

const value& _struct::operator[](const wchar_t *s) const
{
  const char *t = soap_wchar2s(NULL, s);
  const value& r = operator[](t);
  free((void*)t);
  return r;
}

_struct_iterator _struct::begin() const
{
  return _struct_iterator(this);
}

_struct_iterator _struct::end() const
{
  _struct_iterator i(this);
  i += __size;
  return i;
}

bool _struct_const_iterator::operator==(const _struct_iterator& that) const
{
  return _member == that._member;
}

_array::_array()
{
  soap_default__array(NULL, this);
}

_array::_array(const _array& a)
{
  data = a.data;
  soap = a.soap;
}

_array::_array(struct soap *soap)
{
  soap_default__array(soap, this);
}

_array::_array(struct soap *soap, int size)
{
  soap_default__array(soap, this);
  if (size > 0)
  {
    data.__size = size;
    data.value = soap_new_value(soap, data.__size);
    if (!data.value)
      throw std::bad_alloc();
    for (int i = 0; i < data.__size; i++)
      soap_default_value(soap, &data.value[i]);
  }
}

_array& _array::operator=(const struct _array& a)
{
  data = a.data;
  soap = a.soap;
  return *this;
}

bool _array::empty() const
{
  return data.__size == 0;
}

int _array::size() const
{
  return data.__size;
}

void _array::size(int n)
{
  if (n < 0)
    n += data.__size;
  if (n >= 0)
  {
    if (data.__size >= n)
      data.__size = n;
    else
      (void)(*this)[n - 1];
  }
}

value& _array::operator[](int n)
{
  if (n < 0)
  {
    n += data.__size;
    if (n < 0)
      n = 0;
  }
  if (data.value == NULL)
  {
    int newsize = size2k(data.__size = n + 1);
    data.value = soap_new_value(soap, newsize);
    if (!data.value)
      throw std::bad_alloc();
    for (int i = 0; i < newsize; i++)
      soap_default_value(soap, &data.value[i]);
  }
  else if (data.__size <= n)
  {
    int oldsize = size2k(data.__size);
    int newsize = size2k(data.__size = n + 1);
    if (oldsize < newsize)
    {
      value *newvalue = soap_new_value(soap, newsize);
      if (!newvalue)
        throw std::bad_alloc();
      int i;
      for (i = 0; i < oldsize; i++)
        newvalue[i] = data.value[i];
      for (; i < newsize; i++)
        soap_default_value(soap, &newvalue[i]);
      (void)soap_unlink(soap, data.value);
      delete[] data.value;
      data.value = newvalue;
    }
  }
  return data.value[n];
}

const value& _array::operator[](int n) const
{
  if (n < 0)
  {
    n += data.__size;
    if (n < 0)
      n = 0;
  }
  if (data.value != NULL && data.__size > n)
    return data.value[n];
  return *new_value(soap);
}

_array_iterator _array::begin() const
{
  return _array_iterator(this);
}

_array_iterator _array::end() const
{
  _array_iterator i(this);
  i += data.__size;
  return i;
}

bool _array_const_iterator::operator==(const _array_iterator& that) const
{
  return _value == that._value;
}

_base64::_base64()
{ }

_base64::_base64(struct soap *soap)
{
  soap_default__base64(soap, this);
}

_base64::_base64(struct soap *soap, int n, unsigned char *p)
{
  soap_default__base64(soap, this);
  __size = n;
  __ptr = p;
}

int _base64::size() const
{
  return __size;
}

unsigned char * _base64::ptr()
{
  return __ptr;
}

void _base64::size(int n)
{
  __size = n;
}

void _base64::ptr(unsigned char *p)
{
  __ptr = p;
}

_rawdata::_rawdata()
{ }

_rawdata::_rawdata(struct soap *soap)
{
  soap_default__rawdata(soap, this);
}

_rawdata::_rawdata(struct soap *soap, char *p)
{
  soap_default__rawdata(soap, this);
  __size = (int)strlen(p);
  __ptr = (unsigned char*)p;
}

_rawdata::_rawdata(struct soap *soap, int n, char *p)
{
  soap_default__rawdata(soap, this);
  __size = n;
  __ptr = (unsigned char*)p;
}

int _rawdata::size() const
{
  return __size;
}

char * _rawdata::ptr()
{
  return (char*)__ptr;
}

void _rawdata::size(int n)
{
  __size = n;
}

void _rawdata::ptr(char *p)
{
  __ptr = (unsigned char*)p;
}

params::params()
{ }

params::params(struct soap *soap)
{
  soap_default_params(soap, this);
}

params::params(struct soap *soap, int size)
{
  soap_default_params(soap, this);
  if (size > 0)
  {
    __size = size;
    param = soap_new_param(soap, __size);
    if (!param)
      throw std::bad_alloc();
    for (int i = 0; i < __size; i++)
      soap_default_param(soap, &param[i]);
  }
}

bool params::empty() const
{
  return __size == 0;
}

int params::size() const
{
  return __size;
}

value& params::operator[](int n)
{
  if (n < 0)
  {
    n += __size;
    if (n < 0)
      n = 0;
  }
  if (param == NULL)
  {
    int newsize = size2k(__size = n + 1);
    param = soap_new_param(soap, newsize);
    if (!param)
      throw std::bad_alloc();
    for (int i = 0; i < newsize; i++)
      soap_default_param(soap, &param[i]);
  }
  else if (__size <= n)
  {
    int oldsize = size2k(__size);
    int newsize = size2k(__size = n + 1);
    if (oldsize < newsize)
    {
      struct param *newparam = soap_new_param(soap, newsize);
      if (!param)
        throw std::bad_alloc();
      int i;
      for (i = 0; i < oldsize; i++)
        newparam[i] = param[i];
      for (; i < newsize; i++)
        soap_default_param(soap, &newparam[i]);
      (void)soap_unlink(soap, param);
      delete[] param;
      param = newparam;
    }
  }
  return param[n].value;
}

const value& params::operator[](int n) const
{
  if (n < 0)
  {
    n += __size;
    if (n < 0)
      n = 0;
  }
  if (param != NULL && __size > n)
    return param[n].value;
  return *new_value(soap);
}

params_iterator params::begin() const
{
  return params_iterator(this);
}

params_iterator params::end() const
{
  params_iterator i(this);
  i += __size;
  return i;
}

bool params_const_iterator::operator==(const params_iterator& that) const
{
  return _param == that._param;
}

methodCall::methodCall()
{ }

methodCall::methodCall(struct soap *soap)
{
  soap_default_methodCall(soap, this);
}

methodCall::methodCall(struct soap *soap, const char *endpoint, const char *name)
{
  soap_default_methodCall(soap, this);
  methodName = soap_strdup(soap, name);
  methodEndpoint = soap_strdup(soap, endpoint);
  methodResponse = NULL;
}

value& methodCall::operator[](int n)
{
  return params[n];
}

const value& methodCall::operator[](int n) const
{
  return params[n];
}

struct params& methodCall::operator()()
{
  if (send() == SOAP_OK)
  {
    if (methodResponse == NULL)
      methodResponse = soap_new_methodResponse(soap);
    if (methodResponse == NULL)
      throw std::bad_alloc();
    if (methodResponse->recv() != SOAP_OK)
      methodResponse = NULL;
  }
  else
    methodResponse = NULL;
  soap_closesock(soap);
  if (methodResponse && methodResponse->params)
    return *methodResponse->params;
  struct params *params = soap_new_params(soap);
  if (!params)
    throw std::bad_alloc();
  soap_default_params(soap, params);
  return *params;
}

struct params& methodCall::operator()(const struct params& args)
{
  /* parameters */
  params = args;
  /* invoke */
  return (*this)();
}

struct params& methodCall::response()
{
  if (methodResponse == NULL)
  {
    methodResponse = soap_new_methodResponse(soap);
    if (!methodResponse)
      throw std::bad_alloc();
    soap_default_methodResponse(soap, methodResponse);
  }
  if (methodResponse->params == NULL)
  {
    methodResponse->params = soap_new_params(soap);
    if (!methodResponse->params)
      throw std::bad_alloc();
    soap_default_params(soap, methodResponse->params);
  }
  return *methodResponse->params;
}

value& methodCall::fault()
{
  if (methodResponse)
    return methodResponse->get_fault();
  return *new_value(soap);
}

const char * methodCall::name() const
{
  if (methodName)
    return methodName;
  return "";
}

int methodCall::error() const
{
  return soap->error;
}

int methodCall::send()
{
  /* no namespaces */
  soap->namespaces = NULL;
  /* no SOAP encodingStyle */
  soap->encodingStyle = NULL;
  /* content length (needed for HTTP non-chunked) */
  soap_begin_count(soap);
  if (soap->mode & SOAP_IO_LENGTH)
    soap_put_methodCall(soap, this, "methodCall", NULL);
  soap_end_count(soap);
  /* connect and send request */
  if (soap_connect(soap, methodEndpoint, NULL)
   || soap_put_methodCall(soap, this, "methodCall", NULL)
   || soap_end_send(soap))
    return soap->error;
  return SOAP_OK;
}

int methodCall::recv()
{
  if (soap_begin_recv(soap)
   || !soap_get_methodCall(soap, this, "methodCall", NULL)
   || soap_end_recv(soap))
    return soap->error;
  return SOAP_OK;
}

methodResponse::methodResponse()
{ }

methodResponse::methodResponse(struct soap *soap)
{
  soap_default_methodResponse(soap, this);
}

value& methodResponse::operator[](int n)
{
  if (params == NULL)
  {
    params = soap_new_params(soap);
    if (!params)
      throw std::bad_alloc();
    soap_default_params(soap, params);
  }
  return (*params)[n];
}

const value& methodResponse::operator[](int n) const
{
  if (params != NULL)
    return (*params)[n];
  return *new_value(soap);
}

value& methodResponse::get_fault()
{
  if (fault == NULL)
  {
    fault = soap_new_fault(soap);
    if (!fault)
      throw std::bad_alloc();
    soap_default_fault(soap, fault);
  }
  return fault->value;
}

value& methodResponse::set_fault(const char *s)
{
  value *value = new_value(soap);
  *value = s;
  return get_fault() = *value;
}

value& methodResponse::set_fault(value& v)
{
  return get_fault() = v;
}

int methodResponse::send()
{
  /* no namespaces */
  soap->namespaces = NULL;
  /* no SOAP encodingStyle */
  soap->encodingStyle = NULL;
  /* content length */
  soap_begin_count(soap);
  if (soap->mode & SOAP_IO_LENGTH)
    soap_put_methodResponse(soap, this, "methodResponse", NULL);
  soap_end_count(soap);
  /* send response */
  if (soap_response(soap, SOAP_OK)
   || soap_put_methodResponse(soap, this, "methodResponse", NULL)
   || soap_end_send(soap))
    return soap->error;
  return SOAP_OK;
}

int methodResponse::recv()
{
  if (soap_begin_recv(soap)
   || !soap_get_methodResponse(soap, this, "methodResponse", NULL)
   || soap_end_recv(soap))
    return soap->error;
  return SOAP_OK;
}

static int size2k(int n)
{
  int k = 2;
  while (k < n)
    k *= 2;
  return k;
}

value * new_value(struct soap *soap)
{
  value *v = soap_new_value(soap);
  if (!v)
    throw std::bad_alloc();
  return init_value(soap, v);
}

value * init_value(struct soap *soap, value *v)
{
  if (v)
    soap_default_value(soap, v);
  return v;
}

#ifdef JSON_NAMESPACE
} // namespace json
#endif
