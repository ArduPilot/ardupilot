/*
        xml-rpc.c

        C API for XML-RPC/JSON data management

        Note: 

        XML-RPC declarations are located in the gSOAP header file xml-rpc.h,
        which is used to generate XML-RPC serializers (soapH.h, soapStub.h, and
        soapC.c) with:

        $ soapcpp2 -c -CSL xml-rpc.h

        To generate the soap_dup_value deep copy function use -Ec:

        $ soapcpp2 -c -Ec -CSL xml-rpc.h

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
#else
# include "soapH.h"
#endif

static int size2k(int n);

struct value * new_value(struct soap *soap)
{
  return init_value(soap, (struct value*)soap_malloc(soap, sizeof(struct value)));
}

struct value * init_value(struct soap *soap, struct value *v)
{
  if (v)
    soap_default_value(soap, v);
  return v;
}

_boolean * bool_of(struct value *v)
{
  if (v->__type != SOAP_TYPE__boolean)
    if ((v->ref = (void*)soap_malloc(v->soap, sizeof(_boolean))))
      *(_boolean*)v->ref = 0;
  v->__type = SOAP_TYPE__boolean;
  v->__any = NULL;
  return (_boolean*)v->ref;
}

_int * int_of(struct value *v)
{
  _int r = 0;
  if (v->__type == SOAP_TYPE__boolean)
    r = (_int)*(_boolean*)v->ref;
  else if (v->__type == SOAP_TYPE__i4)
    r = (_int)*(_i4*)v->ref;
  else if (v->__type == SOAP_TYPE__double)
    r = (_int)(*(_double*)v->ref);
  else if (v->__type == SOAP_TYPE__string)
  {
    soap_s2LONG64(v->soap, (const char*)v->ref, &r);
    v->soap->error = SOAP_OK;
  }
  if (v->__type != SOAP_TYPE__int)
    if ((v->ref = (void*)soap_malloc(v->soap, sizeof(_int))))
      *(_int*)v->ref = r;
  v->__type = SOAP_TYPE__int;
  v->__any = NULL;
  return (_int*)v->ref;
}

_double * double_of(struct value *v)
{
  _double r = 0;
  if (v->__type == SOAP_TYPE__boolean)
    r = (_double)*(_boolean*)v->ref;
  else if (v->__type == SOAP_TYPE__i4)
    r = (_double)*(_i4*)v->ref;
  else if (v->__type == SOAP_TYPE__int)
    r = (_double)(*(_int*)v->ref);
  else if (v->__type == SOAP_TYPE__string)
  {
    soap_s2double(v->soap, (const char*)v->ref, &r);
    v->soap->error = SOAP_OK;
  }
  if (v->__type != SOAP_TYPE__double)
    if ((v->ref = (void*)soap_malloc(v->soap, sizeof(_double))))
      *(_double*)v->ref = r;
  v->__type = SOAP_TYPE__double;
  v->__any = NULL;
  return (_double*)v->ref;
}

const char ** string_of(struct value *v)
{
  if (v->__type == SOAP_TYPE__string || v->__type == SOAP_TYPE__dateTime_DOTiso8601)
    return (const char**)&v->ref;
  if (v->__type == SOAP_TYPE__boolean)
    v->ref = (void*)(*(_boolean*)v->ref ? "true" : "false");
  else if (v->__type == SOAP_TYPE__i4)
    v->ref =  (void*)soap_strdup(v->soap, soap_int2s(v->soap, (int)*(_i4*)v->ref));
  else if (v->__type == SOAP_TYPE__int)
    v->ref =  (void*)soap_strdup(v->soap, soap_LONG642s(v->soap, (LONG64)*(_int*)v->ref));
  else if (v->__type == SOAP_TYPE__double)
    v->ref =  (void*)soap_strdup(v->soap, soap_double2s(v->soap, (double)*(_double*)v->ref));
  else if (v->__type == SOAP_TYPE__base64)
    v->ref =  (void*)soap_s2base64(v->soap, (unsigned char*)((struct _base64*)v->ref)->__ptr, NULL, ((struct _base64*)v->ref)->__size);
  else if (v->__type == SOAP_TYPE__rawdata)
    v->ref =  (void*)soap_strdup(v->soap, (const char*)((struct _rawdata*)v->ref)->__ptr);
  else if (v->__any)
    v->ref = (void*)v->__any;
  else
    v->ref = (void*)"null";
  v->__type = SOAP_TYPE__string;
  v->__any = NULL;
  return (const char**)&v->ref;
}

const char ** dateTime_of(struct value *v)
{
  if (v->__type != SOAP_TYPE__dateTime_DOTiso8601)
    v->ref = (void*)"";
  v->__type = SOAP_TYPE__dateTime_DOTiso8601;
  v->__any = NULL;
  return (const char**)&v->ref;
}

struct _base64 * base64_of(struct value *v)
{
  if (v->__type != SOAP_TYPE__base64)
    if ((v->ref = (void*)soap_malloc(v->soap, sizeof(struct _base64))))
      soap_default__base64(v->soap, (struct _base64*)v->ref);
  v->__type = SOAP_TYPE__base64;
  v->__any = NULL;
  return (struct _base64*)v->ref;
}

struct _rawdata * rawdata_of(struct value *v)
{
  if (v->__type != SOAP_TYPE__rawdata)
    if ((v->ref = (void*)soap_malloc(v->soap, sizeof(struct _rawdata))))
      soap_default__rawdata(v->soap, (struct _rawdata*)v->ref);
  v->__type = SOAP_TYPE__rawdata;
  v->__any = NULL;
  return (struct _rawdata*)v->ref;
}

struct value * value_at(struct value *v, const char *s)
{
  int i = 0;
  struct _struct *_struct = (struct _struct*)v->ref;
  if (s == NULL)
    s = "";
  if (v->__type != SOAP_TYPE__struct || !_struct)
  {
    v->ref = _struct = (struct _struct*)soap_malloc(v->soap, sizeof(struct _struct));
    if (!_struct)
      return NULL;
    soap_default__struct(v->soap, _struct);
  }
  v->__type = SOAP_TYPE__struct;
  v->__any = NULL;
  if (!_struct->member)
  {
    int newsize = size2k(_struct->__size = 1);
    _struct->member = (struct member*)soap_malloc(v->soap, sizeof(struct member) * newsize);
    if (!_struct->member)
      return NULL;
    for (i = 0; i < newsize; i++)
      soap_default_member(v->soap, &_struct->member[i]);
  }
  else
  {
    int oldsize, newsize;
    for (i = 0; i < _struct->__size; i++)
      if (!strcmp(_struct->member[i].name, s))
        return &_struct->member[i].value;
    oldsize = size2k(_struct->__size);
    newsize = size2k(++_struct->__size);
    if (oldsize < newsize)
    {
      struct member *newmember = (struct member*)soap_malloc(v->soap, sizeof(struct member) * newsize);
      if (!newmember)
        return NULL;
      (void)soap_memcpy((void*)newmember, sizeof(struct member) * newsize, (const void*)_struct->member, sizeof(struct member) * oldsize);
      for (i = oldsize; i < newsize; i++)
        soap_default_member(v->soap, &newmember[i]);
      soap_unlink(v->soap, _struct->member);
      free((void*)_struct->member);
      _struct->member = newmember;
    }
  }
  i = _struct->__size - 1;
  _struct->member[i].name = soap_strdup(_struct->soap, s);
  soap_default_value(v->soap, &_struct->member[i].value);
  return &_struct->member[i].value;
}

struct value * value_atw(struct value *v, const wchar_t *s)
{
  const char *t = soap_wchar2s(NULL, s);
  struct value *u = value_at(v, t);
  free((void*)t);
  return u;
}

struct member * nth_member(struct value *v, int n)
{
  struct _struct *_struct = (struct _struct*)v->ref;
  if (n < 0 && _struct)
  {
    n += _struct->__size;
    if (n < 0)
      n = 0;
  }
  if (v->__type != SOAP_TYPE__struct || !_struct || _struct->__size <= n || n < 0)
    return NULL;
  return &_struct->member[n];
}

struct value * nth_value(struct value *v, int n)
{
  if (v->__type == SOAP_TYPE__struct)
  {
    struct member *member = nth_member(v, n);
    if (!member)
      return new_value(v->soap);
    return &member->value;
  }
  else
  {
    int i = 0;
    struct data *data;
    if (v->__type != SOAP_TYPE__array || v->ref == NULL)
    {
      struct _array *_array = (struct _array*)soap_malloc(v->soap, sizeof(struct _array));
      if (!_array)
        return NULL;
      v->ref = _array;
      soap_default__array(v->soap, _array);
    }
    data = &((struct _array*)v->ref)->data;
    v->__type = SOAP_TYPE__array;
    v->__any = NULL;
    if (n < 0)
    {
      n += data->__size;
      if (n < 0)
        n = 0;
    }
    if (!data->value)
    {
      int newsize = size2k(data->__size = n + 1);
      data->value = (struct value*)soap_malloc(v->soap, sizeof(struct value) * newsize);
      if (!data->value)
        return NULL;
      for (i = 0; i < newsize; i++)
        soap_default_value(v->soap, &data->value[i]);
    }
    else if (data->__size <= n)
    {
      int oldsize = size2k(data->__size);
      int newsize = size2k(data->__size = n + 1);
      if (oldsize < newsize)
      {
        struct value *newvalue = (struct value*)soap_malloc(v->soap, sizeof(struct value) * newsize);
        if (!newvalue)
          return NULL;
        (void)soap_memcpy((void*)newvalue, sizeof(struct value) * newsize, (const void*)data->value, sizeof(struct value) * oldsize);
        for (i = oldsize; i < newsize; i++)
          soap_default_value(v->soap, &newvalue[i]);
        soap_unlink(v->soap, data->value);
        free((void*)data->value);
        data->value = newvalue;
      }
    }
    return &data->value[n];
  }
}

void set_struct(struct value *v)
{
  if (v->__type != SOAP_TYPE__struct || !v->ref)
    v->ref = (struct _struct*)soap_malloc(v->soap, sizeof(struct _struct));
  v->__type = SOAP_TYPE__struct;
  v->__any = NULL;
  if (v->ref)
    soap_default__struct(v->soap, (struct _struct*)v->ref);
}

void set_size(struct value *v, int n)
{
  if (v->__type == SOAP_TYPE__array && v->ref)
  {
    struct data *data = &((struct _array*)v->ref)->data;
    if (n < 0)
      n += data->__size;
    if (n >= 0)
    {
      if (data->__size >= n)
        data->__size = n;
      else
        (void)nth_value(v, n - 1);
    }
  }
  else
  {
    v->ref = (void*)soap_malloc(v->soap, sizeof(struct _array));
    if (!v->ref)
      return;
    v->__type = SOAP_TYPE__array;
    v->__any = NULL;
    soap_default__array(v->soap, (struct _array*)v->ref);
    if (n > 0)
      (void)nth_value(v, n - 1);
  }
}

int has_size(const struct value *v)
{
  if (v->__type == SOAP_TYPE__array && v->ref)
    return ((const struct _array*)v->ref)->data.__size;
  if (v->__type == SOAP_TYPE__struct && v->ref)
    return ((const struct _struct*)v->ref)->__size;
  return 0;
}

int is_empty(const struct value *v)
{
  if (v->__type == SOAP_TYPE__array && v->ref)
    return ((const struct _array*)v->ref)->data.__size == 0;
  if (v->__type == SOAP_TYPE__struct && v->ref)
    return ((const struct _struct*)v->ref)->__size == 0;
  return 1;
}

int nth_at(const struct value *v, const char *s)
{
  if (s == NULL)
    return -1;
  if (v->__type == SOAP_TYPE__struct)
  {
    const struct _struct *_struct = (const struct _struct*)v->ref;
    int i;
    if (s == NULL)
      s = "";
    if (_struct)
      for (i = 0; i < _struct->__size; i++)
        if (!strcmp(_struct->member[i].name, s))
          return i;
  }
  return -1;
}

int nth_atw(const struct value *v, const wchar_t *s)
{
  const char *t;
  int i;
  if (s == NULL)
    return -1;
  t = soap_wchar2s(NULL, s);
  i = nth_at(v, t);
  free((void*)t);
  return i;
}

int nth_nth(const struct value *v, int n)
{
  if (v->__type == SOAP_TYPE__array && v->ref)
  {
    int size = ((struct _array*)v->ref)->data.__size;
    return n < 0 ? n + size : n < size ? n : -1;
  }
  return -1;
}

_boolean is_null(const struct value *v)
{
  return v->__type == 0 && !(v->__any && *v->__any);
}

_boolean is_int(const struct value *v)
{
  return v->__type == SOAP_TYPE__i4 || v->__type == SOAP_TYPE__int;
}

_boolean is_double(const struct value *v)
{
  return v->__type == SOAP_TYPE__double;
}

_boolean is_number(const struct value *v)
{
  return v->__type == SOAP_TYPE__i4 || v->__type == SOAP_TYPE__int || v->__type == SOAP_TYPE__double;
}

_boolean is_string(const struct value *v)
{
  return v->__type == SOAP_TYPE__string || (v->__any && *v->__any);
}

_boolean is_bool(const struct value *v)
{
  return v->__type == SOAP_TYPE__boolean;
}

_boolean is_true(const struct value *v)
{
  return v->__type == SOAP_TYPE__boolean && *(_boolean*)v->ref;
}

_boolean is_false(const struct value *v)
{
  return v->__type == SOAP_TYPE__boolean && !*(_boolean*)v->ref;
}

_boolean is_array(const struct value *v)
{
  return v->__type == SOAP_TYPE__array;
}

_boolean is_struct(const struct value *v)
{
  return v->__type == SOAP_TYPE__struct;
}

_boolean is_dateTime(const struct value *v)
{
  return v->__type == SOAP_TYPE__dateTime_DOTiso8601;
}

_boolean is_base64(const struct value *v)
{
  return v->__type == SOAP_TYPE__base64;
}

_boolean is_rawdata(const struct value *v)
{
  return v->__type == SOAP_TYPE__rawdata;
}

struct params * new_params(struct soap *soap)
{
  struct params *p = soap_malloc(soap, sizeof(struct params));
  if (p)
    soap_default_params(soap, p);
  return p;
}

struct value * nth_param(struct params *p, int n)
{
  int i = 0;
  if (n < 0)
  {
    n += p->__size;
    if (n < 0)
      n = 0;
  }
  if (!p->param)
  {
    int newsize = size2k(p->__size = n + 1);
    p->param = (struct param*)soap_malloc(p->soap, sizeof(struct param) * newsize);
    if (!p->param)
      return NULL;
    for (i = 0; i < newsize; i++)
      soap_default_param(p->soap, &p->param[i]);
  }
  else if (p->__size <= n)
  {
    int oldsize = size2k(p->__size);
    int newsize = size2k(p->__size = n + 1);
    if (oldsize < newsize)
    {
      struct param *newparam = (struct param*)soap_malloc(p->soap, sizeof(struct param) * newsize);
      if (!newparam)
        return NULL;
      (void)soap_memcpy((void*)newparam, sizeof(struct param) * newsize, (const void*)p->param, sizeof(struct param) * oldsize);
      for (i = oldsize; i < newsize; i++)
        soap_default_param(p->soap, &newparam[i]);
      soap_unlink(p->soap, p->param);
      free((void*)p->param);
      p->param = newparam;
    }
  }
  return &p->param[n].value;
}

int call_method(struct soap *soap, const char *endpoint, const char *methodName, struct params *p, struct methodResponse *r)
{
  struct methodCall m;
  soap_default_methodCall(soap, &m);
  m.methodName = (char*)methodName;
  if (p)
  {
    m.params.__size = p->__size;
    m.params.param = p->param;
  }
  soap->namespaces = NULL; /* no namespaces */
  soap->encodingStyle = NULL; /* no SOAP encodingStyle */
  /* connect, send request and receive response */
  if (soap_connect(soap, endpoint, NULL)
   || soap_write_methodCall(soap, &m)
   || soap_read_methodResponse(soap, r))
    return soap_closesock(soap); /* closes socket and returns soap->error */
  soap_closesock(soap);
  return SOAP_OK;
}

static int size2k(int n)
{
  int k = 2;
  while (k < n)
    k *= 2;
  return k;
}
