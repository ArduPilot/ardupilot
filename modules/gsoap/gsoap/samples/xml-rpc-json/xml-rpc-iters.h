/*
        xml-rpc-iters.h

        XML-RPC/JSON C++ API iterators

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

#ifndef XML_RPC_ITERS_H
#define XML_RPC_ITERS_H

#ifdef __cplusplus

#include <iterator>

#ifdef JSON_NAMESPACE
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
#endif

#ifdef JSON_NAMESPACE
namespace json {
#endif
class value_const_iterator;
class value_iterator;
class _struct_const_iterator;
class _struct_iterator;
class _array_const_iterator;
class _array_iterator;
class params_const_iterator;
class params_iterator;
#ifdef JSON_NAMESPACE
}
#endif

// force "cyclic" inclusion to obtain type declarations for definitions below
#ifdef JSON_NAMESPACE
#include "jsonStub.h"
#else
#include "soapStub.h"
#endif

#ifdef JSON_NAMESPACE
namespace json {
#endif

/// bidirectional const iterator over values (struct or array or one atomic value)
class value_const_iterator
{
  friend class value_iterator;
 private:
  struct value*                 _value;
  struct member*                _member;
  int                           _index;
 public:
  typedef ptrdiff_t difference_type;
  typedef struct value value_type;
  typedef struct value& reference;
  typedef struct value* pointer;
  typedef std::bidirectional_iterator_tag iterator_category;
                                value_const_iterator() : _value(NULL), _member(NULL), _index(0) { }
                                value_const_iterator(struct value* v) : _value(v),  _member(NULL), _index(0) { }
                                value_const_iterator(struct value* v, struct member* m) : _value(v), _member(m), _index(0) { }
                                value_const_iterator(struct value* v, int i) : _value(v), _member(NULL), _index(i) { }
  bool operator==(const value_iterator& that) const;
  bool operator==(const value_const_iterator& that) const
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
  bool operator!=(const value_iterator& that) const
  {
    return !operator==(that);
  }
  bool operator!=(const value_const_iterator& that) const
  {
    return !operator==(that);
  }
  const struct value& operator*() const
  {
    return *operator->();
  }
  const struct value *operator->() const
  {
    if (_value->__type == SOAP_TYPE__struct)
      return &_member->value;
    if (_value->__type == SOAP_TYPE__array)
      return ((_array*)_value->ref)->data.value + _index;
    return _value;
  }
  int index() const
  {
    if (_value->__type == SOAP_TYPE__struct)
      return (int)(_member - ((_struct*)_value->ref)->member);
    if (_value->__type == SOAP_TYPE__array)
      return _index;
    return 0;
  }
  const char *name() const
  {
    if (_value->__type == SOAP_TYPE__struct)
      return _member->name;
    return "";
  }
  value_const_iterator& operator++()
  {
    if (_value->__type == SOAP_TYPE__struct)
      _member++;
    else if (_value->__type == SOAP_TYPE__array)
      _index++;
    else
      _value = NULL;
    return *this;
  }
  value_const_iterator operator++(int)
  {
    value_const_iterator i = *this;
    if (_value->__type == SOAP_TYPE__struct)
      _member++;
    else if (_value->__type == SOAP_TYPE__array)
      _index++;
    else
      _value = NULL;
    return i;
  }
  value_const_iterator& operator--()
  {
    if (_value->__type == SOAP_TYPE__struct)
      _member--;
    else if (_value->__type == SOAP_TYPE__array)
      _index--;
    else
      _value = NULL;
    return *this;
  }
  value_const_iterator operator--(int)
  {
    value_const_iterator i = *this;
    if (_value->__type == SOAP_TYPE__struct)
      _member--;
    else if (_value->__type == SOAP_TYPE__array)
      _index--;
    else
      _value = NULL;
    return i;
  }
  value_const_iterator& operator+=(int k)
  {
    if (_value->__type == SOAP_TYPE__struct)
      _member += k;
    else if (_value->__type == SOAP_TYPE__array)
      _index += k;
    else if (k)
      _value = NULL;
    return *this;
  }
  value_const_iterator& operator-=(int k)
  {
    if (_value->__type == SOAP_TYPE__struct)
      _member -= k;
    else if (_value->__type == SOAP_TYPE__array)
      _index -= k;
    else if (k)
      _value = NULL;
    return *this;
  }
};

/// bidirectional iterator over values (struct or array or one atomic value)
class value_iterator
{
  friend class value_const_iterator;
 private:
  struct value*                 _value;
  struct member*                _member;
  int                           _index;
 public:
  typedef ptrdiff_t difference_type;
  typedef struct value value_type;
  typedef struct value& reference;
  typedef struct value* pointer;
                                value_iterator() : _value(NULL), _member(NULL), _index(0) { }
                                value_iterator(struct value* v) : _value(v),  _member(NULL), _index(0) { }
                                value_iterator(struct value* v, struct member* m) : _value(v), _member(m), _index(0) { }
                                value_iterator(struct value* v, int i) : _value(v), _member(NULL), _index(i) { }
  operator value_const_iterator() const
  {
    value_const_iterator i;
    i._value = _value;
    i._member = _member;
    i._index = _index;
    return i;
  }
  bool operator==(const value_iterator& that) const
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
  bool operator==(const value_const_iterator& that) const
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
  bool operator!=(const value_iterator& that) const
  {
    return !operator==(that);
  }
  bool operator!=(const value_const_iterator& that) const
  {
    return !operator==(that);
  }
  struct value& operator*()
  {
    return *operator->();
  }
  struct value *operator->()
  {
    if (_value->__type == SOAP_TYPE__struct)
      return &_member->value;
    if (_value->__type == SOAP_TYPE__array)
    {
      if (((_array*)_value->ref)->data.__size <= _index)
        ((_array*)_value->ref)->size(_index + 1);
      return ((_array*)_value->ref)->data.value + _index;
    }
    return _value;
  }
  int index() const
  {
    if (_value->__type == SOAP_TYPE__struct)
      return (int)(_member - ((_struct*)_value->ref)->member);
    if (_value->__type == SOAP_TYPE__array)
      return _index;
    return 0;
  }
  const char *name() const
  {
    if (_value->__type == SOAP_TYPE__struct)
      return _member->name;
    return "";
  }
  value_iterator& operator++()
  {
    if (_value->__type == SOAP_TYPE__struct)
      _member++;
    else if (_value->__type == SOAP_TYPE__array)
      _index++;
    else
      _value = NULL;
    return *this;
  }
  value_iterator operator++(int)
  {
    value_iterator i = *this;
    if (_value->__type == SOAP_TYPE__struct)
      _member++;
    else if (_value->__type == SOAP_TYPE__array)
      _index++;
    else
      _value = NULL;
    return i;
  }
  value_iterator& operator--()
  {
    if (_value->__type == SOAP_TYPE__struct)
      _member--;
    else if (_value->__type == SOAP_TYPE__array)
      _index--;
    else
      _value = NULL;
    return *this;
  }
  value_iterator operator--(int)
  {
    value_iterator i = *this;
    if (_value->__type == SOAP_TYPE__struct)
      _member--;
    else if (_value->__type == SOAP_TYPE__array)
      _index--;
    else
      _value = NULL;
    return i;
  }
  value_iterator& operator+=(int k)
  {
    if (_value->__type == SOAP_TYPE__struct)
      _member += k;
    else if (_value->__type == SOAP_TYPE__array)
      _index += k;
    else if (k)
      _value = NULL;
    return *this;
  }
  value_iterator& operator-=(int k)
  {
    if (_value->__type == SOAP_TYPE__struct)
      _member -= k;
    else if (_value->__type == SOAP_TYPE__array)
      _index -= k;
    else if (k)
      _value = NULL;
    return *this;
  }
};

/// iterates over _struct
class _struct_const_iterator
{
  friend class _struct_iterator;
 private:
  const struct member*          _member;
 public:
                                _struct_const_iterator()                        : _member(NULL)      { }
                                _struct_const_iterator(const struct _struct* s) : _member(s->member) { }
  bool                          operator==(const _struct_iterator& that)       const;
  bool                          operator==(const _struct_const_iterator& that) const { return _member == that._member; }
  bool                          operator!=(const _struct_iterator& that)       const { return !operator==(that); }
  bool                          operator!=(const _struct_const_iterator& that) const { return !operator==(that); }
  const char*                   index() const      { return _member->name; }    ///< get member name
  const struct value&           operator*() const  { return _member->value; }   ///< get member value
  const struct value*           operator->() const { return &_member->value; }  ///< get member value
  _struct_const_iterator&       operator++()       { _member++; return *this; }
  _struct_const_iterator        operator++(int)    { _struct_const_iterator i = *this; _member++; return i; }
  _struct_const_iterator&       operator+=(int k)  { _member += k; return *this; }
  _struct_const_iterator&       operator-=(int k)  { _member -= k; return *this; }
};

/// iterates over _struct
class _struct_iterator
{
  friend class _struct_const_iterator;
 private:
  struct member*                _member;
 public:
                                _struct_iterator()                        : _member(NULL)      { }
                                _struct_iterator(const struct _struct* s) : _member(s->member) { }
  bool                          operator==(const _struct_iterator& that)       const { return _member == that._member; }
  bool                          operator==(const _struct_const_iterator& that) const { return _member == that._member; }
  bool                          operator!=(const _struct_iterator& that)       const { return !operator==(that); }
  bool                          operator!=(const _struct_const_iterator& that) const { return !operator==(that); }
  const char*                   name() const       { return _member->name; }    ///< get member name
  const char*                   index() const      { return name(); }           ///< get member name (deprecated)
  struct value&                 operator*() const  { return _member->value; }   ///< get member value
  struct value*                 operator->() const { return &_member->value; }  ///< get member value
  _struct_iterator&             operator++()       { _member++; return *this; }
  _struct_iterator              operator++(int)    { _struct_iterator i = *this; _member++; return i; }
  _struct_iterator&             operator+=(int k)  { _member += k; return *this; }
  _struct_iterator&             operator-=(int k)  { _member -= k; return *this; }
};

/// iterates over _array
class _array_const_iterator
{
  friend class _array_iterator;
 private:
  const struct value*           _start;
  const struct value*           _value;
 public:
                                _array_const_iterator()                       : _start(NULL),          _value(NULL)          { }
                                _array_const_iterator(const struct _array* a) : _start(a->data.value), _value(a->data.value) { }
  bool                          operator==(const _array_iterator& that)       const;
  bool                          operator==(const _array_const_iterator& that) const { return _value == that._value; }
  bool                          operator!=(const _array_iterator& that)       const { return !operator==(that); }
  bool                          operator!=(const _array_const_iterator& that) const { return !operator==(that); }
  int                           index() const      { return (int)(_value - _start); } ///< get array index
  const struct value&           operator*() const  { return *_value; }                ///< get array value
  const struct value*           operator->() const { return _value; }                 ///< get array value
  _array_const_iterator&        operator++()       { _value++; return *this; }
  _array_const_iterator         operator++(int)    { _array_const_iterator i = *this; _value++; return i; }
  _array_const_iterator&        operator+=(int k)  { _value += k; return *this; }
  _array_const_iterator&        operator-=(int k)  { _value -= k; return *this; }
};

/// iterates over _array
class _array_iterator
{
  friend class _array_const_iterator;
 private:
  struct value*                 _start;
  struct value*                 _value;
 public:
                                _array_iterator()                       : _start(NULL),          _value(NULL)          { }
                                _array_iterator(const struct _array* a) : _start(a->data.value), _value(a->data.value) { }
  bool                          operator==(const _array_iterator& that)       const { return _value == that._value; }
  bool                          operator==(const _array_const_iterator& that) const { return _value == that._value; }
  bool                          operator!=(const _array_iterator& that)       const { return !operator==(that); }
  bool                          operator!=(const _array_const_iterator& that) const { return !operator==(that); }
  int                           index() const      { return (int)(_value - _start); } ///< get array index
  struct value&                 operator*() const  { return *_value; }                ///< get array value
  struct value*                 operator->() const { return _value; }                 ///< get array value
  _array_iterator&              operator++()       { _value++; return *this; }
  _array_iterator               operator++(int)    { _array_iterator i = *this; _value++; return i; }
  _array_iterator&              operator+=(int k)  { _value += k; return *this; }
  _array_iterator&              operator-=(int k)  { _value -= k; return *this; }
};

/// iterates over params
class params_const_iterator
{
  friend class params_iterator;
 private:
  const struct param*           _start;
  const struct param*           _param;
 public:
                                params_const_iterator()                       : _start(NULL),     _param(NULL)     { }
                                params_const_iterator(const struct params* p) : _start(p->param), _param(p->param) { }
  bool                          operator==(const params_iterator& that)       const;
  bool                          operator==(const params_const_iterator& that) const { return _param == that._param; }
  bool                          operator!=(const params_iterator& that)       const { return !operator==(that); }
  bool                          operator!=(const params_const_iterator& that) const { return !operator==(that); }
  int                           index() const      { return (int)(_param - _start); } ///< get parameter index
  const struct value&           operator*() const  { return _param->value; }          ///< get parameter value
  const struct value*           operator->() const { return &_param->value; }         ///< get parameter value
  params_const_iterator&        operator++()       { _param++; return *this; }
  params_const_iterator         operator++(int)    { params_const_iterator i = *this; _param++; return i; }
  params_const_iterator&        operator+=(int k)  { _param += k; return *this; }
  params_const_iterator&        operator-=(int k)  { _param -= k; return *this; }
};

/// iterates over params
class params_iterator
{
  friend class params_const_iterator;
 private:
  struct param*                 _start;
  struct param*                 _param;
 public:
                                params_iterator()                       : _start(NULL),     _param(NULL)     { }
                                params_iterator(const struct params* p) : _start(p->param), _param(p->param) { }
  bool                          operator==(const params_iterator& that)       const { return _param == that._param; }
  bool                          operator==(const params_const_iterator& that) const { return _param == that._param; }
  bool                          operator!=(const params_iterator& that)       const { return !operator==(that); }
  bool                          operator!=(const params_const_iterator& that) const { return !operator==(that); }
  int                           index() const      { return (int)(_param - _start); } ///< get parameter index
  struct value&                 operator*() const  { return _param->value; }          ///< get parameter value
  struct value*                 operator->() const { return &_param->value; }         ///< get parameter value
  params_iterator&              operator++()       { _param++; return *this; }
  params_iterator               operator++(int)    { params_iterator i = *this; _param++; return i; }
  params_iterator&              operator+=(int k)  { _param += k; return *this; }
  params_iterator&              operator-=(int k)  { _param -= k; return *this; }
};

#ifdef JSON_NAMESPACE
} // namespace json
#endif

#endif

#endif
