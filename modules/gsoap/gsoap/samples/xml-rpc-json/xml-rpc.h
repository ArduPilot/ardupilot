/*
        xml-rpc.h

        XML-RPC binding for C and C++

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2012, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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

////////////////////////////////////////////////////////////////////////////////
//
//      C/C++ XML-RPC/JSON API Value Allocation/Initialization
//
////////////////////////////////////////////////////////////////////////////////

/// C/C++ function returns a pointer to a new value
extern struct value * new_value(struct soap *soap);

/// C/C++ function to init or reset a value, returns a pointer to this value
extern struct value * init_value(struct soap *soap, struct value *v);

////////////////////////////////////////////////////////////////////////////////
//
//      C++ XML-RPC/JSON API Array, Struct, and Parameter Iterators
//
////////////////////////////////////////////////////////////////////////////////

#include "xml-rpc-iters.h"      /* deferred for inclusion by C++ compiler */

/// C++ external iterator class for values
extern class value_iterator;

/// C++ external iterator class for values
extern class value_const_iterator;

/// C++ external iterator class for structs
/// C++ external iterator class for structs
extern class _struct_iterator;

/// C++ external iterator class for structs
extern class _struct_const_iterator;

/// C++ external iterator class for arrays
extern class _array_iterator;

/// C++ external iterator class for arrays
extern class _array_const_iterator;

/// C++ only: declare external iterator class for parameters
extern class params_iterator;

/// C++ only: declare external iterator class for parameters
extern class params_const_iterator;

////////////////////////////////////////////////////////////////////////////////
//
//      C/C++ XML-RPC and JSON Types
//
////////////////////////////////////////////////////////////////////////////////

/// Scalar &lt;boolean&gt; element with values 0 (false) or 1 (true)
typedef char            _boolean;

/// Scalar &lt;double&gt; element with double floating point
typedef double          _double;

/// Scalar &lt;i4&gt; element with 32 bit integer
typedef int             _i4;

/// Scalar &lt;int&gt; element with 64 bit integer
typedef LONG64          _int;

/// Scalar &lt;string&gt; element
typedef char *          _string;

/// Scalar &lt;dateTime.iso8601&gt; element with ISO8601 date and time formatted string
typedef char *          _dateTime_DOTiso8601;

/// Represents the &lt;base64&gt; binary data element
struct _base64
{
// C++ function members, not available in C (when using stdsoap2 -c)
 public:
                        _base64();
                        _base64(struct soap*);
                        _base64(struct soap*, int, unsigned char*);
  int                   size() const;   ///< byte size of data
  unsigned char *       ptr();          ///< pointer to data
  void                  size(int);      ///< set byte size of data
  void                  ptr(unsigned char*);///< set pointer to data

// serializable content
 public:
  unsigned char *       __ptr;          ///< pointer to raw binary data block
  int                   __size;         ///< size of raw binary data block
};

/// Represents the &lt;rawdata&gt; binary data element
struct _rawdata
{
// C++ function members, not available in C (when using stdsoap2 -c)
 public:
                        _rawdata();
                        _rawdata(struct soap*);
                        _rawdata(struct soap*, int, char*);
                        _rawdata(struct soap*, char*);
  int                   size() const;   ///< byte size of data
  char *                ptr();          ///< pointer to data
  void                  size(int);      ///< set byte size of data
  void                  ptr(char*);     ///< set pointer to data

// serializable content
 public:
  unsigned char *       __ptr;          ///< pointer to raw binary data block
  int                   __size;         ///< size of raw binary data block
};

/// Represents the &lt;struct&gt; record structure element
struct _struct
{
// C++ function members, not available in C (when using stdsoap2 -c)
 public:
  typedef _struct_iterator       iterator;
  typedef _struct_const_iterator const_iterator;
                        _struct();
                        _struct(const struct _struct&);
                        _struct(struct soap*);
                        _struct(struct soap*, int);
  struct _struct&       operator=(const struct _struct&);
  extern bool           empty() const;  ///< true if struct is empty
  int                   size() const;   ///< number of accessors in struct
  struct value&         operator[](int) const;///< struct index (negative to get from end)
  struct value&         operator[](const char*);///< struct accessor
  struct value&         operator[](const wchar_t*);///< struct accessor
  const struct value&   operator[](const char*) const;///< struct accessor
  const struct value&   operator[](const wchar_t*) const;///< struct accessor
  _struct_iterator      begin() const;  ///< struct iterator begin
  _struct_iterator      end() const;    ///< struct iterator end

// serializable content
 public:
  int                   __size;         ///< number of members
  struct member *       member;         ///< pointer to member array
  struct soap *         soap;           ///< ref to soap struct that manages this type
};

/// Represents the &lt;data&gt; element
struct data
{
// serializable content
 public:
  int                   __size;         ///< number of array elements
  struct value *        value;          ///< pointer to array elements
};

/// Represents the &lt;array&gt; array of values element
struct _array
{
// C++ function members, not available in C (when using stdsoap2 -c)
 public:
  typedef _array_iterator       iterator;
  typedef _array_const_iterator const_iterator;
                        _array();
                        _array(const struct _array&);
                        _array(struct soap*);
                        _array(struct soap*, int);
  struct _array&        operator=(const struct _array&);
  extern bool           empty() const;  ///< true if array is empty
  int                   size() const;   ///< number of array elements
  void                  size(int);      ///< (re)set number of array elements
  struct value&         operator[](int);///< array index (negative to get from end)
  const struct value&   operator[](int) const;///< array index (negative to get from end)
  _array_iterator       begin() const;  ///< array iterator begin
  _array_iterator       end() const;    ///< array iterator end
 
// serializable content
 public:
  struct data           data;           ///< data with values
  struct soap*          soap;           ///< ref to soap struct that manages this type
};

/// Represents the &lt;value&gt; container element
/// The &lt;value&gt; element contains either string data stored in __any or an
/// other type of data stored in a subelement. In case of a subelement, the
/// __type member indicates the type of data pointed to by the ref member.
/// For example, when __type = SOAP_TYPE__int then the ref points to a LONG64,
/// when __type = SOAP_TYPE__string (char*)ref is a string.
struct value
{
// C++ function members, not available in C (when using stdsoap2 -c)
 public:
  typedef value_iterator       iterator;
  typedef value_const_iterator const_iterator;
                        value();
                        value(const struct value&);
                        value(struct soap*);
                        value(struct soap*, extern bool);
                        value(struct soap*, _i4);
                        value(struct soap*, _int);
                        value(struct soap*, _double);
                        value(struct soap*, const char*);
                        value(struct soap*, const std::string&);
                        value(struct soap*, const wchar_t*);
                        value(struct soap*, const std::wstring&);
                        value(struct soap*, ULONG64);
                        value(struct soap*, const struct _array&);
                        value(struct soap*, const struct _struct&);
                        value(struct soap*, const struct _base64&);
                        value(struct soap*, const struct _rawdata&);
                        operator extern bool() const;
                        operator _i4() const;
                        operator _int() const;
                        operator _double() const;
                        operator char*() const;
                        operator std::string() const;
                        operator wchar_t*() const;
                        operator std::wstring() const;
                        operator ULONG64() const;
                        operator struct _array&();
                        operator const struct _array&() const;
                        operator struct _struct&();
                        operator const struct _struct&() const;
                        operator struct _base64&();
                        operator const struct _base64&() const;
                        operator struct _rawdata& ();
                        operator const struct _rawdata& () const;
  struct value&         operator[](int);                  ///< array/struct index (negative to get from end)
  struct value&         operator[](const char*);          ///< struct access
  struct value&         operator[](const std::string&);   ///< struct access
  struct value&         operator[](const wchar_t*);       ///< struct access
  struct value&         operator[](const std::wstring&);  ///< struct access
  const struct value&   operator[](int) const;                  ///< array/struct index (negative to get from end)
  const struct value&   operator[](const char*) const;          ///< struct access
  const struct value&   operator[](const std::string&) const;   ///< struct access
  const struct value&   operator[](const wchar_t*) const;       ///< struct access
  const struct value&   operator[](const std::wstring&) const;  ///< struct access
  extern bool           operator=(extern bool);
  _i4                   operator=(_i4);
  _int                  operator=(_int);
  _double               operator=(_double);
  ULONG64               operator=(ULONG64);
  const char *          operator=(const char*);
  char *                operator=(char*);
  char *                operator=(const std::string&);
  const char *          operator=(const wchar_t*);
  char *                operator=(wchar_t*);
  char *                operator=(const std::wstring&);
  struct _array&        operator=(const struct _array&);
  struct _struct&       operator=(const struct _struct&);
  struct _base64&       operator=(const struct _base64&);
  struct _rawdata&      operator=(const struct _rawdata&);
  struct value&         operator=(const struct value&);
  extern void           size(int);              ///< set/allocate size of array
  extern int            size() const;           ///< returns array/struct size or 0
  extern bool           empty() const;          ///< true if empty array or struct
  extern int            nth(int) const;         ///< returns nth index if index is in bounds, < 0 otherwise
  extern int            nth(const char*) const; ///< returns nth index of name in struct, < 0 otherwise
  extern int            nth(const wchar_t*) const; ///< returns nth index of name in struct, < 0 otherwise
  extern bool           has(int) const;         ///< true if array index is in bounds
  extern bool           has(const char*) const; ///< true if struct has name as a key
  extern bool           has(const wchar_t*) const; ///< true if struct has name as a key
  extern bool           is_null() const;        ///< true if value is not set (JSON null)
  extern bool           is_bool() const;        ///< true if value is Boolean type
  extern bool           is_false() const;       ///< true if value is Boolean false
  extern bool           is_true() const;        ///< true if value is Boolean true
  extern bool           is_int() const;         ///< true if value is int type
  extern bool           is_double() const;      ///< true if value is double type
  extern bool           is_number() const;      ///< true if value is a number (int or float)
  extern bool           is_string() const;      ///< true if value is string type
  extern bool           is_dateTime() const;    ///< true if value is dateTime
  extern bool           is_array() const;       ///< true if value is array type
  extern bool           is_struct() const;      ///< true if value is struct type
  extern bool           is_base64() const;      ///< true if value is base64 type
  extern bool           is_rawdata() const;     ///< true if value is rawdata
  value_iterator        begin();                ///< value iterator begin
  value_iterator        end();                  ///< value iterator end
 
// serializable content
 public:
  int                   __type 0;       ///< optional SOAP_TYPE_X, where X is a type name
  void *                ref;            ///< ref to data
  _string               __any;          ///< &lt;value&gt; string content in XML-RPC (not JSON), if any
  struct soap *         soap;           ///< ref to soap struct that manages this type
};

/// Represents the &lt;member&gt; element of a &lt;struct&gt;
struct member
{ 
// serializable content
 public:
  char *                name;           ///< struct accessor name
  struct value          value;          ///< struct accessor value
};

////////////////////////////////////////////////////////////////////////////////
//
//      C/C++ XML-RPC API Method Parameters and Call Objects
//
////////////////////////////////////////////////////////////////////////////////

/// Represents the &lt;params&gt; of a &lt;methodCall&gt; request and response
struct params
{
// C++ function members, not available in C (when using stdsoap2 -c)
 public:
  typedef params_iterator       iterator;
  typedef params_const_iterator const_iterator;
                        params();
                        params(struct soap*);
                        params(struct soap*, int);
  extern bool           empty() const;  ///< true if no parameters
  int                   size() const;   ///< number of parameters
  struct value&         operator[](int);///< parameter index (negative to get from end)
  const struct value&   operator[](int) const;///< parameter index (negative to get from end)
  params_iterator       begin() const;  ///< parameter accessor iterator begin
  params_iterator       end() const;    ///< parameter accessor iterator end

// serializable content
 public:
  int                   __size;         ///< number of parameters
  struct param *        param;          ///< pointer to array of parameters
  struct soap *         soap;           ///< ref to soap struct that manages this type
};

/// Represents a &lt;param&gt; of the &lt;params&gt; of a &lt;methodCall&gt;
struct param
{
// serializable content
 public:
  struct value          value;          ///< parameter value
};

/// Represents the &lt;methodResponse&gt; element with response &lt;params&gt; and &lt;fault&gt;
struct methodResponse
{
// C++ function members, not available in C (when using stdsoap2 -c)
 public:
                        methodResponse();
                        methodResponse(struct soap*);
  struct value&         operator[](int);///< response parameter accessor index
  const struct value&   operator[](int) const;///< response parameter accessor index
  struct value&         get_fault(void);///< get fault, if set
  struct value&         set_fault(const char*);///< set fault
  struct value&         set_fault(struct value&);///< set fault
  int                   recv();         ///< receive response
  int                   send();         ///< send response

// serializable content
 public:
  struct params *       params;         ///< response return parameters, if any
  struct fault *        fault;          ///< response fault, if any
  struct soap *         soap;           ///< ref to soap struct that manages this type
};
  
/// Represents the &lt;methodCall&gt; element with &lt;methodName&gt; and request &lt;params&gt; for remote invocation
struct methodCall
{
// private state info
 private:
  char *                methodEndpoint; ///< XML-RPC endpoint
  struct methodResponse*methodResponse; ///< holds the response after the call

// C++ function members, not available in C (when using stdsoap2 -c)
 public:
                        methodCall();
                        methodCall(struct soap*);
                        methodCall(struct soap*, const char *endpoint, const char *methodname);
                        ///< instantiate with endpoint and method name
  struct value&         operator[](int);///< method parameter accessor index
  const struct value&   operator[](int) const;///< method parameter accessor index
  struct params&        operator()();   ///< method invocation
  struct params&        operator()(const struct params&);
                        ///< method invocation with param list
  struct params&        response();     ///< get last response
  struct value&         fault();        ///< fault value of response
  const char *          name() const;   ///< get method name
  int                   error() const;  ///< gSOAP error code
  int                   recv();         ///< receive call
  int                   send();         ///< send call

// serializable content
 public:
  char *                methodName;     ///< name of the method
  struct params         params;         ///< input request parameters
  struct soap *         soap;           ///< ref to soap struct that manages this type
};

/// Represents the &lt;fault&gt; container element with a value
struct fault
{
// serializable content
 public:
  struct value          value;          ///< value of the fault
};

////////////////////////////////////////////////////////////////////////////////
//
//      C XML-RPC/JSON API Functions
//
////////////////////////////////////////////////////////////////////////////////

/// C function returns pointer to Boolean, coerces v to Boolean if needed
extern _boolean *bool_of(struct value *v);

/// C function returns pointer to int, coerces v to int if needed
extern _int *int_of(struct value *v);

/// C function returns pointer to double, coerces v to double if needed
extern _double *double_of(struct value *v);

/// C function returns pointer to string (pointer to char * to set or get), coerces v to string if needed
extern const char **string_of(struct value *v);

/// C function returns pointer to string of ISO 8601, coerces v to string if needed (get time with soap_s2dateTime and set time with soap_dateTime2s)
extern const char **dateTime_of(struct value *v);

/// C function returns pointer to base64 struct, coerces v to base64 struct if needed
extern struct _base64 *base64_of(struct value *v);

/// C function returns pointer to string of RAW JSON
extern struct _rawdata *rawdata_of(struct value* v);

/// C function returns pointer to member value of a struct, coerces v to struct if needed
extern struct value *value_at(struct value *v, const char *s);

/// C function returns pointer to member value of a struct, coerces v to struct if needed
extern struct value *value_atw(struct value *v, const wchar_t *s);

/// C function returns the nth index of a name in a struct, < 0 otherwise
extern int nth_at(const struct value *v, const char *s);

/// C function returns the nth index of a name in a struct, < 0 otherwise
extern int nth_atw(const struct value *v, const wchar_t *s);

/// C function returns the nth index if an nth index in the array exists, < 0 otherwise
extern int nth_nth(const struct value *v, int n);

/// C function returns pointer to nth member (name and value) of a struct or NULL when not exists
extern struct member *nth_member(struct value *v, int n);

/// C function returns pointer to array element value at index n, coerces v to array with value at n if needed
extern struct value *nth_value(struct value *v, int n);

/// C function returns true if value is not set or assigned (JSON null)
extern _boolean is_null(const struct value *v);

/// C function returns true if value is a 32 or a 64 bit int
extern _boolean is_int(const struct value *v);

/// C function returns true if value is a 64 bit double floating point
extern _boolean is_double(const struct value *v);

/// C function returns true if value is a number (int or float)
extern _boolean is_number(const struct value *v);

/// C function returns true if value is a string
extern _boolean is_string(const struct value *v);

/// C function returns true if value is a Boolean "true" or "false" value
extern _boolean is_bool(const struct value *v);

/// C function returns true if value is Boolean "true"
extern _boolean is_true(const struct value *v);

/// C function returns true if value is Boolean "false"
extern _boolean is_false(const struct value *v);

/// C function returns true if array of values
extern _boolean is_array(const struct value *v);

/// C function returns true if structure, a.k.a. a JSON object
extern _boolean is_struct(const struct value *v);

/// C function returns true if ISO 8601, always false for received JSON
extern _boolean is_dateTime(const struct value *v);

/// C function returns true if base64, always false for received JSON
extern _boolean is_base64(const struct value *v);

/// C function returns true if RAW JSON, always false for received JSON
extern _boolean is_rawdata(const struct value *v);

/// C function to create an empty struct
extern void set_struct(struct value *v);

/// C function set/allocate size of array
extern void set_size(struct value *v, int n);

/// C function returns the size of an array or struct
extern int has_size(const struct value *v);

/// C function returns true (1) if struct/array is empty or when value is not an struct/array, 0 otherwise
extern int is_empty(const struct value *v);

////////////////////////////////////////////////////////////////////////////////
//
//      C XML-RPC API Method Parameters and Call Function
//
////////////////////////////////////////////////////////////////////////////////

/// C function returns pointer to new parameters for XML-RPC methodCall
extern struct params *new_params(struct soap *soap);

/// C function to clear parameters, returns a pointer to the empty parameters
extern struct params *init_params(struct soap *soap, struct params *p);

/// C function returns pointer to parameter value at index n, creates new parameter if needed
extern struct value *nth_param(struct params *p, int n);

/// C function to invoke XML-RPC methodCall on endpoint given methodCall m populates methodResponse r, returns SOAP_OK or error
extern int call_method(struct soap *soap, const char *endpoint, const char *methodName, struct params *p, struct methodResponse *r);
