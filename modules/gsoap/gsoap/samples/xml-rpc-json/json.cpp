/*
        json.c[pp]

        JSON C/C++ API

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

#include "json.h"

#ifdef JSON_NAMESPACE
#ifdef __cplusplus
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
#endif

#ifdef JSON_NAMESPACE
#ifdef __cplusplus
namespace json {
#endif
#endif

/******************************************************************************\
 *
 *      JSON error
 *
\******************************************************************************/

int json_error(struct soap *soap, struct value *v)
{
  if (soap->error && v)
  {
    const char *s, *t;
    soap_set_fault(soap);
    s = soap_fault_string(soap);
    t = soap_fault_detail(soap);
    /* set JSON error property (Google JSON Style Guide) */
#ifdef __cplusplus
    (*v)["error"]["code"] = soap->error;
    if (s)
    {
      (*v)["error"]["message"] = s;
      if (t)
        (*v)["error"]["errors"][0]["message"] = t;
    }
#else
    *int_of(value_at(value_at(v, "error"), "code")) = soap->error;
    if (s)
    {
      *string_of(value_at(value_at(v, "error"), "message")) = soap_strdup(soap, s);
      if (t)
        *string_of(value_at(nth_value(value_at(value_at(v, "error"), "errors"), 0), "message")) = soap_strdup(soap, t);
    }
#endif
  }
  return soap->error;
}

int json_send_fault(struct soap *soap)
{
  int status = soap->error;
  struct value *v;
  if (status == SOAP_OK || status == SOAP_STOP)
    return soap_closesock(soap);
  if (status >= 200 && status < 300)
    return soap_send_empty_response(soap, status);
  if (status < 400)
    status = 500;
  soap->keep_alive = 0; /* error: close connection by disabling keep-alive */
  v = new_value(soap);
  json_error(soap, v);
  soap->http_content = "application/json; charset=utf-8";
  if (soap_response(soap, SOAP_FILE + status)
   || json_send(soap, v)
   || soap_end_send(soap))
    return soap_closesock(soap);
  return soap_closesock(soap);
}

int json_send_error(struct soap *soap, int status, const char *message, const char *details)
{
  struct value *v = new_value(soap);
  if (status < 200 || status > 599)
    status = 0;
  /* set JSON error property (Google JSON Style Guide) */
#ifdef __cplusplus
  (*v)["error"]["code"] = status;
  if (message)
    (*v)["error"]["message"] = message;
  if (details)
    (*v)["error"]["errors"][0]["message"] = details;
#else
  *int_of(value_at(value_at(v, "error"), "code")) = status;
  if (message)
    *string_of(value_at(value_at(v, "error"), "message")) = soap_strdup(soap, message);
  if (details)
    *string_of(value_at(nth_value(value_at(value_at(v, "error"), "errors"), 0), "message")) = soap_strdup(soap, details);
#endif
  soap->http_content = "application/json; charset=utf-8";
  if (soap_response(soap, SOAP_FILE + status)
   || json_send(soap, v)
   || soap_end_send(soap))
    return soap_closesock(soap);
  return soap_closesock(soap);
}

/******************************************************************************\
 *
 *      JSON output
 *
\******************************************************************************/

int json_write(struct soap *soap, const struct value *v)
{
  if (soap_begin_send(soap)
   || json_send(soap, v)
   || soap_end_send(soap))
    return soap->error;
  return SOAP_OK;
}

/******************************************************************************/

int json_send(struct soap *soap, const struct value *v)
{
  static const char *json_indent_string = ",\n                                                                                ";
  int i, n = 1;
  if (!v)
    return SOAP_OK;
  switch (v->__type)
  {
    case SOAP_TYPE__array:
      if ((soap->mode & SOAP_XML_INDENT))
        n = 2 * (++soap->level % 40) + 2;
      if (soap_send_raw(soap, "["/*"]"*/, 1))
        return soap->error;
      if (v->ref)
      {
        for (i = 0; i < ((struct _array*)v->ref)->data.__size; i++)
        {
          if (soap_send_raw(soap, json_indent_string + (i == 0), n - (i == 0)))
            return soap->error;
          if (json_send(soap, &(((struct _array*)v->ref)->data.value)[i]))
            return soap->error;
        }
      }
      if ((soap->mode & SOAP_XML_INDENT))
      {
        if (i && soap_send_raw(soap, json_indent_string + 1, n - 3))
          return soap->error;
        soap->level--;
      }
      return soap_send_raw(soap, /*"["*/"]", 1);
    case SOAP_TYPE__boolean:
      if (*(_boolean*)v->ref == 1)
        return soap_send_raw(soap, "true", 4);
      return soap_send_raw(soap, "false", 5);
    case SOAP_TYPE__double:
      return soap_send(soap, soap_double2s(soap, (double)*(_double*)v->ref));
    case SOAP_TYPE__i4:
      return soap_send(soap, soap_int2s(soap, (int)*(_i4*)v->ref));
    case SOAP_TYPE__int:
      return soap_send(soap, soap_LONG642s(soap, (LONG64)*(_int*)v->ref));
    case SOAP_TYPE__string:
    case SOAP_TYPE__dateTime_DOTiso8601:
      return json_send_string(soap, (const char*)v->ref);
    case SOAP_TYPE__base64:
      if (soap_send_raw(soap, "\"", 1))
        return soap->error;
      if (v->ref && soap_putbase64(soap, ((struct _base64*)v->ref)->__ptr, ((struct _base64*)v->ref)->__size))
        return soap->error;
      return soap_send_raw(soap, "\"", 1);
    case SOAP_TYPE__rawdata:
      return soap_send_raw(soap, (const char*)(((struct _rawdata*)v->ref)->__ptr), ((struct _rawdata*)v->ref)->__size);
    case SOAP_TYPE__struct:
      if ((soap->mode & SOAP_XML_INDENT))
        n = 2 * (++soap->level % 40) + 2;
      if (soap_send_raw(soap, "{"/*"}"*/, 1))
        return soap->error;
      if (v->ref)
      {
        for (i = 0; i < ((struct _struct*)v->ref)->__size; i++)
        {
          if (soap_send_raw(soap, json_indent_string + (i == 0), n - (i == 0)))
            return soap->error;
          if (json_send_string(soap, (((struct _struct*)v->ref)->member)[i].name)
              || soap_send_raw(soap, ": ", 1 + (n > 1))
              || json_send(soap, &(((struct _struct*)v->ref)->member)[i].value))
            return soap->error;
        }
      }
      if ((soap->mode & SOAP_XML_INDENT))
      {
        if (i && soap_send_raw(soap, json_indent_string + 1, n - 3))
          return soap->error;
        soap->level--;
      }
      return soap_send_raw(soap, /*"{"*/"}", 1);
    default:
      if (v->__any)
        return json_send_string(soap, v->__any);
      return soap_send_raw(soap, "null", 4);
  }
  return SOAP_OK;
}

/******************************************************************************/

int json_send_string(struct soap *soap, const char *s)
{
  const char *t = s;
  int c;
  char buf[8];
  if (!s)
    return soap_send_raw(soap, "\"\"", 2);
  if (soap_send_raw(soap, "\"", 1))
    return soap->error;
  while ((c = *s++))
  {
    if (c < 0x20 && c > 0)
    {
      switch (c)
      {
        case '\b':
          c = 'b';
          break;
        case '\f':
          c = 'f';
          break;
        case '\n':
          c = 'n';
          break;
        case '\r':
          c = 'r';
          break;
        case '\t':
          c = 't';
          break;
      }
      if (c > 0x20)
      {
        buf[0] = '\\';
        buf[1]  = c;
        if (soap_send_raw(soap, t, s - t - 1) || soap_send_raw(soap, buf, 2))
          return soap->error;
        t = s;
      }
      else
      {
        (SOAP_SNPRINTF(buf, sizeof(buf), 7), "\\u%.4x", c);
        if (soap_send_raw(soap, t, s - t - 1) || soap_send_raw(soap, buf, 6))
          return soap->error;
        t = s;
      }
    }
    else if (c == '"' || c == '\\')
    {
      buf[0] = '\\';
      buf[1]  = c;
      if (soap_send_raw(soap, t, s - t - 1) || soap_send_raw(soap, buf, 2))
        return soap->error;
      t = s;
    }
    else if ((c & 0x80))
    {
      if ((soap->omode & SOAP_ENC_LATIN) && (soap->omode & SOAP_C_UTFSTRING)) /* UTF-8 to ISO 8859-1 */
      {
        if (c < 0xE0 && (c & 0x1F) <= 0x03)
          buf[0] = ((c & 0x1F) << 6) | (*s++ & 0x3F);
        else
          buf[0] = '?';
        if (soap_send_raw(soap, t, s - t - 1) || soap_send_raw(soap, buf, 1))
          return soap->error;
        t = s;
      }
      else if (!(soap->omode & SOAP_ENC_LATIN) && !(soap->omode & SOAP_C_UTFSTRING)) /* ISO 8859-1 to UTF-8 */
      {
        buf[0] = (char)(0xC0 | ((c >> 6) & 0x1F));
        buf[1] = (char)(0x80 | (c & 0x3F));
        if (soap_send_raw(soap, t, s - t - 1) || soap_send_raw(soap, buf, 2))
          return soap->error;
        t = s;
      }
    }
  }
  if (soap_send_raw(soap, t, s - t - 1) || soap_send_raw(soap, "\"", 1))
    return soap->error;
  return SOAP_OK;
}

/******************************************************************************\
 *
 *      JSON input
 *
\******************************************************************************/

int json_read(struct soap *soap, struct value *v)
{
  soap_default_value(soap, v);
  if (soap_begin_recv(soap)
   || json_recv(soap, v)
   || soap_end_recv(soap))
    return json_error(soap, v);
  return SOAP_OK;
}

/******************************************************************************/

int json_recv(struct soap *soap, struct value *v)
{
  soap_wchar c;
  if (!v)
    return SOAP_OK;
  v->__type = 0;
  v->ref = NULL;
  v->__any = NULL;
  v->soap = soap;
  while (((c = soap_getchar(soap)) > 0 && c <= 0x20) || c == 0xA0)
    continue;
  switch (c)
  {
    case EOF:
      return soap->error = SOAP_EOF;
    case '{'/*'}'*/:
    {
      struct value s;
      if (++soap->level > soap->maxlevel)
        return soap->error = SOAP_LEVEL;
#ifdef __cplusplus
      if (!(v->ref = (void*)soap_new__struct(soap)))
        return soap->error = SOAP_EOM;
#else
      if (!(v->ref = (void*)soap_malloc(soap, sizeof(struct _struct))))
        return soap->error = SOAP_EOM;
#endif
      soap_default__struct(soap, (struct _struct*)v->ref);
      v->__type = SOAP_TYPE__struct;
      while (((c = soap_getchar(soap)) > 0 && c <= 0x20) || c == 0xA0)
        continue;
      if (c == /*'{'*/'}')
        return SOAP_OK;
      soap_unget(soap, c);
      for (;;)
      {
        if (json_recv(soap, &s))
          return soap->error;
        if (s.__type != SOAP_TYPE__string)
          return soap_set_sender_error(soap, "string name expected", (const char*)s.ref, SOAP_SYNTAX_ERROR);
        while (((c = soap_getchar(soap)) > 0 && c <= 0x20) || c == 0xA0)
          continue;
        if (c != ':')
          return soap_set_sender_error(soap, "':' expected", (const char*)s.ref, SOAP_SYNTAX_ERROR);
#ifdef __cplusplus
        if (json_recv(soap, v->operator[]((const char*)s.ref)))
          return soap->error;
#else
        if (json_recv(soap, value_at(v, (const char*)s.ref)))
          return soap->error;
#endif
        while (((c = soap_getchar(soap)) > 0 && c <= 0x20) || c == 0xA0)
          continue;
        if (c == /*'{'*/'}')
          break;
        if ((int)c == EOF)
          return soap->error = SOAP_EOF;
        if (c != ',')
          return soap_set_sender_error(soap, "closing '}' or comma expected", NULL, SOAP_SYNTAX_ERROR);
      }
      soap->level--;
      return SOAP_OK;
    }
    case '['/*']'*/:
    {
      int i;
      if (++soap->level > soap->maxlevel)
        return soap->error = SOAP_LEVEL;
#ifdef __cplusplus
      if (!(v->ref = (void*)soap_new__array(soap)))
        return soap->error = SOAP_EOM;
#else
      if (!(v->ref = (void*)soap_malloc(soap, sizeof(struct _array))))
        return soap->error = SOAP_EOM;
#endif
      soap_default__array(soap, (struct _array*)v->ref);
      v->__type = SOAP_TYPE__array;
      while (((c = soap_getchar(soap)) > 0 && c <= 0x20) || c == 0xA0)
        continue;
      if (c == /*'['*/']')
        return SOAP_OK;
      soap_unget(soap, c);
      for (i = 0; i < (int)soap->maxoccurs; i++)
      {
#ifdef __cplusplus
        if (json_recv(soap, v->operator[](i)))
          return soap->error;
#else
        if (json_recv(soap, nth_value(v, i)))
          return soap->error;
#endif
        while (((c = soap_getchar(soap)) > 0 && c <= 0x20) || c == 0xA0)
          continue;
        if (c == /*'['*/']')
          break;
        if ((int)c == EOF)
          return soap->error = SOAP_EOF;
        if (c != ',')
          return soap_set_sender_error(soap, "closing ']' or comma expected", NULL, SOAP_SYNTAX_ERROR);
      }
      --soap->level;
      return SOAP_OK;
    }
    case '"':
    {
      long l = 0;
      soap->labidx = 0;
      for (;;)
      {
        char *s;
        const char *t = NULL;
        size_t k;
        if (soap_append_lab(soap, NULL, 0))
          return soap->error = SOAP_EOM;
        s = soap->labbuf + soap->labidx;
        k = soap->lablen - soap->labidx;
        soap->labidx = soap->lablen;
        while (k--)
        {
          if (t)
          {
            *s++ = *t++;
            if (!*t)
              t = NULL;
          }
          else
          {
            c = soap_getchar(soap);
            switch (c)
            {
              case EOF:
                return soap->error = SOAP_EOF;
              case '"':
                *s = '\0';
                v->__type = SOAP_TYPE__string;
                if (!(v->ref = soap_strdup(soap, soap->labbuf)))
                  return soap->error = SOAP_EOM;
                if (soap->maxlength > 0 && l > soap->maxlength)
                  return soap->error = SOAP_LENGTH;
                return SOAP_OK;
              case '\\':
                c = soap_getchar(soap);
                switch (c)
                {
                  case EOF:
                    return soap->error = SOAP_EOF;
                  case '"':
                  case '\\':
                  case '/':
                    break;
                  case 'b':
                    c = 8;
                    break;
                  case 'f':
                    c = 12;
                    break;
                  case 'n':
                    c = 10;
                    break;
                  case 'r':
                    c = 13;
                    break;
                  case 't':
                    c = 9;
                    break;
                  case 'u':
                  {
                    char *h;
                    wchar_t wc[2];
                    int i;
                    /* hex to utf8 conversion */
                    h = soap->tmpbuf;
                    for (i = 0; i < 4; i++)
                    {
                      if ((c = soap_getchar(soap)) == (int)EOF)
                        return soap->error = SOAP_EOF;
                      h[i] = c;
                    }
                    h[4] = '\0';
                    wc[0] = (wchar_t)soap_strtol(h, &h, 16);
                    wc[1] = 0;
                    if (h - soap->tmpbuf < 4)
                      return soap->error = SOAP_TYPE;
                    t = soap_wchar2s(soap, wc);
                    c = *t++;
                    if (!*t)
                      t = NULL;
                    break;
                  }
                  default:
                    return soap_set_sender_error(soap, "invalid escape in string", NULL, SOAP_SYNTAX_ERROR);
                }
                *s++ = c;
                l++;
                break;
              default:
                if ((c & 0x80) && (soap->imode & SOAP_ENC_LATIN) && (soap->imode & SOAP_C_UTFSTRING)) /* ISO 8859-1 to utf8 */
                {
                  *s++ = (char)(0xC0 | ((c >> 6) & 0x1F));
                  soap->tmpbuf[0] = (0x80 | (c & 0x3F));
                  soap->tmpbuf[1] = '\0';
                  t = soap->tmpbuf;
                }
                else if ((c & 0x80) && !(soap->imode & SOAP_ENC_LATIN) && !(soap->imode & SOAP_C_UTFSTRING)) /* utf8 to ISO 8859-1 */
                {
                  soap_wchar c1 = soap_getchar(soap);
                  if (c1 == (int)EOF)
                    return soap->error = SOAP_EOF;
                  if (c < 0xE0 && (c & 0x1F) <= 0x03)
                    *s++ = ((c & 0x1F) << 6) | (c1 & 0x3F);
                  else
                    *s++ = '?';
                }
                /* the JSON "standard" does not permit ctrl chars in strings, we silently accept these
                else if (c < 0x20 && c >= 0)
                {
                  return soap_set_sender_error(soap, "invalid control character in string", NULL, SOAP_SYNTAX_ERROR);
                }
                */
                else
                {
                  *s++ = c;
                }
                l++;
            }
          }
        }
        if (soap->maxlength > 0 && l > soap->maxlength)
          return soap->error = SOAP_LENGTH;
      }
    }
    default: /* number, true, false, null */
    {
      char *s = soap->tmpbuf;
      do
      {
        *s++ = c;
        c = soap_getchar(soap);
      } while (c != (int)EOF && (isalnum((int)c) || (int)c == '.' || (int)c == '+' || (int)c == '-') && s - soap->tmpbuf < (int)sizeof(soap->tmpbuf) - 1);
      *s = '\0';
      soap_unget(soap, c);
      if (soap->tmpbuf[0] == '-' || isdigit(soap->tmpbuf[0]))
      {
        LONG64 n = soap_strtoll(soap->tmpbuf, &s, 10);
        if (!*s)
        {
          v->__type = SOAP_TYPE__int;
          if (!(v->ref = soap_malloc(soap, sizeof(_int))))
            return soap->error = SOAP_EOM;
          *(_int*)v->ref = n;
        }
        else
        {
          double x;
          if (soap_s2double(soap, soap->tmpbuf, &x))
            return soap_set_sender_error(soap, "JSON number expected", soap->tmpbuf, SOAP_SYNTAX_ERROR);
          v->__type = SOAP_TYPE__double;
          if (!(v->ref = soap_malloc(soap, sizeof(_double))))
            return soap->error = SOAP_EOM;
          *(_double*)v->ref = x;
        }
      }
      else if (!strcmp(soap->tmpbuf, "true"))
      {
        v->__type = SOAP_TYPE__boolean;
        if (!(v->ref = soap_malloc(soap, sizeof(_boolean))))
          return soap->error = SOAP_EOM;
        *(char*)v->ref = 1;
      }
      else if (!strcmp(soap->tmpbuf, "false"))
      {
        v->__type = SOAP_TYPE__boolean;
        if (!(v->ref = soap_malloc(soap, sizeof(_boolean))))
          return soap->error = SOAP_EOM;
        *(char*)v->ref = 0;
      }
      else if (strcmp(soap->tmpbuf, "null"))
      {
        return soap_set_sender_error(soap, "JSON value expected", soap->tmpbuf, SOAP_SYNTAX_ERROR);
      }
      return SOAP_OK;
    }
  }
}

/******************************************************************************\
 *
 *      JSON REST
 *
\******************************************************************************/

int json_call(struct soap *soap, const char *endpoint, const struct value *in, struct value *out)
{
  if (out)
    soap_default_value(soap, out);
  soap->http_content = "application/json; charset=utf-8";
  if (soap_begin_count(soap)
   || ((soap->mode & SOAP_IO_LENGTH) && json_send(soap, in))
   || soap_end_count(soap)
   || soap_connect_command(soap, in && out ? SOAP_POST_FILE : out ? SOAP_GET : in ? SOAP_PUT : SOAP_DEL, endpoint, NULL)
   || json_send(soap, in)
   || soap_end_send(soap)
   || soap_begin_recv(soap)
   || json_recv(soap, out)
   || soap_end_recv(soap))
  {
    if (out)
      json_error(soap, out);
    else if (soap->error == 200 || soap->error == 201 || soap->error == 202)
      soap->error = SOAP_OK;
  }
  return soap_closesock(soap);
}

#ifdef __cplusplus

/******************************************************************************\
 *
 *      JSON C++ API
 *
\******************************************************************************/

int json_write(struct soap *soap, const struct value& v)
{
  return json_write(soap, &v);
}

int json_send(struct soap *soap, const struct value& v)
{
  return json_send(soap, &v);
}

int json_read(struct soap *soap, struct value& v)
{
  return json_read(soap, &v);
}

int json_recv(struct soap *soap, struct value& v)
{
  return json_recv(soap, &v);
}

int json_call(struct soap *soap, const char *endpoint, const struct value& in, struct value& out)
{
  return json_call(soap, endpoint, &in, &out);
}

int json_call(struct soap *soap, const char *endpoint, const struct value *in, struct value& out)
{
  return json_call(soap, endpoint, in, &out);
}

int json_call(struct soap *soap, const char *endpoint, const struct value& in, struct value *out)
{
  return json_call(soap, endpoint, &in, out);
}

std::ostream& operator<<(std::ostream& o, const struct value& v)
{
  if (v.soap)
  {
    std::ostream *os = v.soap->os;
    v.soap->os = &o;
    if (json_write(v.soap, v))
      o.clear(std::ios::failbit); // writing JSON data failed (must be a stream error)
    v.soap->os = os;
  }
  else
  {
    soap *ctx = soap_new();
    if (ctx)
    {
      ctx->os = &o;
      if (json_write(ctx, v))
        o.clear(std::ios::failbit); // writing JSON data failed (must be a stream error)
      soap_destroy(ctx);
      soap_end(ctx);
      soap_free(ctx);
    }
  }
  return o;
}

std::istream& operator>>(std::istream& i, struct value& v)
{
  if (!v.soap)
    v.soap = soap_new();
  if (v.soap)
  {
    std::istream *is = v.soap->is;
    v.soap->is = &i;
    (void)json_read(v.soap, v);
    v.soap->is = is;
  }
  return i;
}

static int json_promote_type(const value& x, const value& y)
{
  switch (x.__type)
  {
    case SOAP_TYPE__array:
      return x.__type;
    case SOAP_TYPE__struct:
      switch (y.__type)
      {
        case SOAP_TYPE__array:
          return y.__type;
        case SOAP_TYPE__struct:
          return x.__type;
        default:
          return SOAP_TYPE__array;
      }
    case SOAP_TYPE__string:
    case SOAP_TYPE__dateTime_DOTiso8601:
      switch (y.__type)
      {
        case SOAP_TYPE__array:
        case SOAP_TYPE__struct:
          return y.__type;
        default:
          return x.__type;
      }
    case SOAP_TYPE__double:
      switch (y.__type)
      {
        case SOAP_TYPE__array:
        case SOAP_TYPE__struct:
        case SOAP_TYPE__string:
        case SOAP_TYPE__dateTime_DOTiso8601:
          return y.__type;
        default:
          return x.__type;
      }
    case SOAP_TYPE__i4:
    case SOAP_TYPE__int:
      switch (y.__type)
      {
        case SOAP_TYPE__array:
        case SOAP_TYPE__struct:
        case SOAP_TYPE__string:
        case SOAP_TYPE__dateTime_DOTiso8601:
        case SOAP_TYPE__double:
          return y.__type;
        default:
          return x.__type;
      }
    default:
      return y.__type;
  }
}

value json_add(const value& x, const value& y)
{
  switch (json_promote_type(x, y))
  {
    case SOAP_TYPE__boolean:
    case SOAP_TYPE__i4:
    case SOAP_TYPE__int:
      return value(x.soap, (_int)x + (_int)y);
    case SOAP_TYPE__double:
      return value(x.soap, (_double)x + (_double)y);
    case SOAP_TYPE__string:
    case SOAP_TYPE__dateTime_DOTiso8601:
    {
      std::string s = (const char*)x;
      return value(x.soap, s.append((const char*)y));
    }
    case SOAP_TYPE__struct:
    {
      _struct s = x;
      _struct t = y;
      _struct st(x.soap);
      for (_struct::iterator i = s.begin(); i != s.end(); ++i)
        st[i.name()] = *i;
      for (_struct::iterator i = t.begin(); i != t.end(); ++i)
        st[i.name()] = *i;
      return value(x.soap, st);
    }
    case SOAP_TYPE__array:
    {
      _array a = x;
      _array b = y;
      int n = a.size();
      int m = b.size();
      _array ab(x.soap);
      ab.size(n + m);
      for (int i = 0; i < n; ++i)
        ab[i] = a[i];
      for (int i = 0; i < m; ++i)
        ab[n + i] = b[i];
      return value(x.soap, ab);
    }
    default:
      return value(x.soap);
  }
}

value json_sub(const value& x, const value& y)
{
  switch (json_promote_type(x, y))
  {
    case SOAP_TYPE__boolean:
    case SOAP_TYPE__i4:
    case SOAP_TYPE__int:
      return value(x.soap, (_int)x - (_int)y);
    case SOAP_TYPE__double:
      return value(x.soap, (_double)x - (_double)y);
    default:
      return value(x.soap);
  }
}

value json_mul(const value& x, const value& y)
{
  switch (json_promote_type(x, y))
  {
    case SOAP_TYPE__boolean:
    case SOAP_TYPE__i4:
    case SOAP_TYPE__int:
      return value(x.soap, (_int)x * (_int)y);
    case SOAP_TYPE__double:
      return value(x.soap, (_double)x * (_double)y);
    default:
      return value(x.soap);
  }
}

value json_div(const value& x, const value& y)
{
  switch (json_promote_type(x, y))
  {
    case SOAP_TYPE__boolean:
    case SOAP_TYPE__i4:
    case SOAP_TYPE__int:
      return value(x.soap, (_int)x / (_int)y);
    case SOAP_TYPE__double:
      return value(x.soap, (_double)x / (_double)y);
    default:
      return value(x.soap);
  }
}

value json_mod(const value& x, const value& y)
{
  switch (json_promote_type(x, y))
  {
    case SOAP_TYPE__boolean:
    case SOAP_TYPE__i4:
    case SOAP_TYPE__int:
    case SOAP_TYPE__double:
      return value(x.soap, (_int)x % (_int)y);
    default:
      return value(x.soap);
  }
}

bool json_eqv(const value& x, const value& y)
{
  if (x.__type != y.__type &&
      (x.__type != SOAP_TYPE__i4 || y.__type != SOAP_TYPE__int) &&
      (x.__type != SOAP_TYPE__int || y.__type != SOAP_TYPE__i4))
    return false;
  switch (x.__type)
  {
    case SOAP_TYPE__boolean:
    case SOAP_TYPE__i4:
    case SOAP_TYPE__int:
      return (_int)x == (_int)y;
    case SOAP_TYPE__double:
      return (_double)x == (_double)y;
    case SOAP_TYPE__string:
    case SOAP_TYPE__dateTime_DOTiso8601:
      return !strcmp((const char*)x, (const char*)y);
    case SOAP_TYPE__struct:
      if (x.size() != y.size())
        return false;
      else
      {
        const _struct& s = x;
        const _struct& t = y;
        for (_struct::iterator i = s.begin(); i != s.end(); ++i)
        {
          _struct::iterator j;
          for (j = t.begin(); j != t.end(); ++j)
            if (!strcmp(i.name(), j.name()))
              break;
          if (j == t.end() || *i != *j)
            return false;
        }
        return true;
      }
    case SOAP_TYPE__array:
      if (x.size() != y.size())
        return false;
      else
      {
        const _array& a = x;
        const _array& b = y;
        _array::iterator i = a.begin();
        _array::iterator j = b.begin();
        for ( ; i != a.end(); ++i, ++j)
          if (*i != *j)
            return false;
        return true;
      }
    default:
      return false;
  }
}

bool json_leq(const value& x, const value& y)
{
  if (x.__type != y.__type &&
      (x.__type != SOAP_TYPE__i4 || y.__type != SOAP_TYPE__int) &&
      (x.__type != SOAP_TYPE__int || y.__type != SOAP_TYPE__i4))
    return false;
  switch (x.__type)
  {
    case SOAP_TYPE__boolean:
    case SOAP_TYPE__i4:
    case SOAP_TYPE__int:
      return (_int)x <= (_int)y;
    case SOAP_TYPE__double:
      return (_double)x <= (_double)y;
    case SOAP_TYPE__string:
    case SOAP_TYPE__dateTime_DOTiso8601:
      return strcmp((const char*)x, (const char*)y) <= 0;
    default:
      return false;
  }
}

bool json_lne(const value& x, const value& y)
{
  if (x.__type != y.__type &&
      (x.__type != SOAP_TYPE__i4 || y.__type != SOAP_TYPE__int) &&
      (x.__type != SOAP_TYPE__int || y.__type != SOAP_TYPE__i4))
    return false;
  switch (x.__type)
  {
    case SOAP_TYPE__boolean:
    case SOAP_TYPE__i4:
    case SOAP_TYPE__int:
      return (_int)x < (_int)y;
    case SOAP_TYPE__double:
      return (_double)x < (_double)y;
    case SOAP_TYPE__string:
    case SOAP_TYPE__dateTime_DOTiso8601:
      return strcmp((const char*)x, (const char*)y) < 0;
    default:
      return false;
  }
}

#endif

#ifdef JSON_NAMESPACE
#ifdef __cplusplus
} // namespace json
#endif
#endif
