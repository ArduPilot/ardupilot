/*
        httppost.c

        gSOAP HTTP POST/PUT/DELETE plugin for SOAP and non-SOAP payloads.

        See instructions below.

        Revisions:
        register multiple POST content handlers, each for a content type

        Note: multipart/related and multipart/form-data are already handled in gSOAP.

gSOAP XML Web services tools
Copyright (C) 2000-2018, Robert van Engelen, Genivia, Inc. All Rights Reserved.

--------------------------------------------------------------------------------
gSOAP public license.

The contents of this file are subject to the gSOAP Public License Version 1.3
(the "License"); you may not use this file except in compliance with the
License. You may obtain a copy of the License at
http://www.cs.fsu.edu/~engelen/soaplicense.html
Software distributed under the License is distributed on an "AS IS" basis,
WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
for the specific language governing rights and limitations under the License.

The Initial Developer of the Original Code is Robert A. van Engelen.
Copyright (C) 2000-2018 Robert A. van Engelen, Genivia inc. All Rights Reserved.
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

This program is released under the GPL with the additional exemption that
compiling, linking, and/or using OpenSSL is allowed.
--------------------------------------------------------------------------------

        Compile & link with stand-alone gSOAP server.

        Usage (server side):

        Define a NULL-terminated array of type-handler pairs, each with a media
        type and a the handler function:

        struct http_post_handlers my_handlers[] = {
          { "application/json",   json_handler },
          { "image/jpg",          jpeg_handler },
          { "text/html",          html_handler },
          { "POST",               generic_POST_handler },
          { "PUT",                generic_PUT_handler },
          { "PATCH",              generic_PATCH_handler },
          { "DELETE",             generic_DELETE_handler },
          { NULL }
        };

        Note that `*` and `-` can be used as wildcards to match any text and
        any character, respectively:

          { "image*",            image_handler },
          { "text*",             text_handler },
          { "text/---",          three_char_type_text_handler },

        In the above, to be more accurate, we should use a slash / between
        image and the wildcard * (which is not shown in the table above due to
        compilers throwing a fit at the / and * combo in this comment block).

        Media types may have optional parameters after `;` such as `charset`
        and `boundary`.  These parameters can be matched by the media type
        patterns in the table.  Patterns that are more specific must precede
        patterns that are less specific in the table.  For example,
        `"text/xml;*charset=utf-8*"` must precede `"text/xml"` which must
        precede `"text*"`.  Note that `"text/xml"` also matches any parameters
        of the media type of the message reveived, such as `"text/xml;
        charset=utf-8"` (only since gSOAP version 2.8.75).

        Each handler is a function that takes the soap context as a parameter
        and returns SOAP_OK or an error code.

        Register the plugin and the handlers:

        struct soap soap;
        soap_init(&soap);
        soap_register_plugin_arg(&soap, http_post, my_handlers);
        ...
        ... = soap_copy(&soap); // copies plugin too but not its data: plugin data is shared since fcopy is not set
        ...
        soap_done(&soap); // delete plugin (calls plugin->fdelete)

        A POST handler function is triggered by the media type in the
        http_post_handlers table. Use http_http_get_body() as below to retrieve
        HTTP POST body data:

        int image_handler(struct soap *soap)
        {
          const char *buf;
          size_t len;
          // if necessary, check type in soap->http_content
          if (soap->http_content && soap_tag_cmp(soap->http_content, "image/gif"))
            return 404;
          buf = soap_http_get_body(soap, &len);
          if (!buf)
            return soap->error;
          soap_end_recv(soap);
          ... // process image in buf[0..len-1]
          // reply with empty HTTP OK response:
          return soap_send_empty_response(soap, SOAP_OK);
        }

        This function should also produce a valid HTTP response, for example:

        if (we want to return HTML)
        {
          if (soap_response(soap, SOAP_HTML) // use this to return HTML...
           || soap_send(soap, "<HTML>...</HTML>"); // example HTML
           || ...
           || soap_end_send(soap))
            return soap_closesock(soap);
          return SOAP_OK;
        }
        else
        {
          soap->http_content = "image/jpeg"; // a jpeg image
          if (soap_response(soap, SOAP_FILE) // SOAP_FILE sets custom http content
           || soap_send_raw(soap, ..., ...);
           || ...
           || soap_end_send(soap))
            return soap_closesock(soap);
          return SOAP_OK;
        }

        The soap_send(soap, char*) and soap_send_raw(soap, char*, size_t) can
        be used to return content from server.

        The soap_response(soap, SOAP_FILE) call returns a HTTP 200 OK response
        with the HTTP content-type header set to soap->http_content.  To return
        a http error status code use soap_response(soap, SOAP_FILE + status)
        with status between 200 and 599 or 0.

        Usage (client side):

        char *buf;      // to hold the HTTP response body data
        size_t len;
        ...
        if (soap_post_connect(soap, "URL", "SOAP action or NULL", "media type")
         || soap_send(soap, ...)
         || soap_end_send(soap))
          ... // error
        if (soap_begin_recv(&soap)
         || soap_http_body(&soap, &buf, &len)
         || soap_end_recv(&soap))
          ... // error
        ... // use buf[0..len-1]
        soap_closesock(soap);   // close, but keep open only with HTTP keep-alive
        soap_end(soap);         // also deletes buf content

        The soap_send(soap, char*) and soap_send_raw(soap, char*, size_t) can
        be used to send content to the server as shown above.

*/

#include "httppost.h"

#ifdef __cplusplus
extern "C" {
#endif

const char http_post_id[] = HTTP_POST_ID;

static int http_post_init(struct soap *soap, struct http_post_data *data, struct http_post_handlers *handlers);
static void http_post_delete(struct soap *soap, struct soap_plugin *p);
static int http_post_parse_header(struct soap *soap, const char*, const char*);
static http_handler_t http_lookup_handler(struct soap *soap, const char *type, struct http_post_data *data);

static int http_fput(struct soap *soap);
static int http_fpatch(struct soap *soap);
static int http_fdel(struct soap *soap);

int http_post(struct soap *soap, struct soap_plugin *p, void *arg)
{
  p->id = http_post_id;
  p->data = (void*)SOAP_MALLOC(soap, sizeof(struct http_post_data));
  p->fdelete = http_post_delete;
  if (!p->data)
    return SOAP_EOM;
  if (http_post_init(soap, (struct http_post_data*)p->data, (struct http_post_handlers *)arg))
  {
    SOAP_FREE(soap, p->data); /* error: could not init */
    return SOAP_EOM; /* return error */
  }
  return SOAP_OK;
}

static int http_post_init(struct soap *soap, struct http_post_data *data, struct http_post_handlers *handlers)
{
  data->fparsehdr = soap->fparsehdr; /* save old HTTP header parser callback */
  soap->fparsehdr = http_post_parse_header; /* replace HTTP header parser callback with ours */
  data->fput = soap->fput;
  soap->fput = http_fput;
  data->fpatch = soap->fpatch;
  soap->fpatch = http_fpatch;
  data->fdel = soap->fdel;
  soap->fdel = http_fdel;
  data->handlers = handlers;
  return SOAP_OK;
}

static void http_post_delete(struct soap *soap, struct soap_plugin *p)
{
  soap->fparsehdr = ((struct http_post_data*)p->data)->fparsehdr;
  soap->fput = ((struct http_post_data*)p->data)->fput;
  soap->fpatch = ((struct http_post_data*)p->data)->fpatch;
  soap->fdel = ((struct http_post_data*)p->data)->fdel;
  SOAP_FREE(soap, p->data); /* free allocated plugin data (this function is not called for shared plugin data, but only when the final soap_done() is invoked on the original soap struct) */
}

static int http_post_parse_header(struct soap *soap, const char *key, const char *val)
{
  struct http_post_data *data = (struct http_post_data*)soap_lookup_plugin(soap, http_post_id);
  if (!data)
    return SOAP_PLUGIN_ERROR;
  if (data->fparsehdr(soap, key, val)) /* parse HTTP header */
    return soap->error;
  if (!soap_tag_cmp(key, "Content-Type")) /* check content type */
    soap->fform = http_lookup_handler(soap, val, data);
  if (!soap->fform)
    soap->fform = http_lookup_handler(soap, "POST", data);
  return soap->error;
}

static http_handler_t http_lookup_handler(struct soap *soap, const char *type, struct http_post_data *data)
{
  struct http_post_handlers *p;
  const char *params = strchr(type, ';');
  char temp[SOAP_HDRLEN];
  (void)soap;
  if (params)
  {
    size_t n = params - type;
    soap_strncpy(temp, sizeof(temp), type, n);
    temp[n] = '\0';
  }
  for (p = data->handlers; p && p->type; p++)
  {
    if (params)
    {
      if (strchr(p->type, ';'))
      {
        if (!soap_tag_cmp(type, p->type))
          break;
      }
      else if (!soap_tag_cmp(temp, p->type))
      {
        break;
      }
    }
    else if (!soap_tag_cmp(type, p->type))
    {
      break;
    }
  }
  if (p && p->type)
  {
    DBGLOG(TEST,SOAP_MESSAGE(fdebug, "Found HTTP POST plugin handler for '%s'\n", type));
    return p->handler;
  }
  return NULL;
}

static int http_fput(struct soap *soap)
{
  int (*fform)(struct soap*);
  struct http_post_data *data = (struct http_post_data*)soap_lookup_plugin(soap, http_post_id);
  if (!data)
    return SOAP_PLUGIN_ERROR;
  fform = http_lookup_handler(soap, "PUT", data);
  if (fform)
    return fform(soap);
  return data->fput(soap);
}

static int http_fpatch(struct soap *soap)
{
  int (*fform)(struct soap*);
  struct http_post_data *data = (struct http_post_data*)soap_lookup_plugin(soap, http_post_id);
  if (!data)
    return SOAP_PLUGIN_ERROR;
  fform = http_lookup_handler(soap, "PATCH", data);
  if (fform)
    return fform(soap);
  return data->fpatch(soap);
}

static int http_fdel(struct soap *soap)
{
  int (*fform)(struct soap*);
  struct http_post_data *data = (struct http_post_data*)soap_lookup_plugin(soap, http_post_id);
  if (!data)
    return SOAP_PLUGIN_ERROR;
  fform = http_lookup_handler(soap, "DELETE", data);
  if (fform)
    return fform(soap);
  return data->fdel(soap);
}

/******************************************************************************/

/* deprecated: use soap_POST instead */
int soap_post_connect(struct soap *soap, const char *endpoint, const char *action, const char *type)
{
  return soap_POST(soap, endpoint, action, type);
}

/* deprecated: use soap_PUT instead */
int soap_put_connect(struct soap *soap, const char *endpoint, const char *action, const char *type)
{
  return soap_PUT(soap, endpoint, action, type);
}

/* deprecated: use soap_DELETE instead */
int soap_delete_connect(struct soap *soap, const char *endpoint)
{
  return soap_DELETE(soap, endpoint);
}

/* deprecated: use soap_http_get_body instead */
int soap_http_body(struct soap *soap, char **buf, size_t *len)
{
  *buf = soap_http_get_body(soap, len);
  if (*buf)
    return SOAP_OK;
  return soap->error;
}

/******************************************************************************/

#ifdef __cplusplus
}
#endif

