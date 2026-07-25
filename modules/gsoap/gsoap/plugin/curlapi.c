/*
        curlapi.c

        cURL plugin.

gSOAP XML Web services tools
Copyright (C) 2000-2017, Robert van Engelen, Genivia Inc., All Rights Reserved.
This part of the software is released under one of the following licenses:
GPL or the gSOAP public license.
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
Copyright (C) 2000-2017, Robert van Engelen, Genivia Inc., All Rights Reserved.
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
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

/**

@mainpage The CURL plugin

[TOC]

@section curl_0 Overview

The CURL plugin for gSOAP provides a bridge for the gSOAP engine to use libcurl
for internet communications.  While gSOAP provides a full HTTP stack, libcurl
can be used to support additional protocols and features by replacing gSOAP's
HTTP stack.

@section curl_1 CURL plugin setup

To use the CURL plugin:
-# Add `#include "plugin/curlapi.h"` to your client-side code and compile your
   code together with `plugin/curlapi.c`. Link your code with libcurl.
-# Add `curl_global_init(CURL_GLOBAL_ALL)` at the start of your program to
   initialize CURL. Add `curl_global_cleanup()` at the end of your program.
-# In your source code where you create a `soap` context, register the plugin
   with this `soap` context, Or use the `soap` member of a soapcpp2-generated
   C++ proxy class. Use `soap_register_plugin(soap, soap_curl)` to register.
-# Alternatively, if you have a `CURL *curl` handle already set up, then
   register the plugin with `soap_register_plugin_arg(soap, soap_curl, curl)`.
   The benefit of this is that you can set CURL options of the handle. Do not
   delete this handle until the `soap` context is deleted.
-# If you register multiple other plugins with the context, you should register
   the CURL plugin always first.

The plugin is not limited to SOAP calls, you can use it with XML REST and JSON
in gSOAP.  The plugin registry steps are the same for any client-side API
service calls.

The CURL plugin supports SOAP with MTOM attachments, including streaming MTOM.
Other plugins can be combined with this plugin, such as WSSE for WS-Security.

@note The CURL plugin increases the overhead of HTTP calls compared to the
gSOAP HTTP stack. The overhead is due to buffering of the entire outbound
message before sending and buffering of the entire message received.  By
contrast, gSOAP uses a streaming approach and only buffers the socket
communications to (de)serialize XML directly into C/C++ data.

@section curl_2 Configuration and settings

To use the CURL plugin, register the plugin with the current `soap` context
using `soap_register_plugin(soap, soap_curl)`.  This also creates a new CURL
handle that is internally used by the plugin until the `soap` context is
deleted.  For C++ proxy classes generated with soapcpp2, register the plugin
with the `soap` member of the proxy class.

The gSOAP HTTP chunked transfer mode `SOAP_IO_CHUNK` and timeout settings are
also used by the CURL plugin, when set, as follows:

@code
    #include "plugin/curlapi.h"
    ...
    struct soap *soap;
    curl_global_init(CURL_GLOBAL_ALL);
    soap = soap_new1(SOAP_IO_CHUNK | SOAP_XML_INDENT);
    soap_register_plugin(soap, soap_curl);
    soap->connect_timeout = 60;  // 1 minute
    soap->send_timeout = 10;     // 10 seconds
    soap->recv_timeout = 10;     // 10 seconds
    soap->transfer_timeout = 20; // 20 seconds
    ...
    // client program runs
    ...
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
    curl_global_cleanup();
@endcode

It is strongly recommended to set timeouts.  The timeout values specified here
are just examples.  Actual values depend on the application's performance
characteristics.

HTTP proxy settings are used by the CURL plugin.  You can specify the HTTP proxy
settings `soap->proxy_host` and `soap->proxy_port` with the HTTP proxy host
and port, respectively, and specify the HTTP proxy access credentials
`soap->proxy_userid` and `soap->proxy_passwd`.

Also compression is used by the CURL plugin when enabled with `SOAP_ENC_ZLIB`:

@code
    #include "plugin/curlapi.h"
    ...
    struct soap *soap;
    curl_global_init(CURL_GLOBAL_ALL);
    soap = soap_new1(SOAP_IO_CHUNK | SOAP_ENC_ZLIB | SOAP_XML_INDENT);
    soap_register_plugin(soap, soap_curl);
    ...
    // client program runs
    ...
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
    curl_global_cleanup();
@endcode


When an transmission error occurs, use `soap_curl_reset(soap)` to reset the
plugin.  This ensures that the gSOAP IO operations are reset and will behave
again normally.

Alternatively, you can create your own `CURL *curl` handle, configure it, and
pass it to the plugin as follows:

@code
    #include "plugin/curlapi.h"
    ...
    struct soap *soap;
    CURL *curl;
    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();
    curl_easy_setopt(data->curl, CURLOPT_CONNECTTIMEOUT, 60L);
    curl_easy_setopt(data->curl, CURLOPT_TIMEOUT, 10L);
    soap = soap_new1(SOAP_XML_INDENT);
    soap_register_plugin_arg(soap, soap_curl, curl);
    ...
    // client program runs
    ...
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
    curl_easy_cleanup(curl);
    ...
    curl_global_cleanup();
@endcode

Note that C++ proxy classes generated by soapcpp2 with option `-j` have a
`soap` member that should be used to register the plugin with:

@code
    #include "plugin/curlapi.h"
    ...
    Proxy proxy(SOAP_XML_INDENT);
    CURL *curl;
    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();
    curl_easy_setopt(data->curl, CURLOPT_CONNECTTIMEOUT, 60L);
    curl_easy_setopt(data->curl, CURLOPT_TIMEOUT, 10L);
    soap_register_plugin_arg(proxy.soap, soap_curl, curl);
    ...
    // make calls with the proxy object
    ...
    proxy.destroy();
    curl_easy_cleanup(curl);
    ...
    curl_global_cleanup();
@endcode

@section curl_3 SOAP client example

This example shows a calculator client application with CURL and gSOAP.

The soapcpp2 command is applied to `calc.h` with `soapcpp2 -c -CL calc.h`, where
`calc.h` is:

@code
    //gsoap ns service name:            calc Simple calculator service described at https://www.genivia.com/dev.html
    //gsoap ns service protocol:        SOAP
    //gsoap ns service style:           rpc
    //gsoap ns service encoding:        encoded
    //gsoap ns service namespace:       http://websrv.cs.fsu.edu/~engelen/calc.wsdl
    //gsoap ns service location:        http://websrv.cs.fsu.edu/~engelen/calcserver.cgi

    //gsoap ns schema namespace:        urn:calc

    //gsoap ns service method: add Sums two values
    int ns__add(double a, double b, double *result);

    //gsoap ns service method: sub Subtracts two values
    int ns__sub(double a, double b, double *result);

    //gsoap ns service method: mul Multiplies two values
    int ns__mul(double a, double b, double *result);

    //gsoap ns service method: div Divides two values
    int ns__div(double a, double b, double *result);

    //gsoap ns service method: pow Raises a to b
    int ns__pow(double a, double b, double *result);
@endcode

This generates `soapStub.h`, `soapH.h`, `soapC.c`, `soapClient.c`, and
`calc.nsmap`.

To keep this example small, the main program uses the calculator service to add
two values:

@code
    #include "soapH.h"
    #include "calc.nsmap"
    #include "plugin/curlapi.h"

    const char server[] = "http://websrv.cs.fsu.edu/~engelen/calcserver.cgi";

    int main(int argc, char **argv)
    {
      struct soap *soap = soap_new1(SOAP_XML_INDENT);
      double result;
      curl_global_init(CURL_GLOBAL_ALL);
      soap_register_plugin(soap, soap_curl);
      if (soap_call_ns__add(soap, server, "", 2.0, 3.0, &result))
      {
        soap_print_fault(soap, stderr);
        soap_curl_reset(soap);
      }
      else
        printf("2 +3 = %g\n", result);
      soap_destroy(soap);
      soap_end(soap);
      soap_free(soap);
      curl_global_cleanup();
      return 0;
    }
@endcode

We compile this example program together with `stdsoap2.c`, `soapC.c`,
`soapClient.c`, `plugin/curlapi.c` and we link it with libcurl.

As stated previously, to use a current `CURL *curl` handle that you have
created, use `soap_register_plugin_arg(soap, soap_curl, curl)` to register the
plugin.

@section curl_4 JSON REST example

See the gSOAP [JSON documentation](https://www.genivia.com/doc/xml-rpc-json/html/index.html)
for details about using JSON with gSOAP in C and in C++.

A JSON client in C with CURL has the following outline:

@code
    #include "plugin/curlapi.h"
    #include "json.h"
    struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}};
    ...
    struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
    struct value *request = new_value(ctx);
    struct value response;
    curl_global_init(CURL_GLOBAL_ALL);
    soap_register_plugin(ctx, soap_curl);
    ... // here we populate the request data to send
    if (json_call(ctx, "endpoint URL", request, &response))
    {
      soap_print_fault(ctx, stderr);
      soap_curl_reset(ctx);
    }
    else
    {
      ... // use the response data here
    }
    soap_destroy(ctx); // delete objects
    soap_end(ctx);     // delete data
    ...                // here we can make other calls etc.
    soap_free(ctx);    // delete the context
    curl_global_cleanup();
@endcode

As stated previously, to use a current `CURL *curl` handle that you have
created, use `soap_register_plugin_arg(soap, soap_curl, curl)` to register the
plugin.

JSON in C++ is similar to the C example shown with the benefit of the
easy-to-use [JSON C++ API](https://www.genivia.com/doc/xml-rpc-json/html/index.html#cpp).

*/

#include "curlapi.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Plugin identification for plugin registry */
const char soap_curl_id[] = SOAP_CURL_ID;

/******************************************************************************\
 *
 *      Static protos
 *
\******************************************************************************/

static int soap_curl_init(struct soap *soap, struct soap_curl_data *data, CURL *curl);
static void soap_curl_delete(struct soap *soap, struct soap_plugin *p);
static int soap_curl_connect_callback(struct soap *soap, const char *endpoint, const char *host, int port);
static int soap_curl_send_callback(struct soap *soap, const char *buf, size_t len);
static int soap_curl_prepare_init_recv_callback(struct soap *soap);
static int soap_curl_prepare_final_recv_callback(struct soap *soap);
static size_t soap_curl_recv_callback(struct soap *soap, char *buf, size_t size);
static size_t soap_curl_read_callback(void *buffer, size_t size, size_t nitems, void *ptr);
static size_t soap_curl_write_callback(void *buffer, size_t size, size_t nitems, void *ptr);

/******************************************************************************\
 *
 *      Plugin registry functions
 *
\******************************************************************************/

/**
@fn int soap_curl(struct soap *soap, struct soap_plugin *p, void *arg)
@brief Plugin registry function, used with soap_register_plugin and soap_register_plugin_arg.
@param soap context
@param[in,out] p plugin created in registry
@param[in] arg passed from soap_register_plugin_arg
@return SOAP_OK
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_curl(struct soap *soap, struct soap_plugin *p, void *arg)
{
  DBGFUN("soap_curl");
  p->id = soap_curl_id;
  p->data = (void*)SOAP_MALLOC(soap, sizeof(struct soap_curl_data));
  p->fcopy = NULL;
  p->fdelete = soap_curl_delete;
  if (!p->data)
    return SOAP_EOM;
  if (soap_curl_init(soap, (struct soap_curl_data*)p->data, (CURL*)arg))
  {
    SOAP_FREE(soap, p->data);
    return SOAP_EOM;
  }
  return SOAP_OK;
}

/**
@fn int soap_curl_init(struct soap *soap, struct soap_wsa_data *data)
@brief Initializes plugin data.
@param soap context
@param[in,out] data plugin data
@return SOAP_OK
*/
static int soap_curl_init(struct soap *soap, struct soap_curl_data *data, CURL *curl)
{
  DBGFUN("soap_curl_init");
  data->soap = soap;
  data->curl = curl;
  data->own = (curl == NULL);
  data->active = 0;
  data->hdr = NULL;
  data->blk = NULL;
  data->ptr = NULL;
  data->lst = NULL;
  data->mode = soap->omode;
  *data->buf = '\0';
  soap->omode &= ~SOAP_IO;
  soap->omode |= SOAP_IO_BUFFER;
  soap->omode |= SOAP_ENC_PLAIN;
  data->fconnect = soap->fconnect;
  soap->fconnect = soap_curl_connect_callback;
  data->fsend = soap->fsend;
  soap->fsend = soap_curl_send_callback;
  data->frecv = soap->frecv;
  soap->frecv = soap_curl_recv_callback;
  data->fprepareinitrecv = soap->fprepareinitrecv;
  soap->fprepareinitrecv = soap_curl_prepare_init_recv_callback;
  data->fpreparefinalrecv = soap->fpreparefinalrecv;
  soap->fpreparefinalrecv = soap_curl_prepare_final_recv_callback;
  return SOAP_OK;
}

/**
@fn void soap_curl_delete(struct soap *soap, struct soap_plugin *p)
@brief Deletes plugin data.
@param soap context
@param[in,out] p plugin
*/
static void soap_curl_delete(struct soap *soap, struct soap_plugin *p)
{
  (void)soap;
  struct soap_curl_data *data = (struct soap_curl_data*)p->data;
  DBGFUN("soap_curl_delete");
  if (data->lst)
    soap_end_block(soap, data->lst);
  if (data->curl && data->own)
    curl_easy_cleanup(data->curl);
  soap->fsend = data->fsend;
  soap->frecv = data->frecv;
  soap->fprepareinitrecv = data->fprepareinitrecv;
  soap->fpreparefinalrecv = data->fpreparefinalrecv;
  SOAP_FREE(soap, data);
}

/******************************************************************************\
 *
 *      Plugin API calls
 *
\******************************************************************************/

/**
@fn int soap_curl_reset(struct soap *soap)
@brief Reset the plugin so gSOAP IO behaves normally. This is an optional API call, not required except when serializing data after an error.
@param soap context
*/
SOAP_FMAC1
void
SOAP_FMAC2
soap_curl_reset(struct soap *soap)
{
  struct soap_curl_data *data = (struct soap_curl_data*)soap_lookup_plugin(soap, soap_curl_id);
  DBGFUN("soap_curl_reset");
  if (data)
  {
    if (data->lst)
      soap_end_block(soap, data->lst);
    data->lst = NULL;
    data->active = 0;
  }
}

/******************************************************************************\
 *
 *      Callbacks registered by plugin
 *
\******************************************************************************/

/**
@fn int soap_curl_connect_callback(struct soap *soap, const char *endpoint, const char *host, int port) 
@brief The fconnect callback invokes this function to override connecting to an endpoint.
@param soap context
@param endpoint URL to connect to, use "" if the CURL handle has a URL assigned with CURLOPT_URL
@param host not used
@param port not used
@return SOAP_OK or error code
*/
static int soap_curl_connect_callback(struct soap *soap, const char *endpoint, const char *host, int port)
{
  struct soap_curl_data *data = (struct soap_curl_data*)soap_lookup_plugin(soap, soap_curl_id);
  (void)host; (void)port;
  DBGFUN1("soap_curl_connect_callback", "endpoint=%s", endpoint);
  if (!data)
    return soap->error = SOAP_PLUGIN_ERROR;
  if (!data->curl)                 /* no CURL handle passed to soap_register_plugin() */
  {
    data->curl = curl_easy_init(); /* so set up our own */
    if (!data->curl)
      return soap->error = SOAP_EOM;
    data->own = 1;
  }
  if (data->hdr)
    curl_slist_free_all(data->hdr);
  data->hdr = NULL;
  data->blk = NULL;
  data->ptr = NULL;
  if (data->lst)
    soap_end_block(soap, data->lst);
  data->lst = NULL;
  if (endpoint && *endpoint)       /* if endpoint != "" then use it, otherwise use CURL's */
    curl_easy_setopt(data->curl, CURLOPT_URL, endpoint);
  curl_easy_setopt(data->curl, CURLOPT_USERAGENT, SOAP_CURL_ID);
  if (soap->status == SOAP_POST || soap->status == SOAP_POST_FILE)
    curl_easy_setopt(data->curl, CURLOPT_POST, 1L);
  else if (soap->status == SOAP_GET)
    curl_easy_setopt(data->curl, CURLOPT_HTTPGET, 1L);
  else if (soap->status == SOAP_PUT)
    curl_easy_setopt(data->curl, CURLOPT_PUT, 1L);
  else if (soap->status == SOAP_DEL)
    curl_easy_setopt(data->curl, CURLOPT_CUSTOMREQUEST, "DELETE");
  if (soap->status == SOAP_POST || soap->status == SOAP_POST_FILE || soap->status == SOAP_PUT)
  {
    if (soap_http_content_type(soap, SOAP_OK))
    {
      (void)soap_memmove(soap->tmpbuf+14, sizeof(soap->tmpbuf), soap->tmpbuf, sizeof(soap->tmpbuf)-14);
      soap->tmpbuf[sizeof(soap->tmpbuf)-1] = '\0';
      (void)soap_memcpy(soap->tmpbuf, sizeof(soap->tmpbuf), "Content-Type: ", 14);
      data->hdr = curl_slist_append(data->hdr, soap->tmpbuf);
      if (!data->hdr)
        return soap->error = SOAP_EOM;
    }
    if (soap->action)
    {
      (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), strlen(soap->action) + 14), "SOAPAction: \"%s\"", soap->action);
      data->hdr = curl_slist_append(data->hdr, soap->tmpbuf);
      if (!data->hdr)
        return soap->error = SOAP_EOM;
    }
    if (soap->http_extra_header)
    {
      const char *header = soap->http_extra_header;
      const char *next;
      soap->http_extra_header = NULL; /* use http_extra_header once (assign new value before each call) */
      while ((next = strstr(header, "\r\n")) != NULL)
      {
        if (next > header && next < header + sizeof(soap->tmpbuf))
        {
          soap_strncpy(soap->tmpbuf, sizeof(soap->tmpbuf), header, next - header);
          if (*soap->tmpbuf)
          {
            data->hdr = curl_slist_append(data->hdr, soap->tmpbuf);
            if (!data->hdr)
              return soap->error = SOAP_EOM;
          }
        }
        header = next + 2;
      }
      if (*header)
      {
        data->hdr = curl_slist_append(data->hdr, header);
        if (!data->hdr)
          return soap->error = SOAP_EOM;
      }
    }
    if (data->hdr)
      curl_easy_setopt(data->curl, CURLOPT_HTTPHEADER, (void*)data->hdr);
    curl_easy_setopt(data->curl, CURLOPT_READFUNCTION, soap_curl_read_callback);
    curl_easy_setopt(data->curl, CURLOPT_READDATA, (void*)data);
  }
  curl_easy_setopt(data->curl, CURLOPT_WRITEFUNCTION, soap_curl_write_callback);
  curl_easy_setopt(data->curl, CURLOPT_WRITEDATA, (void*)data);
  curl_easy_setopt(data->curl, CURLOPT_ERRORBUFFER, data->buf);
  curl_easy_setopt(data->curl, CURLOPT_NOSIGNAL, 1L);
  if (soap->connect_timeout > 0)
    curl_easy_setopt(data->curl, CURLOPT_CONNECTTIMEOUT, (long)soap->connect_timeout);
  else if (soap->connect_timeout < 0)
    curl_easy_setopt(data->curl, CURLOPT_CONNECTTIMEOUT_MS, -(long)soap->connect_timeout/1000);
  if (soap->transfer_timeout > 0)
    curl_easy_setopt(data->curl, CURLOPT_TIMEOUT, (long)soap->transfer_timeout);
  else if (soap->send_timeout > 0)
    curl_easy_setopt(data->curl, CURLOPT_TIMEOUT, (long)soap->send_timeout);
  else if (soap->send_timeout < 0)
    curl_easy_setopt(data->curl, CURLOPT_TIMEOUT_MS, -(long)soap->send_timeout/1000);
  else if (soap->recv_timeout > 0)
    curl_easy_setopt(data->curl, CURLOPT_TIMEOUT, (long)soap->recv_timeout);
  else if (soap->recv_timeout < 0)
    curl_easy_setopt(data->curl, CURLOPT_TIMEOUT_MS, -(long)soap->recv_timeout/1000);
  if (soap->proxy_host)
  {
    (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), strlen(soap->proxy_host) + 8 + 8), "http://%s:%d", soap->proxy_host,soap->proxy_port);
    curl_easy_setopt(data->curl, CURLOPT_PROXY, (void*)soap->tmpbuf);
    if (soap->proxy_userid)
    {
      size_t len_proxy_userid = strlen(soap->proxy_userid);
      size_t len_proxy_passwd = soap->proxy_passwd ? strlen(soap->proxy_passwd) : 0;
      if ((len_proxy_userid + len_proxy_passwd) * 3 + 2 < sizeof(soap->tmpbuf))
      {
        size_t pos = 0;
        size_t i;
        for (i = 0; i < len_proxy_userid; i++)
        {
          (SOAP_SNPRINTF(soap->tmpbuf + pos, sizeof(soap->tmpbuf) - pos, 4), "%%%02x", (int)((unsigned char)soap->proxy_userid[i]));
          pos += 3;
        }
        soap->tmpbuf[pos] = ':';
        pos++;
        for (i = 0; i < len_proxy_passwd; i++)
        {
          (SOAP_SNPRINTF(soap->tmpbuf + pos, sizeof(soap->tmpbuf) - pos, 4), "%%%02x", (int)((unsigned char)soap->proxy_passwd[i]));
          pos += 3;
        }
        soap->tmpbuf[pos] = '\0';
        curl_easy_setopt(data->curl, CURLOPT_PROXYUSERPWD, (void*)soap->tmpbuf);
      }
    }
  }
  soap->omode &= ~SOAP_IO;       /* reset IO modes */
  soap->omode |= SOAP_IO_BUFFER; /* buffer the output */
  soap->omode |= SOAP_ENC_PLAIN; /* no HTTP headers */
  soap->omode &= ~SOAP_ENC_ZLIB;
  /* store data sent by engine in an isolated blist */
  if (!(data->lst = soap_alloc_block(soap)))
    return soap->error;
  soap->blist = soap->blist->next;
  /* activate callbacks */
  data->active = 1;
  return SOAP_OK;
}

/**
@fn int soap_curl_send_callback(struct soap *soap, const char *buf, size_t len)
@brief The fsend callback invokes this function to override sending data by saving in a blist.
@param soap context
@param buf data to send
@param len number of bytes to send
@return SOAP_OK or error code
*/
static int soap_curl_send_callback(struct soap *soap, const char *buf, size_t len)
{
  struct soap_curl_data *data = (struct soap_curl_data*)soap_lookup_plugin(soap, soap_curl_id);
  char *blk;
  DBGFUN1("soap_curl_send_callback", "len=%zu", len);
  if (!data)
    return soap->error = SOAP_PLUGIN_ERROR;
  if (!data->active)
    return data->fsend(soap, buf, len);
  if (!data->curl || !data->lst)
    return soap->error = SOAP_PLUGIN_ERROR;
  if (len > 0)
  {
    blk = (char*)soap_push_block(soap, data->lst, len);
    if (!blk)
      return soap->error;
    (void)soap_memcpy((void*)blk, len, (const void*)buf, len);
  }
  return SOAP_OK;
}

/**
@fn int soap_curl_prepare_init_recv_callback(struct soap *soap)
@brief The fprepareinitrecv callback invokes this function to override the start of receiving data and ending of sending.
@param soap context
@return SOAP_OK or error code
*/
static int soap_curl_prepare_init_recv_callback(struct soap *soap)
{
  struct soap_curl_data *data = (struct soap_curl_data*)soap_lookup_plugin(soap, soap_curl_id);
  long status;
  const char *s = NULL;
  CURLcode res;
  DBGFUN("soap_curl_prepare_init_recv_callback");
  if (!data || !data->curl)
    return soap->error = SOAP_PLUGIN_ERROR;
  if (!data->active || !data->lst)
  {
    if (data->fprepareinitrecv)
      return data->fprepareinitrecv(soap);
    return SOAP_OK;
  }
  if ((data->mode & SOAP_IO) == SOAP_IO_CHUNK)
  {
    /* HTTP chunking mode was set with soap_init1() */
    data->hdr = curl_slist_append(data->hdr, "Transfer-Encoding: chunked");
    if (!data->hdr)
    {
      soap->error = SOAP_EOM;
      return 0;
    }
    curl_easy_setopt(data->curl, CURLOPT_HTTPHEADER, data->hdr);
  }
  else
  {
    /* content length is the size of the message saved */
    curl_easy_setopt(data->curl, CURLOPT_POSTFIELDSIZE, data->lst->size);
  }
  if ((data->mode & SOAP_ENC_ZLIB))
  {
    /* enable all supported built-in compressions */
    curl_easy_setopt(data->curl, CURLOPT_ACCEPT_ENCODING, "");
  }
  data->hdr = curl_slist_append(data->hdr, "Expect:"); /* remove Expect: 100 */
  if (data->hdr)
  {
    curl_easy_setopt(data->curl, CURLOPT_HTTPHEADER, (void*)data->hdr);
    res = curl_easy_perform(data->curl);
    curl_slist_free_all(data->hdr);
    data->hdr = NULL;
    if (res != CURLE_OK)
      return soap_sender_fault(soap, curl_easy_strerror(res), "origin: soap_curl plugin");
  }
  curl_easy_getinfo(data->curl, CURLINFO_RESPONSE_CODE, &status);
  if (!curl_easy_getinfo(data->curl, CURLINFO_CONTENT_TYPE, &s) && s)
    soap->http_content = soap_strdup(soap, s);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "cURL HTTP response code %ld content type %s\n", status, s ? s : ""));
  soap->status = (int)status;
  if ((soap->status >= 200 && soap->status <= 299) /* OK, Accepted, etc */
   || soap->status == 400                          /* Bad Request */
   || soap->status == 500)                         /* Internal Server Error */
  {
    if (data->fprepareinitrecv)
      return data->fprepareinitrecv(soap);
    return SOAP_OK;
  }
  /* read HTTP body for error details and return HTTP error */
  return soap_set_receiver_error(soap, "HTTP Error", soap_http_get_body(soap, NULL), soap->status);
}

/**
@fn int soap_curl_prepare_final_recv_callback(struct soap *soap)
@brief The fpreparefinalrecv callback resets the recv callback.
@param soap context
@return SOAP_OK or error code
*/
static int soap_curl_prepare_final_recv_callback(struct soap *soap)
{
  struct soap_curl_data *data = (struct soap_curl_data*)soap_lookup_plugin(soap, soap_curl_id);
  DBGFUN("soap_curl_prepare_final_recv_callback");
  if (!data)
    return soap->error = SOAP_PLUGIN_ERROR;
  /* deactivate callbacks */
  data->active = 0;
  if (data->fpreparefinalrecv)
    return data->fpreparefinalrecv(soap);
  return SOAP_OK;
}

/**
@fn size_t soap_curl_recv_callback(struct soap *soap, char *buf, size_t size)
@brief The frecv callback invokes this function to override receiving data that is stored in a blist.
@param soap context
@param buf receive in this buffer
@param size buffer size
@return number of bytes read, 0 for end or error
*/
static size_t soap_curl_recv_callback(struct soap *soap, char *buf, size_t size)
{
  struct soap_curl_data *data = (struct soap_curl_data*)soap_lookup_plugin(soap, soap_curl_id);
  size_t len;
  DBGFUN1("soap_curl_recv_callback", "size=%zu", size);
  if (!data)
  {
    soap->error = SOAP_PLUGIN_ERROR;
    return 0;
  }
  if (!data->active)
    return data->frecv(soap, buf, size);
  if (!data->lst)
  {
    long status;
    curl_easy_getinfo(data->curl, CURLINFO_RESPONSE_CODE, &status);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "cURL HTTP response code %ld\n", status));
    soap->error = (int)status;
    return 0;
  }
  if (!data->blk)
  {
    data->ptr = data->blk = soap_first_block(data->soap, data->lst);
    if (!data->blk)
    {
      soap_end_block(soap, data->lst);
      data->lst = NULL;
      return 0;
    }
  }
  len = soap_block_size(soap, data->lst) - (data->ptr - data->blk);
  if (len > size)
    len = size;
  (void)soap_memcpy((void*)buf, size, (const void*)data->ptr, len);
  data->ptr += len;
  if (data->ptr >= data->blk + soap_block_size(data->soap, data->lst))
  {
    data->ptr = data->blk = soap_next_block(data->soap, data->lst);
    if (!data->blk)
    {
      soap_end_block(soap, data->lst);
      data->lst = NULL;
    }
  }
  soap->length += len;
  return len;
}

/******************************************************************************\
 *
 *      CURL callbacks
 *
\******************************************************************************/

/**
@fn size_t soap_curl_read_callback(void *buffer, size_t size, size_t nitems, void *ptr)
@brief The CURL read callback invokes this function to read data to send.
@param buffer read data into this buffer, buffer is size*nitems large.
@param size
@param nitems
@param ptr points to soap_curl_data plugin data
@return number of bytes read, 0 for end or error
*/
static size_t soap_curl_read_callback(void *buffer, size_t size, size_t nitems, void *ptr)
{
  struct soap_curl_data *data = (struct soap_curl_data*)ptr;
  struct soap *soap = data->soap;
  size_t len;
  DBGFUN2("soap_curl_read_callback", "size=%zu", size, "nitems=%zu", nitems);
  if (!data->lst)
    return 0;
  if (!data->blk)
  {
    data->ptr = data->blk = soap_first_block(soap, data->lst);
    if (!data->blk)
    {
      soap_end_block(soap, data->lst);
      data->lst = NULL;
      return 0;
    }
  }
  len = soap_block_size(soap, data->lst) - (data->ptr - data->blk);
  if (len > size * nitems)
    len = size * nitems;
  (void)soap_memcpy((void*)buffer, size * nitems, (const void*)data->ptr, len);
  data->ptr += len;
  if (data->ptr >= data->blk + soap_block_size(soap, data->lst))
  {
    data->ptr = data->blk = soap_next_block(soap, data->lst);
    if (!data->blk)
    {
      soap_end_block(soap, data->lst);
      data->lst = NULL;
    }
  }
  return len;
}

/**
@fn size_t soap_curl_write_callback(void *buffer, size_t size, size_t nitems, void *ptr)
@brief The CURL write callback invokes this function to write data that was received.
@param buffer data to write of size*nitems bytes total.
@param size
@param nitems
@param ptr points to soap_curl_data plugin data
@return number of bytes written, 0 for error
*/
static size_t soap_curl_write_callback(void *buffer, size_t size, size_t nitems, void *ptr)
{
  struct soap_curl_data *data = (struct soap_curl_data*)ptr;
  struct soap *soap = data->soap;
  size_t len = size * nitems;
  char *s;
  DBGFUN2("soap_curl_write_callback", "size=%zu", size, "nitems=%zu", nitems);
  if (!data->lst)
  {
    /* store data received in an isolated blist */
    if (!(data->lst = soap_alloc_block(soap)))
      return 0;
    soap->blist = soap->blist->next;
  }
  s = (char*)soap_push_block(soap, data->lst, len);
  if (!s)
    return 0;
  (void)soap_memcpy((void*)s, len, buffer, len);
  return len;
}

#ifdef __cplusplus
}
#endif
