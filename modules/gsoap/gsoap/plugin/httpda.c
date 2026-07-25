/*
        httpda.c

        gSOAP HTTP Digest Authentication plugin.
        Adds support for digest authentication RFC2617 and the RFC7616 draft.

gSOAP XML Web services tools
Copyright (C) 2000-2016, Robert van Engelen, Genivia Inc., All Rights Reserved.
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
Copyright (C) 2000-2016, Robert van Engelen, Genivia, Inc., All Rights Reserved.
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

@mainpage

- @ref httpda documents the http_da plugin for HTTP Digest Authentication.

*/

/**

@page httpda The HTTP-DA plugin

[TOC]

@section httpda_0 Introduction

The upgraded HTTP digest authentication plugin for gSOAP adds support
for the RFC7616 draft that is backwards compatible with RFC2617.  The new
plugin adds SHA-256 (and SHA-512/256 when OpenSSL supports it) algorithms,
including the -sess variants.  To maintain backwards compatibility with RFC2617
the MD5 algorithm is still supported but not recommended.

HTTP **digest authentication** does not transmit the user id and password for
authentication.  Instead, a server negotiates credentials (username and/or
password) with a client using cryptographic hashing algorithms with nonce
values to prevent replay attacks.

By contrast, HTTP **basic authentication** is not safe over unencrypted
channels because the password is transmitted to the server unencrypted.
Therefore, this mechanism provides no confidentiality protection for the
transmitted credentials.  HTTPS is typically preferred over or used in
conjunction with HTTP basic authentication.

To support HTTP digest authentication in favor of HTTP basic authentication,
you will need to install OpenSSL and follow these steps to build your projects:

- Compile your project that uses gSOAP source code with `-DWITH_OPENSSL`.
- Link libgsoapssl (libgsoapssl++), or use the `stdsoap2.c[pp]` source.
- Compile and link your code together with `plugin/httpda.c`, `plugin/smdevp.c`,
  and `plugin/threads.c`

The plugin is MT-safe by means of internal mutex locks.  Mutex ensures
exclusive access and updates of the shared session store with nonces to prevent
replay attacks.

@section httpda_1 Client-side usage

HTTP basic authentication is the default authentication mechanism supported by
gSOAP.  You can set the basic authentication credentials at the client-side
with:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    soap.userid = "<userid>";
    soap.passed = "<passwd>";
    if (soap_call_ns__method(&soap, ...))
      ... // error
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

or if you use a proxy object generated with saopcpp2 option -j:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    Proxy proxy(...);
    proxy.soap->userid = "<userid>";
    proxy.soap->passed = "<passwd>";
    if (proxy.method(...))
      ... // error
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

HTTP basic authentication should **never** be used over plain HTTP, because the
credentials (the ID and password) are transmitted in the clear in base64
encoded form which is easily reversible.  This mechanism is safer to use over
HTTPS, because the HTTP headers and body are encrypted.

This upgraded HTTP digest authentication plugin supports RFC7616 and RFC2617.
RFC7616 adds SHA2 and is backwards compatible to clients that use MD5.  The MD5
algorithm is not allowed in FIPS making SHA-256 or SHA-512-256 digest
algorithms mandatory.  The client-side of the plugin handles both RFCs
automatically.

To use HTTP digest authentication with gSOAP, register the http_da plugin as
follows:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "httpda.h"
    struct soap *soap = soap_new();
    soap_register_plugin(soap, http_da);
    ...
    soap_destroy(soap); // deallocate data
    soap_end(soap);     // deallocate temp data
    soap_free(soap);    // deregister plugin and deallocate context
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

or if you use a proxy object generated with saopcpp2 option -j:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "httpda.h"
    Proxy proxy(...);
    soap_register_plugin(proxy.soap, http_da);
    ...
    proxy.destroy(); // deallocate data and temps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To make a client-side service call you will need to create a digest store
`http_da_info`.  The store holds the digest information locally on your machine
to manage repeated authentication challenges from all servers you connect to.
Use `http_da_save()` to add credentials to the store and release the store with
`http_da_release()` when you no longer need the credentials.

The `http_da_info` store is intended for one thread to issue a sequence of
calls that are all authenticated without requiring (re)negotiation.  You should
not share the `http_da_info` store with multiple threads, unless you use mutex
locks.

Here is an example:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    struct soap *soap = soap_new();
    soap_register_plugin(soap, http_da);
    struct http_da_info info;

    if (soap_call_ns__method(soap, ...)) // make a call without authentication
    {
      if (soap.error == 401) // HTTP authentication is required
      {
        http_da_save(soap, &info, "<authrealm>", "<userid>", "<passwd>");
        if (soap_call_ns__method(soap, ...)) // make a call with authentication
          ... // error
        http_da_release(soap, &info); // release if auth is no longer needed
      }
      else
        ... // other error
    }

    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Again, if you use a proxy object then replace the `soap_call_ns__method` with
the proxy method invocation, as was shown earlier.

The `<authrealm>` string is the protected realm of the server that requires
authorization.  This string can be obtained with the `soap.authrealm` string
after an unsuccessful non-authenticated call so you can use it to save
credentials to the digest store:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    struct soap *soap = soap_new();
    soap_register_plugin(soap, http_da);
    struct http_da_info info;

    if (soap_call_ns__method(soap, ...)) // make a call without authentication
    {
      if (soap.error == 401) // HTTP authentication is required
      {
        const char *realm = soap.authrealm;
        http_da_save(soap, &info, realm, "<userid>", "<passwd>");
        if (soap_call_ns__method(soap, ...)) // make a call with authentication
          ... // error
	...
        http_da_release(soap, &info); // deallocate authentication info if auth is no longer needed
      }
      else
        ... // error
    }

    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Before a second call is made to the same endpoint that requires authentication,
you must restore the authentication state with `http_da_restore()`, then use
it, and finally release it with `http_da_release()`:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    struct soap *soap = soap_new();
    soap_register_plugin(soap, http_da);
    struct http_da_info info;
    bool auth = false;

    if (soap_call_ns__method(soap, ...)) // make a call without authentication
    {
      if (soap.error == 401) // HTTP authentication is required
      {
        http_da_save(soap, &info, "<authrealm>", "<userid>", "<passwd>");
        auth = true;
      }
      else
        ... // other error
    }

    if (auth)
      http_da_restore(soap, &info);
    if (soap_call_ns__method(soap, ...)) // make a call with authentication
      ... // error

    soap_destroy(soap); // okay to dealloc data
    soap_end(soap);     // okay to dealloc data

    if (auth)
      http_da_restore(soap, &info);
    if (soap_call_ns__method(soap, ...)) // make a call with authentication
      ... // error

    if (auth)
      http_da_release(soap, &info); // deallocate authentication info

    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For HTTP proxies requiring HTTP digest authenticaiton, use the 'proxy'
functions of the plugin:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    struct soap *soap = soap_new();
    soap_register_plugin(soap, http_da);
    struct http_da_info info;

    if (soap_call_ns__method(soap, ...)) // make a call without authentication
    {
      if (soap.error == 407) // HTTP proxy authentication is required
      {
        http_da_proxy_save(soap, &info, "<authrealm>", "<userid>", "<passwd>");
        auth = true;
      }
      else
        ... // error
    }

    if (auth)
      http_da_proxy_restore(soap, &info);
    if (soap_call_ns__method(soap, ...))
      ... // error

    http_da_proxy_release(soap, &info); // deallocate authentication info

    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

@section httpda_2 Client example

A client authenticating against a server:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    struct soap *soap = soap_new();
    soap_register_plugin(soap, http_da);

    if (soap_call_ns__method(soap, ...)) // make a call without authentication
    {
      if (soap.error == 401) // HTTP authentication is required
      {
        if (!strcmp(soap.authrealm, authrealm)) // check authentication realm
        {
          struct http_da_info info; // to store userid and passwd
          http_da_save(soap, &info, authrealm, userid, passwd);
          // call again, now with credentials
          if (soap_call_ns__method(soap, ...) == SOAP_OK)
          {
            ... // process response data
            soap_end(soap);
            ... // userid and passwd were deallocated (!)
            http_da_restore(soap, &info); // get userid and passwd after soap_end()
            if (!soap_call_ns__method(soap, ...) == SOAP_OK)
              ... // error
            http_da_release(soap, &info); // deallocate authentication info
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A client authenticating against a proxy:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    struct soap *soap = soap_new();
    soap_register_plugin(soap, http_da);

    if (soap_call_ns__method(soap, ...))  // make a call without authentication
    {
      if (soap.error == 407) // HTTP authentication is required
      {
        if (!strcmp(soap.authrealm, authrealm)) // check authentication realm
        {
          struct http_da_info info; // to store userid and passwd
          http_da_proxy_save(soap, &info, authrealm, userid, passwd);
          // call again, now with credentials
          if (soap_call_ns__method(soap, ...) == SOAP_OK)
          {
            ... // process response data
            soap_end(soap);
            ... // userid and passwd were deallocated (!)
            http_da_proxy_restore(soap, &info); // get userid and passwd after soap_end()
            if (!soap_call_ns__method(soap, ...) == SOAP_OK)
              ... // error
            http_da_proxy_release(soap, &info); // deallocate authentication info
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

@section httpda_3 Server-side usage

As explained in the gSOAP user guid, server-side HTTP basic authentication is
enforced by simply checking the `soap.userid` and `soap.passwd` values in a
service method that requires client authentication:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    struct soap *soap = soap_new();
    soap_register_plugin(soap, http_da);
    ...
    soap_serve(soap); // see gSOAP documentation and examples on how to serve requests
    ...

    int ns__method(struct soap *soap, ...)
    {
      if (!soap->userid || !soap->passwd || strcmp(soap->userid, "<userid>") || strcmp(soap->passwd, "<passwd>"))
        return 401; // HTTP authentication required
      ...
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

HTTP digest authentication is verified differently, because digests are
compared, not passwords:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    struct soap *soap = soap_new();
    soap_register_plugin(soap, http_da);
    ...
    soap_serve(soap); // see gSOAP documentation and examples on how to serve requests
    ...

    int ns__method(struct soap *soap, ...)
    {
      if (soap->authrealm && soap->userid)
      {
        passwd = ... // database lookup on userid and authrealm to find passwd
        if (!strcmp(soap->authrealm, authrealm) && !strcmp(soap->userid, userid))
        { 
          if (!http_da_verify_post(soap, passwd)) // HTTP POST DA verification
          {
            ... // process request and produce response
            return SOAP_OK;
          }
        }
      }
      soap->authrealm = authrealm; // realm to send to client
      return 401; // Not authorized, challenge with digest authentication
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The `http_da_verify_post()` function checks the HTTP POST credentials by
computing and comparing a digest of the password.  The HTTP POST method is used
for two-way SOAP/XML communications with request and response messages.
One-way SOAP/XML messaging may use HTTP POST or HTTP GET.  To verify an HTTP
GET operation, use `http_da_verify_get()` instead, for example in the HTTP GET
handler implemented with the HTTP GET plugin.  Likewise, use `http_da_verify_put()`
for HTTP PUT, `http_da_verify_patch()` for HTTP PATCH, and `http_da_verify_del()`
for HTTP DELETE.  To verify other HTTP methods, use the `http_da_verify_method()`
function with the method as a string argument, for example:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    if (!http_da_verify_method(soap, "HEAD", passwd))
    {
      ... // process the request
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Server-side operations that handle other methods than HTTP POST and GET, such
as PATCH, PUT, and DELETE should be implemented with the HTTP POST plugin.
This plugin uses a dispatch table with a handler corresponding to the HTTP
method to serve.

RFC7616 recommends SHA2 over MD5.  The MD5 algorithm is not allowed in FIPS and
SHA-256 or SHA-512-256 are mandatory.  This upgraded HTTP digest plugin uses
SHA-256 as the default algorithm and reverts to MD5 only if required by a
client that does not support RFC7616.

The default SHA-256 digest algorithm is enabled automatically.  However, at the
server side you can also use a plugin registry option to set a different
algorithm as the default:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    soap_register_plugin_arg(soap, http_da, <option>);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

where `<option>` is one of:

- `http_da_md5()` MD5 (not recommended).
- `http_da_md5_sess()` MD5-sess (not recommended).
- `http_da_sha256()` SHA-256 (recommended).
- `http_da_sha256_sess()` SHA-256-sess (recommended).
- `http_da_sha512_256()` SHA-512-256 (not yet supported).
- `http_da_sha512_256_sess()` SHA-512-256-sess (not yet supported).

When non-MD5 option is selected, the server will present that digest algorithm
together with the MD5 authentication algorithm as challenge to the client.  If
the client is upgraded to RFC7616 it selects the newer protocol.  If the client
is not upgraded it will select the older MD5-based protocol.

To revert to RFC2617 use `http_da_md5()`.

@section httpda_4 Server example

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    struct soap *soap = soap_new();
    soap_register_plugin(soap, http_da);
    ...
    soap_serve(soap); // see gSOAP documentation and examples on how to serve requests
    ...

    int ns__method(struct soap *soap, ...)
    {
      if (soap->userid && soap->passwd) // Basic authentication
      {
        if (!strcmp(soap->userid, userid) && !strcmp(soap->passwd, passwd))
        {
          ... // can also check soap->authrealm 
          ... // process request and produce response
          return SOAP_OK;
        }
      }
      else if (soap->authrealm && soap->userid) // Digest authentication
      {
        passwd = ... // database lookup on userid and authrealm to find passwd
        if (!strcmp(soap->authrealm, authrealm) && !strcmp(soap->userid, userid))
        { 
          if (!http_da_verify_post(soap, passwd)) // HTTP POST DA verification
          {
            ... // process request and produce response
            return SOAP_OK;
          }
        }
      }
      soap->authrealm = authrealm; // realm to send to client
      return 401; // Not authorized, challenge with digest authentication
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

@section httpda_5 Limitations

HTTP digest authentication cannot be used with streaming MTOM/MIME/DIME
attachments.  Non-streaming MTOM/MIME/DIME attachments are handled just fine.
MTOM/MIM/DIME attachment streaming is automatically turned off by the plugin
and attachment data is buffered rather than streamed.  

*/

#include "httpda.h"

#ifdef __cplusplus
extern "C" {
#endif

const char http_da_id[] = HTTP_DA_ID;

/** HTTP DA session database */
static struct http_da_session *http_da_session = NULL;

/** HTTP DA session database lock */
static MUTEX_TYPE http_da_session_lock = MUTEX_INITIALIZER;

#define HTTP_DA_NONCELEN (21)
#define HTTP_DA_OPAQUELEN (9)

/******************************************************************************\
 *
 *      Forward decls
 *
\******************************************************************************/

static int http_da_init(struct soap *soap, struct http_da_data *data, int *arg);
static int http_da_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src);
static void http_da_delete(struct soap *soap, struct soap_plugin *p);

static int http_da_post_header(struct soap *soap, const char *key, const char *val);
static int http_da_parse(struct soap *soap);
static int http_da_parse_header(struct soap *soap, const char *key, const char *val);
static int http_da_prepareinitsend(struct soap *soap);
static int http_da_prepareinitrecv(struct soap *soap);
static int http_da_preparesend(struct soap *soap, const char *buf, size_t len);
static int http_da_preparerecv(struct soap *soap, const char *buf, size_t len);
static int http_da_preparefinalrecv(struct soap *soap);

static void http_da_session_start(const char *realm, const char *nonce, const char *opaque);
static int http_da_session_update(const char *realm, const char *nonce, const char *opaque, const char *cnonce, const char *ncount);
static void http_da_session_cleanup();

static void http_da_calc_nonce(struct soap *soap, char nonce[HTTP_DA_NONCELEN]);
static void http_da_calc_opaque(struct soap *soap, char opaque[HTTP_DA_OPAQUELEN]);
static int http_da_calc_HA1(struct soap *soap, struct soap_smd_data *smd_data, const char *alg, const char *userid, const char *realm, const char *passwd, const char *nonce, const char *cnonce, char HA1hex[65]);
static int http_da_calc_response(struct soap *soap, struct soap_smd_data *smd_data, const char *alg, char HA1hex[65], const char *nonce, const char *ncount, const char *cnonce, const char *qop, const char *method, const char *uri, char entityHAhex[65], char response[65], char responseHA[32]);

/******************************************************************************\
 *
 *      Plugin registry
 *
\******************************************************************************/

SOAP_FMAC1
int
SOAP_FMAC2
http_da(struct soap *soap, struct soap_plugin *p, void *arg)
{
  (void)arg;
  p->id = http_da_id;
  p->data = (void*)SOAP_MALLOC(soap, sizeof(struct http_da_data));
  p->fcopy = http_da_copy;
  p->fdelete = http_da_delete;
  if (!p->data)
    return SOAP_EOM;
  if (http_da_init(soap, (struct http_da_data*)p->data, (int*)arg))
  {
    SOAP_FREE(soap, p->data);
    return SOAP_EOM;
  }
  return SOAP_OK;
}

/******************************************************************************/

static int
http_da_init(struct soap *soap, struct http_da_data *data, int *arg)
{
  data->fposthdr = soap->fposthdr;
  soap->fposthdr = http_da_post_header;
  data->fparse = soap->fparse;
  soap->fparse = http_da_parse;
  data->fparsehdr = soap->fparsehdr;
  soap->fparsehdr = http_da_parse_header;
  data->fprepareinitsend = soap->fprepareinitsend;
  soap->fprepareinitsend = http_da_prepareinitsend;
  data->fprepareinitrecv = soap->fprepareinitrecv;
  soap->fprepareinitrecv = http_da_prepareinitrecv;
  if (arg && *arg >= 0 && *arg <= 5)
    data->option = *arg;
  else
    data->option = 2; /* SHA-256 by default */
  data->smd_data.ctx = NULL;
  memset((void*)data->digest, 0, sizeof(data->digest));
  data->nonce = NULL;
  data->opaque = NULL;
  data->qop = NULL;
  data->alg = NULL;
  data->nc = 0;
  data->ncount = NULL;
  data->cnonce = NULL;
  memset((void*)data->response, 0, sizeof(data->response));

  return SOAP_OK;
}

/******************************************************************************/

static int
http_da_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src)
{
  (void)soap;
  dst->data = (void*)SOAP_MALLOC(soap, sizeof(struct http_da_data));
  if (!dst->data)
    return SOAP_EOM;
  (void)soap_memcpy((void*)dst->data, sizeof(struct http_da_data), (const void*)src->data, sizeof(struct http_da_data));
  ((struct http_da_data*)dst->data)->smd_data.ctx = NULL;
  memset((void*)((struct http_da_data*)dst->data)->digest, 0, sizeof(((struct http_da_data*)dst->data)->digest));
  ((struct http_da_data*)dst->data)->nonce = NULL;
  ((struct http_da_data*)dst->data)->opaque = NULL;
  ((struct http_da_data*)dst->data)->qop = NULL;
  ((struct http_da_data*)dst->data)->alg = NULL;
  ((struct http_da_data*)dst->data)->nc = 0;
  ((struct http_da_data*)dst->data)->ncount = NULL;
  ((struct http_da_data*)dst->data)->cnonce = NULL;
  memset((void*)((struct http_da_data*)dst->data)->response, 0, sizeof(((struct http_da_data*)dst->data)->response));
  return SOAP_OK;
}

/******************************************************************************/

static void
http_da_delete(struct soap *soap, struct soap_plugin *p)
{
  if (((struct http_da_data*)p->data)->smd_data.ctx)
    soap_smd_final(soap, &((struct http_da_data*)p->data)->smd_data, NULL, NULL);
  SOAP_FREE(soap, p->data);
}

/******************************************************************************\
 *
 *      Plugin registry options
 *
\******************************************************************************/

/**
@brief Use soap_register_plugin_arg(soap, http_da, http_da_md5()) for MD5 server-side challenge algorithm (not recommended).
*/
SOAP_FMAC1
void *
SOAP_FMAC2
http_da_md5()
{
  static int option = 0;
  return (void*)&option;
}

/******************************************************************************/

/**
@brief Use soap_register_plugin_arg(soap, http_da, http_da_md5_sess()) for MD5-sess server-side challenge algorithm (not recommended).
*/
SOAP_FMAC1
void *
SOAP_FMAC2
http_da_md5_sess()
{
  static int option = 1;
  return (void*)&option;
}

/******************************************************************************/

/**
@brief Use soap_register_plugin_arg(soap, http_da, http_da_sha256()) for MD5 server-side challenge algorithm (recommended).
*/
SOAP_FMAC1
void *
SOAP_FMAC2
http_da_sha256()
{
  static int option = 2;
  return (void*)&option;
}

/******************************************************************************/

/**
@brief Use soap_register_plugin_arg(soap, http_da, http_da_sha256_sess()) for MD5-sess server-side challenge algorithm (recommended).
*/
SOAP_FMAC1
void *
SOAP_FMAC2
http_da_sha256_sess()
{
  static int option = 3;
  return (void*)&option;
}

/******************************************************************************/

/**
@brief Use soap_register_plugin_arg(soap, http_da, http_da_sha512_256()) for MD5 server-side challenge algorithm (not yet supported).
*/
SOAP_FMAC1
void *
SOAP_FMAC2
http_da_sha512_256()
{
  static int option = 4;
  return (void*)&option;
}

/******************************************************************************/

/**
@brief Use soap_register_plugin_arg(soap, http_da, http_da_sha512_256_sess()) for MD5-sess server-side challenge algorithm (not yet supported).
*/
SOAP_FMAC1
void *
SOAP_FMAC2
http_da_sha512_256_sess()
{
  static int option = 5;
  return (void*)&option;
}

/******************************************************************************\
 *
 *      Callbacks
 *
\******************************************************************************/

static int
http_da_post_header(struct soap *soap, const char *key, const char *val)
{
  struct http_da_data *data = (struct http_da_data*)soap_lookup_plugin(soap, http_da_id);

  if (!data)
    return SOAP_PLUGIN_ERROR;

  /* client's HTTP Authorization request */
  if (key && (!strcmp(key, "Authorization") || !strcmp(key, "Proxy-Authorization")))
  {
    char HA1hex[65], entityHAhex[65], response[65], responseHA[32];
    char cnonce[HTTP_DA_NONCELEN];
    char ncount[9];
    const char *qop, *method;
    const char *userid = (*key == 'A' ? soap->userid : soap->proxy_userid);
    const char *passwd = (*key == 'A' ? soap->passwd : soap->proxy_passwd);
    size_t smd_len = 16;

    if (data->alg && !soap_tag_cmp(data->alg, "SHA-256*"))
      smd_len = 32;

    if (soap_smd_final(soap, &data->smd_data, data->digest, NULL))
      return soap->error;

    if (userid && passwd && soap->authrealm && data->nonce)
    {
      http_da_calc_nonce(soap, cnonce);

      if (http_da_calc_HA1(soap, &data->smd_data, data->alg, userid, soap->authrealm, passwd, data->nonce, cnonce, HA1hex))
	return soap->error;

      if (soap->status != SOAP_GET && soap->status != SOAP_CONNECT && data->qop && !soap_tag_cmp(data->qop, "*auth-int*"))
      {
	qop = "auth-int";
	(void)soap_s2hex(soap, (unsigned char*)data->digest, entityHAhex, smd_len);
      }
      else if (data->qop)
      {
	qop = "auth";
      }
      else
      {
	qop = NULL;
      }

      switch (soap->status)
      {
        case SOAP_GET:
          method = "GET";
          break;
        case SOAP_PUT:
          method = "PUT";
          break;
        case SOAP_PATCH:
          method = "PATCH";
          break;
        case SOAP_DEL:
          method = "DELETE";
          break;
        case SOAP_HEAD:
          method = "HEAD";
          break;
        case SOAP_OPTIONS:
          method = "OPTIONS";
          break;
        case SOAP_CONNECT:
          method = "CONNECT";
          break;
        default:
          method = "POST";
      }

      (SOAP_SNPRINTF(ncount, sizeof(ncount), 8), "%8.8lx", data->nc++);

      if (http_da_calc_response(soap, &data->smd_data, data->alg, HA1hex, data->nonce, ncount, cnonce, qop, method, soap->path, entityHAhex, response, responseHA))
	return soap->error;

      if (qop)
      {
	(SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), strlen(soap->authrealm) + strlen(userid) + strlen(data->nonce) + strlen(soap->path) + strlen(qop) + strlen(ncount) + strlen(cnonce) + strlen(response) + 83), "Digest algorithm=%s, realm=\"%s\", username=\"%s\", nonce=\"%s\", uri=\"%s\", qop=\"%s\", nc=%s, cnonce=\"%s\", response=\"%s\"", data->alg ? data->alg : "MD5", soap->authrealm, userid, data->nonce, soap->path, qop, ncount, cnonce, response);
      }
      else
      {
	(SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), strlen(soap->authrealm) + strlen(userid) + strlen(data->nonce) + strlen(soap->path) + strlen(ncount) + strlen(cnonce) + strlen(response) + 59), "Digest algorithm=%s, realm=\"%s\", username=\"%s\", nonce=\"%s\", uri=\"%s\", response=\"%s\"", data->alg ? data->alg : "MD5", soap->authrealm, userid, data->nonce, soap->path, response);
      }

      if (data->opaque)
      {
	size_t l = strlen(soap->tmpbuf);
	(SOAP_SNPRINTF(soap->tmpbuf + l, sizeof(soap->tmpbuf) - l, strlen(data->opaque) + 11), ", opaque=\"%s\"", data->opaque);
      }

      return data->fposthdr(soap, key, soap->tmpbuf);
    }
  }

  /* server's HTTP Authorization challenge/response */
  if (key && (!strcmp(key, "WWW-Authenticate") || !strcmp(key, "Proxy-Authenticate")))
  {
    static const char *algos[] = { "MD5", "MD5-sess", "SHA-256", "SHA-256-sess", "SHA-512-256", "SHA-512-256-sess" };
    const char *alg = algos[data->option];
    char nonce[HTTP_DA_NONCELEN];
    char opaque[HTTP_DA_OPAQUELEN];

    http_da_calc_nonce(soap, nonce);
    http_da_calc_opaque(soap, opaque);

    http_da_session_start(soap->authrealm, nonce, opaque);

    if (data->option > 0)
    {
      (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), strlen(soap->authrealm) + strlen(nonce) + strlen(opaque) + 59), "Digest algorithm=%s, realm=\"%s\", qop=\"auth,auth-int\", nonce=\"%s\", opaque=\"%s\"", alg, soap->authrealm, nonce, opaque);
      if (data->fposthdr(soap, key, soap->tmpbuf))
        return soap->error;
    }

    (SOAP_SNPRINTF(soap->tmpbuf, sizeof(soap->tmpbuf), strlen(soap->authrealm) + strlen(nonce) + strlen(opaque) + 59), "Digest algorithm=MD5, realm=\"%s\", qop=\"auth,auth-int\", nonce=\"%s\", opaque=\"%s\"", soap->authrealm, nonce, opaque);

    return data->fposthdr(soap, key, soap->tmpbuf);
  }

  return data->fposthdr(soap, key, val);
}

/******************************************************************************/

static int
http_da_parse(struct soap *soap)
{ 
  struct http_da_data *data = (struct http_da_data*)soap_lookup_plugin(soap, http_da_id);

  if (!data)
    return SOAP_PLUGIN_ERROR;

  data->qop = NULL;

  /* HTTP GET w/o body with qop=auth-int still requires a digest */
  if (soap_smd_init(soap, &data->smd_data, SOAP_SMD_DGST_MD5, NULL, 0)
   || soap_smd_final(soap, &data->smd_data, data->digest, NULL))
    return soap->error;

  soap->error = data->fparse(soap);
  if (soap->error)
    return soap->error;

  if (data->qop && !soap_tag_cmp(data->qop, "auth-int"))
  {
    if (soap->fpreparerecv != http_da_preparerecv)
    {
      data->fpreparerecv = soap->fpreparerecv;
      soap->fpreparerecv = http_da_preparerecv;
    }
    if (soap->fpreparefinalrecv != http_da_preparefinalrecv)
    {
      data->fpreparefinalrecv = soap->fpreparefinalrecv;
      soap->fpreparefinalrecv = http_da_preparefinalrecv;
    }

    if (soap_smd_init(soap, &data->smd_data, SOAP_SMD_DGST_MD5, NULL, 0))
      return soap->error;
  }
 
  return SOAP_OK;
}

/******************************************************************************/

static int
http_da_parse_header(struct soap *soap, const char *key, const char *val)
{
  struct http_da_data *data = (struct http_da_data*)soap_lookup_plugin(soap, http_da_id);

  if (!data)
    return SOAP_PLUGIN_ERROR;

  /* check if server received Authorization Digest HTTP header from client */
  if (!soap_tag_cmp(key, "Authorization") && !soap_tag_cmp(val, "Digest *"))
  {
    data->alg = soap_strdup(soap, soap_http_header_attribute(soap, val + 7, "algorithm"));
    soap->authrealm = soap_strdup(soap, soap_http_header_attribute(soap, val + 7, "realm"));
    soap->userid = soap_strdup(soap, soap_http_header_attribute(soap, val + 7, "username"));
    soap->passwd = NULL;
    data->nonce = soap_strdup(soap, soap_http_header_attribute(soap, val + 7, "nonce"));
    data->opaque = soap_strdup(soap, soap_http_header_attribute(soap, val + 7, "opaque"));
    data->qop = soap_strdup(soap, soap_http_header_attribute(soap, val + 7, "qop"));
    data->ncount = soap_strdup(soap, soap_http_header_attribute(soap, val + 7, "nc"));
    data->cnonce = soap_strdup(soap, soap_http_header_attribute(soap, val + 7, "cnonce"));
    (void)soap_hex2s(soap, soap_http_header_attribute(soap, val + 7, "response"), data->response, 32, NULL);
    return SOAP_OK;
  }

  /* check if client received WWW-Authenticate Digest HTTP header from server */
  if ((!soap_tag_cmp(key, "WWW-Authenticate") || !soap_tag_cmp(key, "Proxy-Authenticate")) && !soap_tag_cmp(val, "Digest *"))
  {
    const char *authrealm = soap_http_header_attribute(soap, val + 7, "realm");
    if (authrealm && (!soap->authrealm || strcmp(authrealm, soap->authrealm)))
    {
      const char *alg;
      soap->authrealm = soap_strdup(soap, authrealm);
      alg = soap_http_header_attribute(soap, val + 7, "algorithm");
      if (!alg || soap_tag_cmp(alg, "SHA-512-256*"))
      {
        /* got the first authenticate header for this realm that we can accept */
        data->alg = soap_strdup(soap, alg);
        data->nonce = soap_strdup(soap, soap_http_header_attribute(soap, val + 7, "nonce"));
        data->opaque = soap_strdup(soap, soap_http_header_attribute(soap, val + 7, "opaque"));
        data->qop = soap_strdup(soap, soap_http_header_attribute(soap, val + 7, "qop"));
        data->nc = 1;
        data->ncount = NULL;
        data->cnonce = NULL;
      }
      else
      {
        soap->authrealm = NULL;
      }
    }
    return SOAP_OK;
  }

  return data->fparsehdr(soap, key, val);
}

/******************************************************************************/

static int
http_da_prepareinitsend(struct soap *soap)
{
  struct http_da_data *data = (struct http_da_data*)soap_lookup_plugin(soap, http_da_id);

  if (!data)
    return SOAP_PLUGIN_ERROR;

  if ((soap->mode & SOAP_IO) != SOAP_IO_STORE && (soap->mode & (SOAP_ENC_DIME | SOAP_ENC_MIME)))
  {
    /* support non-streaming MIME/DIME attachments by buffering the message */
    soap->omode &= ~SOAP_IO;
    soap->omode |= SOAP_IO_STORE;
    soap->mode &= ~SOAP_IO;
    soap->mode |= SOAP_IO_STORE;
  }
  else
  {
    if ((soap->userid && soap->passwd)
     || (soap->proxy_userid && soap->proxy_passwd))
    {
      if (soap_smd_init(soap, &data->smd_data, SOAP_SMD_DGST_MD5, NULL, 0))
        return soap->error;
      if (soap->fpreparesend != http_da_preparesend)
      {
        data->fpreparesend = soap->fpreparesend;
        soap->fpreparesend = http_da_preparesend;
      }
      if ((soap->mode & SOAP_IO) == SOAP_IO_CHUNK)
        soap->mode |= SOAP_IO_LENGTH;
    }
    else
    {
      if (soap->fpreparesend == http_da_preparesend)
      {
        soap->fpreparesend = data->fpreparesend;
      }
    }
  }

  if (data->fprepareinitsend)
    return data->fprepareinitsend(soap);

  return SOAP_OK;
}

/******************************************************************************/

static int
http_da_prepareinitrecv(struct soap *soap)
{
  struct http_da_data *data = (struct http_da_data*)soap_lookup_plugin(soap, http_da_id);

  if (!data)
    return SOAP_PLUGIN_ERROR;

  if (soap->fpreparerecv == http_da_preparerecv)
    soap->fpreparerecv = data->fpreparerecv;
  if (soap->fpreparefinalrecv == http_da_preparefinalrecv)
    soap->fpreparefinalrecv = data->fpreparefinalrecv;

  if (data->fprepareinitrecv)
    return data->fprepareinitrecv(soap);

  return SOAP_OK;
}

/******************************************************************************/

static int
http_da_preparesend(struct soap *soap, const char *buf, size_t len)
{
  struct http_da_data *data = (struct http_da_data*)soap_lookup_plugin(soap, http_da_id);

  if (!data)
    return SOAP_PLUGIN_ERROR;

  if (soap_smd_update(soap, &data->smd_data, buf, len))
    return soap->error;

  if (data->fpreparesend)
    return data->fpreparesend(soap, buf, len);

  return SOAP_OK;
}

/******************************************************************************/

static int
http_da_preparerecv(struct soap *soap, const char *buf, size_t len)
{
  struct http_da_data *data = (struct http_da_data*)soap_lookup_plugin(soap, http_da_id);

  if (!data)
    return SOAP_PLUGIN_ERROR;

  if (soap_smd_update(soap, &data->smd_data, buf, len))
    return soap->error;

  if (data->fpreparerecv)
    return data->fpreparerecv(soap, buf, len);

  return SOAP_OK;
}

/******************************************************************************/

static int
http_da_preparefinalrecv(struct soap *soap)
{
  struct http_da_data *data = (struct http_da_data*)soap_lookup_plugin(soap, http_da_id);

  if (!data)
    return SOAP_PLUGIN_ERROR;

  if (soap_smd_final(soap, &data->smd_data, data->digest, NULL))
    return soap->error;

  soap->fpreparerecv = data->fpreparerecv;
  soap->fpreparefinalrecv = data->fpreparefinalrecv;

  if (soap->fpreparefinalrecv)
    return soap->fpreparefinalrecv(soap);

  return SOAP_OK;
}

/******************************************************************************\
 *
 *      Client-side digest authentication state management
 *
\******************************************************************************/

/**
@brief Saves the credentials to the digest store to use repeatedly for authentication.
@param soap context
@param info a pointer to the digest store
@param realm string (e.g. use soap->authrealm) of the server
@param userid the user ID string
@param passwd the user password string
*/
SOAP_FMAC1
void
SOAP_FMAC2
http_da_save(struct soap *soap, struct http_da_info *info, const char *realm, const char *userid, const char *passwd)
{
  struct http_da_data *data = (struct http_da_data*)soap_lookup_plugin(soap, http_da_id);
  if (!data)
    return;

  soap->authrealm = info->authrealm = soap_strdup(NULL, realm);
  info->userid = soap_strdup(NULL, userid);
  soap->userid = info->userid;
  info->passwd = soap_strdup(NULL, passwd);
  soap->passwd = info->passwd;
  info->nonce = soap_strdup(NULL, data->nonce);
  info->opaque = soap_strdup(NULL, data->opaque);
  info->qop = soap_strdup(NULL, data->qop);
  info->alg = soap_strdup(NULL, data->alg);
}

/******************************************************************************/

/**
@brief Saves the credentials to the digest store to use repeatedly for proxy authentication.
@param soap context
@param info a pointer to the digest store
@param realm string (e.g. use soap->authrealm) of the server
@param userid the user ID string
@param passwd the user password string
*/
SOAP_FMAC1
void
SOAP_FMAC2
http_da_proxy_save(struct soap *soap, struct http_da_info *info, const char *realm, const char *userid, const char *passwd)
{
  struct http_da_data *data = (struct http_da_data*)soap_lookup_plugin(soap, http_da_id);
  if (!data)
    return;

  soap->authrealm = info->authrealm = soap_strdup(NULL, realm);
  info->userid = soap_strdup(NULL, userid);
  soap->proxy_userid = info->userid;
  info->passwd = soap_strdup(NULL, passwd);
  soap->proxy_passwd = info->passwd;
  info->nonce = soap_strdup(NULL, data->nonce);
  info->opaque = soap_strdup(NULL, data->opaque);
  info->qop = soap_strdup(NULL, data->qop);
  info->alg = soap_strdup(NULL, data->alg);
}

/******************************************************************************/

/**
@brief Retrieves the credentials from the digest store to use for authentication.
@param soap context
@param info a pointer to the digest store
*/
SOAP_FMAC1
void
SOAP_FMAC2
http_da_restore(struct soap *soap, struct http_da_info *info)
{
  struct http_da_data *data = (struct http_da_data*)soap_lookup_plugin(soap, http_da_id);
  if (!data)
    return;
  soap->authrealm = info->authrealm;
  soap->userid = info->userid;
  soap->passwd = info->passwd;
  data->nonce = info->nonce;
  data->opaque = info->opaque;
  data->qop = info->qop;
  data->alg = info->alg;
}

/******************************************************************************/

/**
@brief Retrieves the credentials from the digest store to use for proxy authentication.
@param soap context
@param info a pointer to the digest store
*/
SOAP_FMAC1
void
SOAP_FMAC2
http_da_proxy_restore(struct soap *soap, struct http_da_info *info)
{
  struct http_da_data *data = (struct http_da_data*)soap_lookup_plugin(soap, http_da_id);
  if (!data)
    return;
  soap->authrealm = info->authrealm;
  soap->proxy_userid = info->userid;
  soap->proxy_passwd = info->passwd;
  data->nonce = info->nonce;
  data->opaque = info->opaque;
  data->qop = info->qop;
  data->alg = info->alg;
}

/******************************************************************************/

/**
@brief Releases the digest store and frees memory
@param soap context
@param info a pointer to the digest store
*/
SOAP_FMAC1
void
SOAP_FMAC2
http_da_release(struct soap *soap, struct http_da_info *info)
{
  struct http_da_data *data = (struct http_da_data*)soap_lookup_plugin(soap, http_da_id);
  if (!data)
    return;

  soap->authrealm = NULL;
  soap->userid = NULL;
  soap->passwd = NULL;

  data->nonce = NULL;
  data->opaque = NULL;
  data->qop = NULL;
  data->alg = NULL;

  if (info->authrealm)
  {
    free((void*)info->authrealm);
    info->authrealm = NULL;
  }
  if (info->userid)
  {
    free((void*)info->userid);
    info->userid = NULL;
  }
  if (info->passwd)
  {
    free((void*)info->passwd);
    info->passwd = NULL;
  }
  if (info->nonce)
  {
    free((void*)info->nonce);
    info->nonce = NULL;
  }
  if (info->opaque)
  {
    free((void*)info->opaque);
    info->opaque = NULL;
  }
  if (info->qop)
  {
    free((void*)info->qop);
    info->qop = NULL;
  }
  if (info->alg)
  {
    free((void*)info->alg);
    info->alg = NULL;
  }
}

/******************************************************************************/

/**
@brief Releases the digest store for proxy authentication and frees memory
@param soap context
@param info a pointer to the digest store
*/
SOAP_FMAC1
void
SOAP_FMAC2
http_da_proxy_release(struct soap *soap, struct http_da_info *info)
{
  soap->proxy_userid = NULL;
  soap->proxy_passwd = NULL;

  http_da_release(soap, info);
}

/******************************************************************************\
 *
 *      Server-side digest authentication verification
 *
\******************************************************************************/

/**
@brief Verifies the password credentials at the server side when used in an HTTP POST service operation.
@param soap context
@param passwd the user password string
@return SOAP_OK or error when verification failed
*/
SOAP_FMAC1
int
SOAP_FMAC2
http_da_verify_post(struct soap *soap, const char *passwd)
{
  return http_da_verify_method(soap, "POST", passwd);
}

/******************************************************************************/

/**
@brief Verifies the password credentials at the server side when used in an HTTP GET service operation.
@param soap context
@param passwd the user password string
@return SOAP_OK or error when verification failed
*/
SOAP_FMAC1
int
SOAP_FMAC2
http_da_verify_get(struct soap *soap, const char *passwd)
{
  return http_da_verify_method(soap, "GET", passwd);
}

/******************************************************************************/

/**
@brief Verifies the password credentials at the server side when used in an HTTP PUT service operation.
@param soap context
@param passwd the user password string
@return SOAP_OK or error when verification failed
*/
SOAP_FMAC1
int
SOAP_FMAC2
http_da_verify_put(struct soap *soap, const char *passwd)
{
  return http_da_verify_method(soap, "PUT", passwd);
}

/******************************************************************************/

/**
@brief Verifies the password credentials at the server side when used in an HTTP PATCH service operation.
@param soap context
@param passwd the user password string
@return SOAP_OK or error when verification failed
*/
SOAP_FMAC1
int
SOAP_FMAC2
http_da_verify_patch(struct soap *soap, const char *passwd)
{
  return http_da_verify_method(soap, "PATCH", passwd);
}

/******************************************************************************/

/**
@brief Verifies the password credentials at the server side when used in an HTTP DELETE service operation.
@param soap context
@param passwd the user password string
@return SOAP_OK or error when verification failed
*/
SOAP_FMAC1
int
SOAP_FMAC2
http_da_verify_del(struct soap *soap, const char *passwd)
{
  return http_da_verify_method(soap, "DELETE", passwd);
}

/******************************************************************************/

/**
@brief Verifies the password credentials at the server side when used in the specified HTTP method.
@param soap context
@param method the HTTP method, e.g. "POST", "GET", "PUT", "DELETE", "PATCH", "HEAD", "OPTIONS"
@param passwd the user password string
@return SOAP_OK or error when verification failed
*/
SOAP_FMAC1
int
SOAP_FMAC2
http_da_verify_method(struct soap *soap, const char *method, const char *passwd)
{
  struct http_da_data *data = (struct http_da_data*)soap_lookup_plugin(soap, http_da_id);
  char HA1hex[65], entityHAhex[65], response[65], responseHA[32];
  size_t smd_len = 16;

  if (!data)
    return SOAP_ERR;

  if (data->alg && !soap_tag_cmp(data->alg, "SHA-256*"))
    smd_len = 32;

  /* reject if none or basic authentication was used */
  if (!soap->authrealm
   || !soap->userid
   || soap->passwd)     /* passwd is set when basic auth is used */
    return SOAP_ERR;

  /* require at least qop="auth" to prevent replay attacks */
  if (!data->qop)
    return SOAP_ERR;

  if (http_da_session_update(soap->authrealm, data->nonce, data->opaque, data->cnonce, data->ncount))
    return SOAP_ERR;

  if (http_da_calc_HA1(soap, &data->smd_data, data->alg, soap->userid, soap->authrealm, passwd, data->nonce, data->cnonce, HA1hex))
    return soap->error;

  if (!soap_tag_cmp(data->qop, "auth-int"))
    (void)soap_s2hex(soap, (unsigned char*)data->digest, entityHAhex, smd_len);

  if (http_da_calc_response(soap, &data->smd_data, data->alg, HA1hex, data->nonce, data->ncount, data->cnonce, data->qop, method, soap->path, entityHAhex, response, responseHA))
    return soap->error;

  /* check digest response values */
  if (memcmp(data->response, responseHA, smd_len))
    return SOAP_ERR;

  return SOAP_OK;
}

/******************************************************************************\
 *
 *      Digest authentication session database management
 *
\******************************************************************************/

static void
http_da_session_start(const char *realm, const char *nonce, const char *opaque)
{
  struct http_da_session *session;
  time_t now = time(NULL);

  if (now % 10 == 0) /* don't do this all the time to improve efficiency */
    http_da_session_cleanup();

#ifdef SOAP_DEBUG
  fprintf(stderr, "Starting session realm=%s nonce=%s\n", realm, nonce);
#endif

  MUTEX_LOCK(http_da_session_lock);

  session = (struct http_da_session*)malloc(sizeof(struct http_da_session));
  if (session)
  {
    session->next = http_da_session;
    session->modified = now;
    session->realm = soap_strdup(NULL, realm);
    session->nonce = soap_strdup(NULL, nonce);
    session->opaque = soap_strdup(NULL, opaque);
    session->nc = 0;
    http_da_session = session;
  }

  MUTEX_UNLOCK(http_da_session_lock);
}

/******************************************************************************/

static int
http_da_session_update(const char *realm, const char *nonce, const char *opaque, const char *cnonce, const char *ncount)
{
  struct http_da_session *session;

  if (!realm || !nonce || !opaque || !cnonce || !ncount)
  {
#ifdef SOAP_DEBUG
    fprintf(stderr, "Debug message: authentication update failed, missing authentication data\n");
#endif
    return SOAP_ERR;
  }
#ifdef SOAP_DEBUG
  fprintf(stderr, "Debug message: updating session realm=%s nonce=%s\n", realm, nonce);
#endif

  MUTEX_LOCK(http_da_session_lock);

  for (session = http_da_session; session; session = session->next)
    if (session->realm && session->nonce && session->opaque && !strcmp(session->realm, realm) && !strcmp(session->nonce, nonce) && !strcmp(session->opaque, opaque))
      break;

  if (session)
  {
    unsigned long nc = soap_strtoul(ncount, NULL, 16);

    if (session->nc >= nc)
    {
      session->modified = 0; /* replay attack: terminate session */
      session = NULL;
    }
    else
    {
      session->nc = nc;
      session->modified = time(NULL);
    }
  }

  MUTEX_UNLOCK(http_da_session_lock);

  if (!session)
    return SOAP_ERR;

  return SOAP_OK;
}

/******************************************************************************/

static void
http_da_session_cleanup()
{
  struct http_da_session **session;
  time_t now = time(NULL);

  MUTEX_LOCK(http_da_session_lock);

  session = &http_da_session;
  while (*session)
  {
    if ((*session)->modified + HTTP_DA_SESSION_TIMEOUT < now)
    {
      struct http_da_session *p = *session;

#ifdef SOAP_DEBUG
      fprintf(stderr, "Deleting session realm=%s nonce=%s\n", p->realm, p->nonce);
#endif

      if (p->realm)
        free((void*)p->realm);
      if (p->nonce)
        free((void*)p->nonce);
      if (p->opaque)
        free((void*)p->opaque);

      *session = p->next;
      free((void*)p);
    }
    else
      session = &(*session)->next;
  }

  MUTEX_UNLOCK(http_da_session_lock);
}

/******************************************************************************\
 *
 *      Calculate hex nonce and opaque values
 *
\******************************************************************************/

static void
http_da_calc_nonce(struct soap *soap, char nonce[HTTP_DA_NONCELEN])
{
  static unsigned short count = 0xCA53;
  (void)soap;
  (SOAP_SNPRINTF(nonce, HTTP_DA_NONCELEN, 20), "%8.8x%4.4hx%8.8x", (unsigned int)time(NULL), count++, (unsigned int)soap_random);
}

/******************************************************************************/

static void
http_da_calc_opaque(struct soap *soap, char opaque[HTTP_DA_OPAQUELEN])
{
  (void)soap;
  (SOAP_SNPRINTF(opaque, HTTP_DA_OPAQUELEN, 8), "%8.8x", (unsigned int)soap_random);
}

/******************************************************************************\
 *
 *      Calculate HA1, HA2, and response digest as per RFC2617 and RFC7617
 *
\******************************************************************************/

static int
http_da_calc_HA1(struct soap *soap, struct soap_smd_data *smd_data, const char *alg, const char *userid, const char *realm, const char *passwd, const char *nonce, const char *cnonce, char HA1hex[65])
{
  int smd_alg = SOAP_SMD_DGST_MD5;
  size_t smd_len = 16;
  char HA1[32];

  if (alg && !soap_tag_cmp(alg, "SHA-256*"))
  {
    smd_alg = SOAP_SMD_DGST_SHA256;
    smd_len = 32;
  }

  if (soap_smd_init(soap, smd_data, smd_alg, NULL, 0)
   || soap_smd_update(soap, smd_data, userid, strlen(userid))
   || soap_smd_update(soap, smd_data, ":", 1)
   || soap_smd_update(soap, smd_data, realm, strlen(realm))
   || soap_smd_update(soap, smd_data, ":", 1)
   || soap_smd_update(soap, smd_data, passwd, strlen(passwd))
   || soap_smd_final(soap, smd_data, HA1, NULL))
    return soap->error;

  if (alg && !soap_tag_cmp(alg, "*-sess"))
  {
    if (soap_smd_init(soap, smd_data, smd_alg, NULL, 0)
     || soap_smd_update(soap, smd_data, HA1, smd_len))
      return soap->error;

    if (nonce)
    {
      if (soap_smd_update(soap, smd_data, ":", 1)
       || soap_smd_update(soap, smd_data, nonce, strlen(nonce)))
        return soap->error;
    }

    if (soap_smd_update(soap, smd_data, ":", 1)
     || soap_smd_update(soap, smd_data, cnonce, strlen(cnonce))
     || soap_smd_final(soap, smd_data, HA1, NULL))
      return soap->error;
  }

  (void)soap_s2hex(soap, (unsigned char*)HA1, HA1hex, smd_len);

  return SOAP_OK;
};

/******************************************************************************/

static int
http_da_calc_response(struct soap *soap, struct soap_smd_data *smd_data, const char *alg, char HA1hex[65], const char *nonce, const char *ncount, const char *cnonce, const char *qop, const char *method, const char *uri, char entityHAhex[65], char response[65], char responseHA[32])
{
  int smd_alg = SOAP_SMD_DGST_MD5;
  size_t smd_len = 16;
  char HA2[32], HA2hex[65];

  if (alg && !soap_tag_cmp(alg, "SHA-256*"))
  {
    smd_alg = SOAP_SMD_DGST_SHA256;
    smd_len = 32;
  }

  if (soap_smd_init(soap, smd_data, smd_alg, NULL, 0)
   || soap_smd_update(soap, smd_data, method, strlen(method))
   || soap_smd_update(soap, smd_data, ":", 1)
   || soap_smd_update(soap, smd_data, uri, strlen(uri)))
    return soap->error;

  if (qop && !soap_tag_cmp(qop, "auth-int"))
  {
    if (soap_smd_update(soap, smd_data, ":", 1)
     || soap_smd_update(soap, smd_data, entityHAhex, 2*smd_len))
      return soap->error;
  }

  if (soap_smd_final(soap, smd_data, HA2, NULL))
    return soap->error;

  (void)soap_s2hex(soap, (unsigned char*)HA2, HA2hex, smd_len);

  if (soap_smd_init(soap, smd_data, smd_alg, NULL, 0)
   || soap_smd_update(soap, smd_data, HA1hex, 2*smd_len))
    return soap->error;

  if (nonce)
  {
    if (soap_smd_update(soap, smd_data, ":", 1)
     || soap_smd_update(soap, smd_data, nonce, strlen(nonce)))
      return soap->error;
  }

  if (qop && *qop)
  {
    if (soap_smd_update(soap, smd_data, ":", 1)
     || soap_smd_update(soap, smd_data, ncount, strlen(ncount))
     || soap_smd_update(soap, smd_data, ":", 1)
     || soap_smd_update(soap, smd_data, cnonce, strlen(cnonce))
     || soap_smd_update(soap, smd_data, ":", 1)
     || soap_smd_update(soap, smd_data, qop, strlen(qop)))
      return soap->error;
  }

  if (soap_smd_update(soap, smd_data, ":", 1)
   || soap_smd_update(soap, smd_data, HA2hex, 2*smd_len)
   || soap_smd_final(soap, smd_data, responseHA, NULL))
    return soap->error;

  (void)soap_s2hex(soap, (unsigned char*)responseHA, response, smd_len);

  return SOAP_OK;
}

/******************************************************************************/

#ifdef __cplusplus
}
#endif
