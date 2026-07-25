/*
        wsseapi-lite.c

        WS-Security, lite version (time stamp and user name token only).

gSOAP XML Web services tools
Copyright (C) 2000-2015, Robert van Engelen, Genivia Inc., All Rights Reserved.
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
Copyright (C) 2000-2015, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

- @ref wsse documents the wsse lite API for WS-Security 1.0/1.1 support.

*/

/**

@page wsse WS-Security lite

[TOC]

@section wsse_5 Security Header

The material in this section relates to the WS-Security specification section 5.

To use the wsse lite API:
-# Run wsdl2h -t typemap.dat on a WSDL of a service that requires WS-Security
   headers. The typemap.dat file is used to recognize and translate Security
   header blocks.
-# Run soapcpp2 on the header file produced by wsdl2h.
-# Use the wsse lite API functions described below to add time stamp and user
   name tokens.

If HTTPS is required with OpenSSL then please follow the instructions in
Section @ref wsse_11 to ensure thread-safety of WS-Security with HTTPS.

The wsse lite API is located in:

- `gsoap/plugin/wsseapi-lite.h` wsse lite API.
- `gsoap/plugin/wsseapi-lite.c` wsse lite API for C and C++.

You will also need:

- `gsoap/custom/struct_timeval.c` compile and link this file (C and C++).
- compile all sources with `-DWITH_OPENSSL` to enable HTTPS.
- if you have zlib installed, compile all sources also with `-DWITH_GZIP`.
- link with `-lssl -lcrypto -lz -gsoapssl++` (or `-lgsoapssl` for C, or compile `stdsoap2.cpp` for C++ and `stdsoap2.c` for C).

The gSOAP header file for soapcpp2 should import wsse.h (or the older 2002
version wsse2.h):

@code
    #import "wsse.h"
@endcode

The wsdl2h tool adds the necessary imports to the generated header file if the
WSDL declares the use of WS-Security. If not, you may have to add the import
manually before running soapcpp2.

The wsse lite API consists of a set of functions to populate and verify
WS-Security headers and message body content. For more details, we refer to the
following sections that correspond to the WS-Security specification sections:

- Section 6 @ref wsse_6
- Section 10 @ref wsse_10
- @ref wsse_11

The basic API is introduced below.

To add an empty Security header block to the SOAP header, use:

@code
    soap_wsse_add_Security(soap);
@endcode

To delete a Security header, use:

@code
    soap_wsse_delete_Security(soap);
@endcode

Adding an empty Security header block is not very useful. In the following, we
present the higher-level functions of the wsse lite API to populate and verify
Security header content.

@note
The soap context includes an actor value soap.actor that is populated and
rendered as the SOAP-ENV:actor (SOAP 1.1) or SOAP-ENV:role (SOAP 1.2) attribute
in XML within the generic SOAP Header. The attribute is optional, but should be
used to target a recipient such as an intermediate node to process the SOAP
header.  In contrast, actor or role attributes within Security header blocks
target specific recipients to process the Security header block. The gSOAP
implementation does not automate this feature and application should set and
check the actor/role attribute when necessary. In addition, the current
implementation supports the inclusion of a single Security header block in the
SOAP header.

To populate the SOAP-ENV:actor or SOAP-ENV:role attribute within the Security
header, use:

@code
    soap_wsse_add_Security_actor(soap, "recipient");
@endcode

To obtain the actor or role value (e.g. after receiving a message), use:

@code
    _wsse__Security *security = soap_wsse_Security(soap);
    if (security)
    {
      ... = security->SOAP_ENV__actor; // SOAP 1.1
      ... = security->SOAP_ENV__role;  // SOAP 1.2
@endcode

The SOAP-ENV:mustUnderstand attribute is automatically added and checked by the
gSOAP engine. A gSOAP application compiled without Security support will reject
Security headers.

Security header blocks are attached to the soap context, which means that the
information will be automatically kept to support multiple invocations.

@section wsse_6 Security Tokens

The material in this section relates to the WS-Security specification section 6.

@subsection wsse_6_2 User Name Token

To add a user name token to the Security header block, use:

@code
    soap_wsse_add_UsernameTokenText(soap, "Id", "username", NULL);
@endcode

The `Id` value is optional and not used in the wsse lite API. These `Id`s are
serialized as wsu:Id identifiers for cross-referencing XML elements.

To add a user name token with clear text password, use:

@code
    soap_wsse_add_UsernameTokenText(soap, "Id", "username", "password");
@endcode

It is strongly recommended to use `soap_wsse_add_UsernameTokenText` only in
combination with HTTPS encrypted transmission or not at all. A better
alternative is to use password digests (not supported in this wsse lite API).

Clear-text passwords are verified with `soap_wsse_verify_Password`. To
verify a password at the receiving side to authorize a request (e.g. within a
Web service operation), use:

@code
    int ns__myMethod(struct soap *soap, ...)
    {
      const char *username = soap_wsse_get_Username(soap);
      const char *password;
      if (!username)
        return soap->error; // no username: return FailedAuthentication (from soap_wsse_get_Username)
      password = ...; // lookup password of username
      if (soap_wsse_verify_Password(soap, password))
      {
        int err = soap->error;
        soap_wsse_delete_Security(soap); // remove old security headers
        return err; // password verification failed: return FailedAuthentication
      }
      return SOAP_OK;
    }
@endcode

Note that the `soap_wsse_get_Username` functions sets the
wsse:FailedAuthentication fault upon failure. It is common for the API
functions functions to return SOAP_OK or a wsse fault that should be passed to
the sender by returning soap->error from service operations. The fault is
displayed with the `soap_print_fault` function.

@section wsse_10 Security Timestamps

The material in this section relates to the WS-Security specification section
10.

To add a timestamp with the creation time to the Security header, use:

@code
    soap_wsse_add_Timestamp(soap, NULL, 0); // no expiration
@endcode

The lifetime of a message (in seconds) is passed as the third argument, which
will be displayed as the timestamp expiration time:

@code
    soap_wsse_add_Timestamp(soap, NULL, 10); // 10 seconds lifetime
@endcode

@section wsse_11 WS-Security and HTTPS

HTTPS is used at the client side with the usual "https:" URL addressing, shown
here:

@code
    #include "wsseapi-lite.h"
    #include "threads.h"
    struct soap *soap;
    if (CRYPTO_thread_setup())
      ... // error
    soap = soap_new1(SOAP_XML_CANONICAL | SOAP_XML_INDENT);
    if (soap_ssl_client_context(&soap,
      SOAP_SSL_DEFAULT, // requires server authentication
      NULL,             // keyfile for client authentication to server
      NULL,             // the keyfile password
      "cacerts.pem",    // cafile CA certificates to authenticate the server
      NULL,             // capath CA directory path to certificates
      NULL
    ))
      ... // error
    soap->cafile = "cacerts.pem";  // same as above (or overrides the above)
    soap->capath = "dir/to/certs"; // and/or point to CA certs
    soap->crlfile = "revoked.pem"; // use CRL (optional)
    soap_wsse_delete_Security(soap); // remove any previous header content
    soap_wsse_add_UsernameTokenText(soap, NULL, "username", "password");
    soap_wsse_add_Timestamp(soap, NULL, 10); // 10 seconds lifetime
    if (soap_call_ns__myMethod(soap, "https://...", ...))
      ... // error
    ... // process response results
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
    CRYPTO_thread_cleanup();
@endcode

With OpenSSL, the CRYPTO threads should be set up before any threads are
created.

The `soap_ssl_client_context` only needs to be set up once. Use the following
flags:

- `SOAP_SSL_DEFAULT` requires server authentication, CA certs should be used
- `SOAP_SSL_NO_AUTHENTICATION` disables server authentication
- `SOAP_SSL_SKIP_HOST_CHECK` disables server authentication host check
- `SOAP_SSL_ALLOW_EXPIRED_CERTIFICATE` to accept self-signed certificates, expired certificates, and certificates without CRL.

The server uses the following:

@code
    #include "wsseapi-lite.h"
    #include "threads.h"
    SOAP_SOCKET m, s;
    int port = 443;
    struct soap *soap;
    if (CRYPTO_thread_setup())
      ... // error
    soap = soap_new1(SOAP_XML_CANONICAL | SOAP_XML_INDENT);
    if (soap_ssl_server_context(&soap,
      SOAP_SSL_DEFAULT, // requires server to authenticate, but not the client
      server.pem,       // keyfile for authentication to client
      "password",       // the keyfile password
      NULL,             // CA certificates to authenticate the client
      NULL,             // CA directory path to certificates
      NULL,             // use RSA 2048 bits (or give file name with DH param)
      NULL,
      NULL
    ))
      ... // error
    if (!soap_valid_socket(m = soap_bind(soap, NULL, port, 100))
      ... // error
    for (;;)
    {
      if (!soap_valid_socket(s = soap_accept(soap)))
        ... // error
      else
      {
        struct soap *tsoap = soap_copy(soap);
        while (THREAD_CREATE(&tid, (void*(*)(void*))&process_request, (void*)tsoap))
          sleep(1);
      }
    }
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
    CRYPTO_thread_cleanup();
@endcode

where we define a process_request function that is executed by the thread to
process the request (on a copy of the soap context struct):

@code
  void *process_request(struct soap *soap)
  {
    if (soap_ssl_accept(soap)
     || soap_serve(soap))
      ... // error
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
  }
@endcode

The `soap_ssl_server_context` only needs to be set up once. Use the following
flags:

- `SOAP_SSL_DEFAULT` requires server authentication, but no client authentication
- `SOAP_SSL_REQUIRE_CLIENT_AUTHENTICATION` requires client authentication

With OpenSSL, we need to define the thread set up and clean up operations as
follows:

@code
  struct CRYPTO_dynlock_value
  {
    MUTEX_TYPE mutex;
  };

  static MUTEX_TYPE *mutex_buf;

  static struct CRYPTO_dynlock_value *dyn_create_function(const char *file, int line)
  {
    struct CRYPTO_dynlock_value *value;
    value = (struct CRYPTO_dynlock_value*)malloc(sizeof(struct CRYPTO_dynlock_value));
    if (value)
      MUTEX_SETUP(value->mutex);
    return value;
  }

  static void dyn_lock_function(int mode, struct CRYPTO_dynlock_value *l, const char *file, int line)
  {
    if (mode & CRYPTO_LOCK)
      MUTEX_LOCK(l->mutex);
    else
      MUTEX_UNLOCK(l->mutex);
  }

  static void dyn_destroy_function(struct CRYPTO_dynlock_value *l, const char *file, int line)
  {
    MUTEX_CLEANUP(l->mutex);
    free(l);
  }

  void locking_function(int mode, int n, const char *file, int line)
  {
    if (mode & CRYPTO_LOCK)
      MUTEX_LOCK(mutex_buf[n]);
    else
      MUTEX_UNLOCK(mutex_buf[n]);
  }

  unsigned long id_function()
  {
    return (unsigned long)THREAD_ID;
  }

  int CRYPTO_thread_setup()
  {
    int i;
    mutex_buf = (MUTEX_TYPE*)malloc(CRYPTO_num_locks() * sizeof(pthread_mutex_t));
    if (!mutex_buf)
      return SOAP_EOM;
    for (i = 0; i < CRYPTO_num_locks(); i++)
      MUTEX_SETUP(mutex_buf[i]);
    CRYPTO_set_id_callback(id_function);
    CRYPTO_set_locking_callback(locking_function);
    CRYPTO_set_dynlock_create_callback(dyn_create_function);
    CRYPTO_set_dynlock_lock_callback(dyn_lock_function);
    CRYPTO_set_dynlock_destroy_callback(dyn_destroy_function);
    return SOAP_OK;
  }

  void CRYPTO_thread_cleanup()
  {
    int i;
    if (!mutex_buf)
      return;
    CRYPTO_set_id_callback(NULL);
    CRYPTO_set_locking_callback(NULL);
    CRYPTO_set_dynlock_create_callback(NULL);
    CRYPTO_set_dynlock_lock_callback(NULL);
    CRYPTO_set_dynlock_destroy_callback(NULL);
    for (i = 0; i < CRYPTO_num_locks(); i++)
      MUTEX_CLEANUP(mutex_buf[i]);
    free(mutex_buf);
    mutex_buf = NULL;
  }
@endcode

For additional details and examples, see the user guide and examples in the
gSOAP package directory `gsoap/samples/ssl`.

*/

#include "wsseapi-lite.h"

#if defined(SOAP_WSA_2003) || defined(SOAP_WSA_2004) || defined(SOAP_WSA_200408) || defined(SOAP_WSA_2005)
#include "wsaapi.h"
#endif

/** Clock skew between machines (in sec) to fit message expiration in window */
#define SOAP_WSSE_CLKSKEW       (300)

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************\
 *
 *      Common URIs
 *
\******************************************************************************/

const char *wsse_PasswordTextURI = "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordText";

/******************************************************************************\
 *
 *      wsse:Security header element
 *
\******************************************************************************/

/**
@fn _wsse__Security* soap_wsse_add_Security(struct soap *soap)
@brief Adds Security header element.
@param soap context
@return _wsse__Security object
*/
SOAP_FMAC1
struct _wsse__Security *
SOAP_FMAC2
soap_wsse_add_Security(struct soap *soap)
{
  DBGFUN("soap_wsse_add_Security");
  /* if we don't have a SOAP Header, create one */
  soap_header(soap);
  /* if we don't have a wsse:Security element in the SOAP Header, create one */
  if (!soap->header->wsse__Security)
  {
    soap->header->wsse__Security = (_wsse__Security*)soap_malloc(soap, sizeof(_wsse__Security));
    if (!soap->header->wsse__Security)
      return NULL;
    soap_default__wsse__Security(soap, soap->header->wsse__Security);
  }
  return soap->header->wsse__Security;
}

/******************************************************************************/

/**
@fn _wsse__Security* soap_wsse_add_Security_actor(struct soap *soap, const char *actor)
@brief Adds Security header element with actor or role attribute.
@param soap context
@param actor string
@return _wsse__Security object
*/
SOAP_FMAC1
struct _wsse__Security *
SOAP_FMAC2
soap_wsse_add_Security_actor(struct soap *soap, const char *actor)
{
  _wsse__Security *security = soap_wsse_add_Security(soap);
  DBGFUN1("soap_wsse_add_Security_actor", "actor=%s", actor);
  if (soap->namespaces && !strcmp(soap->namespaces[0].ns, "http://schemas.xmlsoap.org/soap/envelope/"))
    security->SOAP_ENV__actor = soap_strdup(soap, actor);
  else
    security->SOAP_ENV__role = soap_strdup(soap, actor);
  return security;
}

/******************************************************************************/

/**
@fn void soap_wsse_delete_Security(struct soap *soap)
@brief Deletes Security header element.
@param soap context
*/
SOAP_FMAC1
void
SOAP_FMAC2
soap_wsse_delete_Security(struct soap *soap)
{
  DBGFUN("soap_wsse_delete_Security");
  if (soap->header)
    soap->header->wsse__Security = NULL;
}

/******************************************************************************/

/**
@fn _wsse__Security* soap_wsse_Security(struct soap *soap)
@brief Returns Security header element if present.
@param soap context
@return _wsse__Security object or NULL
*/
SOAP_FMAC1
struct _wsse__Security *
SOAP_FMAC2
soap_wsse_Security(struct soap *soap)
{
  if (soap->header)
    return soap->header->wsse__Security;
  return NULL;
}

/******************************************************************************\
 *
 *      wsse:Security/wsu:Timestamp header element
 *
\******************************************************************************/

/**
@fn int soap_wsse_add_Timestamp(struct soap *soap, const char *id, time_t lifetime)
@brief Adds Timestamp element with optional expiration date+time (lifetime).
@param[in] soap context
@param[in] id for signature referencing or NULL
@param[in] lifetime expressed in time_t units, or 0 for no expiration
@return SOAP_OK
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_Timestamp(struct soap *soap, const char *id, time_t lifetime)
{
  _wsse__Security *security = soap_wsse_add_Security(soap);
  time_t now = time(NULL);
  char *created = soap_strdup(soap, soap_dateTime2s(soap, now));
  char *expired = lifetime ? soap_strdup(soap, soap_dateTime2s(soap, now + lifetime)) : NULL;
  DBGFUN1("soap_wsse_add_Timestamp", "id=%s", id?id:"");
  /* allocate a Timestamp if we don't have one already */
  if (!security->wsu__Timestamp)
  {
    security->wsu__Timestamp = (_wsu__Timestamp*)soap_malloc(soap, sizeof(_wsu__Timestamp));
    if (!security->wsu__Timestamp)
      return soap->error = SOAP_EOM;
  }
  soap_default__wsu__Timestamp(soap, security->wsu__Timestamp);
  /* populate the wsu:Timestamp element */
  security->wsu__Timestamp->wsu__Id = soap_strdup(soap, id);
  security->wsu__Timestamp->Created = created;
  security->wsu__Timestamp->Expires = expired;
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn _wsu__Timestamp *soap_wsse_Timestamp(struct soap *soap)
@brief Returns Timestamp element if present.
@param soap context
@return _wsu__Timestamp object or NULL
*/
SOAP_FMAC1
struct _wsu__Timestamp *
SOAP_FMAC2
soap_wsse_Timestamp(struct soap *soap)
{
  _wsse__Security *security = soap_wsse_Security(soap);
  if (security)
    return security->wsu__Timestamp;
  return NULL;
}

/******************************************************************************/

/**
@fn int soap_wsse_verify_Timestamp(struct soap *soap)
@brief Verifies the Timestamp/Expires element against the current time.
@param soap context
@return SOAP_OK or SOAP_FAULT with wsu:MessageExpired fault

Sets wsu:MessageExpired fault if wsu:Timestamp is expired. The
SOAP_WSSE_CLKSKEW value is used as a margin to mitigate clock skew. Keeps
silent when no timestamp is supplied or no expiration date is included in the
wsu:Timestamp element.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_verify_Timestamp(struct soap *soap)
{
  _wsu__Timestamp *timestamp = soap_wsse_Timestamp(soap);
  DBGFUN("soap_wsse_verify_Timestamp");
  /* if we have a timestamp with an expiration date, check it */
  if (timestamp && timestamp->Expires)
  {
    time_t now = time(NULL), expired;
    soap_s2dateTime(soap, timestamp->Expires, &expired);
    if (expired + SOAP_WSSE_CLKSKEW <= now)
    {
      const char *code = soap_wsu__tTimestampFault2s(soap, wsu__MessageExpired);
      return soap_wsse_sender_fault_subcode(soap, code, "Message has expired", timestamp->Expires);
    }
  }
  return SOAP_OK;
}

/******************************************************************************\
 *
 *      wsse:Security/UsernameToken header element
 *
\******************************************************************************/

/**
@fn int soap_wsse_add_UsernameTokenText(struct soap *soap, const char *id, const char *username, const char *password)
@brief Adds UsernameToken element with optional clear-text password.
@param soap context
@param[in] id string for signature referencing or NULL
@param[in] username string
@param[in] password string or NULL to omit the password
@return SOAP_OK

Passwords are sent in the clear, so transport-level encryption is required.

@note
This release supports the use of at most one UsernameToken in the header.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_UsernameTokenText(struct soap *soap, const char *id, const char *username, const char *password)
{
  _wsse__Security *security = soap_wsse_add_Security(soap);
  DBGFUN2("soap_wsse_add_UsernameTokenText", "id=%s", id?id:"", "username=%s", username?username:"");
  /* allocate a UsernameToken if we don't have one already */
  if (!security->UsernameToken)
  {
    security->UsernameToken = (_wsse__UsernameToken*)soap_malloc(soap, sizeof(_wsse__UsernameToken));
    if (!security->UsernameToken)
      return soap->error = SOAP_EOM;
  }
  soap_default__wsse__UsernameToken(soap, security->UsernameToken);
  /* populate the UsernameToken */
  security->UsernameToken->wsu__Id = soap_strdup(soap, id);
  security->UsernameToken->Username = soap_strdup(soap, username);
  /* allocate and populate the Password */
  if (password)
  {
    security->UsernameToken->Password = (_wsse__Password*)soap_malloc(soap, sizeof(_wsse__Password));
    if (!security->UsernameToken->Password)
      return soap->error = SOAP_EOM;
    soap_default__wsse__Password(soap, security->UsernameToken->Password);
    security->UsernameToken->Password->Type = (char*)wsse_PasswordTextURI;
    security->UsernameToken->Password->__item = soap_strdup(soap, password);
  }
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn _wsse__UsernameToken* soap_wsse_UsernameToken(struct soap *soap, const char *id)
@brief Returns UsernameToken element if present.
@param soap context
@param[in] id string of UsernameToken or NULL
@return _wsse__UsernameToken object or NULL

@note
This release supports the use of at most one UsernameToken in the header.
*/
SOAP_FMAC1
struct _wsse__UsernameToken *
SOAP_FMAC2
soap_wsse_UsernameToken(struct soap *soap, const char *id)
{
  _wsse__Security *security = soap_wsse_Security(soap);
  if (security
   && security->UsernameToken
   && (!id || (security->UsernameToken->wsu__Id
            && !strcmp(security->UsernameToken->wsu__Id, id))))
    return security->UsernameToken;
  return NULL;
}

/******************************************************************************/

/**
@fn const char* soap_wsse_get_Username(struct soap *soap)
@brief Returns UsernameToken/username string or wsse:FailedAuthentication fault.
@param soap context
@return UsernameToken/username string or NULL with wsse:FailedAuthentication fault error set
@see soap_wsse_verify_Password

The returned username should be used to lookup the user's password in a
dictionary or database for server-side authentication with
soap_wsse_verify_Password.
*/
SOAP_FMAC1
const char *
SOAP_FMAC2
soap_wsse_get_Username(struct soap *soap)
{
  _wsse__UsernameToken *token = soap_wsse_UsernameToken(soap, NULL);
  DBGFUN("soap_wsse_get_Username");
  if (token)
    return token->Username;
  soap_wsse_fault(soap, wsse__FailedAuthentication, "Username authentication required");
  return NULL;
}

/******************************************************************************/

/**
@fn int soap_wsse_verify_Password(struct soap *soap, const char *password)
@brief Verifies the supplied password or sets wsse:FailedAuthentication fault.
@param soap context
@param[in] password string to verify against
@return SOAP_OK (authorized) or SOAP_FAULT with wsse:FailedAuthentication fault

The verification supports both clear-text password verification only.

@note
This release supports the use of at most one UsernameToken in the header.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_verify_Password(struct soap *soap, const char *password)
{
  _wsse__UsernameToken *token = soap_wsse_UsernameToken(soap, NULL);
  DBGFUN("soap_wsse_verify_Password");
  /* if we have a UsernameToken with a Password, check it */
  if (token && token->Password)
  {
    /* password digest or text? */
    if (token->Password->Type
     && !strcmp(token->Password->Type, wsse_PasswordTextURI))
    {
      /* check password text */
      if (!strcmp(token->Password->__item, password))
        return SOAP_OK;
    }
  }
  return soap_wsse_fault(soap, wsse__FailedAuthentication, NULL);
}

/******************************************************************************\
 *
 *      Faults
 *
\******************************************************************************/

/**
@fn int soap_wsse_sender_fault_subcode(struct soap *soap, const char *faultsubcode, const char *faultstring, const char *faultdetail)
@brief Sets sender SOAP Fault (sub)code for server fault response.
@param soap context
@param[in] faultsubcode sub code string
@param[in] faultstring fault string
@param[in] faultdetail detail string
@return SOAP_FAULT
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_sender_fault_subcode(struct soap *soap, const char *faultsubcode, const char *faultstring, const char *faultdetail)
{
#if defined(SOAP_WSA_2003) || defined(SOAP_WSA_2004) || defined(SOAP_WSA_200408) || defined(SOAP_WSA_2005)
  return soap_wsa_sender_fault_subcode(soap, faultsubcode, faultstring, faultdetail);
#else
  return soap_sender_fault_subcode(soap, faultsubcode, faultstring, faultdetail);
#endif
}

/******************************************************************************/

/**
@fn int soap_wsse_receiver_fault_subcode(struct soap *soap, const char *faultsubcode, const char *faultstring, const char *faultdetail)
@brief Sets receiver SOAP Fault (sub)code for server fault response.
@param soap context
@param[in] faultsubcode sub code string
@param[in] faultstring fault string
@param[in] faultdetail detail string
@return SOAP_FAULT
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_receiver_fault_subcode(struct soap *soap, const char *faultsubcode, const char *faultstring, const char *faultdetail)
{
#if defined(SOAP_WSA_2003) || defined(SOAP_WSA_2004) || defined(SOAP_WSA_200408) || defined(SOAP_WSA_2005)
  return soap_wsa_receiver_fault_subcode(soap, faultsubcode, faultstring, faultdetail);
#else
  return soap_receiver_fault_subcode(soap, faultsubcode, faultstring, faultdetail);
#endif
}

/******************************************************************************/

/**
@fn int soap_wsse_sender_fault(struct soap *soap, const char *faultstring, const char *faultdetail)
@brief Sets sender SOAP Fault for server fault response.
@param soap context
@param[in] faultstring fault string
@param[in] faultdetail detail string
@return SOAP_FAULT
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_sender_fault(struct soap *soap, const char *faultstring, const char *faultdetail)
{
  return soap_wsse_sender_fault_subcode(soap, NULL, faultstring, faultdetail);
}

/******************************************************************************/

/**
@fn int soap_wsse_receiver_fault(struct soap *soap, const char *faultstring, const char *faultdetail)
@brief Sets receiver SOAP Fault for server fault response.
@param soap context
@param[in] faultstring fault string
@param[in] faultdetail detail string
@return SOAP_FAULT
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_receiver_fault(struct soap *soap, const char *faultstring, const char *faultdetail)
{
  return soap_wsse_receiver_fault_subcode(soap, NULL, faultstring, faultdetail);
}

/******************************************************************************/

/**
@fn int soap_wsse_fault(struct soap *soap, wsse__FaultcodeEnum fault, const char *detail)
@brief Sets SOAP Fault (sub)code for server response.
@param soap context
@param[in] fault is one of wsse:FaultcodeEnum
@param[in] detail string with optional text message
@return SOAP_FAULT
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_fault(struct soap *soap, wsse__FaultcodeEnum fault, const char *detail)
{
  const char *code = soap_wsse__FaultcodeEnum2s(soap, fault);
  DBGFUN2("soap_wsse_fault", "fault=%s", code?code:"", "detail=%s", detail?detail:"");
  /* remove incorrect or incomplete Security header */
  soap_wsse_delete_Security(soap);
  /* populate the SOAP Fault as per WS-Security spec */
  /* detail = NULL; */ /* uncomment when detail text not recommended */
  /* use WSA to populate the SOAP Header when WSA is used */
  switch (fault)
  {
    case wsse__UnsupportedSecurityToken:
      return soap_wsse_sender_fault_subcode(soap, code, "An unsupported token was provided", detail);
    case wsse__UnsupportedAlgorithm:
      return soap_wsse_sender_fault_subcode(soap, code, "An unsupported signature or encryption algorithm was used", detail);
    case wsse__InvalidSecurity:
      return soap_wsse_sender_fault_subcode(soap, code, "An error was discovered processing the <wsse:Security> header", detail);
    case wsse__InvalidSecurityToken:
      return soap_wsse_sender_fault_subcode(soap, code, "An invalid security token was provided", detail);
    case wsse__FailedAuthentication:
      return soap_wsse_sender_fault_subcode(soap, code, "The security token could not be authenticated or authorized", detail);
    case wsse__FailedCheck:
      return soap_wsse_sender_fault_subcode(soap, code, "The signature or decryption was invalid", detail);
    case wsse__SecurityTokenUnavailable:
      return soap_wsse_sender_fault_subcode(soap, code, "Referenced security token could not be retrieved", detail);
  }
  return SOAP_FAULT;
}

/******************************************************************************\
 *
 *      Misc functions
 *
\******************************************************************************/

/**
@fn int soap_wsse_set_wsu_id(struct soap *soap, const char *tags)
@brief Sets the elements that are to be extended with wsu:Id attributes. The
wsu:Id attribute values are set to the string value of the tag's QName by
replacing colons with hyphens to produce an xsd:ID value.
@param soap context
@param[in] tags string of space-separated qualified and unqualified element tag names
@return SOAP_OK
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_set_wsu_id(struct soap *soap, const char *tags)
{
  DBGFUN1("soap_wsse_set_wsu_id", "tags=%s", tags?tags:"(null)");
  soap->wsuid = soap_strdup(soap, tags);
  return SOAP_OK;
}

/******************************************************************************/

#ifdef __cplusplus
}
#endif

