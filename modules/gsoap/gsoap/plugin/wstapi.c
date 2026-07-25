/*
        wstapi.c

        WS-Trust with SAML 2.0, WS-Security 1.0/1.1, WS-Addressing 2005/08

        Extensible framework implements partial WS-Trust logic

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
Copyright (C) 2000-2016, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

This program is released under the GPL with the additional exemption that
compiling, linking, and/or using OpenSSL is allowed.
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

/**

@mainpage The gSOAP WS-Trust Extensible Framework

@section wst_1 The gSOAP WS-Trust Extensible Framework

[TOC]

The material in this section relates to the WS-Trust specification.

The WS-Trust framework is extensible.  New client-side and server-side WS-Trust
operations can be added.  Several predefined operations are included to get you
started.  The list of predefined operations will be expanded over time.  Please
inquire Genivia tech support services.

To use the WS-Trust framework, make sure that the `wst.h` specification is
imported in the .h file for soapcpp2, e.g. after running wsdl2h check the
generated .h file:

@code
    #import "wst.h"
@endcode

If the import is not there, add it manually.  Then run soapcpp2 as usual with
option `-Iimport` to import wst.h from the import directory.

The wst.h and the WS-Trust-dependent wstx.h and other gSOAP-specific .h header
files are located in the import directory of the gSOAP package. These files
define the WS-Trust and other WS-* protocol header elements and types. The
wstx.h header file defines the WS-Trust RequestSecurityTokenRequest and
RequestSecurityTokenRequestCollection operations.

Compile your code with `-DWITH_DOM` and `-DWITH_OPENSSL` to enable WS-Security
plugin API features.

Compile and link your code with wsseapi.c and wstapi.c, and include wsseapi.h
and wstapi.h in your code.

Internally, the wstapi.c code enables SOAP 1.2 messaging.  This will not affect
your SOAP 1.1 messaging.

@section wst_2 WS-Trust Bindings

The WS-Trust bindings in wst.h in the import directory were generated from the
WS-Trust schema for you with the wsdl2h tool and WS/WS-typemap.dat as follows:

    wsdl2h -cgyex -o wst.h -t WS/WS-typemap.dat WS/WS-Trust.xsd

The following modifications to wst.h are **required** to be made after
generating wst.h with wsdl2h:

- Remove `//gsoapopt`
- Change `http://docs.oasis-open.org/ws-sx/ws-trust/200512` to remove the trailing `/` (slash)
- Change `//gsoap wst schema namespace` directive to `//gsoap wst schema import` directive
- Add `#import "wsp_appliesto.h"`
- Add `#import "wstx.h"` at the end of the definitions in wst.h

@section wst_3 Expanding the Current WS-Trust Bindings

To expand or customize the WS-Trust bindings by adding (or removing) content
model elements to the RequestSecurityToken and RequestSecurityTokenResponse,
edit WS/WS-typemap.dat for the following two definition blocks:

    wst__RequestSecurityTokenType = $\
        _wsp__AppliesTo_*                    wsp__AppliesTo;
    wst__RequestSecurityTokenType = $\
        char*                                KeyType;
    wst__RequestSecurityTokenType = $\
        char*                                RequestType;
    wst__RequestSecurityTokenType = $\
        char*                                TokenType;
    wst__RequestSecurityTokenType = $\
        wst__EntropyType*                    Entropy;
    wst__RequestSecurityTokenType = $\
        char*                                ComputedKeyAlgorithm;
    wst__RequestSecurityTokenType = $\
        unsigned int*                        KeySize;
    wst__RequestSecurityTokenType = $\
        struct wst__BinaryExchangeType*      BinaryExchange;
    wst__RequestSecurityTokenType = $\
        struct wst__AuthenticatorType*       Authenticator;

    wst__RequestSecurityTokenResponseType = $\
        struct wst__RequestedSecurityTokenType* RequestedSecurityToken;
    wst__RequestSecurityTokenResponseType = $\
        struct wst__RequestedReferenceType*  RequestedAttachedReference;
    wst__RequestSecurityTokenResponseType = $\
        struct wst__RequestedReferenceType*  RequestedUnattachedReference;
    wst__RequestSecurityTokenResponseType = $\
        struct wst__RequestedProofTokenType* RequestedProofToken;
    wst__RequestSecurityTokenResponseType = $\
        struct wst__RequestedTokenCancelledType* RequestedTokenCancelled;
    wst__RequestSecurityTokenResponseType = $\
        char*                                KeyType;
    wst__RequestSecurityTokenResponseType = $\
        char*                                RequestType;
    wst__RequestSecurityTokenResponseType = $\
        char*                                TokenType;
    wst__RequestSecurityTokenResponseType = $\
        wst__EntropyType*                    Entropy;
    wst__RequestSecurityTokenResponseType = $\
        struct wst__BinaryExchangeType*      BinaryExchange;
    wst__RequestSecurityTokenResponseType = $\
        struct wst__AuthenticatorType*       Authenticator;

For example, to add the `wst:Lifetime` element to the
RequestSecurityTokenResponse add the following two lines:

    wst__RequestSecurityTokenResponseType = $\
        wst__LifetimeType*                   Lifetime;

where `wst__LifetimeType` is declared in wst.h.  The pointer makes it optional.

Then follow the instructions in the previous section to regenerate wst.h.

Given the new `Lifetime` element, the wstapi.c framework can be extended to use
this element information as follows:

@code
    struct wst__RequestSecurityTokenResponseType response;
    time_t expires = 0; // no expiration
    ...
    if (soap_call___wst__RequestSecurityToken(soap, endpoint, soap_wst_rst_action, &request, &response))
    {
      soap_set_namespaces(soap, saved_namespaces);
      return soap->error;
    }
    soap_set_namespaces(soap, saved_namespaces);
    if (response.Lifetime && response.Lifetime->wsu__Expires)
      soap_s2dateTime(soap, response.Lifetime->wsu__Expires, &expires); // set expiration
@endcode

@section wst_4 Predefined WS-Trust Operations

This section lists the predefined WS-Trust operations implemented in wstapi.c.

@subsection wst_soap_wst_request_saml_token

@code
    int soap_wst_request_saml_token(struct soap *soap, const char *endpoint, int soapver, const char *applyto, const char *username, const char *password, saml1__AssertionType **saml1, saml2__AssertionType **saml2)
@endcode

Request SAML 1.0 or SAML 2.0 token, with `endpoint` service endpoint URL (send
to), `soapver` SOAP version with 1 = SOAP 1.1, 2 = SOAP 1.2 (SOAP 1.2 is
recommended), `applyto` is your service domain, `username` to authenticate or
NULL, `password` to authenticate or NULL, `saml1` if non-NULL, requests SAML
1.0 and upon return points to a pointer that is set to the SAML 1.0 assertion
received, `saml2` if non-NULL, requests SAML 2.0 and upon return points to a
pointer that is set to the SAML 2.0 assertion received.

Returns `SOAP_OK` on success when the assertion could be verified, with `saml1`
or `saml2` set.

For example:

@code
    #include "wstapi.h"
    #include "wsaapi.h"
    #include "wsseapi.h"
    #include "wst.nsmap"

    static int ssl_verify(int ok, X509_STORE_CTX *store) { return 1; } // ignore all cert warnings (bad)

    ...

    struct soap *soap = soap_new1(SOAP_XML_INDENT);
    int soapver = 2; // SOAP 1.2
    const char *to = "https://yourcompany.com/adfs/services/trust/13/UsernameMixed";
    const char *applyto = "yourcompany.com";
    const char *username = "yourusername";
    const char *password = "yourpassword";
    saml2__AssertionType *saml2;
    // register wsa plugin (optional, only if the client requires WS-Addressing rerouted messaging)
    soap_register_plugin(soap, soap_wsa);
    // register wsse plugin
    soap_register_plugin(soap, soap_wsse);
    // HTTPS settings
    if (soap_ssl_client_context(soap, SOAP_SSL_DEFAULT, NULL, NULL, "cacerts.pem", NULL, NULL))
    {
      soap_print_fault(soap, stderr);
      exit(1);
    }
    // HTTPS and SAML certificate verification callback
    soap->fsslverify = ssl_verify;
    // SAML 2.0 token request
    if (soap_wst_request_saml_token(soap, to, soapver, applyto, username, password, NULL, &saml2))
    {
      soap_print_fault(soap, stderr);
    }
    else
    {
      // display subset of the assertion information
      if (saml2)
      {
        int i;
        for (i = 0; i < saml2->__size_AssertionType; i++)
        {
          if (saml2->__union_AssertionType[i].saml2__Statement)
          {
            // omitted from displaying
          }
          if (saml2->__union_AssertionType[i].saml2__AuthnStatement)
          {
            if (saml2->__union_AssertionType[i].saml2__AuthnStatement->saml2__AuthnContext)
            {
              if (saml2->__union_AssertionType[i].saml2__AuthnStatement->saml2__AuthnContext->saml2__AuthnContextClassRef)
                printf("AuthnStatement Context: %s\n", saml2->__union_AssertionType[i].saml2__AuthnStatement->saml2__AuthnContext->saml2__AuthnContextClassRef);
            }
          }
          if (saml2->__union_AssertionType[i].saml2__AuthzDecisionStatement)
          {
            // omitted from displaying
          }
          if (saml2->__union_AssertionType[i].saml2__AttributeStatement)
          {
            int j;
            for (j = 0; j < saml2->__union_AssertionType[i].saml2__AttributeStatement->__size_AttributeStatementType; j++)
            {
              if (saml2->__union_AssertionType[i].saml2__AttributeStatement->__union_AttributeStatementType[j].saml2__Attribute)
              {
                int k;
                for (k = 0; k < saml2->__union_AssertionType[i].saml2__AttributeStatement->__union_AttributeStatementType[j].saml2__Attribute->__sizeAttributeValue; k++)
                  printf("Type: %s\nValue: %s\n", saml2->__union_AssertionType[i].saml2__AttributeStatement->__union_AttributeStatementType[j].saml2__Attribute->Name, saml2->__union_AssertionType[i].saml2__AttributeStatement->__union_AttributeStatementType[j].saml2__Attribute->saml2__AttributeValue[k]);
              }
            }
          }
        }
        if (saml2->saml2__Conditions)
        {
          if (saml2->saml2__Conditions->NotBefore)
            printf("Not before %s\n", soap_dateTime2s(soap, *saml2->saml2__Conditions->NotBefore));
          if (saml2->saml2__Conditions->NotOnOrAfter)
            printf("Not on or after %s\n", soap_dateTime2s(soap, *saml2->saml2__Conditions->NotOnOrAfter));
        }

      }
      else
      {
        printf("No SAML 2.0 statements!\n");
      }
    }
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
@endcode

This prints several of the assertion's properties, including the conditions
under which the assertion is valid.  The `NotBefore` and `NotOnOrAfter`
conditions can be checked against the current time as follows:

@code
    time_t now = time(NULL);
    if (saml2->saml2__Conditions->NotBefore && *saml2->saml2__Conditions->NotBefore > now)
      ... error // not valid yet
    if (saml2->saml2__Conditions->NotOnOrAfter && *saml2->saml2__Conditions->NotOnOrAfter <= now)
      ... error // expired
@endcode

@subsection wst_soap_wst_request_psha1_token

@code
    int soap_wst_request_psha1_token(struct soap *soap, const char *endpoint, int soapver, const char *applyto, const char *username, const char *password, char *psha1, size_t psha1len)
@endcode

Request P_SHA1 token with `endpoint` service endpoint URL (send to), `soapver` SOAP version with 1 = SOAP 1.1, 2 = SOAP 1.2 (SOAP 1.2 is recommended), `applyto` your service domain, `username` to authenticate or NULL, `password` to authenticate or NULL, `psha1` is filled with the P_SHA1 result token of `psa1len` bytes.

Returns `SOAP_OK` on success.

@code
    #include "wsaapi.h"
    #include "wstapi.h"
    #include "wsseapi.h"
    #include "wst.nsmap"

    static int ssl_verify(int ok, X509_STORE_CTX *store) { return 1; } // ignore all cert warnings (bad)

    ...

    struct soap *soap = soap_new1(SOAP_XML_INDENT);
    int soapver = 2; // SOAP 1.2
    const char *to = "https://yourcompany.com/adfs/services/trust/13/UsernameMixed";
    const char *applyto = "yourcompany.com";
    const char *username = "yourusername";
    const char *password = "yourpassword";
    char psha1[256];
    // register wsa plugin (optional, only if the client requires WS-Addressing rerouted messaging)
    soap_register_plugin(soap, soap_wsa);
    // register wsse plugin
    soap_register_plugin(soap, soap_wsse);
    // HTTPS settings
    if (soap_ssl_client_context(soap, SOAP_SSL_DEFAULT, NULL, NULL, "cacerts.pem", NULL, NULL))
    {
      soap_print_fault(soap, stderr);
      exit(1);
    }
    // HTTPS certificate verification callback
    soap->fsslverify = ssl_verify;
    // PSHA1 token request
    if (soap_wst_request_psha1_token(soap, to, soapver, applyto, username, password, psha1, 256))
    {
      soap_print_fault(soap, stderr);
    }
    else
    {
      // use psha1[0..255]
    }
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
@endcode

@subsection wst_soap_wst_request_psha256_token

Similar to the previous section, request a P_SHA256 token with:

@code
    int soap_wst_request_psha256_token(struct soap *soap, const char *endpoint, int soapver, const char *applyto, const char *username, const char *password, char *psha256, size_t psha256len)
@endcode

@section wst_5 Using the wst Plugin for Servers

To implement a WS-Trust server in C, run soapcpp2 as follows:

    soapcpp2 -c -L file.h

where file.h has an `#import "wst.h"`.  This generates the soapServer.c and soapC.c code you need to compile with wstapi.c, wsaapi.c, wsseapi.c, smdevp.c, and mecevp.c.  Link with libgsoapssl.a (or stdsoap2.c and dom.c).  Use `-DWITH_OPENSSL` and `-DWITH_DOM` to compile the source code.

For C++, use:

    soapcpp2 -L file.h

This generates the soapServer.cpp and soapC.cpp code you need to compile with wstapi.c, wsaapi.c, wsseapi.c, smdevp.c, and mecevp.c.  Link with libgsoapssl++.a (or stdsoap2.cpp and dom.cpp).  Use `-DWITH_OPENSSL` and `-DWITH_DOM` to compile the source code.

If you prefer to use soapcpp2 option `-j` (or `-i`) to generate C++ server objects, please run soacpp2 again as follows:

    soapcpp2 -j -L file.h
    soapcpp2 -CL -pwst import/wst.h

This generates wstClient.cpp, which should be compiled together with the rest of your project code.

You should define the following service operations:

@code
    int wstService::RequestSecurityToken(wst__RequestSecurityTokenType *request, wst__RequestSecurityTokenResponseType *response)
    {
      ...
    }
    int wstService::RequestSecurityTokenCollection(struct wst__RequestSecurityTokenCollectionType *request, struct wst__RequestSecurityTokenResponseCollectionType *response)
    {
      ...
    }
@endcode

If you are combinding WS-Trust with other service operations, then you must also chain the service operations at the server side as follows:

@code
    if (soap_begin_serve(service.soap) == SOAP_OK)
      if (service.dispatch() == SOAP_NO_METHOD)
        soap_serve_request(service.soap);
@endcode

where the `service` object is an instance of the application services generated by soapcpp2 `-j`.

*/

#include "wstapi.h"

const char *soap_wst_rst_action = SOAP_NAMESPACE_OF_wst "/RST/Issue";
const char *soap_wst_rstr_action = SOAP_NAMESPACE_OF_wst "/RSTR/Issue";
const char *soap_wst_rstc_action = SOAP_NAMESPACE_OF_wst "/RSTC/Issue";
const char *soap_wst_rstrc_action = SOAP_NAMESPACE_OF_wst "/RSTRC/IssueFinal";

/* generated with soapcpp2 -1 -Iimport import/wst.h */
struct Namespace soap11_namespaces[] = {
  {"SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/", "http://www.w3.org/*/soap-envelope", NULL},
  {"SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/", "http://www.w3.org/*/soap-encoding", NULL},
  {"xsi", "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*/XMLSchema-instance", NULL},
  {"xsd", "http://www.w3.org/2001/XMLSchema", "http://www.w3.org/*/XMLSchema", NULL},
  {"c14n", "http://www.w3.org/2001/10/xml-exc-c14n#", NULL, NULL},
  {"ds", "http://www.w3.org/2000/09/xmldsig#", NULL, NULL},
  {"saml1", SOAP_NAMESPACE_OF_saml1, NULL, NULL},
  {"saml2", SOAP_NAMESPACE_OF_saml2, NULL, NULL},
  {"xenc", "http://www.w3.org/2001/04/xmlenc#", NULL, NULL},
  {"wsc", "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512", "http://schemas.xmlsoap.org/ws/2005/02/sc", NULL},
  {"wsse", SOAP_NAMESPACE_OF_wsse, "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd", NULL},
  {"chan", "http://schemas.microsoft.com/ws/2005/02/duplex", NULL, NULL},
  {"wsa5", "http://www.w3.org/2005/08/addressing", "http://schemas.xmlsoap.org/ws/2004/08/addressing", NULL},
  {"wsp", "http://schemas.xmlsoap.org/ws/2004/09/policy", NULL, NULL},
  {"wsu", SOAP_NAMESPACE_OF_wsu, NULL, NULL},
  {"wst", SOAP_NAMESPACE_OF_wst, "http://schemas.xmlsoap.org/ws/2005/02/trust", NULL},
  {NULL, NULL, NULL, NULL}
};

/* generated with soapcpp2 -2 -Iimport import/wst.h */
struct Namespace soap12_namespaces[] = {
  {"SOAP-ENV", "http://www.w3.org/2003/05/soap-envelope", "http://schemas.xmlsoap.org/soap/envelope/", NULL},
  {"SOAP-ENC", "http://www.w3.org/2003/05/soap-encoding", "http://schemas.xmlsoap.org/soap/encoding/", NULL},
  {"xsi", "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*/XMLSchema-instance", NULL},
  {"xsd", "http://www.w3.org/2001/XMLSchema", "http://www.w3.org/*/XMLSchema", NULL},
  {"c14n", "http://www.w3.org/2001/10/xml-exc-c14n#", NULL, NULL},
  {"ds", "http://www.w3.org/2000/09/xmldsig#", NULL, NULL},
  {"saml1", SOAP_NAMESPACE_OF_saml1, NULL, NULL},
  {"saml2", SOAP_NAMESPACE_OF_saml2, NULL, NULL},
  {"xenc", "http://www.w3.org/2001/04/xmlenc#", NULL, NULL},
  {"wsc", "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512", "http://schemas.xmlsoap.org/ws/2005/02/sc", NULL},
  {"wsse", SOAP_NAMESPACE_OF_wsse, "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd", NULL},
  {"chan", "http://schemas.microsoft.com/ws/2005/02/duplex", NULL, NULL},
  {"wsa5", "http://www.w3.org/2005/08/addressing", "http://schemas.xmlsoap.org/ws/2004/08/addressing", NULL},
  {"wsp", "http://schemas.xmlsoap.org/ws/2004/09/policy", NULL, NULL},
  {"wsu", SOAP_NAMESPACE_OF_wsu, NULL, NULL},
  {"wst", SOAP_NAMESPACE_OF_wst, "http://schemas.xmlsoap.org/ws/2005/02/trust", NULL},
  {NULL, NULL, NULL, NULL}
};

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************\
 *
 *      Client-side WS-Trust Operations
 *
\******************************************************************************/

/**
@fn int soap_wst_request_saml_token(struct soap *soap, const char *endpoint, int soapver, const char *applyto, const char *username, const char *password, saml1__AssertionType **saml1, saml2__AssertionType **saml2)
@brief Request SAML 1.0 or SAML 2.0 token.  Verifies the SAML signature, which requires soap->cafile to be set.   Does not verify the conditions of the SAML token, such as NotBefore and NotOnOrAfter, which has to be done explicitly as shown in the documentation.
@param soap context
@param endpoint service endpoint URL (send to)
@param soapver SOAP version 1 = SOAP 1.1, 2 = SOAP 1.2 (recommended)
@param applyto service domain
@param username authentication or NULL
@param password authentication or NULL
@param saml1 if non-NULL, requests SAML 1.0 and upon return points to a pointer that is set to the SAML 1.0 assertion received
@param saml2 if non-NULL, requests SAML 2.0 and upon return points to a pointer that is set to the SAML 2.0 assertion received
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wst_request_saml_token(struct soap *soap, const char *endpoint, int soapver, const char *applyto, const char *username, const char *password, saml1__AssertionType **saml1, saml2__AssertionType **saml2)
{
  struct wst__RequestSecurityTokenType request;
  struct wst__RequestSecurityTokenResponseType response;
  const struct Namespace *saved_namespaces = soap->namespaces;
  DBGFUN("soap_wst_request_saml_token");
  /* SOAP 1.1 or 1.2 */
  if (soapver == 1)
    soap_set_namespaces(soap, soap11_namespaces);
  else
    soap_set_namespaces(soap, soap12_namespaces);
  /* set KeyType, RequestType, TokenType */
  soap_default_wst__RequestSecurityTokenType(soap, &request);
  request.KeyType = (char*)SOAP_NAMESPACE_OF_wst "/Bearer";
  request.RequestType = (char*)SOAP_NAMESPACE_OF_wst "/Issue";
  if (saml1)
  {
    request.TokenType = (char*)"urn:oasis:names:tc:SAML:1.0:assertion";
    *saml1 = NULL;
  }
  else if (saml2)
  {
    request.TokenType = (char*)"urn:oasis:names:tc:SAML:2.0:assertion";
    *saml2 = NULL;
  }
  /* WS-Policy and WS-Addressing headers */
  request.wsp__AppliesTo = (struct _wsp__AppliesTo_*)soap_malloc(soap, sizeof(struct _wsp__AppliesTo_));
  if (!request.wsp__AppliesTo)
    return soap->error = SOAP_EOM;
  soap_default__wsp__AppliesTo_(soap, request.wsp__AppliesTo);
  request.wsp__AppliesTo->SOAP_WSA(EndpointReference) = (SOAP_WSA(EndpointReferenceType)*)soap_malloc(soap, sizeof(SOAP_WSA(EndpointReferenceType)));
  if (!request.wsp__AppliesTo->SOAP_WSA(EndpointReference))
    return soap->error = SOAP_EOM;
  SOAP_WSA_(soap_default,EndpointReferenceType)(soap, request.wsp__AppliesTo->SOAP_WSA(EndpointReference));
  request.wsp__AppliesTo->SOAP_WSA(EndpointReference)->Address = (char*)applyto;
  soap_wsa_request(soap, NULL, endpoint, soap_wst_rst_action);
  /* add credentials */
  if (username && password)
    soap_wsse_add_UsernameTokenDigest(soap, "User", username, password);
  /* verify init enables signature verification */
  soap_wsse_verify_init(soap);
  if (soap_call___wst__RequestSecurityToken(soap, endpoint, soap_wst_rst_action, &request, &response))
  {
    short version = soap->version;
    soap_set_namespaces(soap, saved_namespaces);
    soap->version = version;
    return soap->error;
  }
  soap_set_namespaces(soap, saved_namespaces);
  if (response.RequestedSecurityToken && response.TokenType)
  {
    if (saml1 && !strcmp(response.TokenType, "urn:oasis:names:tc:SAML:1.0:assertion"))
      *saml1 = response.RequestedSecurityToken->saml1__Assertion;
    else if (saml2 && !strcmp(response.TokenType, "urn:oasis:names:tc:SAML:2.0:assertion"))
      *saml2 = response.RequestedSecurityToken->saml2__Assertion;
  }
  if (saml1 && *saml1)
  {
    /* verify assertion using the enveloped signature that contains a X509 certificate */
    if (soap_wsse_verify_with_signature(soap, (*saml1)->ds__Signature))
      return soap->error;
  }
  else if (saml2 && *saml2)
  {
    /* verify assertion using the enveloped signature that contains a X509 certificate */
    if (soap_wsse_verify_with_signature(soap, (*saml2)->ds__Signature))
      return soap->error;
  }
  return SOAP_OK;
}

/**
@fn int soap_wst_request_psha1_token(struct soap *soap, const char *endpoint, int soapver, const char *applyto, const char *username, const char *password, char *psha1, size_t psha1len)
@brief Request PSHA1 token.
@param soap context
@param endpoint service endpoint URL (send to)
@param soapver SOAP version 1 = SOAP 1.1, 2 = SOAP 1.2 (recommended)
@param applyto service domain
@param username authentication or NULL
@param password authentication or NULL
@param psha1 filled with the PSHA1 result token of psa1len bytes
@param psha1len token size in bytes
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wst_request_psha1_token(struct soap *soap, const char *endpoint, int soapver, const char *applyto, const char *username, const char *password, char *psha1, size_t psha1len)
{
  struct wst__RequestSecurityTokenType request;
  struct wst__RequestSecurityTokenResponseType response;
  char HA[16];
  unsigned int keysize = (unsigned int)psha1len;
  const struct Namespace *saved_namespaces = soap->namespaces;
  DBGFUN("soap_wst_request_psha1_token");
  /* SOAP 1.1 or 1.2 */
  if (soapver == 1)
    soap_set_namespaces(soap, soap11_namespaces);
  else
    soap_set_namespaces(soap, soap12_namespaces);
  /* set KeyType, RequestType, ComputedKeyAlgorithm */
  soap_default_wst__RequestSecurityTokenType(soap, &request);
  request.KeyType = (char*)SOAP_NAMESPACE_OF_wst "/SymmetricKey";
  request.RequestType = (char*)SOAP_NAMESPACE_OF_wst "/Issue";
  request.ComputedKeyAlgorithm = (char*)SOAP_NAMESPACE_OF_wst "/CK/PSHA1";
  request.KeySize = &keysize;
  soap_wsse_rand_nonce(HA, 16);
  request.Entropy = (wst__EntropyType*)soap_malloc(soap, sizeof(wst__EntropyType));
  soap_default_wst__EntropyType(soap, request.Entropy);
  request.Entropy->BinarySecret = (wst__BinarySecretType*)soap_malloc(soap, sizeof(wst__BinarySecretType));
  request.Entropy->BinarySecret->Type = (char*)SOAP_NAMESPACE_OF_wst "/Nonce";
  request.Entropy->BinarySecret->__item = soap_s2base64(soap, (unsigned char*)HA, NULL, 16);
  /* WS-Policy and WS-Addressing headers */
  request.wsp__AppliesTo = (struct _wsp__AppliesTo_*)soap_malloc(soap, sizeof(struct _wsp__AppliesTo_));
  if (!request.wsp__AppliesTo)
    return soap->error = SOAP_EOM;
  soap_default__wsp__AppliesTo_(soap, request.wsp__AppliesTo);
  request.wsp__AppliesTo->SOAP_WSA(EndpointReference) = (SOAP_WSA(EndpointReferenceType)*)soap_malloc(soap, sizeof(SOAP_WSA(EndpointReferenceType)));
  if (!request.wsp__AppliesTo->SOAP_WSA(EndpointReference))
    return soap->error = SOAP_EOM;
  SOAP_WSA_(soap_default,EndpointReferenceType)(soap, request.wsp__AppliesTo->SOAP_WSA(EndpointReference));
  request.wsp__AppliesTo->SOAP_WSA(EndpointReference)->Address = (char*)applyto;
  soap_wsa_request(soap, NULL, endpoint, soap_wst_rst_action);
  /* add credentials */
  if (username && password)
    soap_wsse_add_UsernameTokenDigest(soap, "User", username, password);
  /* verify init enables signature verification */
  soap_wsse_verify_init(soap);
  if (soap_call___wst__RequestSecurityToken(soap, endpoint, soap_wst_rst_action, &request, &response))
  {
    short version = soap->version;
    soap_set_namespaces(soap, saved_namespaces);
    soap->version = version;
    return soap->error;
  }
  soap_set_namespaces(soap, saved_namespaces);
  if (response.Entropy && response.Entropy->BinarySecret)
  {
    int len;
    const char *seed = soap_base642s(soap, response.Entropy->BinarySecret->__item, NULL, 0, &len);
    if (soap_psha1(soap, HA, 16, seed, len, psha1, psha1len))
      return soap->error;
  }
  return SOAP_OK;
}

#ifdef __cplusplus
}
#endif
