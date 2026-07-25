/*
        wsseapi.c

        WS-Security plugin

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
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

/**

@mainpage

- @ref wsse documents the wsse plugin for WS-Security 1.0/1.1 support.
- @ref smdevp documents the smdevp signature/digest engine.
- @ref mecevp documents the mecevp encryption engine.
- @ref threads documents a portable threads and locking API.

*/

/**

@page wsse The WS-Security plugin

[TOC]

@section wsse_0 Standards compliance

The WS-Security plugin conforms to:

- WS-Security 1.1 (and backward compatibility with WS-Security 1.0)
- XML Signature Syntax and Processing (XMLDSIG)
- XML Encryption Syntax and Processing (XMLENC)
- SAML 1.0 and 2.0

WS-Security 1.1 with the following 1.1.1 additions:

- Web Services Security SOAP Message Security 1.1.1
- Web Services Security Username Token Profile 1.1.1
- Web Services Security Kerberos Token Profile 1.1.1
- Web Services Security SAML Token Profile 1.1.1
- Web Services Security X.509 Certificate Token Profile 1.1.1
- Web Services Security Rights Expression Language (REL) Token Profile 1.1.1

Note on Basic Security Profile 1.1 compliance: the gSOAP wsse plugin cannot
automatically verify that the wsse API calls made by an application comply with
the profile's requirements.  Users should verify the security
considerations stated by the [Basic Security Profile 1.1](http://docs.oasis-open.org/ws-brsp/BasicSecurityProfile/v1.1/cs01/BasicSecurityProfile-v1.1-cs01.html),
in particular giving close attention to section 19.

@section wsse_5 Security Header

The material in this section relates to the WS-Security specification.

@subsection wsse_5_1 Getting started

To use the wsse plugin:
-# Run wsdl2h -t typemap.dat on a WSDL of a service that requires WS-Security
   headers. The typemap.dat file is used to recognize and translate Security
   header blocks for XML signature and encryption.  The generated file also
   includes WS-Policy instructions with WS-Security requirements to follow,
   when WS-Policy is typically present in the WSDL.
-# Run soapcpp2 on the header file produced by wsdl2h.
-# (Re-)compile stdsoap2.c/pp, dom.c/pp, smdevp.c, mecevp.c, wsseapi.c and the
   generated source files with the `-DWITH_DOM` and `-DWITH_OPENSSL` compiler
   flags set. The smdevp.c, mecevp.c, and wsseapi.c files are located in the
   'plugin' directory.
-# Use the wsse plugin API functions described below to add and verify
   Security headers, sign and verify messages, and to encrypt/decrypt messages.

An example WS-Security client/server application can be found in
gsoap/samples/wsse that illustrates the use of the API to cover a wide range of
WS-Security features.  As a demo of the API, this example is not intended as a
typical WS-Security client or server application.

Another example WS-Security client/server application that is designed to
interoperate with WCF can be found in gsoap/samples/WCF/Basic/MessageSecurity.

@warning
The security token handler callback function parameters have changed in 2.8.34
and greater with the addition of KeyIdentifier information `keyid` and
`keyidlen`.  To register your own security token handler function with the
plugin, make sure that your callback functions matches these function parameters:
@code
    const void *security_token_handler(struct soap *soap, int *alg, const char *keyname, const unsigned char *keyid, int keyidlen, int *keylen);
@endcode

The wsse engine is thread safe. However, if HTTPS is required then please
follow the instructions in Section @ref wsse_11 to ensure thread-safety of
WS-Security when combined with HTTPS.

The wsse API code is implemented in:

- `gsoap/plugin/wsseapi.h` wsse API declarations.
- `gsoap/plugin/wsseapi.c` wsse API for C and C++.

You will also need:

- `gsoap/custom/struct_timeval.c` compile and link this file (C and C++).
- `gsoap/plugin/smdevp.c` compile and link this file (C and C++).
- `gsoap/plugin/mecevp.c` compile and link this file (C and C++).
- compile all sources with `-DWITH_OPENSSL -DWITH_DOM`.
- if you have zlib installed, compile all sources also with `-DWITH_GZIP`.
- link with `-lssl -lcrypto -lz -lgsoapssl++` (or `-lgsoapssl` for C, or compile `stdsoap2.cpp` for C++ and `stdsoap2.c` for C).

The gSOAP header file (generated with wsdl2h, and containing the data binding
interface for soapcpp2) should import wsse.h:

@code
    #import "wsse.h"
@endcode

This declaration supports WS-Security 1.0 by default and accepts WS-Security
1.1.  Vice versa, to support WS-Security 1.1 by default and accept 1.0:

@code
    #import "wsse11.h"
@endcode

The wsdl2h tool adds the necessary import directives to the generated header
file if the WSDL declares the use of WS-Security.  If not, you may have to add
the import directive shown above manually before running soapcpp2.  Instead of
manually adding the directive, you can let wsdl2h do this for you by adding the
following lines to typemap.dat:

@code
    [
    #import "wsse.h"
    ]
@endcode

The wsdl2h tool uses typemap.dat to add or modify the generated code.

@subsection wsse_5_2 Warning

The following sections describe the wsse plugin API.  The narrative must not
be interpreted as a set of requirements to implement WS-Security, should not be
used as a guide to select certain keys and key sizes, or as a recommendation in
general.  Rather, the API description explains which API functions are
available to implement WS-Security operations vis-a-vis sections of the
WS-Security standard.  It is implicitly assumed that WS-Security API
requirement policies are followed by the developers of the WS-Security Web API,
such as defined by [Basic Security Profiles](http://docs.oasis-open.org/ws-brsp/BasicSecurityProfile/v1.1/cs01/BasicSecurityProfile-v1.1-cs01.html).

When implementing WS-Security services, it is important to follow the WS-Policy
instructions generated by the wsdl2h tool for WSDLs with WS-Policy elements.
In any case, consult the latest basic security profiles for WS-Security,
including important security considerations, see
[Basic Security Profile 1.1 section 19](http://docs.oasis-open.org/ws-brsp/BasicSecurityProfile/v1.1/cs01/BasicSecurityProfile-v1.1-cs01.html#_Toc396926339).

No guarantees are provided when the developer uses API calls of the wsse plugin
to produce implementations that deviate from established policies and profiles or
if he/she fails to check messages for the presence of headers that are required
by policies, including but not limited to authentication requirements, message
expiration validation, and verification that message parts are signed as
required, using the API functions documented below.  The developers of the wsse
plugin took great care to validate and test its functionality in the field to
ensure it operates properly when following security policies and profiles.
The documentation, examples, and demos are meant to exemplify the API.  Genivia
and its developers waive any responsibility and liability when the software and
examples are used by users or by third-parties in ways that modify their
functionalities that deviate from established Web services security policies
and Basic Security Profiles.

WS-Security is not a replacement of TLS.  Secure transport with HTTPS is
typically required to transport WS-Security messages.

@subsection wsse_5_3 Introduction

The wsse API consists of a set of functions to populate and verify WS-Security
headers and message body content. For more details, we refer to the following
sections that correspond to the WS-Security specification sections:

- Section 6 @ref wsse_6
- Section 7 @ref wsse_7
- Section 8 @ref wsse_8
- Section 9 @ref wsse_9
- Section 10 @ref wsse_10
- @ref wsse_11
- @ref wsse_12
- @ref wsse_13
- @ref wsse_wsc

The API is introduced below.

To add an empty Security header block to the SOAP header, use:

@code
    soap_wsse_add_Security(soap);
@endcode

To delete a Security header, use:

@code
    soap_wsse_delete_Security(soap);
@endcode

Adding an empty Security header block is not very useful. In the following, we
present the higher-level functions of the wsse plugin to populate and verify
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

@subsection wsse_6_2 User Name Tokens

To add a user name token to the Security header block, use:

@code
    soap_wsse_add_UsernameTokenText(soap, "Id", "username", NULL);
@endcode

The `Id` value is optional. When non-NULL the user name token is included in the
digital signature to protect its integrity. It is common for the wsse plugin
functions to accept such `Id`s, which are serialized as wsu:Id identifiers for
cross-referencing XML elements. The signature engine of the wsse plugin is
designed to automatically sign all wsu:Id attributed elements to simplify the
code you need to write to implement the signing process.

To add a user name token with clear text password, use:

@code
    soap_wsse_add_UsernameTokenText(soap, "Id", "username", "password");
@endcode

It is strongly recommended to use `soap_wsse_add_UsernameTokenText` only in
combination with HTTPS encrypted transmission or not at all. A better
alternative is to use password digests (and still use HTTPS as preferred). With
password digest authentication, the digest value of a password (with message
creation time and a random nonce) is compared on both sides, thus eliminating
the need to exchange a password over the wire.

To add a user name token with password digest, use:

@code
    soap_wsse_add_UsernameTokenDigest(soap, "Id", "username", "password");
@endcode

Although the password string is passed to this function, it is not rendered in
XML or stored in a message log. Only digests are compared on both sides, not
the passwords. This authentication method adds a timestamp and nonce to prevent
message replay attacks.

It has been argued that this approach adopted by the WS-Security protocol is
still vulnerable since the application retrieves the password in text form
requiring a database to store passwords in clear text. However, a digest
algorithm can be used to hash the passwords and store their digests instead,
which eliminates the need to store clear-text passwords. This is a common
approach adopted by Unix for decades.

By setting the `Id` value to a unique string, the user name token is also
digitally signed by the signature engine further preventing tampering with its
value.

You must use `soap_wsse_add_UsernameTokenDigest` for each message exchange
to refresh the password digest even when the user name and password are not
changed. Otherwise, the receiver might flag the message as a replay attack.

To specify a time stamp for the digest instead of the current time, use:

@code
    time_t when = ...;
    soap_wsse_add_UsernameTokenDigest_at(soap, "Id", "username", "password", when);
@endcode

Clear-text passwords and password digests are verified with
`soap_wsse_verify_Password`. To verify a password at the receiving side to
authorize a request (e.g. within a Web service operation), use:

@code
    int ns__myMethod(struct soap *soap, ...)
    {
      const char *username = soap_wsse_get_Username(soap);
      const char *password;
      if (!username)
      {
        soap_wsse_delete_Security(soap); // remove old security headers
        return soap->error; // no username: return FailedAuthentication (from soap_wsse_get_Username)
      }
      password = ...; // lookup password of username
      if (soap_wsse_verify_Password(soap, password))
      {
        int err = soap->error;
        soap_wsse_delete_Security(soap); // remove old security headers
        // if it is required to return signed faults, then add the following six lines here:
        if (soap_wsse_add_BinarySecurityTokenX509(soap, "X509Token", cert)
         || soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(soap, "#X509Token")
         || soap_wsse_sign_body(soap, SOAP_SMD_SIGN_RSA_SHA256, rsa_private_key, 0)
        {
          soap_wsse_delete_Security(soap); // remove security headers (failed construction)
          return soap->error;
        }
        return err; // password verification failed: return FailedAuthentication
      }
      ... // process request, then sign the response message:
      if (soap_wsse_add_BinarySecurityTokenX509(soap, "X509Token", cert)
       || soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(soap, "#X509Token")
       || soap_wsse_sign_body(soap, SOAP_SMD_SIGN_RSA_SHA256, rsa_private_key, 0)
      {
        soap_wsse_delete_Security(soap); // remove security headers (failed construction)
        return soap->error;
      }
      return SOAP_OK;
    }
@endcode

Note that the `soap_wsse_get_Username` functions sets the
wsse:FailedAuthentication fault upon failure. It is common for the wsse plugin
functions to return `SOAP_OK` or a wsse fault that should be passed to the sender
by returning soap->error from service operations. The fault is displayed with
the `soap_print_fault` function. To return signed faults back to the client, a
signature is constructed as shown in the code snippet above. When the signature
construction itself fails, we delete the partially constructed signature and
return the fault to the client.

Password digest authentication prevents message replay attacks. The wsse plugin
keeps a database of password digests to thwart replay attacks. This is the only
part in the plugin code that requires mutex provided by threads.h. Of course,
this only works correctly if the server is persistent, such as a stand-alone
service. Note that CGI-based services do not keep state. Machine clocks must be
synchronized and clock skew should not exceed `SOAP_WSSE_CLKSKEW` at the
server side.

@subsection wsse_6_3 Binary Security Tokens

X509 certificates are commonly included in Security header blocks as binary
security tokens. A certificate is used to verify the digital signature of a
digitally signed message using the public key embedded within the certificate.
The certificate itself is signed by a certificate authority (CA) that vouches
for the authenticity of the certificate, i.e. to prove the identify of the
message originator. This verification process is important, because digital
signatures are useless without verification: an attacker could simply replace
the message, sign it, and replace the certificate.

Certificates are automatically verified by the wsse plugin signature engine
when received and accessed, which means that the certificates of the CAs must
be made accessible to the wsse plugin as follows:

@code
    soap->cafile = "cacerts.pem";  // use this
    soap->capath = "dir/to/certs"; // and/or point to CA certs
    soap->crlfile = "revoked.pem"; // use CRL (optional)
@endcode

The `soap_wsse_verify_X509` function checks the validity of a certificate.
The check is automatically performed. The check is also performed when
retrieving the certificate from a Security header block, either automatically
by the wsse plugin's signature verification engine or manually as follows:

@code
    X509 *cert = soap_wsse_get_BinarySecurityTokenX509(soap, "Id");
@endcode

where `Id` is the identification string of the binary security token or NULL to
get the first found in the Security header.

The X509 certificate returned by this function should be freed with `X509_free`
to deallocate the certificate data:

@code
    if (cert)
      X509_free(cert);
    cert = NULL;
@endcode

The verification is an expensive process that will be optimized in future
releases by caching the certificate chain.

To attach a binary security token stored in a PEM file to a Security header
block for transmission, use:

@code
    soap_wsse_add_BinarySecurityTokenPEM(soap, NULL, "mycert.pem")
@endcode

A binary security token can be automatically signed by setting its `Id`
attribute:

@code
    soap_wsse_add_BinarySecurityTokenPEM(soap, "X509Token", "mycert.pem")
@endcode

Repeatedly loading a certificate from a PEM file is inefficient. To reuse a
certificate loaded from a PEM file for multiple invocations, use:

@code
    FILE *fd = fopen("mycert.pem", "r");
    X509 *cert = PEM_read_X509(fd, NULL, NULL, NULL);
    fclose(fd);
    if (soap_wsse_add_BinarySecurityTokenX509(soap, "X509Token", cert))
      ... // an error occurred
@endcode

Other types of binary security tokens can be added to the Security header block using:

@code
    soap_wsse_add_BinarySecurityToken(soap, "Id", "valueType", data, datalen);
@endcode

@section wsse_6_4 SAML and Other Tokens

The use and processing rules for tokens such as SAML assertions is specific to
an application.  SAML 1.0 and 2.0 tokens are supported with the following
functions to retrieve them from Security header blocks:

@code
    saml1__AssertionType *assertion1 = soap_wsse_get_saml1(soap);
@endcode

@code
    saml2__AssertionType *assertion2 = soap_wsse_get_saml2(soap);
@endcode

The pointers returned are non-NULL when these tokens are present.  You can
verify that a token is signed by the signature of the Security header with:

@code
    saml2__AssertionType *assertion2 = soap_wsse_get_saml2(soap);
    if (!assertion2 || soap_wsse_verify_element(soap, SOAP_NAMESPACE_OF_saml2, "Assertion") == 0)
      ... error // no Assertion or Assertion not signed by WS-Security signature (zero Assertion elements signed)
@endcode

If the SAML token received contains a signature and/or time range conditions
then you should verify that the SAML token is valid after receiving it in a
Security header block of a WS-Security message:

@code
    if (saml2->saml2__Conditions)
    {
      time_t now = time(NULL);
      if (saml2->saml2__Conditions->NotBefore && *saml2->saml2__Conditions->NotBefore > now)
        ... error // not valid yet
      if (saml2->saml2__Conditions->NotOnOrAfter && *saml2->saml2__Conditions->NotOnOrAfter <= now)
        ... error // expired
    }
    if (saml2->ds__Signature)
      if (soap_wsse_verify_with_signature(soap, saml2->ds__Signature))
        ... error // Assertion has signature but token is invalid
@endcode

The above assumes that a WS-Security message was received that was signed and
decrypted (when applicable).

@note The resolution of the dateTime values of `NotBefore` and `NotOnOrAfter`
is determined by the clock resolution of a time representation.  The `time_t`
resolution is seconds.  Therefore, the `struct timeval` serializer is used to
increase the resolution to microseconds (by using
`#import "custom/struct_timeval.h"` in `gsoap/import/saml2.h`.

To add a SAML token to the WS-Security headers, use
`soap_wsse_add_saml1(struct soap*, const char *id)` or
`soap_wsse_add_saml2(struct soap*, const char *id)`:

@code
    time_t now = time(NULL);
    saml1__AssertionType *assertion1 = soap_wsse_add_saml1(soap, "SAML1");
    if (!assertion1)
      ... // error
    assertion1->IssueInstant = now;
    assertion1->Issuer = (char*)"MyCompany";
    assertion1->saml1__Conditions = soap_new_saml1__ConditionsType(soap, -1);
    if (!assertion1->saml1__Conditions)
      ... // error
    // valid from now for up to one hour
    assertion1->saml1__Conditions->NotBefore = soap_new_dateTime(soap, -1)
    if (!assertion1->saml1__Conditions->NotBefore)
      ... // error
    *assertion1->saml1__Conditions->NotBefore = now;
    assertion1->saml1__Conditions->NotOnOrAfter = soap_new_dateTime(soap, -1)
    if (!assertion1->saml1__Conditions->NotOnOrAfter)
      ... // error
    *assertion1->saml1__Conditions->NotOnOrAfter = now + 3600;
    ...
@endcode

and, respectively:

@code
    time_t now = time(NULL);
    saml2__AssertionType *assertion2 = soap_wsse_add_saml2(soap, "SAML2");
    if (!assertion2)
      ... // error
    assertion2->IssueInstant = now;
    assertion2->saml2__Issuer = (struct saml2__NameIDType*)soap_malloc(soap, sizeof(struct saml2__NameIDType));
    soap_default_saml2__NameIDType(soap, assertion2->saml2__Issuer);
    assertion2->saml2__Issuer->__item = (char*)"MyCompany";
    // valid from now for up to one hour
    assertion2->saml2__Conditions->NotBefore = soap_new_dateTime(soap, -1)
    if (!assertion2->saml2__Conditions->NotBefore)
      ... // error
    *assertion2->saml2__Conditions->NotBefore = now;
    assertion2->saml2__Conditions->NotOnOrAfter = soap_new_dateTime(soap, -1)
    if (!assertion2->saml2__Conditions->NotOnOrAfter)
      ... // error
    *assertion2->saml2__Conditions->NotOnOrAfter = now + 3600;
    ...
@endcode

The code shown above adds an empty SAML token to the Security header block
after which the SAML assertion issuer, subject, conditions, statements, and
attributes should be set.  Once these are set, the assertion can be signed
with a ds:Signature and X509 certificate added to the assertion to create an
enveloped signature:

@code
    EVP_PKEY *rsa_private_key; // private key
    X509 *cert;                // certificate (e.g. in "cacert.pem")
    ...
    saml2__AssertionType *assertion2 = soap_wsse_add_saml2(soap, "SAML2");
    if (!assertion2)
      ... // error
    ... // set SAML issuer, subject, conditions, statements, and attributes
    if (soap_wsse_sign_saml2(soap, assertion2, SOAP_SMD_SIGN_RSA_SHA256, private_key, 0, cert))
      ... error // could not sign and/or add cert to X509Data
    soap->cafile = "cacert.pem"; // file that contains the public certificate
    if (soap_wsse_verify_saml2(soap, assertion2))
      ... error // coult not verify the signature, e.g. invalid key-certificate pair
@endcode

It is a good habit to verify a SAML token that was created in memory with
`int soap_wsse_verify_saml1(struct soap*, saml1__AssertionType *saml1)` or
`int soap_wsse_verify_saml2(struct soap*, saml2__AssertionType *saml2)` as
shown.  This step is optional, but can be useful to detect if the private key
and certificate are uncorrelated and should not be used.

The private key and certificate values can be obtained as shown in Section
@ref wsse_8_2a.

For implementing other types of tokens, you are encouraged to modify the
import/wsse.h file to add more tokens to the `_wsse__Security` header block:

@code
    struct somens__SomeTokenType { @char *wsu__Id; ... };

    typedef struct _wsse__Security
    {       ...
            struct saml1__AssertionType*            saml1__Assertion;
            struct saml2__AssertionType*            saml2__Assertion;
            struct somens__SomeTokenType*           somens__SomeToken; // added an optional token
            ...                                                        // add more if needed
            @char*                                  SOAP_ENV__actor;
            @char*                                  SOAP_ENV__role;
    } _wsse__Security;
@endcode

The tokens can be set with:

@code
    _wsse__Security *security = soap_wsse_Security(soap);
    security->somens__SomeToken = (struct somens__SomeTokenType*)soap_malloc(soap, sizeof(struct somens__SomeTokenType));
    soap_default_somens__SomeTokenType(soap, security->somens__SomeToken);
    security->somens__SomeToken->wsu__Id = "myToken"; // allows for auto-signing this element
    ...
@endcode
 
For tokens in DOM XML form, use the `xsd__anyType` DOM element:

@code
    typedef struct _wsse__Security
    {       ...
            xsd__anyType*                           somens__SomeToken; // added an optional token in DOM form
    } _wsse__Security;
@endcode

The token in DOM form can be signed if you set the wsu:Id attribute to a unique
value say "MyToken":

@code
    _wsse__Security *security = soap_wsse_Security(soap);
    security->somens__SomeToken = (xsd__anyType*)soap_malloc(soap, sizeof(xsd__anyType));
    soap_default_xsd__anyType(soap, security->somens__SomeToken);
    soap_att_text(soap_att(soap_add_security->somens__SomeToken, NULL, "wsu:Id"), "MyToken");
    ...
@endcode

We recommend to use [domcpp](http://www.genivia.com/doc/dom/html/index.html) to
generate code to set the token to send messages and get its values after
receiving messages.

For tokens in XML "string" text form, use the `_XML` literal string (a `char*`
type with XML content):

@code
    typedef struct _wsse__Security
    {       ...
            _XML                                    somens__SomeToken; // added an optional token in string form
    } _wsse__Security;
@endcode

However, beware that XML text cannot be signed by the signature as a Security
header (unless you embed it within a new element in the Security header block
and set that element's wsu:Id attribute).

@section wsse_7 Token References

The material in this section relates to the WS-Security specification section 7.

To use a certificate for signature verification, add a direct security token
reference URI for the token to the KeyInfo, for example:

@code
    soap_wsse_add_KeyInfo_SecurityTokenReferenceURI(soap, "URI", "valueType");
@endcode

and:

@code
    soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(soap, "URI");
@endcode

For X509 certificates we use this to add a binary security token with the
certificate and a reference to the local token:

@code
    if (soap_wsse_add_BinarySecurityTokenX509(soap, "X509Token", cert)
     || soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(soap, "#X509Token"))
      ... // an error occurred
@endcode

This follows the recommended practice to place Security token references in
the KeyInfo element of a Signature. The KeyInfo is used to verify the validity
of a signature value.

Key identifiers can be used as well:

@code
    soap_wsse_add_KeyInfo_SecurityTokenReferenceKeyIdentifier(soap, "Id", "valueType", data, datalen);
@endcode

Embedded references are added with:

@code
    soap_wsse_add_KeyInfo_SecurityTokenReferenceEmbedded(soap, "Id", "valueType");
@endcode

Full support for embedded references requires coding to add tokens and
assertions, as well as to consume embedded references at the receiving side.
There is no automated mechanism to take the embedded references and process
them accordingly.

The use of key names is not recommended, but in case they are required they can
be added with:

@code
    soap_wsse_add_KeyInfo_KeyName(soap, "name");
@endcode

@section wsse_8 Signatures

The material in this section relates to the WS-Security specification section 8.

When signatures are used with encryption (@ref wsse_9), then encryption is
always applied after signing.  It is generally known that it is safe to perform
encryption after signing, but not vice versa.  In particular, this order allows
for the encryption of the signature and its digests, as required by
[Basic Security Profile 1.1](http://docs.oasis-open.org/ws-brsp/BasicSecurityProfile/v1.1/cs01/BasicSecurityProfile-v1.1-cs01.html) section 19.4.

First, the wsse plugin must be registered to sign and verify messages:

@code
    soap_register_plugin(soap, soap_wsse);
@endcode

XML signatures are usually computed over normalized XML (to ensure the XML
processors of intermediate nodes can accurately reproduce the XML). To this
end, the exclusive canonical XML standard (exc-c14n) is required, which is set
using the `SOAP_XML_CANONICAL` flag:

@code
    struct soap *soap = soap_new1(SOAP_XML_CANONICAL);
    soap_register_plugin(soap, soap_wsse);
@endcode

To send messages with inclusive canonicalization, in addition to the
`SOAP_XML_CANONICAL` flag also use: 

@code
    soap_wsse_set_InclusiveNamespaces(soap, "*");
@endcode

However, exclusive canonicalization is recommended over inclusive
canonicalization, or no canonicalization at all.  WS Basic Security profiles
1.0 and 1.1 require exclusive canonicalization.

Flags to consider:

- `SOAP_XML_CANONICAL` recommended to enable exc-c14n (exclusive canonicalization).
- `SOAP_XML_INDENT` optional, to emit more readable XML (see warning below).
- `SOAP_IO_CHUNK` efficient HTTP-chunked streaming messages.
- `SOAP_ENC_GZIP` for HTTP compression (also enables HTTP chunking).

@warning
Interoperability with WCF WS-Security is not guaranteed when `SOAP_XML_INDENT`
is enabled.  Avoid using `SOAP_XML_INDENT` for interoperability.  The
implementation of canonicalization in WCF with respect to the normalization of
white space between XML tags differs from the protocol standards.

Next, decide which signature algorithm is appropriate to use:

- HMAC-SHA uses a secret key (also known as a shared key in symmetric
  cryptography) to sign the SHA digest of the SignedInfo element.
- DSA-SHA uses a DSA private key to sign the SHA digest of the SignedInfo
  element.
- RSA-SHA uses a RSA private key to sign the SHA digest of the SignedInfo
  element.
- ECDSA-SHA uses a Elliptic Curve DSA private key to sign the SHA digest of the
  SignedInfo element.

HMAC-SHA is the simplest method, but relies on the fact that you have to make
absolutely sure the key is kept secret on both the sending and receiving side.
As long as the secret key is confidential, messages are securely signed.
However, this is virtually impossible when exchanging messages with untrusted
disparate parties. The advantage of HMAC-SHA is the speed by which messages
are signed and verified.

Algorithms HMAC SHA1, SHA256, and SHA512 are supported:

- `SOAP_SMD_HMAC_SHA1`   http://www.w3.org/2000/09/xmldsig#hmac-sha1
- `SOAP_SMD_HMAC_SHA224` http://www.w3.org/2001/04/xmldsig-more#hmac-sha224
- `SOAP_SMD_HMAC_SHA256` http://www.w3.org/2001/04/xmldsig-more#hmac-sha256
- `SOAP_SMD_HMAC_SHA384` http://www.w3.org/2001/04/xmldsig-more#hmac-sha384
- `SOAP_SMD_HMAC_SHA512` http://www.w3.org/2001/04/xmldsig-more#hmac-sha512

DSA-SHA and RSA-SHA rely on public key cryptography. In simplified terms, a
message is signed using the (confidential!) private key. The public key is used
to verify the signature. Since only the originating party could have used its
private key to sign the message, the integrity of the message is guaranteed. Of
course, we must trust the public key came from the originator (it is often
included as an X509 certificate in the message). To this end, a trusted
certificate authority should have signed the public key, thereby creating a
X509 certificate that contains the public key and the identity of the message
originator.

The following DSA, RSA, and ECDSA algorithms are supported:

- `SOAP_SMD_SIGN_DSA_SHA1`     http://www.w3.org/2000/09/xmldsig#dsa-sha1
- `SOAP_SMD_SIGN_DSA_SHA256`   http://www.w3.org/2000/09/xmldsig-more#dsa-sha256
- `SOAP_SMD_SIGN_RSA_SHA1`     http://www.w3.org/2000/09/xmldsig#rsa-sha1
- `SOAP_SMD_SIGN_RSA_SHA224`   http://www.w3.org/2001/04/xmldsig-more#rsa-sha224
- `SOAP_SMD_SIGN_RSA_SHA256`   http://www.w3.org/2001/04/xmldsig-more#rsa-sha256
- `SOAP_SMD_SIGN_RSA_SHA384`   http://www.w3.org/2001/04/xmldsig-more#rsa-sha384
- `SOAP_SMD_SIGN_RSA_SHA512`   http://www.w3.org/2001/04/xmldsig-more#rsa-sha512
- `SOAP_SMD_SIGN_ECDSA_SHA1`   http://www.w3.org/2001/04/xmldsig-more#ecdsa-sha1
- `SOAP_SMD_SIGN_ECDSA_SHA224` http://www.w3.org/2001/04/xmldsig-more#ecdsa-sha224
- `SOAP_SMD_SIGN_ECDSA_SHA256` http://www.w3.org/2001/04/xmldsig-more#ecdsa-sha256
- `SOAP_SMD_SIGN_ECDSA_SHA384` http://www.w3.org/2001/04/xmldsig-more#ecdsa-sha384
- `SOAP_SMD_SIGN_ECDSA_SHA512` http://www.w3.org/2001/04/xmldsig-more#ecdsa-sha512

An optional callback function can be passed to the plugin that is responsible
for providing a certificate or key to the wsse engine to verify a signed
message. For example, when a security token is absent from an DSA-SHA or
RSA-SHA signed message then the only mechanism to automatically verify the
signature is to let the callback produce a certificate:

@code
    soap_register_plugin(soap, soap_wsse);
    soap_wsse_set_security_token_handler(soap, security_token_handler);

    const void *security_token_handler(struct soap *soap, int *alg, const char *keyname, const unsigned char *keyid, int keyidlen, int *keylen)
    {
      // Get the user name from UsernameToken in message
      const char *uid = soap_wsse_get_Username(soap);
      switch (*alg)
      {
        case SOAP_SMD_VRFY_DSA_SHA1:
        case SOAP_SMD_VRFY_DSA_SHA256:
        case SOAP_SMD_VRFY_RSA_SHA1:
        case SOAP_SMD_VRFY_RSA_SHA224:
        case SOAP_SMD_VRFY_RSA_SHA256:
        case SOAP_SMD_VRFY_RSA_SHA384:
        case SOAP_SMD_VRFY_RSA_SHA512:
        case SOAP_SMD_VRFY_ECDSA_SHA1:
        case SOAP_SMD_VRFY_ECDSA_SHA224:
        case SOAP_SMD_VRFY_ECDSA_SHA256:
        case SOAP_SMD_VRFY_ECDSA_SHA384:
        case SOAP_SMD_VRFY_ECDSA_SHA512:
          if (uid)
          {
            // Lookup uid to retrieve the X509 certificate to verify the signature
            const X509 *cert = ...; 
            return (const void*)cert;
          }
          return NULL; // no certificate: fail
        case SOAP_SMD_HMAC_SHA1:
        case SOAP_SMD_HMAC_SHA224:
        case SOAP_SMD_HMAC_SHA256:
        case SOAP_SMD_HMAC_SHA384:
        case SOAP_SMD_HMAC_SHA512:
          if (uid)
          {
            // Lookup uid to retrieve the HMAC SHA key to verify the signature
            const void *key = ...; 
            *alg = ...;
            *keylen = ...;
            return key;
          }
          return NULL; // no certificate: fail
        case SOAP_MEC_ENV_DEC_DES_CBC:
        case SOAP_MEC_ENV_DEC_AES128_CBC:
        case SOAP_MEC_ENV_DEC_AES192_CBC:
        case SOAP_MEC_ENV_DEC_AES256_CBC:
        case SOAP_MEC_ENV_DEC_AES512_CBC: // reserved for future use
        case SOAP_MEC_ENV_DEC_AES128_GCM: // GCM requires OpenSSL 1.0.2 or higher
        case SOAP_MEC_ENV_DEC_AES192_GCM: // GCM requires OpenSSL 1.0.2 or higher
        case SOAP_MEC_ENV_DEC_AES256_GCM: // GCM requires OpenSSL 1.0.2 or higher
        case SOAP_MEC_ENV_DEC_AES512_GCM: // GCM requires OpenSSL 1.0.2 or higher
          if (keyname)
          {
            // use this to get key or X509 certificate from a key store using the keyname value:
            // 1. keyname is set to the subject name of the certificate, if a
            //    certificate is present in the SecurityTokenReference/KeyIdentifier
            //    when ValueType is http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-x509-token-profile-1.0#X509v3
            // 2. keyname is set to the string concatenation
            //     "{X509IssuerName}#{X509SerialNumber}" of the X509IssuerName
            //     and X509SerialNumber present in X509Data/X509IssuerSerial
            // 3. keyname is set to X509Data/X509SubjectName
            return ...;
          }
          else if (keyid)
          {
            // use this to get the key from a key store using the keyid[0..keyidlen-1]:
            // 1. keyid and keyidlen are set to the data in
            //    SecurityTokenReference/KeyIdentifier when the ValueType is
            //    http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-x509-token-profile-1.0#X509SubjectKeyIdentifier
            return ...;
          }
          break;
        case SOAP_MEC_DEC_DES_CBC:
        case SOAP_MEC_DEC_AES128_CBC:
        case SOAP_MEC_DEC_AES192_CBC:
        case SOAP_MEC_DEC_AES256_CBC:
        case SOAP_MEC_DEC_AES512_CBC: // reserved for future use
        case SOAP_MEC_DEC_AES128_GCM: // GCM requires OpenSSL 1.0.2 or higher
        case SOAP_MEC_DEC_AES192_GCM: // GCM requires OpenSSL 1.0.2 or higher
        case SOAP_MEC_DEC_AES256_GCM: // GCM requires OpenSSL 1.0.2 or higher`
        case SOAP_MEC_DEC_AES512_GCM: // GCM requires OpenSSL 1.0.2 or higher
          if (keyname)
          {
            // use the keyname to get the shared secret key associated for decryption
            *keylen = ... // length of the shared secret key
            return ...;
          }
          break;
      }
      return NULL; // fail
    }
@endcode

@warning
The security token handler callback function parameters have changed in 2.8.34
and greater with the addition of KeyIdentifier information `keyid` and
`keyidlen`.

@subsection wsse_8_2a Signing Messages

After the plugin is registered and a signature algorithm selected, the
`soap_wsse_sign` function or the `soap_wsse_sign_body` function is used
to initiate the signature engine to automatically sign outbound messages.

The code to sign the SOAP Body of a message using HMAC-SHA1 is:

@code
    static char hmac_key[16] =
    { 0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99, 0x88,
      0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00 };
    struct soap *soap = soap_new1(SOAP_XML_CANONICAL);
    soap_register_plugin(soap, soap_wsse);
    if (soap_wsse_sign_body(soap, SOAP_SMD_HMAC_SHA1, hmac_key, sizeof(hmac_key))
      ... // an error occurred
    else if (soap_call_ns__myMethod(soap, ...))
      ... // a transmission error occurred
@endcode

The `hmac_key` above is some secret key you generated for the sending side and
receiving side (don't use the one shown here). Instead of SHA1 above, you can
also use the more secure SHA224, SHA256, SHA384 and SHA512 hashes.

As always, use `soap_print_fault` to display the error message.

To sign the body of an outbound SOAP message using RSA-SHA (DSA-SHA is
similar), we include the X509 certificate with the public key as a
BinarySecurityToken in the header and a KeyInfo reference to the token to let
receivers use the public key in the trusted and verified (!) certificate to
verify the authenticity of the message:

@code
    FILE *fd;
    EVP_PKEY *rsa_private_key;
    X509 *cert;
    struct soap *soap = soap_new1(SOAP_XML_CANONICAL);
    soap_register_plugin(soap, soap_wsse);
    fd = fopen("privkey.pem", "r");
    rsa_private_key = PEM_read_PrivateKey(fd, NULL, NULL, "password");
    fclose(fd);
    fd = fopen("cert.pem", "r");
    X509 *cert = PEM_read_X509(fd, NULL, NULL, NULL);
    fclose(fd);
    if (soap_wsse_add_BinarySecurityTokenX509(soap, "X509Token", cert)
     || soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(soap, "#X509Token")
     || soap_wsse_sign_body(soap, SOAP_SMD_SIGN_RSA_SHA256, rsa_private_key, 0))
      ... // an error occurred
    else if (soap_call_ns__myMethod(soap, ...))
      ... // a transmission error occurred
@endcode

The private key and its certificate are often placed in the same file, see e.g.
server.pem in the package.

To summarize the signing process:
-# Register the wsse plugin.
-# Obtain an HMAC secret key or a DSA/RSA/ECDSA private key.
-# For DSA or RSA, obtain the X509 certificate with the public key signed by a
   certificate authority.
-# Add the X509 certificate as a BinarySecurityToken to the header.
-# Add a KeyInfo BinarySecurityTokenReference.
-# Invoke `soap_wsse_sign_body` and/or `soap_wsse_sign` with
   `soap_wsse_sign_only` to sign the message.
-# Always check the function return values for errors. You don't want to
   produce and accept messages with an invalid Security headers.

@subsection wsse_8_2b Signing Message Parts

The `soap_wsse_sign_body` function signs the entire SOAP body but nothing else.
If it is desirable to sign individual parts of a message the
`soap_wsse_sign_only` and `soap_wsse_sign` functions should be used. All
message parts with wsu:Id attributes are signed.  These message parts should
not be nested (nested elements will not be separately signed). By default, all
and only those XML elements with wsu:Id attributes are signed. Therefore, the
wsu:Id attribute values used in a message must be unique within the message.
Although usually not required, the default signing rule can be overridden with
the `soap_wsse_sign_only` function, see @ref wsse_8_3.

For example, consider a transaction in which we only want to sign a contract in
the SOAP Body. This allows us to modify the rest of the message or extract the
contract in XML and pass it on with the signature.

The gSOAP header file includes a myContract declaration:

@code
    struct ns__myContract
    {
      @char* wsu__Id = "Contract";
      char* name;
      char* title;
      char* terms;
    };
    int ns__myMethod(struct ns__myContract agreement, bool* accepted);
@endcode

The default value of the wsu:Id is "Contract" so that we can instantiate the
struct, automatically sign it, and send it as follows:

@code
    struct ns__myContract contract;
    bool accept;
    soap_default_ns__myContract(soap, &contract);
    contract.name = ...;
    contract.title = ...;
    contract.terms = ...;
    if (soap_wsse_add_BinarySecurityTokenX509(soap, "X509Token", cert)
     || soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(soap, "#X509Token")
     || soap_wsse_sign(soap, SOAP_SMD_SIGN_RSA_SHA256, rsa_private_key, 0))
      ... // an error occurred
    else if (soap_call_ns__myMethod(soap, contract, &accept))
      ... // a transmission error occurred
@endcode

The above example shows a wsu:Id attribute embedded (hardcoded) in a struct.
When it is not possible to add the `wsu__Id` member, for example when the type is
a string instead of a struct, it is suggested to specify the XML element to be
signed with the `soap_wsse_set_wsu_id(soap, "space-separated string of
element names")`. Use it before each call or in the server operation
(when returning XML data from a service operation). This lets the engine add
wsu:Id="tag" attribute-value pair to the element's tag name. For example:

@code
    soap_wsse_set_wsu_id(soap, "ns:myContract"); // <ns:myContract wsu:Id="ns:myContract">...
    soap_wsse_set_InclusiveNamespaces(soap, "ns xsd"); // QNames have 'ns' and 'xsd' values
    if (soap_wsse_add_BinarySecurityTokenX509(soap, "X509Token", cert)
     || soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(soap, "#X509Token")
     || soap_wsse_sign(soap, SOAP_SMD_SIGN_RSA_SHA256, rsa_private_key, 0))
      ... // an error occurred
    soap_wsse_set_wsu_id(soap, NULL); // reset
    soap_wsse_set_InclusiveNamespaces(soap, NULL); // reset
@endcode

This code adds the wsu:Id="ns-myContract" to the ns:myContract element. Here,
the `wsu__Id` value in the struct MUST NOT be set. Otherwise, two wsu:Id
attributes are present which is invalid. Also, the element signed must be
unique in the message. That is, there cannot be more than one matching element,
otherwise the resulting signature is invalid.

@note
To reset the automatic wsu:Id attributes addition, pass NULL to
`soap_wsse_set_wsu_id` as shown above. This is automatically performed when a new
message is received (but not automatically in a sequence of one-way sends for
example).

@note
It is generally known that QName content of elements and attribute values may
lead to verification issues with exclusive canonicalization
(`SOAP_XML_CANONICAL`), because XML processors may not recognize prefixes in
such QName contexts as visually utilized. With QName content in elements and
attributes and `SOAP_XML_CANONICAL` enabled, we should use
`soap_wsse_set_InclusiveNamespaces(soap, "prefixlist")` to define which
namespace prefixes (space-separated in the string) should be considered
inclusive. For example, xsi:type attribute values are QNames with xsd types
(and perhaps other types), meaning we should add "xsd" to the inclusive
namespace prefix list with `soap_wsse_set_InclusiveNamespaces(soap, "xsd")`
to ensure xsi:type="xsd:TYPE" attributes with QName content are properly signed
and not susceptible to certain wrapping attacks.  A quick way to include all
prefixes in the signed contents and thereby thwart signature wrapping attacks
is to use `soap_wsse_set_InclusiveNamespaces(soap, "+")`.

@note
When signing parts of the body as outlined above, use `soap_wsse_sign`
(do NOT use `soap_wsse_sign_body`).

@warning
Do not attempt to sign an element with a wsu:Id that is a subelement of another
element with a wsu:Id, that is, do not sign inner nested wsu:Id elements. The
element that you will try to sign will not be canonicalized and will lead to a
failure of the signature verification. When elements with wsu:Id are nested,
sign the outermost element.

We recommend to sign the entire SOAP Body using `soap_wsse_sign_body` and
reserve the use of `soap_wsse_set_wsu_id` for SOAP Header elements, such as
WS-Addressing elements. For example, to add and sign WS-Addressing 2005 headers
(which are activated with an `#import "wsa5.h"` in the header file for
soapcpp2):

@code
    #include "wsaapi.h"
    #include "wsseapi.h"
    soap_register_plugin(soap, soap_wsa);
    soap_register_plugin(soap, soap_wsse);
    ...
    soap_wsse_set_wsu_id(soap, "wsa5:From wsa5:To wsa5:ReplyTo wsa5:FaultTo wsa5:Action wsa5:MessageID");
    if (soap_wsa_request(soap, RequestMessageID, ToAddress, RequestAction)
     || soap_wsa_add_From(soap, FromAddress) // optional: add a 'From' address
     || soap_wsa_add_FaultTo(soap, FaultToAddress))
      ... // error: out of memory
    if (soap_wsse_add_BinarySecurityTokenX509(soap, "X509Token", cert)
     || soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(soap, "#X509Token")
     || soap_wsse_sign_body(soap, SOAP_SMD_SIGN_RSA_SHA256, rsa_private_key, 0))
      ... // an error occurred
    else if (soap_call_ns__myMethod(soap, ...))
      ... // a transmission error occurred
    soap_wsse_set_wsu_id(soap, NULL);
@endcode

This code signs the wsa5 header elements that are set with `soap_wsa_request`,
see the WS-Addressing "wsa" API in the gSOAP documentation for more information
on the use of WS-Addressing). It is fine to specify more elements with
`soap_wsse_set_wsu_id` than actually present in the XML payload. The other
WS-Addressing headers are not present and are not signed.

If your are using WS-Addressing 2004 (which is activated with an
`#import "wsa.h"` in the header file for soapcpp2) then change one line:

@code
    soap_wsse_set_wsu_id(soap, "wsa:From wsa:To wsa:ReplyTo wsa:FaultTo wsa:Action wsa:MessageID");
@endcode

@note
`soap_wsse_set_wsu_id` should only be set once for each `soap_wsse_sign`
or `soap_wsse_sign_body`. Each new call overrides the previous setting.

@warning
Never use `soap_wsse_set_wsu_id` to set the wsu:Id for an element that
occurs more than once in the payload, since each will have the same wsu:Id
attribute that may lead to a WS-Signature failure.

@subsection wsse_8_3 Signing Security Headers and Tokens

To sign security tokens such as user names, passwords, and binary security
tokens, just assign their Id values with a unique string, such as "Time" for
timestamps and "User" for user names. For example:

@code
    soap_wsse_add_Timestamp(soap, "Time", 600);
    soap_wsse_add_UsernameTokenDigest(soap, "User", "username", "password");
    ... // the rest of the signing code
@endcode

Note that by default all wsu:Id-attributed elements are signed. To filter a
subset of wsu:Id-attributed elements for signatures, use the
`soap_wsse_sign_only` function to specify a subset of the elements that have
wsu:Id values as follows:

@code
    soap_wsse_add_Timestamp(soap, "Time", 600);
    soap_wsse_add_UsernameTokenDigest(soap, "User", "username", "password");
    soap_wsse_add_BinarySecurityTokenX509(soap, "X509Token", cert);
    soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(soap, "#X509Token");
    soap_wsse_sign_body(soap, SOAP_SMD_SIGN_RSA_SHA256, rsa_private_key, 0);
    soap_wsse_sign_only(soap, "Time User Body"); // OK to use after soap_wsse_sign_body
@endcode

The wsu:Id values are provides with the `add` functions, such as "User" and
"X509Token". The SOAP Body always has a wsu:Id value "Body" when
`soap_wsse_sign_body` is used.

Note that in the above we MUST set the X509Token name for cross-referencing
with a wsu:Id, which normally results in automatically signing that token
unless filtered out with `soap_wsse_sign_only`. The SOAP Body wsu:Id is
always "Body" and should be part of the `soap_wsse_sign_only` set of wsu:Id
names to sign.

When using `soap_wsse_set_wsu_id` we need to use the tag name with
`soap_wsse_sign_only`. For example:

@code
    soap_wsa_request(soap, RequestMessageID, ToAddress, RequestAction);
    soap_wsse_set_wsu_id(soap, "wsa5:To wsa5:From wsa5:ReplyTo wsa5:Action");
    soap_wsse_add_UsernameTokenDigest(soap, "User", "username", "password");
    soap_wsse_add_BinarySecurityTokenX509(soap, "X509Token", cert);
    soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(soap, "#X509Token");
    soap_wsse_sign_body(soap, SOAP_SMD_SIGN_RSA_SHA256, rsa_private_key, 0);
    soap_wsse_sign_only(soap, "wsa5:To wsa5:From wsa5:ReplyTo wsa5:Action User Body");
@endcode

@note
`soap_wsse_sign_only` should only be set once for each `soap_wsse_sign`
or `soap_wsse_sign_body`. Each new call overrides the previous.

@note
To reset the filtering of signed tokens and elements, pass NULL to
`soap_wsse_sign_only`. This is automatically performed when a new message is
received (but not automatically in a sequence of one-way sends for example).

@subsection wsse_8_4 Signature Validation

To automatically verify the signature of an inbound message signed with DSA or
RSA algorithms, assuming the message contains the X509 certificate as a binary
security token, use:

@code
    struct soap *soap = soap_new1(SOAP_XML_CANONICAL);
    soap_register_plugin(soap, soap_wsse);
    soap_wsse_verify_auto(soap, SOAP_SMD_NONE, NULL, 0);
    soap->cafile = "cacerts.pem";  // file with CA certs of peers
    soap->capath = "dir/to/certs"; // and/or point to CA certs directory
    soap->crlfile = "revoked.pem"; // use CRL (optional)

    // server:
    if (soap_serve(soap))
      ... // an error occurred

    // client:
    if (soap_call_ns__myMethod(soap, ...))
      ... // an error occurred
@endcode

All locally referenced and signed elements in the signed message will be
verified with `soap_wsse_verify_auto` using the default settings set with
`SOAP_SMD_NONE`. Elements that are not signed cannot be verified. Also elements
referenced with absolute URIs that are not part of the message are not
automatically verified. The received message is stored in a DOM accessible with
soap->dom. This enables further analysis of the message content.

For a post-parsing check to verify if an XML element was signed in an inbound
message, use:

@code
    soap_wsse_verify_auto(soap, SOAP_SMD_NONE, NULL, 0);
    soap->cafile = "cacerts.pem";  // file with CA certs of peers
    soap->capath = "dir/to/certs"; // and/or point to CA certs directory
    soap->crlfile = "revoked.pem"; // use CRL (optional)
    ... // client call
    if (soap_wsse_verify_element(soap, "namespaceURI", "tag") == 1)
      ... // only one element with matching tag and namespace is signed
@endcode

The `soap_wsse_verify_element` function returns the number of matching elements
signed.

The signed element nesting rules are obeyed, so if the matching element is a
descendent of a signed element, it is signed as well.

Because it is a post check, a client should invoke `soap_wsse_verify_element`
after the call completed. A service should invoke this function within the
service operation routine, i.e. when the message request is accepted and about
to be processed.

For example, to check whether the wsu:Timestamp element was signed, e.g. after
checking that it is present and message expiration checked with
`soap_wsse_verify_Timestamp`, use:

@code
    if (!soap_wsse_verify_Timestamp(soap))
    {
      soap_wsse_delete_Security(soap);
      ... // error
    }
    else if (soap_wsse_verify_element(soap, "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd", "Timestamp") > 0)
      ... // timestamp was signed
@endcode

To check the SOAP Body (either using SOAP 1.1 or 1.2), simply use
`soap_wsse_verify_body`.

The `soap_wsse_verify_auto` function keeps processing signed (and unsigned)
messages as they arrive. For unsigned messages this can be expensive and the
verification engine should be shut down using `soap_wsse_verify_done`.

There can be two problems with signature verification. First, some
WS-Security implementations include SignedInfo/Reference/@URI without targeting
an element, which will produce an error that a Reference URI target does not
exist. To ignore these references, use `SOAP_WSSE_IGNORE_EXTRA_REFS`. Second,
certificates provided by the peer are not verifiable unless the signing CA
certificate is included in the cafile or capath. To disable peer certificate
verification, set the fsslverify callback to return 1 as follows:

@code
    static int ssl_verify(int ok, X509_STORE_CTX *store)
    {
      // put certificate verification here, return 0 when fails 1 when ok
      return 1;
    }
    ...
    soap_wsse_verify_auto(soap, SOAP_SMD_NONE | SOAP_WSSE_IGNORE_EXTRA_REFS, NULL, 0);
    soap->cafile = "cacerts.pem";  // file with CA certs of peers
    soap->capath = "dir/to/certs"; // and/or point to CA certs directory
    soap->crlfile = "revoked.pem"; // use CRL (optional)
    soap->fsslverify = ssl_verify; // set certificate verification callback
@endcode

To reject peer certificates under all conditions except specific permitted
conditions such as self-signed certificates in the chain, use the following
code as a guide (see OpenSSL documentation on `X509_STORE_CTX_get_error`):

@code
    static int ssl_verify(int ok, X509_STORE_CTX *store)
    {
      if (!ok)
      {
        char buf[1024];
        int err = X509_STORE_CTX_get_error(store);
        X509 *cert = X509_STORE_CTX_get_current_cert(store);
        switch (err)
        {
          case X509_V_ERR_CERT_NOT_YET_VALID:
          case X509_V_ERR_CERT_HAS_EXPIRED:
          case X509_V_ERR_DEPTH_ZERO_SELF_SIGNED_CERT:
          case X509_V_ERR_SELF_SIGNED_CERT_IN_CHAIN:
          case X509_V_ERR_UNABLE_TO_GET_CRL:
          case X509_V_ERR_CRL_NOT_YET_VALID:
          case X509_V_ERR_CRL_HAS_EXPIRED:
            X509_STORE_CTX_set_error(store, X509_V_OK);
            ok = 1;
            break;
          default:
            fprintf(stderr, "SSL verify error %d or warning with certificate at depth %d: %s\n", err, X509_STORE_CTX_get_error_depth(store), X509_verify_cert_error_string(err));
            X509_NAME_oneline(X509_get_issuer_name(cert), buf, sizeof(buf)-1);
            fprintf(stderr, "  certificate issuer:  %s\n", buf);
            X509_NAME_oneline(X509_get_subject_name(cert), buf, sizeof(buf)-1);
            fprintf(stderr, "  certificate subject: %s\n", buf);
        }
      }
      return ok;
    }
    ...
    soap_wsse_verify_auto(soap, SOAP_SMD_NONE | SOAP_WSSE_IGNORE_EXTRA_REFS, NULL, 0);
    soap->cafile = "cacerts.pem";  // file with CA certs of peers
    soap->capath = "dir/to/certs"; // and/or point to CA certs directory
    soap->crlfile = "revoked.pem"; // use CRL (optional)
    soap->fsslverify = ssl_verify;
@endcode

To verify the HMAC signature of an inbound message, the HMAC key must be
supplied:

@code
    static char hmac_key[16] = // the same secret key that was used to sign
    { 0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99, 0x88,
      0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00 };
    struct soap *soap = soap_new1(SOAP_XML_CANONICAL);
    soap_register_plugin(soap, soap_wsse);
    soap_wsse_verify_auto(soap, SOAP_SMD_HMAC_SHA1, hmac_key, sizeof(hmac_key));
    soap->cafile = "cacerts.pem";  // file with CA certs of peers
    soap->capath = "dir/to/certs"; // and/or point to CA certs directory
    soap->crlfile = "revoked.pem"; // use CRL (optional)

    // server:
    if (soap_serve(soap))
      ... // an error occurred

    // client:
    if (soap_call_ns__myMethod(soap, ...))
      ... // an error occurred
@endcode

To summarize the signature verification process:

-# Register the wsse plugin.
-# For HMAC, obtain the HMAC secret key
-# Use `soap_wsse_verify_auto` to verify inbound messages.
-# Set the cafile (or capath) to verify certificates of the peers and crlfile
   (optional)
-# After receiving a message, the DOM in soap->dom can be traversed for further
   analysis.
-# Always check the function return values for errors. You don't want to accept
   a request or response message with an invalid Security header.
-# Use `soap_wsse_verify_done` to terminate verification, e.g. to consume
   plain messages more efficiently.

@section wsse_9 Encryption

The material in this section relates to the WS-Security specification section 9.

When encryption is used with signing (@ref wsse_8), then encryption is always
applied after signing.  It is generally known that it is safe to perform
encryption after signing, but not vice versa.  In particular, this order allows
for the encryption of the signature and its digests, as required by
[Basic Security Profile 1.1](http://docs.oasis-open.org/ws-brsp/BasicSecurityProfile/v1.1/cs01/BasicSecurityProfile-v1.1-cs01.html) section 19.4.
The profile also requires the signature in the header to be encrypted and the
entire SOAP Body, rather than parts of the SOAP Body.  To this end, use
`soap_wsse_add_EncryptedKey_encrypt_only(..., "ds:Signature SOAP-ENV:Body")` as
described further below.

First, the wsse plugin must be registered:

@code
    struct soap *soap = soap_new1(SOAP_XML_CANONICAL);
    soap_register_plugin(soap, soap_wsse);
@endcode

To send messages with inclusive canonicalization, in addition to the
`SOAP_XML_CANONICAL` flag also use: 

@code
    soap_wsse_set_InclusiveNamespaces(soap, "*");
@endcode

However, exclusive canonicalization is recommended over inclusive
canonicalization, or no canonicalization at all.  WS Basic Security profile 1.0
requires exclusive canonicalization.

Flags to consider:

- `SOAP_XML_CANONICAL` recommended to enable exc-c14n (exclusive canonicalization).
- `SOAP_XML_INDENT` optional, to emit more readable XML (see warning).
- `SOAP_IO_CHUNK` efficient HTTP-chunked streaming messages.
- `SOAP_ENC_GZIP` for HTTP compression (also enables HTTP chunking).

@warning
Interoperability with WCF WS-Security is not guaranteed when `SOAP_XML_INDENT`
is enabled.  Avoid using `SOAP_XML_INDENT` for interoperability.  The
implementation of C14N in WCF with respect to the normalization of white space
between XML tags differs from the protocol standards.

@subsection wsse_9_1 Encrypting Messages

Encryption should be used in combination with signing. A signature ensures
message integrity while encryption ensures confidentially. Encrypted messages
can be tampered with unless integrity is ensured. Therefore, the reader should
be familiar with the material in Section @ref wsse_8 should to sign and verify
message content.

Messages are encrypted using either public key cryptography or by using a
symmetric secret key. A symmetric secret key should only be shared between the
sender and receiver (or any trusted communicating peer).

Encryption with public key cryptography uses an "envelope" process, where the
public key of the recipient is used to encrypt a temporary (ephemeral) secret
key that is sent together with the secret key-encrypted message to the
recipient. The recipient decrypts the ephemeral key and uses it to decrypt the
message. The public key is usually part of a X509 certificate. The public key
(containing the subject information) is added to the Security header and used
for encryption of the SOAP Body as follows:

@code
    X509 *cert = ...; // from PEM file or local secure store as shown above
    if (soap_wsse_add_EncryptedKey(soap, SOAP_MEC_ENV_ENC_DES_CBC, "Cert", cert, NULL, NULL, NULL))
      soap_print_fault(soap, stderr);
@endcode

`SOAP_MEC_ENV_ENC_DES_CBC` specifies envelope encoding with triple DES CBC and
PKCS1 RSA-1_5. Use `(SOAP_MEC_ENV_ENC_AES256_CBC | SOAP_MEC_OAEP)` for AES256
CBC with OAEP padding (OAEP is recommended over RSA-1_5 or use GCM).

The envelope encryption options are:

- `SOAP_MEC_ENV_ENC_DES_CBC`                    RSA-1_5 envelope encryption with triple DES CBC
- `SOAP_MEC_ENV_ENC_AES256_CBC`                 RSA-1_5 envelope encryption with AES256 CBC
- `SOAP_MEC_ENV_ENC_AES256_GCM`                 envelope authenticated encryption with AES256 GCM
- `SOAP_MEC_ENV_ENC_AES256_CBC | SOAP_MEC_OAEP` OAEP envelope encryption with AES256 CBC

where, in the above, AES256 can also be replaced with AES128 or AES192.

The "Cert" parameter is a unique URI to reference the key from the encrypted
SOAP Body. The above enables the encryption engine for the next message to be
sent, either at the client or server side. The server should use this withing a
server operation (before returning) to enable the service operation response to
be encrypted.

To include a subject key ID in the Security header instead of the entire public
key, specify the subject key ID parameter:

@code
    X509 *cert = ...; // from PEM file or local secure store as shown above
    if (soap_wsse_add_EncryptedKey(soap, SOAP_MEC_ENV_ENC_DES_CBC, "Cert", cert, "Subject Key ID", NULL, NULL))
      soap_print_fault(soap, stderr);
@endcode

The difference with the previous example where no subject key ID was specified
is that the Security header only contains the subject key ID and no longer the
public key in base64 format.

To exclude the encrypted key certificate from the message and include a
X509Data element with IssuerName and SerialNumber:

@code
    X509 *cert = ...; // from PEM file or local secure store as shown above
    if (soap_wsse_add_EncryptedKey(soap, SOAP_MEC_ENV_ENC_DES_CBC, "Cert", cert, NULL, "CN=Root Agency", "-79441640260855276448009124614332182350"))
      soap_print_fault(soap, stderr);
@endcode

The issuer name and serial number (must be in decimal for
`soap_wsse_add_EncryptedKey`) of a certificate can be obtained as follows:

@code
    X509 *cert = ...; // from PEM file or local secure store as shown above
    BIGNUM *bn = BN_new();
    char issuer[256], *serial;
    X509_NAME_oneline(X509_get_issuer_name(cert), issuer, sizeof(issuer)-1);
    ASN1_INTEGER_to_BN(X509_get_serialNumber(cert), bn);
    serial = BN_bn2dec(bn);
    OPENSSL_free(bn);
    ...
    if (soap_wsse_add_EncryptedKey(soap, SOAP_MEC_ENV_ENC_DES_CBC, "Cert", cert, NULL, issuer+1, serial))
      soap_print_fault(soap, stderr);
    ...
    OPENSSL_free(serial);
@endcode

Note that in the above code the leading slash in "/CN=Root Agency" is excluded
from the issuer name.

When excluding the encrypted key certificate from the message, the token
handler callback must be provided on the receiving end to obtain the
certificate that corresponds to the issuer name and serial number. 

To encrypt specific elements of the SOAP Header, such as the signature, and
Body rather than just the SOAP Body alone, use
`soap_wsse_add_EncryptedKey_encrypt_only` to specify elements to encrypt in
combination with `soap_wsse_set_wsu_id` to specify these elements:

@code
    X509 *cert = ...; // from PEM file or local secure store as shown above
    // the SOAP Body contains one <ns:myContract> and one <ns:myPIN> (not nested)
    soap_wsse_set_wsu_id(soap, "ns:myContract ns:myPIN");
    if (soap_wsse_add_EncryptedKey_encrypt_only(soap, SOAP_MEC_ENV_ENC_DES_CBC, "Cert", cert, NULL, NULL, NULL, "ds:Signature ns:myContract ns:myPIN"))
      soap_print_fault(soap, stderr);
@endcode

To encrypt the SOAP Body and SOAP Header element(s), such as ds:Signature,
use "SOAP-ENV:Body" with `soap_wsse_add_EncryptedKey_encrypt_only` to specify
these elements:

@code
    X509 *cert = ...; // from PEM file or local secure store as shown above
    if (soap_wsse_add_EncryptedKey_encrypt_only(soap, SOAP_MEC_ENV_ENC_DES_CBC, "Cert", cert, NULL, NULL, NULL, "ds:Signature SOAP-ENV:Body"))
      soap_print_fault(soap, stderr);
@endcode

@note
The `soap_wsse_set_wsu_id` MUST be used to specify all element tag names to
encrypt. Additional elements MAY be specified in `soap_wsse_set_wsu_id` (for
example elements to digitally sign). You do not have to use this function to
set the wsu:Id of the SOAP Body which always has a wsu:Id with "Body".

@note
The elements identified by the tag names in `soap_wsse_set_wsu_id` to
encrypt MUST occur NO MORE THAN ONCE in the XML message.

For symmetric encryption with a shared secret key, generate a 160-bit triple
DES key and make sure both the sender and reciever can use the key without it
being shared by any other party (key exchange problem). Then use the
`soap_wsse_encrypt_body` function to encrypt the SOAP Body as follows:

@code
    char des_key[20] = ...; // 20-byte (160-bit) DES shared secret key
    if (soap_wsse_encrypt_body(soap, SOAP_MEC_ENC_DES_CBC, des_key, sizeof(des_key)))
      soap_print_fault(soap, stderr);
@endcode

The symmetric encryption options are:

- `SOAP_MEC_ENC_DES_CBC`             symmetric encryption with triple DES CBC
- `SOAP_MEC_ENC_AES256_CBC`          symmetric encryption with AES256 CBC
- `SOAP_MEC_ENC_AES256_GCM`          symmetric authenticated encryption with AES256 GCM

where, in the above, AES256 can also be replaced with AES128 or AES192.

For example, symmetric encryption with AES256:

@code
    char aes256_key[32] = ...; // 32-byte (256-bit) AES256 shared secret key
    if (soap_wsse_encrypt_body(soap, SOAP_MEC_ENC_AES256_CBC, aes256_key, sizeof(aes256_key)))
      soap_print_fault(soap, stderr);
@endcode

To symmetrically encrypt specific elements of the SOAP Body rather than the
entire SOAP Body, use `soap_wsse_encrypt_only` to specify the elements to
encrypt in combination with `soap_wsse_set_wsu_id` to specify these elements:

@code
    char des_key[20] = ...; // 20-byte (160-bit) secret key
    // the SOAP Body contains one <ns:myContract> and one <ns:myPIN> (not nested)
    soap_wsse_set_wsu_id(soap, "ns:myContract ns:myPIN");
    if (soap_wsse_encrypt_only(soap, SOAP_MEC_ENC_DES_CBC, des_key, sizeof(des_key), "ds:Signature ns:myContract ns:myPIN"))
      soap_print_fault(soap, stderr);
@endcode

@note
The `soap_wsse_set_wsu_id` MUST be used to specify all element tag names to
encrypt. Additional elements MAY be specified in `soap_wsse_set_wsu_id` (for
example elements to digitally sign).

@note
The elements identified by the tag names in `soap_wsse_set_wsu_id` to
encrypt MUST occur EXACTLY ONCE in the SOAP Body.

@subsection wsse_9_2 Decrypting Message Parts

The wsse engine automatically decrypts message parts, but requires a private
key or secret shared key to do so. A default key can be given to enable
decryption, but it will fail if a non-compatible key was used for encryption.
In that case a token handler callback should be defined by the user to select a
proper decryption key based on the available subject key name or identifier
embedded in the encrypted message.

An example of a token handler callback:

@code
    soap_register_plugin_arg(soap, soap_wsse);
    soap_wsse_set_security_token_handler(soap, security_token_handler);

    const void *security_token_handler(struct soap *soap, int *alg, const char *keyname, const unsigned char *keyid, int keyidlen, int *keylen)
    {
      // Get the user name from UsernameToken in message
      const char *uid = soap_wsse_get_Username(soap);
      switch (*alg)
      {
        case SOAP_SMD_VRFY_DSA_SHA1:
        case SOAP_SMD_VRFY_DSA_SHA256:
        case SOAP_SMD_VRFY_RSA_SHA1:
        case SOAP_SMD_VRFY_RSA_SHA224:
        case SOAP_SMD_VRFY_RSA_SHA256:
        case SOAP_SMD_VRFY_RSA_SHA384:
        case SOAP_SMD_VRFY_RSA_SHA512:
        case SOAP_SMD_VRFY_ECDSA_SHA1:
        case SOAP_SMD_VRFY_ECDSA_SHA224:
        case SOAP_SMD_VRFY_ECDSA_SHA256:
        case SOAP_SMD_VRFY_ECDSA_SHA384:
        case SOAP_SMD_VRFY_ECDSA_SHA512:
          if (uid)
          {
            // Lookup uid to retrieve the X509 certificate to verify the signature
            const X509 *cert = ...; 
            return (const void*)cert;
          }
          break;
        case SOAP_SMD_HMAC_SHA1:
        case SOAP_SMD_HMAC_SHA224:
        case SOAP_SMD_HMAC_SHA256:
        case SOAP_SMD_HMAC_SHA384:
        case SOAP_SMD_HMAC_SHA512:
          if (uid)
          {
            // Lookup uid to retrieve the HMAC SHA key to verify the signature
            const void *key = ...; 
            *alg = ...;
            *keylen = ...;
            return key;
          }
          return NULL; // no certificate: fail
        case SOAP_MEC_ENV_DEC_DES_CBC:
        case SOAP_MEC_ENV_DEC_AES128_CBC:
        case SOAP_MEC_ENV_DEC_AES192_CBC:
        case SOAP_MEC_ENV_DEC_AES256_CBC:
        case SOAP_MEC_ENV_DEC_AES512_CBC: // reserved for future use
        case SOAP_MEC_ENV_DEC_AES128_GCM: // GCM requires OpenSSL 1.0.2 or higher
        case SOAP_MEC_ENV_DEC_AES192_GCM: // GCM requires OpenSSL 1.0.2 or higher
        case SOAP_MEC_ENV_DEC_AES256_GCM: // GCM requires OpenSSL 1.0.2 or higher
        case SOAP_MEC_ENV_DEC_AES512_GCM: // GCM requires OpenSSL 1.0.2 or higher
          if (keyname)
          {
            // use this to get the key or certificate from a key store using the keyname value:
            // 1. keyname is set to the subject name of the certificate, if a
            //    certificate is present in the SecurityTokenReference/KeyIdentifier
            //    when ValueType is http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-x509-token-profile-1.0#X509v3
            // 2. keyname is set to the string concatenation
            //     "{X509IssuerName}#{X509SerialNumber}" of the X509IssuerName
            //     and X509SerialNumber present in X509Data/X509IssuerSerial
            // 3. keyname is set to X509Data/X509SubjectName
            return ...;
          }
          else if (keyid)
          {
            // use this to get the key from a key store using the keyid[0..keyidlen-1]:
            // 1. keyid and keyidlen are set to the data in
            //    SecurityTokenReference/KeyIdentifier when the ValueType is
            //    http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-x509-token-profile-1.0#X509SubjectKeyIdentifier
            return ...;
          }
          break;
        case SOAP_MEC_DEC_DES_CBC:
        case SOAP_MEC_DEC_AES128_CBC:
        case SOAP_MEC_DEC_AES192_CBC:
        case SOAP_MEC_DEC_AES256_CBC:
        case SOAP_MEC_DEC_AES512_CBC: // reserved for future use
        case SOAP_MEC_DEC_AES128_GCM: // GCM requires OpenSSL 1.0.2 or higher
        case SOAP_MEC_DEC_AES192_GCM: // GCM requires OpenSSL 1.0.2 or higher
        case SOAP_MEC_DEC_AES256_GCM: // GCM requires OpenSSL 1.0.2 or higher`
        case SOAP_MEC_DEC_AES512_GCM: // GCM requires OpenSSL 1.0.2 or higher
          if (keyname)
          {
            // use the keyname to get the shared secret key associated for decryption
            *keylen = ... // length of the shared secret key
            return ...;
          }
          break;
      }
      return NULL; // fail
    }
@endcode

The last two case-arms are used to return a key associated with the keyname
paramater, which is a string that contains the subject key id from the public
key information in an encrypted message or the subject key ID string that was
set with `soap_wsse_add_EncryptedKey` at the sender side.

@warning
The security token handler callback function parameters have changed in 2.8.34
and greater with the addition of KeyIdentifier information `keyid` and
`keyidlen`.

To set the default private key for envelope decryption, use:

@code
    EVP_PKEY *rsa_private_key = ...;
    soap_wsse_decrypt_auto(soap, SOAP_MEC_ENV_DEC_DES_CBC, rsa_private_key, 0);
@endcode

The envelope decryption options are:

- `SOAP_MEC_ENV_DEC_DES_CBC`                    RSA-1_5 envelope decryption with triple DES CBC
- `SOAP_MEC_ENV_DEC_AES256_CBC`                 RSA-1_5 envelope decryption with AES256 CBC
- `SOAP_MEC_ENV_DEC_AES256_GCM`                 envelope authenticated decryption with AES256 GCM
- `SOAP_MEC_ENV_DEC_AES256_CBC | SOAP_MEC_OAEP` OAEP envelope decryption with AES256 CBC

where, in the above, AES256 can be replaced with AES128 or AES192.

To set the default shared secret key for symmetric decryption, use:

@code
    char des_key[20] = ...; // 20-byte (160-bit) triple DES key
    soap_wsse_decrypt_auto(soap, SOAP_MEC_DEC_DES_CBC, des_key, sizeof(des_key));
@endcode

The symmetric decryption options are:

- `SOAP_MEC_DEC_DES_CBC`             symmetric decryption with triple DES CBC
- `SOAP_MEC_DEC_AES256_CBC`          symmetric decryption with AES256 CBC
- `SOAP_MEC_DEC_AES256_GCM`          symmetric authenticated decryption with AES256 GCM

where, in the above, AES256 can be replaced with AES128 or AES192.

For example, symmetric decryption with AES256:

@code
    char aes256_key[32] = ...; // 32-byte (256-bit) AES256 key
    soap_wsse_decrypt_auto(soap, SOAP_MEC_DEC_AES256_CBC, aes256_key, sizeof(aes256_key));
@endcode

If a default key is not set, the token handler callback should be used as
discussed above in this section. Do NOT set a default key if a token handler is
used to handle multiple different keys. The default key mechanism is simpler to
use when only one decryption key is used to decrypt all encrypted messages.

To remove the default key, use:

@code
    soap_wsse_decrypt_auto(soap, SOAP_MEC_NONE, NULL, 0);
@endcode

@subsection wsse_9_3 Example Client and Server

The code for a client is shown below that uses signatures and encryption:

@code
    FILE *fd;
    EVP_PKEY *rsa_private_key;
    X509 *cert;
    // create new context with recommended C14N enabled
    struct soap *soap = soap_new1(SOAP_XML_CANONICAL);
    // register the plugin(s)
    soap_register_plugin(soap, soap_wsse);
    // enable peer certificate verification
    soap->cafile = "cacerts.pem";  // file with CA certs of peers
    soap->capath = "dir/to/certs"; // and/or point to CA certs directory
    soap->crlfile = "revoked.pem"; // use CRL (optional)
    soap->fsslverify = ssl_verify; // optional, a callback to verify peer certificates
    // get the private key for signing and decryption
    fd = fopen("privkey.pem", "r");
    rsa_private_key = PEM_read_PrivateKey(fd, NULL, NULL, "password");
    fclose(fd);
    // get the certificate to include in the security header
    fd = fopen("cert.pem", "r");
    X509 *cert = PEM_read_X509(fd, NULL, NULL, NULL);
    fclose(fd);
    // enable decryption with the private key
    soap_wsse_decrypt_auto(soap, SOAP_MEC_ENV_DEC_DES_CBC, rsa_private_key, 0);
    // enable signature verification
    soap_wsse_verify_auto(soap, SOAP_SMD_NONE, NULL, 0);
    // add the certificate X509 token and token reference, sign the Body using the private key
    if (soap_wsse_add_BinarySecurityTokenX509(soap, "X509Token", cert)
     || soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(soap, "#X509Token")
     || soap_wsse_sign_body(soap, SOAP_SMD_SIGN_RSA_SHA256, rsa_private_key, 0))
      ... // an error occurred
    // encrypt the Body and the signature using the public key in cert (the cert of the peer loaded from PEM)
    soap_wsse_set_wsu_id(soap, "ds:Signature");
    if (soap_wsse_add_EncryptedKey_encrypt_only(soap, SOAP_MEC_ENV_ENC_DES_CBC, "Cert", cert, NULL, NULL, NULL, "ds:Signature SOAP-ENV:Body"))
      ... // an error occurred
    if (soap_call_ns__myMethod(soap, ...))
      ... // a transmission error occurred
    ...
    EVP_PKEY_free(rsa_private_key);
    X509_free(cert);
@endcode

When using HTTPS, the `soap->cafile`, `soap->capath` are already set with
`soap_ssl_client_context()`. The explicit assignments shown above are not
needed.

See @ref wsse_8_4 on how to implement the optional `ssl_verify` callback to
verify peer certificates.

You may want to register a token handler callback if the peer does not include
its X509 certificate in the security header. The token handler callback should
retrieve the certificate, e.g. given its id and serial number.

The server-side service is implemented as follows:

@code
    EVP_PKEY *rsa_private_key; // OK to declare global
    X509 *cert;                // OK to declare global
    ..
    FILE *fd;
    // create new context with recommended C14N enabled
    struct soap *soap = soap_new1(SOAP_XML_CANONICAL);
    // register the plugin(s)
    soap_register_plugin(soap, soap_wsse);
    // enable peer certificate verification
    soap->cafile = "cacerts.pem";  // file with CA certs of peers
    soap->capath = "dir/to/certs"; // and/or point to CA certs directory
    soap->crlfile = "revoked.pem"; // use CRL (optional)
    soap->fsslverify = ssl_verify; // optional, a callback to verify peer certificates
    // get the private key for signing and decryption
    fd = fopen("privkey.pem", "r");
    rsa_private_key = PEM_read_PrivateKey(fd, NULL, NULL, "password");
    fclose(fd);
    // get the certificate to include in the security header
    fd = fopen("cert.pem", "r");
    X509 *cert = PEM_read_X509(fd, NULL, NULL, NULL);
    fclose(fd);
    ...
    if (!soap_valid_socket(soap_bind(soap, NULL, port, 100)))
      ... // an error occurred
    while (soap_valid_socket(soap_accept(soap)))
    {
      // enable signature verification
      soap_wsse_verify_auto(soap, SOAP_SMD_NONE, NULL, 0);
      // enable decryption with the private key
      soap_wsse_decrypt_auto(soap, SOAP_MEC_ENV_DEC_DES_CBC, rsa_private_key, 0);
      // serve one request
      if (soap_serve(soap))
      {
        soap_wsse_delete_Security(soap);
        soap_print_fault(soap, stderr);
      }
    }
    ...
    EVP_PKEY_free(rsa_private_key);
    X509_free(cert);
@endcode

where an example service operation could be:

@code
    int ns__myMethod(struct soap *soap, ...)
    {
      ...
      // remove old security headers
      soap_wsse_delete_Security(soap);
      // encrypt the Body and the signature using the public key in cert (the cert of the peer loaded from PEM)
      soap_wsse_set_wsu_id(soap, "ds:Signature");
      // add the certificate X509 token and token reference, sign the Body using the private key
      // encrypt the Body and the signature using the public key in cert (the cert of the peer loaded from PEM)
      if (soap_wsse_add_BinarySecurityTokenX509(soap, "X509Token", cert)
       || soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(soap, "#X509Token")
       || soap_wsse_sign_body(soap, SOAP_SMD_SIGN_RSA_SHA256, rsa_private_key, 0)
       || soap_wsse_add_EncryptedKey_encrypt_only(soap, SOAP_MEC_ENV_ENC_DES_CBC, "Cert", cert, NULL, NULL, NULL, "ds:Signature SOAP-ENV:Body"))
      {
        soap_wsse_delete_Security(soap); // remove incomplete security headers
        return soap->error;
      }
      return SOAP_OK;
    }
@endcode

The service operation signs the Body using its private key and encrypts the
response Body and signature using a public key from the peer's certificate.

To implement a server that supports HTTP keep-alive, a `soap->fserveloop`
callback function should be assigned. This callback is executed in the
`soap_serve()` loop to call `soap_wsse_verify_auto()` and
`soap_wsse_decrypt_auto()` to ensure that the continuous inbound message stream
can be verified and decrypted:

@code
    struct soap *soap = soap_new1(SOAP_XML_CANONICAL | SOAP_IO_KEEPALIVE);
    soap_register_plugin(soap, soap_wsse);
    ...
    soap->fserveloop = set_verify_decrypt_auto;
    ...
    while (soap_valid_socket(soap_accept(soap)))
    {
      set_verify_decrypt_auto(soap);
      // serve multiple requests (when keep alive)
      if (soap_serve(soap))
      {
        soap_wsse_delete_Security(soap);
        soap_print_fault(soap, stderr);
      }
    }

    int set_verify_decrypt_auto(struct soap *soap)
    {
      // enable signature verification
      soap_wsse_verify_auto(soap, SOAP_SMD_NONE, NULL, 0);
      // enable decryption with the private key
      soap_wsse_decrypt_auto(soap, SOAP_MEC_ENV_DEC_DES_CBC, rsa_private_key, 0);
      return SOAP_OK;
    }
@endcode

@section wsse_10 Security Timestamps

The material in this section relates to the WS-Security specification section
10.

To add a timestamp with the creation time to the Security header, use:

@code
    soap_wsse_add_Timestamp(soap, NULL, 0); // 0 means no expiration
@endcode

The lifetime of a message (in seconds) is passed as the third argument, which
will be displayed as the timestamp expiration time:

@code
    soap_wsse_add_Timestamp(soap, NULL, 10); // 10 seconds lifetime
@endcode

Timestamps, like other header elements, are not automatically secured with a
digital signature. To secure a timestamp, we add an identifier (wsu:Id) to each
element we want the WS-Security plugin to sign thereby making it impossible for
someone to tamper with that part of the message. To do this for the timestamp,
we simply pass a unique identification string as the second argument:

@code
    soap_wsse_add_Timestamp(soap, "Time", 10); // timestamp will be signed
@endcode

After receiving a message, the receiver can verify the presence and validity of
the timestamp and whether it was signed with:

@code
    if (!soap_wsse_verify_Timestamp(soap))
    {
      soap_wsse_delete_Security(soap);
      ... // error, no timestamp or timestamp has expired
    }
    else if (soap_wsse_verify_element(soap, "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd", "Timestamp") > 0)
      ... // timestamp was signed
@endcode

@section wsse_11 WS-Security and HTTPS

HTTPS is used at the client side with the usual "https:" URL addressing, shown
here with the registration of the wsse plugin and setting up locks for
thread-safe use of SSL for HTTPS:

@code
    #include "wsseapi.h"
    #include "threads.h"
    struct soap *soap;
    if (CRYPTO_thread_setup())
      ... // error
    soap = soap_new1(SOAP_XML_CANONICAL);
    soap_register_plugin(soap, soap_wsse);
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
    ... // set up WS-Security for signatures/encryption etc
    if (soap_call_ns__myMethod(soap, "https://...", ...))
      ... // error
    ... // process response results
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
    CRYPTO_thread_cleanup();
@endcode

The CRYPTO threads should be set up before any threads are created.

The `soap_ssl_client_context` only needs to be set up once. Use the following
flags:

- `SOAP_SSL_DEFAULT` requires server authentication, CA certs should be used
- `SOAP_SSL_NO_AUTHENTICATION` disables server authentication
- `SOAP_SSL_SKIP_HOST_CHECK` disables server authentication host check
- `SOAP_SSL_ALLOW_EXPIRED_CERTIFICATE` to accept self-signed certificates, expired certificates, and certificates without CRL.

The server uses the following:

@code
    #include "wsseapi.h"
    #include "threads.h"
    SOAP_SOCKET m, s;
    int port = 443;
    struct soap *soap;
    if (CRYPTO_thread_setup())
      ... // error
    soap = soap_new1(SOAP_XML_CANONICAL);
    soap_register_plugin(soap, soap_wsse);
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

where we define a `process_request()` function that is executed by the thread
to process the request (on a copy of the soap context struct):

@code
    void *process_request(struct soap *soap)
    {
      ... // set up WS-Security for signatures/encryption etc
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

We also should implement the mutex setup and cleanup operations as follows:

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
gSOAP package directory gsoap/samples/ssl.

@section wsse_12 Miscellaneous

The Security header block was generated from the WS-Security schema with the
wsdl2h tool and WS/WS-typemap.dat:

    wsdl2h -cegxy -o wsse.h -t WS/WS-typemap.dat WS/wsse.xsd

The same process was used to generate the header file ds.h from the XML digital
signatures core schema, and the xenc.h encryption schema:

    wsdl2h -cuxy -o ds.h -t WS/WS-typemap.dat WS/ds.xsd
    wsdl2h -cuxy -o xenc.h -t WS/WS-typemap.dat WS/xenc.xsd

The import/wsse.h file has the following definition for the Security header
block:

@code
typedef struct _wsse__Security
{       struct _wsu__Timestamp*                 wsu__Timestamp;
        struct _wsse__UsernameToken*            UsernameToken;
        struct _wsse__BinarySecurityToken*      BinarySecurityToken;
        struct xenc__EncryptedKeyType*          xenc__EncryptedKey;
        struct _xenc__ReferenceList*            xenc__ReferenceList;
        struct ds__SignatureType*               ds__Signature;
        @char*                                  SOAP_ENV__actor;
        @char*                                  SOAP_ENV__role;
} _wsse__Security;
@endcode

The `_wsse__Security` header is modified by a WS/WS-typemap.dat mapping rule to
include additional details.

@section wsse_13 Limitations and Security Considerations

- Encryption of simple content (CDATA content) directly with
  `soap_wsse_add_EncryptedKey_encrypt_only` is not supported.  You can encrypt
  elements with complex content (complexType and complexContent elements that
  have sub elements).  This is not a limitation for decryption with the WSSE
  plugin, which is not limited to elements with complex content.

- `EncryptedHeader` elements (WS-Security 1.1.1) are not supported.  Any or all
  subelements of a SOAP Header may be encrypted, but not the SOAP Header
  itself, replaced by `<wsse11:EncryptedHeader>` with encrypted contents.

- Encryption is performed after signing (likewise, signatures are verified
  after decryption).  Signing after encryption is not supported in the current
  plugin release.  It is generally known that it is safer to perform encryption
  after signing as the wsse plugin performs, and not vice versa.  In particular,
  this order allows for the encryption of the signature and its digests, as
  required by [Basic Security Profile 1.1](http://docs.oasis-open.org/ws-brsp/BasicSecurityProfile/v1.1/cs01/BasicSecurityProfile-v1.1-cs01.html) section 19.4.  To this end, use
  `soap_wsse_add_EncryptedKey_encrypt_only(..., "ds:Signature SOAP-ENV:Body")`.

- Signing and encrypting XML containing QName content may lead to verification
  issues, because the W3C C14N exclusive canonicalization protocol has known
  limitations with QName content normalization.  In fact, not signing the
  prefix bindings as in C14N exclusive canonicalization is a security risk,
  because the prefix-URI binding can be modified when placed at an ancestor
  node that is not included in the signed XML content.  Use
  `soap_wsse_set_InclusiveNamespaces(soap, "prefixlist")` to define all
  namespace prefixes (space-separated in the string) that should be considered
  inclusive. All prefixes used in QName content should be listed to mitigate
  the security risks outlined. The WSSE engine recognizes xsi:type,
  SOAP-ENC:arrayType, SOAP-ENC:itemType attribute QNames. Therefore, soapcpp2
  option `-t` is always safe to use, but a non-gSOAP receiver may still fail.

@section wsse_14 Defending Against Signature Wrapping Attacks

Signature wrapping attacks exploit a vulnerability in the XML DSig standard by
tricking the signature verifier to verify the signature of the signed content
but when this content is moved to a different place in the XML document, for
example where the content is ignored.  In this attack, a signed XML element
identified with an `id` attribute is moved in the document and replaced with an
unsigned replacement element with aribitrary content that the attacker created
and the receiver will use instead.  We refer to online articles and
publications on signature wrapping attacks for more details.

To defend against signature wrapping attacks, it is recommended to sign the
SOAP Body instead of individual elements of the SOAP Body.  A receiver must
verify that the SOAP Body received is indeed signed and verify that other parts
of the message such as critical SOAP Headers are signed.  You can do this on
the receiving end by calling `soap_wsse_verify_body(soap)` and check that the
return value is nonzero.  This prevents signature wrapping attacks on the SOAP
Body.  If individual element(s) of the SOAP Body must be signed instead of the
body itself, then make sure to use call
`soap_wsse_verify_element(soap, "namespaceURI", "tag")` on the receiving end
and check that the return value is nonzero to verify that all elements matching
the namespaceURI and tag are signed.

If SOAP Headers are signed such as the timestamp and username token then make
sure to verify that the timestamp was indeed signed by calling
`soap_wsse_verify_element(soap, SOAP_NAMESPACE_OF_wsu, "Timestamp")`
and check that the return value is nonzero. Likewise, to verify that the
usernameToken authentication credentials are signed, call
`soap_wsse_verify_element(soap, SOAP_NAMESPACE_OF_wsse, "UsernameToken")`
and check that the return value is nonzero.

To prevent signature wrapping attacks on XML namespace prefixes used in QNames,
which are vulnerable when the prefix is bound to a namespace URI in an
ancester node to the signed content, use
`soap_wsse_set_InclusiveNamespaces(soap, "prefixlist")`.  This makes the
namespace prefixes in the list (space-separated in the string) inclusive.
Use `soap_wsse_set_InclusiveNamespaces(soap, "+")` to automatically add all
prefixes defined in the namespace table (i.e. the .nsmap file) to the inclusive
namespace list (this requires gSOAP 2.8.64 or greater).

@section wsse_wsc WS-SecureConversation and WS-Trust

To use a WS-SecureConversation security context token (SCT) with WS-Security:

@code
    const char *identifier = "...";
    soap_wsse_add_SecurityContextToken(soap, "SCT", identifier);
@endcode

In this example a context has been established and the secret that is
identified by the 'identifier' string is known to both parties. This secret is
used to sign the message body. The "SCT" is a wsu:Id, which is used as a
reference to sign the token.

To compute PSHA1 with base64 input strings `client_secret_base64` and
`server_secret_base64` to output a base64-encoded `psha1[0..psha1len-1]` string
`psha1_base64`:

@code
    int psha1len = 32; // or greater
    int n, m;
    const char *client_secret = soap_base642s(soap, client_secret_base64, NULL, 0, &n);
    const char *server_secret = soap_base642s(soap, server_secret_base64, NULL, 0, &m);
    char psha1[psha1len];
    char *psha1_base64;
    if (soap_psha1(soap, client_secret, n, server_secret, m, psha1, psha1len))
      .. error // insufficient memory
    psha1_base64 = soap_s2base64(soap, (unsigned char*)psha1, NULL, psha1len);
@endcode

Similarly, PSHA256 can be computed by calling `soap_psha256()`.
*/

#include "wsseapi.h"
#include "threads.h"    /* need threads to enable mutex for MT */

#if defined(SOAP_WSA_2003) || defined(SOAP_WSA_2004) || defined(SOAP_WSA_200408) || defined(SOAP_WSA_2005)
#include "wsaapi.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Plugin identification for plugin registry */
const char soap_wsse_id[14] = SOAP_WSSE_ID;

/** Maximum number of SignedInfo References */
#define SOAP_WSSE_MAX_REF       (100)

/** Clock skew between machines (in sec) to fit message expiration in window */
#define SOAP_WSSE_CLKSKEW       (300)

/** Size of the random nonce */
#define SOAP_WSSE_NONCELEN      (20)
/** Digest authentication accepts messages that are not older than creation time + SOAP_WSSE_NONCETIME */
#define SOAP_WSSE_NONCETIME     (SOAP_WSSE_CLKSKEW + 240)

/******************************************************************************\
 *
 *      Common URIs
 *
\******************************************************************************/

const char *wsse_PasswordTextURI = "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordText";
const char *wsse_PasswordDigestURI = "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordDigest";
const char *wsse_Base64BinaryURI = "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-soap-message-security-1.0#Base64Binary";
const char *wsse_HexBinaryURI = "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-soap-message-security-1.0#HexBinary";
const char *wsse_X509v3URI = "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-x509-token-profile-1.0#X509v3";
const char *wsse_X509v3SubjectKeyIdentifierURI = "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-x509-token-profile-1.0#X509SubjectKeyIdentifier";

const char *ds_envsigURI = "http://www.w3.org/2000/09/xmldsig#enveloped-signature";

const char *ds_sha1URI = "http://www.w3.org/2000/09/xmldsig#sha1";
const char *ds_sha224URI = "http://www.w3.org/2001/04/xmldsig-more#sha224";
const char *ds_sha256URI = "http://www.w3.org/2001/04/xmlenc#sha256";
const char *ds_sha384URI = "http://www.w3.org/2001/04/xmldsig-more#sha384";
const char *ds_sha512URI = "http://www.w3.org/2001/04/xmlenc#sha512";

const char *ds_hmac_sha1URI = "http://www.w3.org/2000/09/xmldsig#hmac-sha1";
const char *ds_hmac_sha224URI = "http://www.w3.org/2001/04/xmldsig-more#hmac-sha224";
const char *ds_hmac_sha256URI = "http://www.w3.org/2001/04/xmldsig-more#hmac-sha256";
const char *ds_hmac_sha384URI = "http://www.w3.org/2001/04/xmldsig-more#hmac-sha384";
const char *ds_hmac_sha512URI = "http://www.w3.org/2001/04/xmldsig-more#hmac-sha512";

const char *ds_dsa_sha1URI = "http://www.w3.org/2000/09/xmldsig#dsa-sha1";
const char *ds_dsa_sha256URI = "http://www.w3.org/2009/xmldsig11#dsa-sha256";

const char *ds_rsa_sha1URI   = "http://www.w3.org/2000/09/xmldsig#rsa-sha1";
const char *ds_rsa_sha224URI = "http://www.w3.org/2001/04/xmldsig-more#rsa-sha224";
const char *ds_rsa_sha256URI = "http://www.w3.org/2001/04/xmldsig-more#rsa-sha256";
const char *ds_rsa_sha384URI = "http://www.w3.org/2001/04/xmldsig-more#rsa-sha384";
const char *ds_rsa_sha512URI = "http://www.w3.org/2001/04/xmldsig-more#rsa-sha512";

const char *ds_ecdsa_sha1URI   = "http://www.w3.org/2001/04/xmldsig-more#ecdsa-sha1";
const char *ds_ecdsa_sha224URI = "http://www.w3.org/2001/04/xmldsig-more#ecdsa-sha224";
const char *ds_ecdsa_sha256URI = "http://www.w3.org/2001/04/xmldsig-more#ecdsa-sha256";
const char *ds_ecdsa_sha384URI = "http://www.w3.org/2001/04/xmldsig-more#ecdsa-sha384";
const char *ds_ecdsa_sha512URI = "http://www.w3.org/2001/04/xmldsig-more#ecdsa-sha512";

const char *xenc_3desURI      = "http://www.w3.org/2001/04/xmlenc#tripledes-cbc";
const char *xenc_aes128cbcURI = "http://www.w3.org/2001/04/xmlenc#aes128-cbc";
const char *xenc_aes192cbcURI = "http://www.w3.org/2001/04/xmlenc#aes192-cbc";
const char *xenc_aes256cbcURI = "http://www.w3.org/2001/04/xmlenc#aes256-cbc";
const char *xenc_aes512cbcURI = "http://www.w3.org/2001/04/xmlenc#aes512-cbc";
const char *xenc_aes128gcmURI = "http://www.w3.org/2009/xmlenc11#aes128-gcm";
const char *xenc_aes192gcmURI = "http://www.w3.org/2009/xmlenc11#aes192-gcm";
const char *xenc_aes256gcmURI = "http://www.w3.org/2009/xmlenc11#aes256-gcm";
const char *xenc_aes512gcmURI = "http://www.w3.org/2009/xmlenc11#aes512-gcm";

const char *xenc_elementURI = "http://www.w3.org/2001/04/xmlenc#Element";
const char *xenc_contentURI = "http://www.w3.org/2001/04/xmlenc#Content";

const char *xenc_rsa15URI = "http://www.w3.org/2001/04/xmlenc#rsa-1_5";
const char *xenc_rsaesURI = "http://www.w3.org/2001/04/xmlenc#rsa-oaep-mgf1p";

const char *wsse_URI = SOAP_NAMESPACE_OF_wsse;
const char *wsu_URI = SOAP_NAMESPACE_OF_wsu;
const char *saml1_URI = SOAP_NAMESPACE_OF_saml1;
const char *saml2_URI = SOAP_NAMESPACE_OF_saml2;
const char *xenc_URI = "http://www.w3.org/2001/04/xmlenc#";
const char *ds_URI = "http://www.w3.org/2000/09/xmldsig#";
const char *c14n_URI = "http://www.w3.org/TR/2001/REC-xml-c14n-20010315";
const char *c14n_wc_URI = "http://www.w3.org/TR/2001/REC-xml-c14n-20010315#WithComments";
const char *exc_c14n_URI = "http://www.w3.org/2001/10/xml-exc-c14n#";
const char *exc_c14n_wc_URI = "http://www.w3.org/2001/10/xml-exc-c14n#WithComments";

/******************************************************************************\
 *
 *      Digest authentication session
 *
\******************************************************************************/

/**
@struct soap_wsse_session
@brief Digest authentication session data.
*/
struct soap_wsse_session {
  struct soap_wsse_session *next;       /**< Next session in list */
  time_t expired;                       /**< Session expiration */
  char hash[SOAP_SMD_SHA1_SIZE];        /**< SHA1 digest */
  char nonce[1]; /**< Nonce string flows into region below this struct */
};

/** The digest authentication session database */
static struct soap_wsse_session *soap_wsse_session = NULL;

/** Lock for digest authentication session database exclusive access */
static MUTEX_TYPE soap_wsse_session_lock = MUTEX_INITIALIZER;

/******************************************************************************\
 *
 *      Static protos
 *
\******************************************************************************/

static char* soap_wsse_ids(struct soap *soap, const char *tags, int sub);

static int soap_wsse_session_verify(struct soap *soap, const char hash[SOAP_SMD_SHA1_SIZE], const char *created, const char *nonce);
static void soap_wsse_session_cleanup(struct soap *soap);
static void calc_digest(struct soap *soap, const char *created, const char *nonce, int noncelen, const char *password, char hash[SOAP_SMD_SHA1_SIZE]);

static int soap_wsse_init(struct soap *soap, struct soap_wsse_data *data, const void *(*arg)(struct soap*, int*, const char*, const unsigned char*, int, int*));
static int soap_wsse_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src);
static void soap_wsse_delete(struct soap *soap, struct soap_plugin *p);

static int soap_wsse_preparesend(struct soap *soap, const char *buf, size_t len);
static int soap_wsse_preparefinalsend(struct soap *soap);
static void soap_wsse_preparecleanup(struct soap *soap, struct soap_wsse_data *data);
static int soap_wsse_preparefinalrecv(struct soap *soap);

static int soap_wsse_element_begin_in(struct soap *soap, const char *tag);
static int soap_wsse_element_end_in(struct soap *soap, const char *tag1, const char *tag2);
static int soap_wsse_element_begin_out(struct soap *soap, const char *tag, int id, const char *type);
static int soap_wsse_element_end_out(struct soap *soap, const char *tag);

static size_t soap_wsse_verify_nested(struct soap *soap, struct soap_dom_element *dom, const char *URI, const char *tag);

static int soap_p_hash(struct soap *soap, const char *hmac_key, size_t hmac_key_len, const char *secret, size_t secretlen, int alg, char HA[], size_t HA_len, char temp[], char *phash, size_t phashlen);

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
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  DBGFUN("soap_wsse_delete_Security");
  if (data)
  {
    if (soap->fpreparesend == soap_wsse_preparesend)
      soap->fpreparesend = data->fpreparesend;
    if (soap->fpreparefinalsend == soap_wsse_preparefinalsend)
      soap->fpreparefinalsend = data->fpreparefinalsend;
    data->fpreparesend = NULL;
    data->fpreparefinalsend = NULL;
    data->sigid = NULL;
    data->encid = NULL;
    if (data->prefixlist)
      SOAP_FREE(soap, data->prefixlist);
    data->prefixlist = NULL;
  }
  soap->feltbegout = NULL;
  soap->feltendout = NULL;
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
 *      wsse:Security/ds:Signature header element
 *
\******************************************************************************/

/**
@fn ds__SignatureType* soap_wsse_add_Signature(struct soap *soap)
@brief Adds Signature header element.
@param soap context
@return ds__SignatureType object
*/
SOAP_FMAC1
struct ds__SignatureType *
SOAP_FMAC2
soap_wsse_add_Signature(struct soap *soap)
{
  _wsse__Security *security = soap_wsse_add_Security(soap);
  DBGFUN("soap_wsse_add_Signature");
  /* if we don't have a ds:Signature, create one */
  if (!security->ds__Signature)
  {
    security->ds__Signature = (ds__SignatureType*)soap_malloc(soap, sizeof(ds__SignatureType));
    if (!security->ds__Signature)
      return NULL;
    soap_default_ds__SignatureType(soap, security->ds__Signature); 
  }
  return security->ds__Signature;
}

/******************************************************************************/

/**
@fn void soap_wsse_delete_Signature(struct soap *soap)
@brief Deletes Signature header element.
@param soap context
*/
SOAP_FMAC1
void
SOAP_FMAC2
soap_wsse_delete_Signature(struct soap *soap)
{
  _wsse__Security *security = soap_wsse_Security(soap);
  DBGFUN("soap_wsse_delete_Signature");
  if (security)
    security->ds__Signature = NULL;
}

/******************************************************************************/

/**
@fn ds__SignatureType* soap_wsse_Signature(struct soap *soap)
@brief Returns Signature header element if present.
@param soap context
@return ds__SignatureType object or NULL
*/
SOAP_FMAC1
struct ds__SignatureType *
SOAP_FMAC2
soap_wsse_Signature(struct soap *soap)
{
  _wsse__Security *security = soap_wsse_Security(soap);
  if (security)
    return security->ds__Signature;
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
  security->UsernameToken->Salt = NULL;
  security->UsernameToken->Iteration = NULL;
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
@fn int soap_wsse_add_UsernameTokenDigest(struct soap *soap, const char *id, const char *username, const char *password)
@brief Adds UsernameToken element for digest authentication.
@param soap context
@param[in] id string for signature referencing or NULL
@param[in] username string
@param[in] password string
@return SOAP_OK

Computes SHA1 digest of the time stamp, a nonce, and the password. The digest
provides the authentication credentials. Passwords are NOT sent in the clear.

@note
This release supports the use of at most one UsernameToken in the header.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_UsernameTokenDigest(struct soap *soap, const char *id, const char *username, const char *password)
{
  return soap_wsse_add_UsernameTokenDigest_at(soap, id, username, password, time(NULL));
}

/******************************************************************************/

/**
@fn int soap_wsse_add_UsernameTokenDigest_at(struct soap *soap, const char *id, const char *username, const char *password, time_t when)
@brief Adds UsernameToken element for digest authentication.
@param soap context
@param[in] id string for signature referencing or NULL
@param[in] username string
@param[in] password string
@param[in] when the time stamp to use for the digest hash
@return SOAP_OK

Computes SHA1 digest of the time stamp, a nonce, and the password. The digest
provides the authentication credentials. Passwords are NOT sent in the clear.

@note
This release supports the use of at most one UsernameToken in the header.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_UsernameTokenDigest_at(struct soap *soap, const char *id, const char *username, const char *password, time_t when)
{
  _wsse__Security *security = soap_wsse_add_Security(soap);
  const char *created = soap_dateTime2s(soap, when);
  char HA[SOAP_SMD_SHA1_SIZE], HABase64[29];
  char nonce[SOAP_WSSE_NONCELEN], *nonceBase64;
  DBGFUN2("soap_wsse_add_UsernameTokenDigest", "id=%s", id?id:"", "username=%s", username?username:"");
  /* generate a nonce */
  soap_wsse_rand_nonce(nonce, SOAP_WSSE_NONCELEN);
  nonceBase64 = soap_s2base64(soap, (unsigned char*)nonce, NULL, SOAP_WSSE_NONCELEN);
  /* The specs are not clear: compute digest over binary nonce or base64 nonce? */
  /* compute SHA1(created, nonce, password) */
  calc_digest(soap, created, nonce, SOAP_WSSE_NONCELEN, password, HA);
  /* Hm...?
     calc_digest(soap, created, nonceBase64, strlen(nonceBase64), password, HA);
   */
  soap_s2base64(soap, (unsigned char*)HA, HABase64, SOAP_SMD_SHA1_SIZE);
  /* populate the UsernameToken with digest */
  soap_wsse_add_UsernameTokenText(soap, id, username, HABase64);
  /* populate the remainder of the password, nonce, and created */
  security->UsernameToken->Password->Type = (char*)wsse_PasswordDigestURI;
  security->UsernameToken->Nonce = (struct wsse__EncodedString*)soap_malloc(soap, sizeof(struct wsse__EncodedString));
  security->UsernameToken->Salt = NULL;
  security->UsernameToken->Iteration = NULL;
  if (!security->UsernameToken->Nonce)
    return soap->error = SOAP_EOM;
  soap_default_wsse__EncodedString(soap, security->UsernameToken->Nonce);
  security->UsernameToken->Nonce->__item = nonceBase64;
  security->UsernameToken->Nonce->EncodingType = (char*)wsse_Base64BinaryURI;
  security->UsernameToken->wsu__Created = soap_strdup(soap, created);
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

The verification supports both clear-text password verification and digest
password authentication. For digest authentication a history mechanism with a
digest authentication session database ensures protection against replay
attacks.

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
     && !strcmp(token->Password->Type, wsse_PasswordDigestURI))
    {
      /* check password digest: compute SHA1(created, nonce, password) */
      if (token->Nonce
       && token->wsu__Created
       && strlen(token->Password->__item) == 28)        /* digest pw len = 28 */
      {
        char HA1[SOAP_SMD_SHA1_SIZE], HA2[SOAP_SMD_SHA1_SIZE];
        /* The specs are not clear: compute digest over binary nonce or base64 nonce? The formet appears to be the case: */
        int noncelen;
        const char *nonce;
        if (!token->Nonce->EncodingType || !strcmp(token->Nonce->EncodingType, wsse_Base64BinaryURI))
          nonce = soap_base642s(soap, token->Nonce->__item, NULL, 0, &noncelen);
        else if (!strcmp(token->Nonce->EncodingType, wsse_HexBinaryURI))
          nonce = soap_hex2s(soap, token->Nonce->__item, NULL, 0, &noncelen);
        else
          return soap_wsse_fault(soap, wsse__FailedAuthentication, NULL);
        /* compute HA1 = SHA1(created, nonce, password) */
        calc_digest(soap, token->wsu__Created, nonce, noncelen, password, HA1);
        /*
        calc_digest(soap, token->wsu__Created, token->Nonce->__item, strlen(token->Nonce->__item), password, HA1);
        */
        /* get HA2 = supplied digest from base64 Password */
        soap_base642s(soap, token->Password->__item, HA2, SOAP_SMD_SHA1_SIZE, NULL);
        /* compare HA1 to HA2 */
        if (!memcmp(HA1, HA2, SOAP_SMD_SHA1_SIZE))
        {
          /* authorize if HA1 and HA2 identical and not replay attack */
          if (!soap_wsse_session_verify(soap, HA1, token->wsu__Created, token->Nonce->__item))
            return SOAP_OK;
          return soap->error; 
        }
      }
    }
    else
    {
      /* check password text */
      if (token->Password->__item && !strcmp(token->Password->__item, password))
        return SOAP_OK;
    }
  }
  return soap_wsse_fault(soap, wsse__FailedAuthentication, NULL);
}

/******************************************************************************\
 *
 *      wsse:Security/BinarySecurityToken header element
 *
\******************************************************************************/

/**
@fn int soap_wsse_add_BinarySecurityToken(struct soap *soap, const char *id, const char *valueType, const unsigned char *data, int size)
@brief Adds BinarySecurityToken element.
@param soap context
@param[in] id string for signature referencing or NULL
@param[in] valueType string
@param[in] data points to binary token data
@param[in] size is length of binary token
@return SOAP_OK
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_BinarySecurityToken(struct soap *soap, const char *id, const char *valueType, const unsigned char *data, int size)
{
  _wsse__Security *security = soap_wsse_add_Security(soap);
  DBGFUN2("wsse_add_BinarySecurityToken", "id=%s", id?id:"", "valueType=%s", valueType?valueType:"");
  /* allocate BinarySecurityToken if we don't already have one */
  if (!security->BinarySecurityToken)
  {
    security->BinarySecurityToken = (_wsse__BinarySecurityToken*)soap_malloc(soap, sizeof(_wsse__BinarySecurityToken));
    if (!security->BinarySecurityToken)
      return soap->error = SOAP_EOM;
  }
  soap_default__wsse__BinarySecurityToken(soap, security->BinarySecurityToken);
  /* populate the BinarySecurityToken */
  security->BinarySecurityToken->wsu__Id = soap_strdup(soap, id);
  security->BinarySecurityToken->ValueType = soap_strdup(soap, valueType);
  security->BinarySecurityToken->EncodingType = (char*)wsse_Base64BinaryURI;
  security->BinarySecurityToken->__item = soap_s2base64(soap, data, NULL, size);
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_add_BinarySecurityTokenX509(struct soap *soap, const char *id, X509 *cert)
@brief Adds BinarySecurityToken element with X509 certificate.
@param soap context
@param[in] id string for signature reference
@param[in] cert points to the X509 certificate
@return SOAP_OK or SOAP_EOM

This function uses i2d_X509 from the the OpenSSL library to convert an X509
object to binary DER format.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_BinarySecurityTokenX509(struct soap *soap, const char *id, X509 *cert)
{
  int derlen;
  unsigned char *der, *s;
  if (!cert)
    return soap_wsse_fault(soap, wsse__InvalidSecurityToken, "Missing certificate");
  /* determine the storage requirement */
  derlen = i2d_X509(cert, NULL);
  if (derlen < 0)
    return soap_wsse_fault(soap, wsse__InvalidSecurityToken, "Invalid certificate");
  /* use the gSOAP engine's look-aside buffer to temporarily hold the cert */
  if (soap_store_lab(soap, NULL, derlen))
    return SOAP_EOM;
  s = der = (unsigned char*)soap->labbuf;
  /* store in DER format */
  i2d_X509(cert, &s);
  /* populate the BinarySecurityToken with base64 certificate data */
  return soap_wsse_add_BinarySecurityToken(soap, id, wsse_X509v3URI, der, derlen);
}

/******************************************************************************/

/**
@fn int soap_wsse_add_BinarySecurityTokenPEM(struct soap *soap, const char *id, const char *filename)
@brief Adds BinarySecurityToken element from a PEM file.
@param soap context
@param[in] id string for signature reference
@param[in] filename
@return SOAP_OK or SOAP_FAULT with wsse__InvalidSecurity fault when file cannot be read or does not contain a valid certificate

This function uses PEM_read_X509 from the the OpenSSL library to read a
certificate from a PEM formatted file.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_BinarySecurityTokenPEM(struct soap *soap, const char *id, const char *filename)
{
  FILE *fd;
  DBGFUN2("soap_wsse_add_BinarySecurityTokenPEM", "id=%s", id?id:"", "filename=%s", filename?filename:"");
  fd = fopen(filename, "r");
  if (fd)
  {
    /* read the certificate */
    X509 *cert = PEM_read_X509(fd, NULL, NULL, NULL);
    fclose(fd);
    /* if okay, populate the BinarySecurityToken element */
    if (cert)
    {
      int err = soap_wsse_add_BinarySecurityTokenX509(soap, id, cert);
      X509_free(cert);
      return err;
    }
  }
  return soap_wsse_fault(soap, wsse__InvalidSecurityToken, "Missing certificate");
}

/******************************************************************************/

/**
@fn _wsse__BinarySecurityToken* soap_wsse_BinarySecurityToken(struct soap *soap, const char *id)
@brief Returns BinarySecurityToken element if present.
@param soap context
@param[in] id string of token to get or NULL to get the first
@return _wsse__BinarySecurityToken object or NULL
*/
SOAP_FMAC1
struct _wsse__BinarySecurityToken *
SOAP_FMAC2
soap_wsse_BinarySecurityToken(struct soap *soap, const char *id)
{
  _wsse__Security *security = soap_wsse_Security(soap);
  if (security
   && security->BinarySecurityToken
   && (!id || (security->BinarySecurityToken->wsu__Id
            && !strcmp(security->BinarySecurityToken->wsu__Id, id))))
    return security->BinarySecurityToken;
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "No BinarySecurityToken id=%s, pointer=%p to token found\n", id, security->BinarySecurityToken));
  return NULL;
}

/******************************************************************************/

/**
@fn int soap_wsse_get_BinarySecurityToken(struct soap *soap, const char *id, char **valueType, unsigned char **data, int *size)
@brief Get wsse:BinarySecurityToken element token data in binary form.
@param soap context
@param[in] id string of token to get or NULL to get the first
@param[out] valueType string
@param[out] data points to binary token data
@param[out] size is length of binary token
@return SOAP_OK or SOAP_FAULT with wsse:SecurityTokenUnavailable fault
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_get_BinarySecurityToken(struct soap *soap, const char *id, char **valueType, unsigned char **data, int *size)
{
  _wsse__BinarySecurityToken *token = soap_wsse_BinarySecurityToken(soap, id);
  DBGFUN1("soap_wsse_get_BinarySecurityToken", "id=%s", id?id:"");
  *data = NULL;
  if (token)
  {
    *valueType = token->ValueType;
    if (!token->EncodingType || !strcmp(token->EncodingType, wsse_Base64BinaryURI))
      *data = (unsigned char*)soap_base642s(soap, token->__item, NULL, 0, size);
    else if (!strcmp(token->EncodingType, wsse_HexBinaryURI))
      *data = (unsigned char*)soap_hex2s(soap, token->__item, NULL, 0, size);
    if (*data)
      return SOAP_OK;
  }
  return soap_wsse_fault(soap, wsse__SecurityTokenUnavailable, "BinarySecurityToken required");
}

/******************************************************************************/

/**
@fn X509* soap_wsse_get_BinarySecurityTokenX509(struct soap *soap, const char *id)
@brief Get X509 wsse:BinarySecurityToken certificate and verify its content. This call must be followed by an X509_free to deallocate the X509 certificate data.
@param soap context
@param[in] id string of token to get or NULL to get the first
@return X509 certificate (dynamically allocated) or NULL with wsse:SecurityTokenUnavailable fault
*/
SOAP_FMAC1
X509*
SOAP_FMAC2
soap_wsse_get_BinarySecurityTokenX509(struct soap *soap, const char *id)
{
  X509 *cert = NULL;
  char *valueType = NULL;
#if (OPENSSL_VERSION_NUMBER >= 0x0090800fL)
  const unsigned char *data = NULL;
#else
  unsigned char *data = NULL;
#endif
  int size;
  DBGFUN1("soap_wsse_get_BinarySecurityTokenX509", "id=%s", id?id:"");
  if (!soap_wsse_get_BinarySecurityToken(soap, id, &valueType, (unsigned char**)&data, &size)
   && valueType
   && !strcmp(valueType, wsse_X509v3URI))
    cert = d2i_X509(NULL, &data, size);
  /* verify the certificate */
  if (cert && soap_wsse_verify_X509(soap, cert))
  {
    X509_free(cert);
    cert = NULL;
  }
  return cert;
}

/******************************************************************************/

/**
@fn int soap_wsse_verify_X509(struct soap *soap, X509 *cert)
@brief Verifies X509 certificate against soap->cafile, soap->capath, and soap->crlfile.
@param soap context
@param[in] cert X509 certificate
@return SOAP_OK or fault

This is an expensive operation. Whenever a new soap context is created, the
cafile and objects are loaded into that context each time we need to verify a
certificate.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_verify_X509(struct soap *soap, X509 *cert)
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  X509_STORE_CTX *verify;
  DBGFUN("soap_wsse_verify_X509");
  if (!data)
    return soap_set_receiver_error(soap, "soap_wsse_verify_X509", "Plugin not registered", SOAP_PLUGIN_ERROR);
  if (!cert)
    return soap_wsse_sender_fault(soap, "soap_wsse_verify_X509", "No certificate");
  if (!data->store)
  {
    data->store = X509_STORE_new();
    if (!data->store)
      return soap_wsse_receiver_fault(soap, "soap_wsse_verify_X509", "Could not create X509_STORE object");
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Setting up a new X509 store\n"));
    X509_STORE_set_verify_cb_func(data->store, soap->fsslverify);
    if (soap->cafile || soap->capath)
    {
      if (X509_STORE_load_locations(data->store, soap->cafile, soap->capath) != 1)
        return soap_wsse_receiver_fault(soap, "soap_wsse_verify_X509", "Could not load CA PEM file or path");
    }
#if (OPENSSL_VERSION_NUMBER > 0x00907000L)
    if (soap->crlfile)
    {
      if (*soap->crlfile)
      {
        X509_LOOKUP *lookup;
        lookup = X509_STORE_add_lookup(data->store, X509_LOOKUP_file());
        if (!lookup)
          return soap_wsse_receiver_fault(soap, "soap_wsse_verify_X509", "Could not create X509_LOOKUP object");
        if (X509_load_crl_file(lookup, soap->crlfile, X509_FILETYPE_PEM) != 1)
          return soap_wsse_receiver_fault(soap, "soap_wsse_verify_X509", "Could not read CRL PEM file");
      }
      X509_STORE_set_flags(data->store, X509_V_FLAG_CRL_CHECK | X509_V_FLAG_CRL_CHECK_ALL);
    }
#endif
  }
  verify = X509_STORE_CTX_new();
  if (!verify)
    return soap_wsse_receiver_fault(soap, "soap_wsse_verify_X509", "Could not create X509_STORE_CTX object");
#if (OPENSSL_VERSION_NUMBER > 0x00907000L)
  if (X509_STORE_CTX_init(verify, data->store, cert, NULL) != 1)
  {
    X509_STORE_CTX_free(verify);
    return soap_wsse_receiver_fault(soap, "soap_wsse_verify_X509", "Could not initialize X509_STORE_CTX object");
  }
#else
  X509_STORE_CTX_init(verify, data->store, cert, NULL);
#endif
#ifdef SOAP_DEBUG
  {
    char buf[1024];
    X509_NAME_oneline(X509_get_issuer_name(cert), buf, sizeof(buf)-1);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Certificate issuer %s\n", buf));
    X509_NAME_oneline(X509_get_subject_name(cert), buf, sizeof(buf)-1);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Certificate subject %s\n", buf));
  }
#endif
  if (X509_verify_cert(verify) != 1)
  {
    X509_STORE_CTX_free(verify);
    return soap_wsse_sender_fault(soap, "soap_wsse_verify_X509", "Invalid certificate");
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Certificate is valid\n"));
  X509_STORE_CTX_free(verify);
  return SOAP_OK;
}

/******************************************************************************\
 *
 *      wsc:SecurityContextToken
 *
\******************************************************************************/

/**
@fn int soap_wsse_add_SecurityContextToken(struct soap *soap, const char *id, const char *identifier)
@brief Adds wsc:SecurityContextToken/Identifier and SecurityTokenReference to it.
@param soap context
@param[in] id string for signature reference (required)
@param[in] identifier wsc:Identifier value (required)
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_SecurityContextToken(struct soap *soap, const char *id, const char *identifier)
{
  char *URI = NULL;
  size_t l;
  _wsse__Security *security = soap_wsse_add_Security(soap);
  DBGFUN2("soap_wsse_add_SecurityContextToken", "id=%s", id, "identifier=%s", identifier?identifier:"");
  /* allocate wsc:SecurityContextToken if we don't already have one */
  if (!security->wsc__SecurityContextToken)
  {
    security->wsc__SecurityContextToken = (struct wsc__SecurityContextTokenType*)soap_malloc(soap, sizeof(struct wsc__SecurityContextTokenType));
    if (!security->wsc__SecurityContextToken)
      return soap->error = SOAP_EOM;
  }
  soap_default_wsc__SecurityContextTokenType(soap, security->wsc__SecurityContextToken);
  /* populate the wsc:SecurityContextToken */
  l = strlen(id);
  URI = (char*)soap_malloc(soap, l + 2);
  if (!URI)
    return soap->error = SOAP_EOM;
  *URI = '#';
  soap_strcpy(URI + 1, l + 1, id);
  security->wsc__SecurityContextToken->wsu__Id = URI + 1;
  security->wsc__SecurityContextToken->Identifier = soap_strdup(soap, identifier);
  /* set SecurityTokenReference */
  return soap_wsse_add_KeyInfo_SecurityTokenReferenceURI(soap, URI, NULL);
}

/******************************************************************************/

/**
@fn const char *soap_wsse_get_SecurityContextToken(struct soap *soap)
@brief Returns wsc:SecurityContextToken/Identifier string value or NULL.
@param soap context
@return wsc:SecurityContextToken/Identifier string value value or NULL
*/
SOAP_FMAC1
const char *
SOAP_FMAC2
soap_wsse_get_SecurityContextToken(struct soap *soap)
{
  _wsse__Security *security = soap_wsse_Security(soap);
  DBGFUN("soap_wsse_SecurityContextToken");
  if (security->wsc__SecurityContextToken && security->wsc__SecurityContextToken->wsu__Id)
  {
    const char *URI = soap_wsse_get_KeyInfo_SecurityTokenReferenceURI(soap);
    if (URI && !strcmp(URI, security->wsc__SecurityContextToken->wsu__Id))
      return security->wsc__SecurityContextToken->Identifier;
  }
  return NULL;
}

/******************************************************************************\
 *
 *      ds:Signature/SignedInfo
 *
\******************************************************************************/

/**
@fn ds__SignedInfoType* soap_wsse_add_SignedInfo(struct soap *soap)
@brief Adds SignedInfo element.
@param soap context
@return ds__SignedInfoType object
*/
SOAP_FMAC1
struct ds__SignedInfoType *
SOAP_FMAC2
soap_wsse_add_SignedInfo(struct soap *soap)
{
  ds__SignatureType *signature = soap_wsse_add_Signature(soap);
  if (!signature->SignedInfo)
  {
    signature->SignedInfo = (ds__SignedInfoType*)soap_malloc(soap, sizeof(ds__SignedInfoType));
    if (!signature->SignedInfo)
      return NULL;
    soap_default_ds__SignedInfoType(soap, signature->SignedInfo);
  }
  return signature->SignedInfo;
}

/******************************************************************************/

/**
@fn int soap_wsse_add_SignedInfo_Reference(struct soap *soap, const char *URI, unsigned int level, const char *transform, const char *prefixlist, int alg, const char *HA)
@brief Adds SignedInfo element with Reference URI, transform algorithm used, and digest value.
@param soap context
@param[in] URI reference
@param[in] level XML depth of element signed
@param[in] transform string should be exc_c14n_URI for exc-c14n, or c14n_URI or NULL for default (no canonicalization)
@param[in] prefixlist used by the exc-c14n transform or NULL
@param[in] alg is the digest algorithm used
@param[in] HA is the digest in binary form
@return SOAP_OK or SOAP_EOM when references exceed SOAP_WSSE_MAX_REF

This function can be called to add more references to the wsse:SignedInfo
element. A maximum number of SOAP_WSSE_MAX_REF references can be added. The
digest method is always SHA1. 

@note
XPath transforms cannot be specified in this release.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_SignedInfo_Reference(struct soap *soap, const char *URI, unsigned int level, const char *transform, const char *prefixlist, int alg, const char *HA)
{
  ds__SignedInfoType *signedInfo = soap_wsse_add_SignedInfo(soap);
  ds__ReferenceType *reference;
  DBGFUN3("soap_wsse_add_SignedInfo_Reference", "URI=%s", URI ? URI : "(null)", "transform=%s", transform ? transform : "(null)", "alg=%x", alg);
  /* if this is the first reference, allocate SOAP_WSSE_MAX_REF references */
  if (signedInfo->__sizeReference == 0)
  {
    signedInfo->Reference = (ds__ReferenceType**)soap_malloc(soap, SOAP_WSSE_MAX_REF*sizeof(ds__ReferenceType*));
  }
  else
  {
    /* maximum number of references exceeded? */
    if (signedInfo->__sizeReference >= SOAP_WSSE_MAX_REF)
      return soap->error = SOAP_EOM;
  }
  /* allocate fresh new reference */
  reference = (ds__ReferenceType*)soap_malloc(soap, sizeof(ds__ReferenceType));
  if (!reference)
    return soap->error = SOAP_EOM;
  soap_default_ds__ReferenceType(soap, reference);
  /* populate the URI */
  reference->URI = soap_strdup(soap, URI);
  /* if a transform algorithm was used, populate the Transforms element */
  if (transform)
  {
    reference->Transforms = (ds__TransformsType*)soap_malloc(soap, sizeof(ds__TransformsType));
    if (!reference->Transforms)
      return soap->error = SOAP_EOM;
    soap_default_ds__TransformsType(soap, reference->Transforms);
    /* only one transform, unless level == 0 to create enveloped signatures */
    if (level <= 1)
    {
      reference->Transforms->__sizeTransform = 2;
      reference->Transforms->Transform = (ds__TransformType*)soap_malloc(soap, 2 * sizeof(ds__TransformType));
      if (reference->Transforms->Transform)
      {
        soap_default_ds__TransformType(soap, &reference->Transforms->Transform[1]);
        reference->Transforms->Transform[1].Algorithm = (char*)ds_envsigURI;
      }
    }
    else
    {
      reference->Transforms->__sizeTransform = 1;
      reference->Transforms->Transform = (ds__TransformType*)soap_malloc(soap, sizeof(ds__TransformType));
    }
    if (!reference->Transforms->Transform)
      return soap->error = SOAP_EOM;
    soap_default_ds__TransformType(soap, reference->Transforms->Transform);
    reference->Transforms->Transform->Algorithm = (char*)transform;
    /* populate the c14n:InclusiveNamespaces element */
    if (prefixlist && *prefixlist && *prefixlist != '*')
    {
      reference->Transforms->Transform->c14n__InclusiveNamespaces = (_c14n__InclusiveNamespaces*)soap_malloc(soap, sizeof(_c14n__InclusiveNamespaces));
      if (!reference->Transforms->Transform->c14n__InclusiveNamespaces)
        return soap->error = SOAP_EOM;
      soap_default__c14n__InclusiveNamespaces(soap, reference->Transforms->Transform->c14n__InclusiveNamespaces);
      if (*prefixlist == '+')
      {
        struct Namespace *ns = soap->local_namespaces;
        size_t n = 0;
        char *p;
        for (ns = soap->local_namespaces; ns && ns->id; ns++)
          n += strlen(ns->id) + 1;
        reference->Transforms->Transform->c14n__InclusiveNamespaces->PrefixList = p = (char*)soap_malloc(soap, n);
        if (p)
        {
          for (ns = soap->local_namespaces; ns && ns->id; )
          {
            strcpy(p, ns->id);
            p += strlen(p);
            ns++;
            if (ns->id)
              *p++ = ' ';
          }
          *p = '\0';
        }
      }
      else
      {
        reference->Transforms->Transform->c14n__InclusiveNamespaces->PrefixList = (char*)prefixlist;
      }
    }
  }
  /* populate the DigestMethod element */
  reference->DigestMethod = (ds__DigestMethodType*)soap_malloc(soap, sizeof(ds__DigestMethodType));
  if (!reference->DigestMethod)
    return soap->error = SOAP_EOM;
  soap_default_ds__DigestMethodType(soap, reference->DigestMethod);
  /* the DigestMethod algorithm SHA1, SHA256, SHA512  */
  switch (alg & SOAP_SMD_HASH)
  {
    case SOAP_SMD_SHA1:
      reference->DigestMethod->Algorithm = (char*)ds_sha1URI;
      break;
#if (OPENSSL_VERSION_NUMBER >= 0x0090800fL)
    case SOAP_SMD_SHA224:
      reference->DigestMethod->Algorithm = (char*)ds_sha224URI;
      break;
    case SOAP_SMD_SHA256:
      reference->DigestMethod->Algorithm = (char*)ds_sha256URI;
      break;
    case SOAP_SMD_SHA384:
      reference->DigestMethod->Algorithm = (char*)ds_sha384URI;
      break;
    case SOAP_SMD_SHA512:
      reference->DigestMethod->Algorithm = (char*)ds_sha512URI;
      break;
#endif
    default:
      return soap_wsse_fault(soap, wsse__UnsupportedAlgorithm, "SHA224/256/384/512 requires OpenSSL 0.9.8 or greater");
  }
  /* populate the DigestValue element */
  reference->DigestValue = soap_s2base64(soap, (unsigned char*)HA, NULL, soap_smd_size(alg, NULL));
  if (!reference->DigestValue)
    return soap->error;
  /* add the fresh new reference to the array */
  signedInfo->Reference[signedInfo->__sizeReference] = reference;
  signedInfo->__sizeReference++;
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_add_SignedInfo_SignatureMethod(struct soap *soap, const char *method, int canonical)
@brief Adds SignedInfo element with SignatureMethod.
@param soap context
@param[in] method is the URI of the signature algorithm (e.g. ds_rsa_sha1)
@param[in] canonical flag indicating that SignedInfo is signed in exc-c14n form or in c14n when soap->c14ninclude == "*" (i.e. all prefixes are inclusive)
@return SOAP_OK
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_SignedInfo_SignatureMethod(struct soap *soap, const char *method, int canonical)
{
  ds__SignedInfoType *signedInfo = soap_wsse_add_SignedInfo(soap);
  DBGFUN2("soap_wsse_add_SignedInfo_SignatureMethod", "method=%s", method?method:"", "canonical=%d", canonical);
  /* if signed in exc-c14n form, populate CanonicalizationMethod element */
  if (canonical)
  {
    signedInfo->CanonicalizationMethod = (ds__CanonicalizationMethodType*)soap_malloc(soap, sizeof(ds__CanonicalizationMethodType));
    if (!signedInfo->CanonicalizationMethod)
      return soap->error = SOAP_EOM;
    soap_default_ds__CanonicalizationMethodType(soap, signedInfo->CanonicalizationMethod);
    if (soap->c14ninclude && *soap->c14ninclude == '*')
      signedInfo->CanonicalizationMethod->Algorithm = (char*)c14n_URI;
    else
      signedInfo->CanonicalizationMethod->Algorithm = (char*)exc_c14n_URI;
  }
  /* populate SignatureMethod element */
  signedInfo->SignatureMethod = (ds__SignatureMethodType*)soap_malloc(soap, sizeof(ds__SignatureMethodType));
  if (!signedInfo->SignatureMethod)
    return soap->error = SOAP_EOM;
  soap_default_ds__SignatureMethodType(soap, signedInfo->SignatureMethod);
  signedInfo->SignatureMethod->Algorithm = (char*)method;
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn ds__SignedInfoType* soap_wsse_SignedInfo(struct soap *soap)
@brief Returns SignedInfo element if present.
@param soap context
@return ds__SignedInfoType object or NULL
*/
SOAP_FMAC1
struct ds__SignedInfoType *
SOAP_FMAC2
soap_wsse_SignedInfo(struct soap *soap)
{
  ds__SignatureType *signature = soap_wsse_Signature(soap);
  if (signature)
    return signature->SignedInfo;
  return NULL;
}

/******************************************************************************/

/**
@fn int soap_wsse_get_SignedInfo_SignatureMethod(struct soap *soap, int *alg, int *bits)
@brief Get SignatureMethod algorithm.
@param soap context
@param[out] alg is a signature algorithm, such as SOAP_SMD_HMAC_SHA1/224/256/384/512, SOAP_SMD_VRFY_DSA_SHA1/256, SOAP_SMD_VRFY_RSA_SHA1/224/256/384/512 or SOAP_SMD_ECDSA_SHA1/224/256/384/512
@param[out] bits is set to HMACOutputLength if present and valid, 0 otherwise
@return SOAP_OK or SOAP_FAULT with wsse:UnsupportedAlgorithm, wsse:FailedCheck, or wsse__InvalidSecurity fault
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_get_SignedInfo_SignatureMethod(struct soap *soap, int *alg, int *bits)
{
  ds__SignedInfoType *signedInfo = soap_wsse_SignedInfo(soap);
  DBGFUN("soap_wsse_get_SignedInfo_SignatureMethod");
  *alg = SOAP_SMD_NONE;
  *bits = 0;
  /* if we have a SignedInfo element, get the algorithm */
  if (signedInfo
   && signedInfo->SignatureMethod
   && signedInfo->SignatureMethod->Algorithm)
  {
    const char *method = signedInfo->SignatureMethod->Algorithm;
    if (!strcmp(method, ds_hmac_sha1URI))
      *alg = SOAP_SMD_HMAC_SHA1;
    else if (!strcmp(method, ds_hmac_sha224URI))
      *alg = SOAP_SMD_HMAC_SHA224;
    else if (!strcmp(method, ds_hmac_sha256URI))
      *alg = SOAP_SMD_HMAC_SHA256;
    else if (!strcmp(method, ds_hmac_sha384URI))
      *alg = SOAP_SMD_HMAC_SHA384;
    else if (!strcmp(method, ds_hmac_sha512URI))
      *alg = SOAP_SMD_HMAC_SHA512;
    else if (!strcmp(method, ds_dsa_sha1URI))
      *alg = SOAP_SMD_VRFY_DSA_SHA1;
    else if (!strcmp(method, ds_dsa_sha256URI))
      *alg = SOAP_SMD_VRFY_DSA_SHA256;
    else if (!strcmp(method, ds_rsa_sha1URI))
      *alg = SOAP_SMD_VRFY_RSA_SHA1;
    else if (!strcmp(method, ds_rsa_sha224URI))
      *alg = SOAP_SMD_VRFY_RSA_SHA224;
    else if (!strcmp(method, ds_rsa_sha256URI))
      *alg = SOAP_SMD_VRFY_RSA_SHA256;
    else if (!strcmp(method, ds_rsa_sha384URI))
      *alg = SOAP_SMD_VRFY_RSA_SHA384;
    else if (!strcmp(method, ds_rsa_sha512URI))
      *alg = SOAP_SMD_VRFY_RSA_SHA512;
    else if (!strcmp(method, ds_ecdsa_sha1URI))
      *alg = SOAP_SMD_VRFY_ECDSA_SHA1;
    else if (!strcmp(method, ds_ecdsa_sha224URI))
      *alg = SOAP_SMD_VRFY_ECDSA_SHA224;
    else if (!strcmp(method, ds_ecdsa_sha256URI))
      *alg = SOAP_SMD_VRFY_ECDSA_SHA256;
    else if (!strcmp(method, ds_ecdsa_sha384URI))
      *alg = SOAP_SMD_VRFY_ECDSA_SHA384;
    else if (!strcmp(method, ds_ecdsa_sha512URI))
      *alg = SOAP_SMD_VRFY_ECDSA_SHA512;
    else
      return soap_wsse_fault(soap, wsse__UnsupportedAlgorithm, method);
    if (signedInfo->SignatureMethod->HMACOutputLength)
    {
      if ((*alg & SOAP_SMD_ALGO) != SOAP_SMD_HMAC)
        return soap_wsse_fault(soap, wsse__InvalidSecurity, "Invalid SignatureMethod HMACOutputLength");
      *bits = *signedInfo->SignatureMethod->HMACOutputLength;
      if (*bits < (int)(4 * soap_smd_size(*alg, NULL)) || *bits > (int)(8 * soap_smd_size(*alg, NULL)))
        return soap_wsse_fault(soap, wsse__InvalidSecurity, "Invalid SignatureMethod HMACOutputLength");
    }
    return SOAP_OK;
  }
  return soap_wsse_fault(soap, wsse__FailedCheck, "Signature with SignedInfo SignatureMethod required");
}

/******************************************************************************\
 *
 *      ds:Signature/SignatureValue
 *
\******************************************************************************/

/**
@fn int soap_wsse_add_SignatureValue(struct soap *soap, int alg, const void *key, int keylen)
@brief Adds SignedInfo/SignatureMethod element, signs the SignedInfo element, and adds the resulting SignatureValue element.
@param soap context
@param[in] alg is SOAP_SMD_HMAC_SHA1/224/256/384/512, SOAP_SMD_SIGN_DSA_SHA1/256, SOAP_SMD_SIGN_RSA_SHA1/224/256/384/512, or SOAP_SMD_SIGN_ECDSA_SHA1/224/256/384/512
@param[in] key to use to sign (HMAC or DSA/RSA/ECDSA EVP_PKEY)
@param[in] keylen length of HMAC key
@return SOAP_OK, SOAP_EOM, or fault

To sign the SignedInfo element with this function, populate SignedInfo with
Reference elements first using soap_wsse_add_SignedInfo_Reference. The
SignedInfo element must not be modified after signing.

The SOAP_XML_INDENT and SOAP_XML_CANONICAL flags are used to serialize the SignedInfo to compute the signature.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_SignatureValue(struct soap *soap, int alg, const void *key, int keylen)
{
  ds__SignatureType *signature = soap_wsse_add_Signature(soap);
  const char *method = NULL;
  char *sig = NULL;
  int siglen;
  int err;
  const char *c14ninclude = soap->c14ninclude;
  DBGFUN1("soap_wsse_add_SignatureValue", "alg=%x", alg);
  /* determine signature algorithm to use */
  switch (alg)
  {
    case SOAP_SMD_HMAC_SHA1:
      method = ds_hmac_sha1URI;
      break;
#if (OPENSSL_VERSION_NUMBER >= 0x0090800fL)
    case SOAP_SMD_HMAC_SHA224:
      method = ds_hmac_sha224URI;
      break;
    case SOAP_SMD_HMAC_SHA256:
      method = ds_hmac_sha256URI;
      break;
    case SOAP_SMD_HMAC_SHA384:
      method = ds_hmac_sha384URI;
      break;
    case SOAP_SMD_HMAC_SHA512:
      method = ds_hmac_sha512URI;
      break;
#endif
    case SOAP_SMD_SIGN_DSA_SHA1:
      method = ds_dsa_sha1URI;
      break;
#if (OPENSSL_VERSION_NUMBER >= 0x0090800fL)
    case SOAP_SMD_SIGN_DSA_SHA256:
      method = ds_dsa_sha256URI;
      break;
#endif
    case SOAP_SMD_SIGN_RSA_SHA1:
      method = ds_rsa_sha1URI;
      break;
#if (OPENSSL_VERSION_NUMBER >= 0x0090800fL)
    case SOAP_SMD_SIGN_RSA_SHA224:
      method = ds_rsa_sha224URI;
      break;
    case SOAP_SMD_SIGN_RSA_SHA256:
      method = ds_rsa_sha256URI;
      break;
    case SOAP_SMD_SIGN_RSA_SHA384:
      method = ds_rsa_sha512URI;
      break;
    case SOAP_SMD_SIGN_RSA_SHA512:
      method = ds_rsa_sha512URI;
      break;
#endif
    case SOAP_SMD_SIGN_ECDSA_SHA1:
      method = ds_ecdsa_sha1URI;
      break;
#if (OPENSSL_VERSION_NUMBER >= 0x0090800fL)
    case SOAP_SMD_SIGN_ECDSA_SHA224:
      method = ds_ecdsa_sha224URI;
      break;
    case SOAP_SMD_SIGN_ECDSA_SHA256:
      method = ds_ecdsa_sha256URI;
      break;
    case SOAP_SMD_SIGN_ECDSA_SHA384:
      method = ds_ecdsa_sha512URI;
      break;
    case SOAP_SMD_SIGN_ECDSA_SHA512:
      method = ds_ecdsa_sha512URI;
      break;
#endif
    default:
      return soap_wsse_fault(soap, wsse__UnsupportedAlgorithm, "Unsupported algorithm (requires OpenSSL 0.9.8 or greater)");
  }
  /* populate SignedInfo/SignatureMethod based on SOAP_XML_CANONICAL flag */
  soap_wsse_add_SignedInfo_SignatureMethod(soap, method, (soap->mode & SOAP_XML_CANONICAL));
  /* use the gSOAP engine's look-aside buffer to temporarily hold the sig */
  if (soap_store_lab(soap, NULL, soap_smd_size(alg, key)))
    return soap->error = SOAP_EOM;
  sig = soap->labbuf;
  /* we will serialize SignedInfo as it appears exactly in the SOAP Header */
  soap->part = SOAP_IN_HEADER;
  /* set indent level for XML SignedInfo as it appears in the SOAP Header */
  soap->level = 4;
  /* prevent xmlns:ds namespace inclusion when non-exclusive is used */
  if (!(soap->mode & SOAP_XML_CANONICAL))
    soap_push_namespace(soap, "ds", ds_URI);
  soap->c14ninclude = NULL;
  /* use smdevp engine to sign SignedInfo */
  err = soap_smd_begin(soap, alg, key, keylen);
  if (!err)
    err = soap_out_ds__SignedInfoType(soap, "ds:SignedInfo", 0, signature->SignedInfo, NULL);
  soap->c14ninclude = c14ninclude;
  if (soap_smd_end(soap, sig, &siglen) || err)
    return soap_wsse_fault(soap, wsse__InvalidSecurity, "Could not sign");
  /* populate the SignatureValue element */
  signature->SignatureValue = soap_s2base64(soap, (unsigned char*)sig, NULL, siglen);
  if (!signature->SignatureValue)
    return soap->error;
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_verify_Signature(struct soap *soap)
@brief Verifies the Signature and the signed elements.
@param soap context
@return SOAP_OK, SOAP_EOM, or fault
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_verify_Signature(struct soap *soap)
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  ds__SignedInfoType *signedInfo = soap_wsse_SignedInfo(soap);
  int err = SOAP_OK, alg, bits, keylen = 0;
  EVP_PKEY *pkey = NULL;
  const void *key = NULL;
  DBGFUN("soap_wsse_verify_Signature");
  if (!signedInfo)
    return soap_wsse_fault(soap, wsse__FailedCheck, "Signature with SignedInfo required");
  if (!data)
    return soap->error = SOAP_PLUGIN_ERROR;
  /* determine which signature algorithm was used */
  if (soap_wsse_get_SignedInfo_SignatureMethod(soap, &alg, &bits))
    return soap->error;
  /* for HMAC-SHA1, the secret key might be stored in the KeyIdentifier */
  if ((alg & SOAP_SMD_ALGO) == SOAP_SMD_HMAC)
  {
    const char *valueType = soap_wsse_get_KeyInfo_SecurityTokenReferenceKeyIdentifierValueType(soap, soap_wsse_KeyInfo(soap));
    /* if in the KeyIdentifier, retrieve it */
    if (valueType && !strcmp(valueType, ds_hmac_sha1URI))
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Using HMAC key from KeyIdentifier to verify signature\n"));
      key = (void*)soap_wsse_get_KeyInfo_SecurityTokenReferenceKeyIdentifier(soap, soap_wsse_KeyInfo(soap), &keylen);
    }
    /* next, try the plugin's security token handler */
    if (!key)
    {
      if (data->security_token_handler)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Getting HMAC key through security_token_handler callback\n"));
        key = data->security_token_handler(soap, &alg, NULL, NULL, 0, &keylen);
      }
    }
    /* still no key: try to get it from the plugin */
    if (!key && alg == (data->vrfy_alg & SOAP_SMD_MASK))
    {
      /* get the HMAC secret key from the plugin */
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Using HMAC key from plugin to verify signature\n"));
      key = data->vrfy_key;
      keylen = data->vrfy_keylen;
    }
  }
  else
  {
    /* get the certificate from the KeyInfo reference */
    X509 *cert, *cert1;
    cert = cert1 = soap_wsse_get_KeyInfo_SecurityTokenReferenceX509(soap);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Got cert=%p\n", cert));
    /* next, try the plugin's security token handler */
    if (!cert)
    {
      /* no cert, get it through the token handler */
      if (data->security_token_handler)
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Getting certificate through security_token_handler callback\n"));
        cert = (X509*)data->security_token_handler(soap, &alg, NULL, NULL, 0, &keylen);
      }
    }
    /* obtain the public key from the cert */
    if (cert)
    {
      pkey = X509_get_pubkey((X509*)cert);
      key = (void*)pkey;
      soap->error = SOAP_OK;
    }
    else if (alg == (data->vrfy_alg & SOAP_SMD_MASK))
    {
      /* get the public key from the plugin */
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Using public key from plugin to verify signature\n"));
      key = data->vrfy_key;
      soap->error = SOAP_OK;
    }
    if (cert1)
      X509_free(cert1);
  }
  /* if still no key then fault else verify SignedInfo with signature and check digests of local elements */
  if (!key)
    err = soap_wsse_fault(soap, wsse__SecurityTokenUnavailable, NULL);
  else if (soap_wsse_verify_SignatureValue(soap, alg, key, keylen)
        || soap_wsse_verify_SignedInfo(soap))
    err = soap->error;
  if (pkey)
    EVP_PKEY_free(pkey);
  return err;
}

/******************************************************************************/

/**
@fn int soap_wsse_verify_SignatureValue(struct soap *soap, int alg, const void *key, int keylen)
@brief Verifies the SignatureValue of a SignedInfo element.
@param soap context
@param[in] alg is a signature algorith, such as SOAP_SMD_HMAC_SHA1/224/256/384/512, SOAP_SMD_VRFY_DSA_SHA1/256, SOAP_SMD_VRFY_RSA_SHA1/224/256/384/512 or SOAP_SMD_VRFY_ECDSA_SHA1/224/256/384/512 determined by the SignedInfo/SignatureMethod
@param[in] key to use to verify (HMAC or DSA/RSA/ECDSA EVP_PKEY)
@param[in] keylen length of HMAC key
@return SOAP_OK, SOAP_EOM, or fault

This function searches for the SignedInfo element in the soap->dom DOM tree to
verify the signature in the SignatureValue element. Using the DOM ensures we
will verify the signature of a SignedInfo as it was exactly received by the
parser, by using the -DWITH_DOM compile flag and SOAP_XML_DOM runtime flag. If
there is no DOM, it verifies the signature of the deserialized SignedInfo
element in the SOAP Header. However, serializing deserialized data may change
the octet stream that was signed, unless we're using gSOAP as producers and
consumers (with the SOAP_XML_INDENT flag reset).
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_verify_SignatureValue(struct soap *soap, int alg, const void *key, int keylen)
{
  ds__SignatureType *signature = soap_wsse_Signature(soap);
  DBGFUN1("soap_wsse_verify_SignatureValue", "alg=%x", alg);
  /* always need an HMAC secret key or DSA/RSA public key to verify */
  if (!key)
    return soap_wsse_fault(soap, wsse__SecurityTokenUnavailable, NULL);
  /* verify the SignedInfo element with the SignatureValue element */
  if (signature
   && signature->SignedInfo
   && signature->SignatureValue)
  {
    char *sig = NULL;
    const char *sigval = NULL;
    int method, bits, siglen, sigvallen;
    struct soap_dom_element *elt = NULL;
    /* check that we are using the intended signature algorithm */
    if (soap_wsse_get_SignedInfo_SignatureMethod(soap, &method, &bits))
      return soap->error;
    if (alg != method)
      return soap_wsse_fault(soap, wsse__FailedCheck, "Incorrect signature algorithm used");
    /* retrieve the signature */
    sigval = soap_base642s(soap, signature->SignatureValue, NULL, 0, &sigvallen);
    /* search the DOM for SignedInfo */
    elt = soap_dom_find(soap->dom, soap->dom, ds_URI, "SignedInfo", 0);
    if (elt)
    {
      int err = SOAP_OK;
      const char *c14ninclude = soap->c14ninclude;
      /* should not include leading whitespace in signature verification */
      elt->lead = NULL;
      /* use smdevp engine to verify SignedInfo */
      if ((alg & SOAP_SMD_ALGO) == SOAP_SMD_HMAC)
      {
        sig = (char*)soap_malloc(soap, soap_smd_size(alg, key));
      }
      else
      {
        sig = (char*)sigval;
        siglen = sigvallen;
      }
      if (signature->SignedInfo->CanonicalizationMethod && signature->SignedInfo->CanonicalizationMethod->Algorithm)
      {
        struct soap_dom_element *prt;
        struct soap_dom_attribute *att;
        if (signature->SignedInfo->CanonicalizationMethod->c14n__InclusiveNamespaces)
          soap->c14ninclude = signature->SignedInfo->CanonicalizationMethod->c14n__InclusiveNamespaces->PrefixList;
        else
          soap->c14ninclude = NULL;
        /* recanonicalize DOM while keeping content "as is" */
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Verifying signed canonicalized DOM with C14N prefix list '%s'\n", soap->c14ninclude ? soap->c14ninclude : ""));
        soap->mode &= ~SOAP_XML_DOM;
        soap->mode |= SOAP_XML_CANONICAL | SOAP_DOM_ASIS;
        if (strcmp(signature->SignedInfo->CanonicalizationMethod->Algorithm, c14n_URI)
         && strcmp(signature->SignedInfo->CanonicalizationMethod->Algorithm, c14n_wc_URI)
         && strcmp(signature->SignedInfo->CanonicalizationMethod->Algorithm, exc_c14n_URI)
         && strcmp(signature->SignedInfo->CanonicalizationMethod->Algorithm, exc_c14n_wc_URI))
          return soap_wsse_fault(soap, wsse__UnsupportedAlgorithm, "Invalid canonicalization method");
        err = soap_smd_begin(soap, alg, key, keylen);
        /* emit all xmlns attributes of ancestors */
        while (soap->nlist)
        {
          struct soap_nlist *np = soap->nlist->next;
          SOAP_FREE(soap, soap->nlist);
          soap->nlist = np;
        }
        /* push xmlns:ns="..." */
        for (prt = elt->prnt; prt; prt = prt->prnt)
        {
          for (att = prt->atts; att; att = att->next)
          {
            DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM attribute = %s\n", att->name));
            if (att->name && att->text && !strncmp(att->name, "xmlns:", 6) && !soap_lookup_ns(soap, att->name + 6, strlen(att->name + 6)))
              (void)soap_attribute(soap, att->name, att->text);
          }
        }
        /* push xmlns="..." */
        for (prt = elt->prnt; prt; prt = prt->prnt)
        {
          for (att = prt->atts; att; att = att->next)
          {
            if (att->name && att->text && !strcmp(att->name, "xmlns"))
            {
              (void)soap_attribute(soap, att->name, att->text);
              prt = NULL;
              break;
            }
          }
          if (!prt)
            break;
        }
      }
      else
      {
        /* compute digest over DOM "as is" */
        soap->mode &= ~(SOAP_XML_CANONICAL | SOAP_XML_DOM);
        soap->mode |= SOAP_DOM_ASIS;
        err = soap_smd_begin(soap, alg, key, keylen);
      }
      /* do not dump namespace table xmlns bindings */
      soap->ns = 2;
      /* compute digest */
      soap->feltbegout = NULL;
      soap->feltendout = NULL;
      if (!err)
        err = soap_out_xsd__anyType(soap, NULL, 0, elt, NULL);
      soap->c14ninclude = c14ninclude;
      if (soap_smd_end(soap, sig, &siglen) || err)
        return soap_wsse_fault(soap, wsse__FailedCheck, "The signed SignedInfo SignatureValue is invalid");
      if ((alg & SOAP_SMD_ALGO) == SOAP_SMD_HMAC)
      {
        if (bits == 0)
        {
          if (siglen != sigvallen || memcmp(sig, sigval, siglen))
            return soap_wsse_fault(soap, wsse__FailedCheck, "The HMAC-signed SignedInfo is invalid");
        }
        else if (bits > 8 * siglen || memcmp(sig, sigval, 8 * bits))
        {
          return soap_wsse_fault(soap, wsse__FailedCheck, "The HMAC-signed SignedInfo is invalid");
        }
      }
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Signature in DOM is valid\n"));
      return SOAP_OK;
    }
    else
    {
      int err = SOAP_OK;
      const char *c14nexclude = soap->c14nexclude;
      const char *c14ninclude = soap->c14ninclude;
      soap_mode mode = soap->mode;
      short part = soap->part;
      /* serialize the SignedInfo element as it appeared in the SOAP Header */
      soap->level = 4;
      soap->c14ninclude = NULL;
      soap->part = SOAP_IN_HEADER; /* header encoding rules (literal) */
      soap->mode &= ~SOAP_XML_DOM;
      if (signature->SignedInfo->CanonicalizationMethod)
        soap->mode |= SOAP_XML_CANONICAL;
      else
        soap->mode &= ~SOAP_XML_CANONICAL;
      if ((alg & SOAP_SMD_ALGO) == SOAP_SMD_HMAC)
      {
        sig = (char*)soap_malloc(soap, soap_smd_size(alg, key));
      }
      else
      {
        sig = (char*)sigval;
        siglen = sigvallen;
      }
      err = soap_smd_begin(soap, alg, key, keylen);
      if (!err)
        err = soap_out_ds__SignedInfoType(soap, "ds:SignedInfo", 0, signature->SignedInfo, NULL);
      soap->mode = mode;
      soap->c14nexclude = c14nexclude;
      soap->c14ninclude = c14ninclude;
      soap->part = part;
      if (soap_smd_end(soap, sig, &siglen) || err)
        return soap_wsse_fault(soap, wsse__FailedCheck, "The signed serialized SignedInfo SignatureValue is invalid");
      if ((alg & SOAP_SMD_ALGO) == SOAP_SMD_HMAC)
      {
        if (bits == 0)
        {
          if (siglen != sigvallen || memcmp(sig, sigval, siglen))
            return soap_wsse_fault(soap, wsse__FailedCheck, "The HMAC-signed serialized SignedInfo is invalid");
        }
        else if (bits > 8 * siglen || memcmp(sig, sigval, 8 * bits))
        {
          return soap_wsse_fault(soap, wsse__FailedCheck, "The HMAC-signed serialized SignedInfo is invalid");
        }
      }
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Signature is valid\n"));
      return SOAP_OK;
    }
  }
  return soap_wsse_fault(soap, wsse__FailedCheck, "Signature with SignedInfo and SignatureValue required");
}

/******************************************************************************/

/**
@fn int soap_wsse_verify_SignedInfo(struct soap *soap)
@brief Verifies the digest values of the XML elements referenced by the SignedInfo References.
@param soap context
@return SOAP_OK or fault

This function searches for the SignedInfo element in the soap->dom DOM tree to
verify the digests contained therein. Using the DOM ensures we will verify the
digests of the locally signed elements as they were exactly received by the
parser, by using the -DWITH_DOM compile flag and SOAP_XML_DOM runtime flag. If
there is no DOM, the function fails.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_verify_SignedInfo(struct soap *soap)
{
  ds__SignedInfoType *signedInfo = soap_wsse_SignedInfo(soap);
  DBGFUN("soap_wsse_verify_SignedInfo");
  if (signedInfo)
  {
    struct soap_dom_element *elt;
    int i;
    /* must have at least one reference element */
    if (signedInfo->__sizeReference == 0)
      return soap_wsse_fault(soap, wsse__InvalidSecurity, "Signature SignedInfo Reference required");
    /* remove ds:Signature from the DOM to support enveloped signatures */
    elt = soap_dom_find(soap->dom, soap->dom, ds_URI, "Signature", 0);
    if (elt)
    {
      elt->elts = NULL;
      elt->atts = NULL;
      elt->nstr = NULL;
      elt->name = NULL;
      elt->text = NULL;
      elt->code = NULL;
    }
    /* As an alternative to the current implementation, this might be a good
       place to re-canonicalize the entire DOM to improve interop. Two DOMs can
       be used: one with non-c14n XML and one with c14n XML so we can handle
       multiple different transforms.
    */
    /* for each reference element, check the digest */
    for (i = 0; i < signedInfo->__sizeReference; i++)
    {
      ds__ReferenceType *reference = signedInfo->Reference[i];
      /* reference element is complete? */
      if (!reference->URI
       || !reference->DigestMethod
       || !reference->DigestMethod->Algorithm
       || !reference->DigestValue)
        return soap_wsse_fault(soap, wsse__InvalidSecurity, "Incomplete SignedInfo Reference");
      /* reference is local? */
      if (*reference->URI == '#')
      {
        int alg, canonical = 0;
        const char *c14ninclude = soap->c14ninclude;
        unsigned char hash[SOAP_SMD_MAX_SIZE];
        /* digest algorithm */
        if (!strcmp(reference->DigestMethod->Algorithm, ds_sha1URI))
          alg = SOAP_SMD_DGST_SHA1;
#if (OPENSSL_VERSION_NUMBER >= 0x0090800fL)
        else if (!strcmp(reference->DigestMethod->Algorithm, ds_sha224URI))
          alg = SOAP_SMD_DGST_SHA224;
        else if (!strcmp(reference->DigestMethod->Algorithm, ds_sha256URI))
          alg = SOAP_SMD_DGST_SHA256;
        else if (!strcmp(reference->DigestMethod->Algorithm, ds_sha384URI))
          alg = SOAP_SMD_DGST_SHA384;
        else if (!strcmp(reference->DigestMethod->Algorithm, ds_sha512URI))
          alg = SOAP_SMD_DGST_SHA512;
#endif
        else
          return soap_wsse_fault(soap, wsse__UnsupportedAlgorithm, reference->DigestMethod->Algorithm);
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Verifying digest of locally referenced data %s alg=%x\n", reference->URI, alg));
        soap->c14ninclude = NULL;
        /* if reference has a transform, it should be an c14n or exc-c14n transform */
        if (reference->Transforms)
        {
          int i;
          for (i = 0; i < reference->Transforms->__sizeTransform; i++)
          {
            if (!reference->Transforms->Transform[i].Algorithm)
              return soap_wsse_fault(soap, wsse__UnsupportedAlgorithm, NULL);
            if (!strcmp(reference->Transforms->Transform[i].Algorithm, c14n_URI)
             || !strcmp(reference->Transforms->Transform[i].Algorithm, c14n_wc_URI))
            {
              canonical = 1;
            }
            else if (!strcmp(reference->Transforms->Transform[i].Algorithm, exc_c14n_URI)
                  || !strcmp(reference->Transforms->Transform[i].Algorithm, exc_c14n_wc_URI))
            {
              canonical = 1;
              if (reference->Transforms->Transform[i].c14n__InclusiveNamespaces)
                soap->c14ninclude = reference->Transforms->Transform[i].c14n__InclusiveNamespaces->PrefixList;
            }
            else if (strcmp(reference->Transforms->Transform[i].Algorithm, ds_envsigURI))
            {
              DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Unsupported Transform Algorithm %s\n", reference->Transforms->Transform[i].Algorithm));
              /* ignore application-specific Transforms
                 return soap_wsse_fault(soap, wsse__UnsupportedAlgorithm, reference->Transforms->Transform[i].Algorithm);
              */
            }
          }
        }
        /* convert base64 digest to binary */
        soap_base642s(soap, reference->DigestValue, (char*)hash, SOAP_SMD_MAX_SIZE, NULL);
        /* verify the digest of a locally signed element */
        if (soap_wsse_verify_digest(soap, alg, canonical, reference->URI + 1, hash))
          return soap->error;
        soap->c14ninclude = c14ninclude;
      }
    }
    return SOAP_OK;
  }
  return soap_wsse_fault(soap, wsse__InvalidSecurity, "Signature SignedInfo required");
}

/******************************************************************************/

/**
@fn int soap_wsse_verify_digest(struct soap *soap, int alg, int canonical, const char *id, unsigned char hash[SOAP_SMD_MAX_SIZE])
@brief Verifies the digest value of an XML element referenced by id against the hash.
@param soap context
@param[in] alg digest algorithm
@param[in] canonical flag indicating that element is signed in c14n (inclusive or exclusive)
@param[in] id string of the XML element to verify
@param[in] hash digest value to verify against
@return SOAP_OK or fault
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_verify_digest(struct soap *soap, int alg, int canonical, const char *id, unsigned char hash[SOAP_SMD_MAX_SIZE])
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  struct soap_dom_element *elt, *dom = NULL;
  DBGFUN3("soap_wsse_verify_digest", "alg=%x", alg, "canonical=%d", canonical, "id=%s", id);
  if (!data)
    return soap_set_receiver_error(soap, "soap_wsse_verify_digest", "Plugin not registered", SOAP_PLUGIN_ERROR);
  /* traverse the DOM to find the element with matching wsu:Id or ds:Id */
  for (elt = soap->dom; elt; elt = soap_dom_next_element(elt, NULL))
  {
    struct soap_dom_attribute *att;
    for (att = elt->atts; att; att = att->next)
    {
      /* check if attribute is an wsu:Id or ds:Id or ID*/
      if (att->name
       && ((att->nstr && (!strcmp(att->nstr, wsu_URI) || !strcmp(att->nstr, ds_URI)) && (!strcmp(att->name, "Id") || !soap_tag_cmp(att->name, "*:Id")))
        || (!att->nstr && (!strcmp(att->name, "ID") || !strcmp(att->name, "AssertionID")))))
      {
        /* found a match, compare attribute value with id */
        if (att->text && !strcmp(att->text, id))
        {
          if (dom)
            return soap_wsse_fault(soap, wsse__FailedCheck, "SignedInfo duplicate Id");
          dom = elt;
          /* elt = NULL; break; */ /* improves speed but skips duplicate Id check */
        }
      }
    }
  }
  if (dom)
  {
    unsigned char HA[SOAP_SMD_MAX_SIZE];
    int len, err = SOAP_OK;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Computing digest for Id=%s\n", id));
    /* do not hash leading whitespace */
    dom->lead = NULL;
    /* canonical or as-is? */
    if (canonical)
    {
      struct soap_dom_element *prt;
      struct soap_dom_attribute *att;
      soap->mode &= ~SOAP_XML_DOM;
      soap->mode |= SOAP_XML_CANONICAL | SOAP_DOM_ASIS;
      err = soap_smd_begin(soap, alg, NULL, 0);
      /* emit all xmlns attributes of ancestors */
      while (soap->nlist)
      {
        struct soap_nlist *np = soap->nlist->next;
        SOAP_FREE(soap, soap->nlist);
        soap->nlist = np;
      }
      for (prt = dom->prnt; prt; prt = prt->prnt)
      {
        for (att = prt->atts; att; att = att->next)
        {
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "DOM attribute = %s\n", att->name));
          if (att->name && att->text && !strncmp(att->name, "xmlns:", 6) && !soap_lookup_ns(soap, att->name + 6, strlen(att->name + 6)))
            (void)soap_attribute(soap, att->name, att->text);
        }
      }
      for (prt = dom->prnt; prt; prt = prt->prnt)
      {
        for (att = prt->atts; att; att = att->next)
        {
          if (att->name && att->text && !strcmp(att->name, "xmlns"))
          {
            (void)soap_attribute(soap, att->name, att->text);
            prt = NULL;
            break;
          }
        }
        if (!prt)
          break;
      }
    }
    else
    {
      /* compute digest over DOM "as is" */
      soap->mode &= ~SOAP_XML_CANONICAL;
      soap->mode |= SOAP_DOM_ASIS;
      err = soap_smd_begin(soap, alg, NULL, 0);
    }
    /* do not dump namespace table xmlns bindings */
    soap->ns = 2;
    /* compute digest */
    soap->feltbegout = NULL;
    soap->feltendout = NULL;
    /* root may have prefixes that should be retained as per soap->c14ninclude */
    soap->event = SOAP_SEC_BEGIN;
    if (!err)
      err = soap_out_xsd__anyType(soap, NULL, 0, dom, NULL);
    if (soap_smd_end(soap, (char*)HA, &len) || err)
      return soap_wsse_fault(soap, wsse__FailedCheck, "Digest computation failed");
    /* compare digests, success if identical */
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Comparing digest hashes\n"));
    DBGHEX(TEST, hash, len);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n--\n"));
    DBGHEX(TEST, HA, len);
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n"));
    if (!memcmp(hash, HA, (size_t)len))
      return SOAP_OK;
    return soap_wsse_fault(soap, wsse__FailedCheck, NULL);
  }
  if ((data->vrfy_alg & SOAP_WSSE_IGNORE_EXTRA_REFS))
    return SOAP_OK;
  return soap_wsse_fault(soap, wsse__FailedCheck, "SignedInfo reference URI target not found");
}

/******************************************************************************\
 *
 *      ds:Signature/KeyInfo
 *
\******************************************************************************/

/**
@fn ds__KeyInfoType* soap_wsse_add_KeyInfo(struct soap *soap)
@brief Adds KeyInfo element.
@param soap context
@return ds__KeyInfo object
*/
SOAP_FMAC1
struct ds__KeyInfoType *
SOAP_FMAC2
soap_wsse_add_KeyInfo(struct soap *soap)
{
  ds__SignatureType *signature = soap_wsse_add_Signature(soap);
  if (!signature->KeyInfo)
  {
    signature->KeyInfo = (ds__KeyInfoType*)soap_malloc(soap, sizeof(ds__KeyInfoType));
    if (!signature->KeyInfo)
      return NULL;
  }
  soap_default_ds__KeyInfoType(soap, signature->KeyInfo);
  return signature->KeyInfo;
}

/******************************************************************************/

/**
@fn ds__KeyInfoType* soap_wsse_KeyInfo(struct soap *soap)
@brief Returns KeyInfo element if present.
@param soap context
@return ds__KeyInfo object or NULL
*/
SOAP_FMAC1
struct ds__KeyInfoType *
SOAP_FMAC2
soap_wsse_KeyInfo(struct soap *soap)
{
  ds__SignatureType *signature = soap_wsse_Signature(soap);
  if (signature)
    return signature->KeyInfo;
  return NULL;
}

/******************************************************************************\
 *
 *      ds:Signature/KeyInfo/KeyName
 *
\******************************************************************************/

/**
@fn int soap_wsse_add_KeyInfo_KeyName(struct soap *soap, const char *name)
@brief Adds KeyName element.
@param soap context
@param[in] name string of the KeyName
@return SOAP_OK

@note
The recommended method to add Key information is to utilize KeyIdentifier
instead of KeyName. A KeyName is useful mainly for internal use.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_KeyInfo_KeyName(struct soap *soap, const char *name)
{
  ds__KeyInfoType *keyInfo = soap_wsse_add_KeyInfo(soap);
  DBGFUN1("soap_wsse_add_KeyInfo_KeyName", "name=%s", name);
  /* populate the KeyName element */
  keyInfo->KeyName = soap_strdup(soap, name);
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_get_KeyInfo_KeyName(struct soap *soap)
@brief Returns KeyName element if present.
@param soap context
@return string or NULL
*/
SOAP_FMAC1
const char *
SOAP_FMAC2
soap_wsse_get_KeyInfo_KeyName(struct soap *soap)
{
  ds__KeyInfoType *keyInfo = soap_wsse_KeyInfo(soap);
  DBGFUN("soap_wsse_get_KeyInfo_KeyName");
  if (!keyInfo)
    return NULL;
  return keyInfo->KeyName;
}

/******************************************************************************\
 *
 *      ds:Signature/KeyInfo/wsse:SecurityTokenReference/Reference/@URI
 *
\******************************************************************************/

/**
@fn int soap_wsse_add_KeyInfo_SecurityTokenReferenceURI(struct soap *soap, const char *URI, const char *valueType)
@brief Adds KeyInfo element with SecurityTokenReference URI.
@param soap context
@param[in] URI string referencing a security token
@param[in] valueType string or NULL
@return SOAP_OK
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_KeyInfo_SecurityTokenReferenceURI(struct soap *soap, const char *URI, const char *valueType)
{
  ds__KeyInfoType *keyInfo = soap_wsse_add_KeyInfo(soap);
  DBGFUN2("soap_wsse_add_KeyInfo_SecurityTokenReferenceURI", "URI=%s", URI?URI:"", "valueType=%s", valueType?valueType:"");
  /* allocate SecurityTokenReference element if we don't have one already */
  if (!keyInfo->wsse__SecurityTokenReference)
  {
    keyInfo->wsse__SecurityTokenReference = (_wsse__SecurityTokenReference*)soap_malloc(soap, sizeof(_wsse__SecurityTokenReference));
    if (!keyInfo->wsse__SecurityTokenReference)
      return soap->error = SOAP_EOM;
  }
  soap_default__wsse__SecurityTokenReference(soap, keyInfo->wsse__SecurityTokenReference);
  /* allocate Reference element */
  keyInfo->wsse__SecurityTokenReference->Reference = (_wsse__Reference*)soap_malloc(soap, sizeof(_wsse__Reference));
  if (!keyInfo->wsse__SecurityTokenReference->Reference)
    return soap->error = SOAP_EOM;
  soap_default__wsse__Reference(soap, keyInfo->wsse__SecurityTokenReference->Reference);
  /* populate the Reference element */
  keyInfo->wsse__SecurityTokenReference->Reference->URI = soap_strdup(soap, URI);
  keyInfo->wsse__SecurityTokenReference->Reference->ValueType = soap_strdup(soap, valueType);
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(struct soap *soap, const char *URI)
@brief Adds KeyInfo element with SecurityTokenReference URI to an X509 cert.
@param soap context
@param[in] URI string referencing an X509 certificate
@return SOAP_OK
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(struct soap *soap, const char *URI)
{
  return soap_wsse_add_KeyInfo_SecurityTokenReferenceURI(soap, URI, wsse_X509v3URI);
}

/******************************************************************************/

/**
@fn const char* soap_wsse_get_KeyInfo_SecurityTokenReferenceURI(struct soap *soap)
@brief Returns a SecurityTokenReference URI if present.
@param soap context
@return string or NULL
*/
SOAP_FMAC1
const char *
SOAP_FMAC2
soap_wsse_get_KeyInfo_SecurityTokenReferenceURI(struct soap *soap)
{
  ds__KeyInfoType *keyInfo = soap_wsse_KeyInfo(soap);
  if (keyInfo
   && keyInfo->wsse__SecurityTokenReference
   && keyInfo->wsse__SecurityTokenReference->Reference)
    return keyInfo->wsse__SecurityTokenReference->Reference->URI;
  return NULL;
}

/******************************************************************************/

/**
@fn const char* soap_wsse_get_KeyInfo_SecurityTokenReferenceValueType(struct soap *soap)
@brief Returns a SecurityTokenReference ValueType if present.
@param soap context
@return string or NULL
*/
SOAP_FMAC1
const char *
SOAP_FMAC2
soap_wsse_get_KeyInfo_SecurityTokenReferenceValueType(struct soap *soap)
{
  ds__KeyInfoType *keyInfo = soap_wsse_KeyInfo(soap);
  if (keyInfo
   && keyInfo->wsse__SecurityTokenReference
   && keyInfo->wsse__SecurityTokenReference->Reference)
    return keyInfo->wsse__SecurityTokenReference->Reference->ValueType;
  return NULL;
}

/******************************************************************************/

/**
@fn X509* soap_wsse_get_KeyInfo_SecurityTokenReferenceX509(struct soap *soap)
@brief Returns a X509 certificate when referenced and present as a BinarySecurity token or when embedded in the signature KeyIdentifier or KeyInfo element. This call must be followed by an X509_free to deallocate the X509 certificate data.
@param soap context
@return X509 object or NULL with wsse:SecurityTokenUnavailable fault
*/
SOAP_FMAC1
X509*
SOAP_FMAC2
soap_wsse_get_KeyInfo_SecurityTokenReferenceX509(struct soap *soap)
{
  const char *URI = soap_wsse_get_KeyInfo_SecurityTokenReferenceURI(soap);
  X509 *cert = NULL;
  DBGFUN("soap_wsse_get_KeyInfo_SecurityTokenReferenceX509");
  if (URI && *URI == '#')
  {
    const char *valueType;
    valueType = soap_wsse_get_KeyInfo_SecurityTokenReferenceValueType(soap);
    if (!valueType || !strcmp(valueType, wsse_X509v3URI))
      cert = soap_wsse_get_BinarySecurityTokenX509(soap, URI + 1);
  }
  else if (!URI)
  {
#if (OPENSSL_VERSION_NUMBER >= 0x0090800fL)
    const unsigned char *der = NULL;
#else
    unsigned char *der = NULL;
#endif
    int derlen;
    ds__KeyInfoType *keyInfo = soap_wsse_KeyInfo(soap);
    if (keyInfo && keyInfo->X509Data && keyInfo->X509Data->X509Certificate)
    {
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Using X509 cert from KeyInfo to verify signature\n"));
      der = (unsigned char*)soap_base642s(soap, keyInfo->X509Data->X509Certificate, NULL, 0, &derlen);
    }
    else
    {
      const char *valueType = soap_wsse_get_KeyInfo_SecurityTokenReferenceKeyIdentifierValueType(soap, keyInfo);
      if (valueType && !strcmp(valueType, ds_hmac_sha1URI))
      {
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Using X509 cert from KeyIdentifier to verify signature\n"));
        der = (unsigned char*)soap_wsse_get_KeyInfo_SecurityTokenReferenceKeyIdentifier(soap, keyInfo, &derlen);
      }
    }
    if (der)
    {
      cert = d2i_X509(NULL, &der, derlen);
      /* verify the certificate */
      if (cert && soap_wsse_verify_X509(soap, cert))
      {
        X509_free(cert);
        cert = NULL;
      }
    }
  }
  return cert;
}

/******************************************************************************/

/**
@fn int soap_wsse_add_KeyInfo_X509Certificate(struct soap *soap, X509 *cert)
@brief Adds KeyInfo/X509Data/X509Certificate.
@param soap context
@param[in] cert X509 certificate
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_KeyInfo_X509Certificate(struct soap *soap, X509 *cert)
{
  ds__KeyInfoType *keyInfo = soap_wsse_add_KeyInfo(soap);
  int derlen;
  unsigned char *der, *s;
  DBGFUN("soap_wsse_add_KeyInfo_X509Certificate");
  if (!cert)
    return soap_wsse_fault(soap, wsse__InvalidSecurityToken, "Missing certificate");
  if (!keyInfo)
    return soap->error = SOAP_EOM;
  /* populate the X509Data element */
  keyInfo->X509Data = (struct ds__X509DataType*)soap_malloc(soap, sizeof(struct ds__X509DataType));
  if (!keyInfo->X509Data)
    return soap->error = SOAP_EOM;
  soap_default_ds__X509DataType(soap, keyInfo->X509Data);
  /* determine the storage requirement */
  derlen = i2d_X509(cert, NULL);
  if (derlen < 0)
    return soap_wsse_fault(soap, wsse__InvalidSecurityToken, "Invalid certificate");
  /* use the gSOAP engine's look-aside buffer to temporarily hold the cert */
  if (soap_store_lab(soap, NULL, derlen))
    return SOAP_EOM;
  s = der = (unsigned char*)soap->labbuf;
  /* store in DER format */
  i2d_X509(cert, &s);
  /* populate the X509Certificate with base64 certificate data */
  keyInfo->X509Data->X509Certificate = soap_s2base64(soap, der, NULL, derlen);
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn struct ds__X509IssuerSerialType *soap_wsse_get_KeyInfo_SecurityTokenReferenceX509Data(struct soap *soap)
@brief Returns ds__X509IssuerSerialType with non-NULL X509IssuerName and non-NULL X509SerialNumber of a X509Data element when present and set.
@param soap context
@return pointer to ds__X509IssuerSerialType struct or NULL
*/
SOAP_FMAC1
struct ds__X509IssuerSerialType *
SOAP_FMAC2
soap_wsse_get_KeyInfo_SecurityTokenReferenceX509Data(struct soap *soap)
{
  ds__KeyInfoType *keyInfo = soap_wsse_KeyInfo(soap);
  if (keyInfo
   && keyInfo->wsse__SecurityTokenReference
   && keyInfo->wsse__SecurityTokenReference->ds__X509Data
   && keyInfo->wsse__SecurityTokenReference->ds__X509Data->X509IssuerSerial
   && keyInfo->wsse__SecurityTokenReference->ds__X509Data->X509IssuerSerial->X509IssuerName
   && keyInfo->wsse__SecurityTokenReference->ds__X509Data->X509IssuerSerial->X509SerialNumber)
    return keyInfo->wsse__SecurityTokenReference->ds__X509Data->X509IssuerSerial;
  return NULL;
}

/******************************************************************************\
 *
 *      ds:Signature/KeyInfo/wsse:SecurityTokenReference/Reference/KeyIdentifier
 *
\******************************************************************************/

/**
@fn int soap_wsse_add_KeyInfo_SecurityTokenReferenceKeyIdentifier(struct soap *soap, const char *id, const char *valueType, unsigned char *data, int size)
@brief Adds KeyInfo element with SecurityTokenReference/KeyIdentifier binary data.
@param soap context
@param[in] id string for signature reference
@param[in] valueType string
@param[in] data binary data
@param[in] size of binary data
@return SOAP_OK
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_KeyInfo_SecurityTokenReferenceKeyIdentifier(struct soap *soap, const char *id, const char *valueType, unsigned char *data, int size)
{
  ds__KeyInfoType *keyInfo = soap_wsse_add_KeyInfo(soap);
  DBGFUN2("soap_wsse_add_KeyInfo_SecurityTokenReferenceKeyIdentifier", "id=%s", id?id:"", "valueType=%s", valueType?valueType:"");
  /* allocate SecurityTokenReference if we don't have one already */
  if (!keyInfo->wsse__SecurityTokenReference)
  {
    keyInfo->wsse__SecurityTokenReference = (_wsse__SecurityTokenReference*)soap_malloc(soap, sizeof(_wsse__SecurityTokenReference));
    if (!keyInfo->wsse__SecurityTokenReference)
      return soap->error = SOAP_EOM;
  }
  soap_default__wsse__SecurityTokenReference(soap, keyInfo->wsse__SecurityTokenReference);
  /* allocate KeyIdentifier */
  keyInfo->wsse__SecurityTokenReference->KeyIdentifier = (_wsse__KeyIdentifier*)soap_malloc(soap, sizeof(_wsse__KeyIdentifier));
  if (!keyInfo->wsse__SecurityTokenReference->KeyIdentifier)
    return soap->error = SOAP_EOM;
  soap_default__wsse__KeyIdentifier(soap, keyInfo->wsse__SecurityTokenReference->KeyIdentifier);
  /* populate KeyIdentifier */
  keyInfo->wsse__SecurityTokenReference->KeyIdentifier->wsu__Id = soap_strdup(soap, id);
  keyInfo->wsse__SecurityTokenReference->KeyIdentifier->ValueType = soap_strdup(soap, valueType);
  keyInfo->wsse__SecurityTokenReference->KeyIdentifier->EncodingType = (char*)wsse_Base64BinaryURI;
  keyInfo->wsse__SecurityTokenReference->KeyIdentifier->__item = soap_s2base64(soap, data, NULL, size);
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn const char* soap_wsse_get_KeyInfo_SecurityTokenReferenceKeyIdentifierValueType(struct soap *soap, ds__KeyInfoType *keyInfo)
@brief Returns KeyInfo/SecurityTokenReference/KeyIdentifier/ValueType if present.
@param soap context
@param keyInfo points to ds__KeyInfoType, e.g. using soap_wsse_KeyInfo(soap)
@return string or NULL
*/
SOAP_FMAC1
const char *
SOAP_FMAC2
soap_wsse_get_KeyInfo_SecurityTokenReferenceKeyIdentifierValueType(struct soap *soap, ds__KeyInfoType *keyInfo)
{
  DBGFUN("soap_wsse_get_KeyInfo_SecurityTokenReferenceKeyIdentifierValueType");
  if (!keyInfo
   || !keyInfo->wsse__SecurityTokenReference
   || !keyInfo->wsse__SecurityTokenReference->KeyIdentifier)
    return NULL;
  return keyInfo->wsse__SecurityTokenReference->KeyIdentifier->ValueType;
}

/******************************************************************************/

/**
@fn const unsigned char* soap_wsse_get_KeyInfo_SecurityTokenReferenceKeyIdentifier(struct soap *soap, ds__KeyInfoType *keyInfo, int *size)
@brief Returns KeyInfo/SecurityTokenReference/KeyIdentifier binary data.
@param soap context
@param keyInfo points to ds__KeyInfoType, e.g. using soap_wsse_KeyInfo(soap)
@param[out] size is set to the size of the decoded data
@return data or NULL
*/
SOAP_FMAC1
const unsigned char *
SOAP_FMAC2
soap_wsse_get_KeyInfo_SecurityTokenReferenceKeyIdentifier(struct soap *soap, ds__KeyInfoType *keyInfo, int *size)
{
  DBGFUN("soap_wsse_get_KeyInfo_SecurityTokenReferenceKeyIdentifier");
  if (!keyInfo
   || !keyInfo->wsse__SecurityTokenReference
   || !keyInfo->wsse__SecurityTokenReference->KeyIdentifier)
    return NULL;
  if (!keyInfo->wsse__SecurityTokenReference->KeyIdentifier->EncodingType || !strcmp(keyInfo->wsse__SecurityTokenReference->KeyIdentifier->EncodingType, wsse_Base64BinaryURI))
    return (unsigned char*)soap_base642s(soap, keyInfo->wsse__SecurityTokenReference->KeyIdentifier->__item, NULL, 0, size);
  else
    return (unsigned char*)soap_hex2s(soap, keyInfo->wsse__SecurityTokenReference->KeyIdentifier->__item, NULL, 0, size);
}

/******************************************************************************\
 *
 *      ds:Signature/KeyInfo/wsse:SecurityTokenReference/Reference/Embedded
 *
\******************************************************************************/

/**
@fn int soap_wsse_add_KeyInfo_SecurityTokenReferenceEmbedded(struct soap *soap, const char *id, const char *valueType)
@brief Adds KeyInfo element with Embedded SecurityTokenReference.
@param soap context
@param[in] id string for signature reference
@param[in] valueType string
@return SOAP_OK

@note
This function does not add embedded tokens automatically. See code for comments.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_KeyInfo_SecurityTokenReferenceEmbedded(struct soap *soap, const char *id, const char *valueType)
{
  ds__KeyInfoType *keyInfo = soap_wsse_add_KeyInfo(soap);
  DBGFUN("soap_wsse_get_KeyInfo_SecurityTokenReferenceEmbedded");
  /* allocate SecurityTokenReference if we don't have one already */
  if (!keyInfo->wsse__SecurityTokenReference)
  {
    keyInfo->wsse__SecurityTokenReference = (_wsse__SecurityTokenReference*)soap_malloc(soap, sizeof(_wsse__SecurityTokenReference));
    if (!keyInfo->wsse__SecurityTokenReference)
      return soap->error = SOAP_EOM;
  }
  soap_default__wsse__SecurityTokenReference(soap, keyInfo->wsse__SecurityTokenReference);
  /* allocate Embedded element */
  keyInfo->wsse__SecurityTokenReference->Embedded = (_wsse__Embedded*)soap_malloc(soap, sizeof(_wsse__Embedded));
  if (!keyInfo->wsse__SecurityTokenReference->Embedded)
    return soap->error = SOAP_EOM;
  soap_default__wsse__Embedded(soap, keyInfo->wsse__SecurityTokenReference->Embedded);
  /* populate Embedded element */
  keyInfo->wsse__SecurityTokenReference->Embedded->wsu__Id = soap_strdup(soap, id);
  keyInfo->wsse__SecurityTokenReference->Embedded->ValueType = soap_strdup(soap, valueType);
  /* TODO: Semi-automatically add embedded tokens and assertions. Could use DOM here?
  keyInfo->wsse__SecurityTokenReference->Embedded->xyz = ...;
  */
  return SOAP_OK;
}

/******************************************************************************\
 *
 *      wsse:Security/xenc:EncryptedKey header element
 *
\******************************************************************************/

/**
@fn int soap_wsse_add_EncryptedKey(struct soap *soap, int alg, const char *URI, X509 *cert, const char *subjectkeyid, const char *issuer, const char *serial)
@brief Adds EncryptedKey header element and initiates the encryption of the SOAP Body.
@param soap context
@param[in] alg algorithm to use, SOAP_MEC_ENV_ENC_DES_CBC etc.
@param[in] URI a unique identifier for the key, required for interoperability
@param[in] cert the X509 certificate with public key or NULL
@param[in] subjectkeyid string identification of the subject which when set to non-NULL is used instead of embedding the public key in the message
@param[in] issuer string to include SecurityTokenReference/X509Data 
@param[in] serial string to include SecurityTokenReference/X509Data
@return SOAP_OK or error code

This function adds the encrypted key or subject key ID to the WS-Security
header and initiates encryption of the SOAP Body. An X509 certificate may be
provided, or the subjectkeyid, or the issuer and serial number. The certificate
is embedded in the WS-Security EncryptedKey header. If the subjectkeyid string
is non-NULL the subject key ID is used in the EncryptedKey header instead of
the X509 certificate content.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_EncryptedKey(struct soap *soap, int alg, const char *URI, X509 *cert, const char *subjectkeyid, const char *issuer, const char *serial)
{
  return soap_wsse_add_EncryptedKey_encrypt_only(soap, alg, URI, cert, subjectkeyid, issuer, serial, NULL);
}

/******************************************************************************/

/**
@fn int soap_wsse_add_EncryptedKey_encrypt_only(struct soap *soap, int alg, const char *URI, X509 *cert, const char *subjectkeyid, const char *issuer, const char *serial, const char *tags)
@brief Adds EncryptedKey header element and initiates encryption of the given XML elements specified in the tags string. Should be used in combination with soap_wsse_set_wsu_id to set wsu:Id for given XML element tags.
@param soap context
@param[in] alg algorithm to use, SOAP_MEC_ENV_ENC_DES_CBC etc.
@param[in] URI a unique identifier for the key, required for interoperability
@param[in] cert the X509 certificate with public key or NULL
@param[in] subjectkeyid string identification of the subject which when set to non-NULL is used instead of embedding the public key in the message
@param[in] issuer string to include SecurityTokenReference/X509Data 
@param[in] serial string to include SecurityTokenReference/X509Data
@param[in] tags space-separated string of element tag names to encrypt
@return xenc__EncryptedKeyType object

This function adds the encrypted key or subject key ID to the WS-Security
header and initiates encryption of the SOAP Body. An X509 certificate may be
provided, or the subjectkeyid, or the issuer and serial number. The certificate
is embedded in the WS-Security EncryptedKey header. If the subjectkeyid string
is non-NULL the subject key ID is used in the EncryptedKey header instead of
the X509 certificate content. Only the XML elements given in the 'tags' string
as wsu:Id attributed elements are encrypted.

@warning
Use @ref soap_wsse_add_EncryptedKey_encrypt_only only in combination with
@ref soap_wsse_set_wsu_id with the tag names of the elements to be encrypted.
OTHERWISE THE GIVEN XML ELEMENTS ARE NOT ENCRYPTED AND WILL BE SENT IN THE
CLEAR.

@warning
The elements identified with @ref soap_wsse_set_wsu_id to encrypt MUST occur
EXACTLY ONCE in the SOAP Body.

@warning
Encryption/decryption of elements with simple content (CDATA content) IS NOT
SUPPORTED. This means that elements you want to encrypt with this function must
have complex content. That is, only encrypt elements with sub elements or
encrypt the entire SOAP Body.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_EncryptedKey_encrypt_only(struct soap *soap, int alg, const char *URI, X509 *cert, const char *subjectkeyid, const char *issuer, const char *serial, const char *tags)
{
  EVP_PKEY *pubk;
  unsigned char *key;
  int keylen;
  _wsse__Security *security;
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  DBGFUN("soap_wsse_add_EncryptedKey_encrypt_only");
  if (!data)
    return soap_set_receiver_error(soap, "soap_wsse_add_EncryptedKey_encrypt_only", "Plugin not registered", SOAP_PLUGIN_ERROR);
  security = soap_wsse_add_Security(soap);
  /* if we don't have a xenc:EncryptedKey, create one */
  if (!security->xenc__EncryptedKey)
  {
    security->xenc__EncryptedKey = (xenc__EncryptedKeyType*)soap_malloc(soap, sizeof(xenc__EncryptedKeyType));
    if (!security->xenc__EncryptedKey)
      return soap->error = SOAP_EOM;
  }
  soap_default_xenc__EncryptedKeyType(soap, security->xenc__EncryptedKey); 
  security->xenc__EncryptedKey->Id = soap_strdup(soap, URI);
  security->xenc__EncryptedKey->EncryptionMethod = (xenc__EncryptionMethodType*)soap_malloc(soap, sizeof(struct xenc__EncryptionMethodType));
  if (!security->xenc__EncryptedKey->EncryptionMethod)
    return soap->error = SOAP_EOM;
  soap_default_xenc__EncryptionMethodType(soap, security->xenc__EncryptedKey->EncryptionMethod); 
  /* RSA Version 1.5 or RSA-OAEP? */
  alg |= (SOAP_MEC_ENV | SOAP_MEC_ENC);
  if (alg & SOAP_MEC_OAEP)
  {
    security->xenc__EncryptedKey->EncryptionMethod->Algorithm = (char*)xenc_rsaesURI;
    security->xenc__EncryptedKey->EncryptionMethod->OAEPparams = NULL;
    security->xenc__EncryptedKey->EncryptionMethod->ds__DigestMethod = (struct ds__DigestMethodType*)soap_malloc(soap, sizeof(struct ds__DigestMethodType));
    if (!security->xenc__EncryptedKey->EncryptionMethod->ds__DigestMethod)
      return soap->error = SOAP_EOM;
    soap_default_ds__DigestMethodType(soap, security->xenc__EncryptedKey->EncryptionMethod->ds__DigestMethod);
    security->xenc__EncryptedKey->EncryptionMethod->ds__DigestMethod->Algorithm = (char*)ds_sha1URI;
  }
  else
  {
    security->xenc__EncryptedKey->EncryptionMethod->Algorithm = (char*)xenc_rsa15URI;
  }
  /* KeyInfo */
  security->xenc__EncryptedKey->ds__KeyInfo = (_ds__KeyInfo*)soap_malloc(soap, sizeof(_ds__KeyInfo));
  if (!security->xenc__EncryptedKey->ds__KeyInfo)
    return soap->error = SOAP_EOM;
  soap_default__ds__KeyInfo(soap, security->xenc__EncryptedKey->ds__KeyInfo); 
  /* allocate SecurityTokenReference */
  security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference = (_wsse__SecurityTokenReference*)soap_malloc(soap, sizeof(_wsse__SecurityTokenReference));
  if (!security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference)
    return soap->error = SOAP_EOM;
  soap_default__wsse__SecurityTokenReference(soap, security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference);
  if (issuer && serial)
  {
    security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->ds__X509Data = (struct ds__X509DataType*)soap_malloc(soap, sizeof(struct ds__X509DataType));
    if (!security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->ds__X509Data)
      return soap->error = SOAP_EOM;
    soap_default_ds__X509DataType(soap, security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->ds__X509Data);
    security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->ds__X509Data->X509IssuerSerial = (struct ds__X509IssuerSerialType*)soap_malloc(soap, sizeof(struct ds__X509IssuerSerialType));
    if (!security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->ds__X509Data->X509IssuerSerial)
      return soap->error = SOAP_EOM;
    soap_default_ds__X509IssuerSerialType(soap, security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->ds__X509Data->X509IssuerSerial);
    security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->ds__X509Data->X509IssuerSerial->X509IssuerName = soap_strdup(soap, issuer);
    security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->ds__X509Data->X509IssuerSerial->X509SerialNumber = soap_strdup(soap, serial);
  }
  else
  {
    /* allocate KeyIdentifier */
    security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->KeyIdentifier = (_wsse__KeyIdentifier*)soap_malloc(soap, sizeof(_wsse__KeyIdentifier));
    if (!security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->KeyIdentifier)
      return soap->error = SOAP_EOM;
    soap_default__wsse__KeyIdentifier(soap, security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->KeyIdentifier);
    /* populate KeyIdentifier */
    security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->KeyIdentifier->EncodingType = (char*)wsse_Base64BinaryURI;
    if (subjectkeyid)
    {
      security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->KeyIdentifier->ValueType = (char*)wsse_X509v3SubjectKeyIdentifierURI;
      security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->KeyIdentifier->__item = soap_s2base64(soap, (unsigned char*)subjectkeyid, NULL, strlen(subjectkeyid));
    }
    else if (cert)
    {
      unsigned char *der, *s;
      int derlen;
      security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->KeyIdentifier->ValueType = (char*)wsse_X509v3URI;
      /* determine the storage requirement */
      derlen = i2d_X509(cert, NULL);
      if (derlen < 0)
        return soap_wsse_fault(soap, wsse__InvalidSecurityToken, "Invalid certificate passed to soap_wsse_add_EncryptedKey_encrypt_only");
      /* use the gSOAP engine's look-aside buffer to temporarily hold the cert */
      if (soap_store_lab(soap, NULL, derlen))
        return SOAP_EOM;
      s = der = (unsigned char*)soap->labbuf;
      /* store in DER format */
      i2d_X509(cert, &s);
      security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->KeyIdentifier->__item = soap_s2base64(soap, der, NULL, derlen);
    }
  }
  /* CipherData */
  security->xenc__EncryptedKey->CipherData = (xenc__CipherDataType*)soap_malloc(soap, sizeof(struct xenc__CipherDataType));
  if (!security->xenc__EncryptedKey->CipherData)
    return soap->error = SOAP_EOM;
  soap_default_xenc__CipherDataType(soap, security->xenc__EncryptedKey->CipherData); 
  /* get the public key */
  pubk = X509_get_pubkey(cert);
  if (!pubk)
    return soap_wsse_fault(soap, wsse__InvalidSecurityToken, "Invalid certificate passed to soap_wsse_add_EncryptedKey_encrypt_only");
  /* start encryption engine, get the encrypted secret key */
  key = (unsigned char*)soap_malloc(soap, soap_mec_size(alg, pubk));
  if (!key)
    return soap->error = SOAP_EOM;
  if (data->mec)
    soap_mec_cleanup(soap, data->mec);
  else
    data->mec = (struct soap_mec_data*)SOAP_MALLOC(soap, sizeof(struct soap_mec_data));
  if (!data->mec)
    return soap->error = SOAP_EOM;
  if (soap_mec_begin(soap, data->mec, alg, pubk, key, &keylen))
  {
    EVP_PKEY_free(pubk);
    return soap->error;
  }
  EVP_PKEY_free(pubk);
  data->enco_alg = alg;
  data->enco_keyname = NULL;
  data->enco_key = key;
  data->enco_keylen = keylen;
  security->xenc__EncryptedKey->CipherData->CipherValue = soap_s2base64(soap, key, NULL, keylen);
  if (!security->xenc__EncryptedKey->CipherData->CipherValue)
    return soap->error = SOAP_EOM;
  data->encid = soap_strdup(soap, tags);
  if (!tags)
  {
    soap->omode |= SOAP_SEC_WSUID;
    if (soap_wsse_add_EncryptedKey_DataReferenceURI(soap, "#SOAP-ENV_Body"))
      return soap->error;
  }
  else
  {
    char *s, *t;
    size_t l = strlen(tags);
    /* make space to insert # to each id converted from a tag name */
    t = (char*)soap_malloc(soap, l + 2);
    if (!t)
      return soap->error = SOAP_EOM;
    *t = '#';
    soap_strcpy(t + 1, l + 1, tags);
    s = soap_wsse_ids(soap, t, '_');
    if (!s)
      return soap->error = SOAP_EOM;
    s++;
    for (;;)
    {
      t = strchr(s, ' ');
      if (t)
        *t = '\0';
      *--s = '#';
      if (soap_wsse_add_EncryptedKey_DataReferenceURI(soap, s))
        return soap->error;
      if (!t)
        break;
      s = t + 1;
      while (*s == ' ')
        s++;
    }
    if (soap_tagsearch(data->encid, "ds:Signature"))
    {
      /* support ds:Signature encryption only with HTTP chunking, otherwise content length is incorrect */
      if ((soap->omode & SOAP_IO) == SOAP_IO_BUFFER || (soap->omode & SOAP_IO) == SOAP_IO_FLUSH)
        soap->omode = (soap->omode & ~SOAP_IO) | SOAP_IO_CHUNK;
    }
  }
  soap->feltbegout = soap_wsse_element_begin_out;
  soap->feltendout = soap_wsse_element_end_out;
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_verify_EncryptedKey(struct soap *soap)
@brief Verifies the EncryptedKey header information (certificate validity requires soap->cafile to be set). Retrieves the decryption key from the token handler callback to decrypt the decryption key.
@param soap context
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_verify_EncryptedKey(struct soap *soap)
{
  struct soap_wsse_data *data;
  int keybase = 1;
  const char *keytype = NULL;
  const char *keydata = NULL;
  const char *keyname = NULL;
  const char *keyalgo = NULL;
  const char *keytemp = NULL;
  data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  if (!data)
    return soap_set_receiver_error(soap, "soap_wsse_verify_EncryptedKey", "Plugin not registered", SOAP_PLUGIN_ERROR);
#if 0 /* deprecated */
  _wsse__Security *security = soap_wsse_Security(soap);
  if (security)
  {
    /* SOAP header is complete when parsing the SOAP Body */
    if (!security->xenc__EncryptedKey)
      return SOAP_OK;
    if (!security->xenc__EncryptedKey->EncryptionMethod
     || !security->xenc__EncryptedKey->EncryptionMethod->Algorithm)
      return soap_wsse_fault(soap, wsse__InvalidSecurity, "Invalid Encryption algorithm or key");
    if (security->xenc__EncryptedKey->ds__KeyInfo)
    {
      if (security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference
       && security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->KeyIdentifier
       && security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->KeyIdentifier->ValueType
       && security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->KeyIdentifier->__item)
      {
        if (security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->KeyIdentifier->EncodingType)
          keybase = !strcmp(security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->KeyIdentifier->EncodingType, wsse_Base64BinaryURI);
        keytype = security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->KeyIdentifier->ValueType;
        keydata = security->xenc__EncryptedKey->ds__KeyInfo->wsse__SecurityTokenReference->KeyIdentifier->__item;
      }
      if (security->xenc__EncryptedKey->ds__KeyInfo->KeyName)
        keyname = security->xenc__EncryptedKey->ds__KeyInfo->KeyName;
      else if (security->xenc__EncryptedKey->ds__KeyInfo->X509Data
            && security->xenc__EncryptedKey->ds__KeyInfo->X509Data->X509SubjectName)
        keyname = security->xenc__EncryptedKey->ds__KeyInfo->X509Data->X509SubjectName;
      keyalgo = security->xenc__EncryptedKey->EncryptionMethod->Algorithm;
    }
    if (security->xenc__EncryptedKey->CipherData
     && security->xenc__EncryptedKey->CipherData->CipherValue)
      keytemp = security->xenc__EncryptedKey->CipherData->CipherValue;
    /* do not process EncryptedKey again */
    security->xenc__EncryptedKey = NULL;
  }
  else
#endif
  {
    /* SOAP header is incomplete while parsing the SOAP Header, so get EncryptedKey from DOM */
    if (soap->dom)
    {
      struct soap_dom_element *ek = soap_dom_find(soap->dom, soap->dom, xenc_URI, "EncryptedKey", 0);
      DBGFUN("soap_wsse_verify_EncryptedKey");
      if (ek)
      {
        struct soap_dom_element *ki;
        struct soap_dom_element *cd;
        struct soap_dom_element *elt;
        struct soap_dom_attribute *att;
        elt = soap_elt_get(ek, xenc_URI, "EncryptionMethod");
        if (elt)
        {
          att = soap_att_get(elt, NULL, "Algorithm");
          if (att)
            keyalgo = soap_att_get_text(att);
        }
        if (!keyalgo)
          return soap_wsse_fault(soap, wsse__InvalidSecurity, "Invalid Encryption algorithm or key");
        ki = soap_elt_get(ek, ds_URI, "KeyInfo");
        if (ki)
        {
          elt = soap_elt_get(ki, wsse_URI, "SecurityTokenReference");
          if (elt)
          {
            elt = soap_elt_get(elt, wsse_URI, "KeyIdentifier");
            if (elt)
            {
              att = soap_att_get(elt, NULL, "EncodingType");
              if (att && soap_att_get_text(att))
                keybase = !strcmp(soap_att_get_text(att), wsse_Base64BinaryURI);
              att = soap_att_get(elt, NULL, "ValueType");
              keytype = soap_att_get_text(att);
              keydata = soap_elt_get_text(elt);
            }
          }
          elt = soap_elt_get(ki, ds_URI, "KeyName");
          if (elt)
          {
            keyname = soap_elt_get_text(elt);
          }
          else
          {
            elt = soap_elt_get(ki, ds_URI, "X509Data");
            if (elt)
            {
              keyname = soap_elt_get_text(soap_elt_get(elt, ds_URI, "X509SubjectName"));
              elt = soap_elt_get(elt, ds_URI, "X509IssuerSerial");
              if (elt)
              {
                keyname = soap_elt_get_text(soap_elt_get(elt, ds_URI, "X509IssuerName"));
                if (keyname)
                {
                  const char *t = soap_elt_get_text(soap_elt_get(elt, ds_URI, "X509SerialNumber"));
                  if (t)
                  {
                    size_t n = strlen(keyname) + strlen(t) + 2;
                    char *s = (char*)soap_malloc(soap, n);
                    (SOAP_SNPRINTF_SAFE(s, n), "%s#%s", keyname, t);
                    keyname = s;
                  }
                }
              }
            }
          }
        }
        cd = soap_elt_get(ek, xenc_URI, "CipherData");
        if (cd)
        {
          elt = soap_elt_get(cd, xenc_URI, "CipherValue");
          keytemp = soap_elt_get_text(elt);
        }
      }
    }
  }
  if (keyalgo)
  {
    int keylen;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Verify EncryptedKey %s alg=%x\n", keyalgo, data->deco_alg));
    if (keytype && keydata)
    {
      if (!strcmp(keytype, wsse_X509v3URI))
      {
        X509 *cert = NULL;
#if (OPENSSL_VERSION_NUMBER >= 0x0090800fL)
        const unsigned char *der;
#else
        unsigned char *der;
#endif
        int derlen;
        if (keybase)
          der = (unsigned char*)soap_base642s(soap, keydata, NULL, 0, &derlen);
        else
          der = (unsigned char*)soap_hex2s(soap, keydata, NULL, 0, &derlen);
        if (!der)
          return soap_wsse_fault(soap, wsse__InvalidSecurity, "Invalid Encryption algorithm or key");
        cert = d2i_X509(&cert, &der, derlen);
        if (soap_wsse_verify_X509(soap, cert))
        {
          if (cert)
            X509_free(cert);
          return soap->error;
        }
        /* get the private key from subject name of cert, if not set */
        if (!data->deco_key && data->security_token_handler)
        {
          char buf[1024];
          X509_NAME_oneline(X509_get_subject_name(cert), buf, sizeof(buf)-1);
          DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Getting private key from cert name '%s' through security_token_handler callback\n", buf));
          data->deco_alg = SOAP_MEC_ENV_DEC_DES_CBC;
          data->deco_key = data->security_token_handler(soap, &data->deco_alg, buf, NULL, 0, &keylen);
          data->deco_keylen = 0;
        }
        if (cert)
          X509_free(cert);
      }
      else if (!data->deco_key && data->security_token_handler && !strcmp(keytype, wsse_X509v3SubjectKeyIdentifierURI))
      {
        const unsigned char *subjectkeyid;
        int subjectkeyidlen;
        if (keybase)
          subjectkeyid = (unsigned char*)soap_base642s(soap, keydata, NULL, 0, &subjectkeyidlen);
        else
          subjectkeyid = (unsigned char*)soap_hex2s(soap, keydata, NULL, 0, &subjectkeyidlen);
        /* get the private key from subject key id */
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Getting private key from key id '%s' through security_token_handler callback\n", subjectkeyid));
        data->deco_alg = SOAP_MEC_ENV_DEC_DES_CBC;
        data->deco_key = data->security_token_handler(soap, &data->deco_alg, NULL, subjectkeyid, subjectkeyidlen, &keylen);
        data->deco_keylen = 0;
      }
    }
    else if (!data->deco_key && data->security_token_handler && keyname)
    {
      /* get the private key from key name or subject name */
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Getting private key from name '%s' through security_token_handler callback\n", keyname));
      data->deco_alg = SOAP_MEC_ENV_DEC_DES_CBC;
      data->deco_key = data->security_token_handler(soap, &data->deco_alg, keyname, NULL, 0, &keylen);
      data->deco_keylen = 0;
    }
    /* start decryption */
    if (data->deco_key && keytemp)
    {
      int keylen;
      unsigned char *key = (unsigned char*)soap_base642s(soap, keytemp, NULL, 0, &keylen);
      if (!strcmp(keyalgo, xenc_rsaesURI))
        data->deco_alg |= SOAP_MEC_OAEP;
      else if (strcmp(keyalgo, xenc_rsa15URI))
        return soap_wsse_fault(soap, wsse__InvalidSecurity, "Invalid Encryption algorithm or key");
      if (data->mec)
        soap_mec_cleanup(soap, data->mec);
      else
        data->mec = (struct soap_mec_data*)SOAP_MALLOC(soap, sizeof(struct soap_mec_data));
      if (!data->mec)
        return soap->error = SOAP_EOM;
      if (soap_mec_begin(soap, data->mec, data->deco_alg, (SOAP_MEC_KEY_TYPE*)data->deco_key, key, &keylen))
        return soap_wsse_fault(soap, wsse__FailedCheck, NULL);
    }
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Verified EncryptedKey alg=%x\n", data->deco_alg));
  }
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn void soap_wsse_delete_EncryptedKey(struct soap *soap)
@brief Deletes EncryptedKey header element.
@param soap context
*/
SOAP_FMAC1
void
SOAP_FMAC2
soap_wsse_delete_EncryptedKey(struct soap *soap)
{
  _wsse__Security *security = soap_wsse_Security(soap);
  DBGFUN("soap_wsse_delete_EncryptedKey");
  if (security)
    security->xenc__EncryptedKey = NULL;
}

/******************************************************************************/

/**
@fn xenc__EncryptedKeyType* soap_wsse_EncryptedKey(struct soap *soap)
@brief Returns EncryptedKey header element if present.
@param soap context
@return xenc__EncryptedKeyType object or NULL
*/
SOAP_FMAC1
struct xenc__EncryptedKeyType *
SOAP_FMAC2
soap_wsse_EncryptedKey(struct soap *soap)
{
  _wsse__Security *security = soap_wsse_Security(soap);
  if (security)
    return security->xenc__EncryptedKey;
  return NULL;
}

/******************************************************************************\
 *
 *      wsse:Security/xenc:EncryptedKey/ReferenceList/DataReference
 *
\******************************************************************************/

/**
@fn int soap_wsse_add_EncryptedKey_DataReferenceURI(struct soap *soap, const char *URI)
@brief Adds a DataReference URI to the EncryptedKey header element.
@param soap context
@param[in] URI value of the URI ID
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_EncryptedKey_DataReferenceURI(struct soap *soap, const char *URI)
{
  _wsse__Security *security = soap_wsse_add_Security(soap);
  _xenc__ReferenceList *ref;
  int k, n = 0;
  DBGFUN1("soap_wsse_add_EncryptedKey_DataReferenceURI", "URI=%s", URI?URI:"");
  if (!security->xenc__EncryptedKey)
  {
    security->xenc__EncryptedKey = (xenc__EncryptedKeyType*)soap_malloc(soap, sizeof(xenc__EncryptedKeyType));
    if (!security->xenc__EncryptedKey)
      return soap->error = SOAP_EOM;
    soap_default_xenc__EncryptedKeyType(soap, security->xenc__EncryptedKey); 
  }
  if (!security->xenc__EncryptedKey->ReferenceList)
  {
    security->xenc__EncryptedKey->ReferenceList = (struct _xenc__ReferenceList*)soap_malloc(soap, sizeof(struct _xenc__ReferenceList));
    if (!security->xenc__EncryptedKey->ReferenceList)
      return soap->error = SOAP_EOM;
    soap_default__xenc__ReferenceList(soap, security->xenc__EncryptedKey->ReferenceList);
  }
  ref = security->xenc__EncryptedKey->ReferenceList;
  k = ref->__size_ReferenceList++;
  /* need to increase space? */
  if (k < 0)
    return soap->error = SOAP_EOM;
  if (k == 0)
    n = 1;
  else if (k >= 1 && (k & (k - 1)) == 0)
    n = 2 * k;
  /* yes we do */
  if (n)
  {
    struct __xenc__union_ReferenceList *tmp = (struct __xenc__union_ReferenceList*)soap_malloc(soap, n * sizeof(struct __xenc__union_ReferenceList));
    int i;
    if (!tmp)
      return soap->error = SOAP_EOM;
    for (i = 0; i < k; i++)
      tmp[i] = ref->__union_ReferenceList[i];
    security->xenc__EncryptedKey->ReferenceList->__union_ReferenceList = tmp;
    ref = security->xenc__EncryptedKey->ReferenceList;
  }
  /* add entry */
  soap_default___xenc__union_ReferenceList(soap, &ref->__union_ReferenceList[k]);
  ref->__union_ReferenceList[k].DataReference = (struct xenc__ReferenceType*)soap_malloc(soap, sizeof(struct xenc__ReferenceType));
  if (!ref->__union_ReferenceList[k].DataReference)
    return soap->error = SOAP_EOM;
  soap_default_xenc__ReferenceType(soap, ref->__union_ReferenceList[k].DataReference);
  ref->__union_ReferenceList[k].DataReference->URI = soap_strdup(soap, URI);
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_add_DataReferenceURI(struct soap *soap, const char *URI)
@brief Adds a DataReference URI to the Security header element.
@param soap context
@param[in] URI value of the URI ID
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_DataReferenceURI(struct soap *soap, const char *URI)
{
  _wsse__Security *security = soap_wsse_add_Security(soap);
  _xenc__ReferenceList *ref;
  int k, n = 0;
  DBGFUN1("soap_wsse_add_DataReferenceURI", "URI=%s", URI?URI:"");
  /* initial alloc */
  if (!security->xenc__ReferenceList)
  {
    security->xenc__ReferenceList = (struct _xenc__ReferenceList*)soap_malloc(soap, sizeof(struct _xenc__ReferenceList));
    if (!security->xenc__ReferenceList)
      return soap->error = SOAP_EOM;
    soap_default__xenc__ReferenceList(soap, security->xenc__ReferenceList);
  }
  ref = security->xenc__ReferenceList;
  k = ref->__size_ReferenceList++;
  /* need to increase space? */
  if (k < 0)
    return soap->error = SOAP_EOM;
  if (k == 0)
    n = 1;
  else if (k >= 1 && (k & (k - 1)) == 0)
    n = 2 * k;
  /* yes we do */
  if (n)
  {
    struct __xenc__union_ReferenceList *tmp = (struct __xenc__union_ReferenceList*)soap_malloc(soap, n * sizeof(struct __xenc__union_ReferenceList));
    int i;
    if (!tmp)
      return soap->error = SOAP_EOM;
    for (i = 0; i < k; i++)
      tmp[i] = ref->__union_ReferenceList[i];
    security->xenc__ReferenceList->__union_ReferenceList = tmp;
    ref = security->xenc__ReferenceList;
  }
  /* add entry */
  soap_default___xenc__union_ReferenceList(soap, &ref->__union_ReferenceList[k]);
  ref->__union_ReferenceList[k].DataReference = (struct xenc__ReferenceType*)soap_malloc(soap, sizeof(struct xenc__ReferenceType));
  if (!ref->__union_ReferenceList[k].DataReference)
    return soap->error = SOAP_EOM;
  soap_default_xenc__ReferenceType(soap, ref->__union_ReferenceList[k].DataReference);
  ref->__union_ReferenceList[k].DataReference->URI = soap_strdup(soap, URI);
  return SOAP_OK;
}

/******************************************************************************\
 *
 *      xenc:EncryptedData/ds:KeyInfo/Keyname
 *
\******************************************************************************/

/**
@fn int soap_wsse_add_EncryptedData_KeyInfo_KeyName(struct soap *soap, const char *keyname)
@brief Adds EncryptedData/ds:KeyInfo/Keyname elements.
@param soap context
@param[in] keyname name of the key
@return SOAP_OK or error code

This function adds the name of the key to each EncryptedData element to
identify the shared secret key used for encryption.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_add_EncryptedData_KeyInfo_KeyName(struct soap *soap, const char *keyname)
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  DBGFUN("soap_wsse_add_EncryptedData_KeyInfo_KeyName");
  if (!data)
    return soap_set_receiver_error(soap, "soap_wsse_add_EncryptedData_KeyInfo_KeyName", "Plugin not registered", SOAP_PLUGIN_ERROR);
  data->enco_keyname = soap_strdup(soap, keyname);
  return SOAP_OK;
}

/******************************************************************************\
 *
 *      wsse:Security/saml1:Assertion
 *
\******************************************************************************/

#ifdef SOAP_NAMESPACE_OF_saml1

/**
@fn saml1__AssertionType *soap_wsse_add_saml1(struct soap *soap, const char *wsuId)
@brief Adds SAML 1.0 Assertion to the wsse:Security header block, default initialized with wsu:Id set to enable signing of the assertion.
@param soap context
@param[in] wsuId string with unique ID value for signing or NULL to omit signing
@return pointer to assertion or NULL when failed to allocate
*/
SOAP_FMAC1
saml1__AssertionType *
SOAP_FMAC2
soap_wsse_add_saml1(struct soap *soap, const char *wsuId)
{
  saml1__AssertionType *assertion = NULL;
  _wsse__Security *security = soap_wsse_add_Security(soap);
  if (security)
  {
    assertion = (saml1__AssertionType*)soap_malloc(soap, sizeof(saml1__AssertionType));
    if (assertion)
    {
      soap_default_saml1__AssertionType(soap, assertion);
      assertion->MajorVersion = (char*)"1";
      assertion->MinorVersion = (char*)"0";
      assertion->wsu__Id = soap_strdup(soap, wsuId);
    }
  }
  return assertion;
}

/******************************************************************************/

/**
@fn int soap_wsse_sign_saml1(struct soap *soap, saml1__Assertion assertion, int alg, const void *key, int keylen, X509 *cert)
@brief Signs a SAML 2.0 assertion.
@param soap context
@param[in] alg is the signature algorithm, such as SOAP_SMD_HMAC_SHA1/224/256/384/512, SOAP_SMD_SIGN_DSA_SHA1/256, SOAP_SMD_SIGN_RSA_SHA1/224/256/384/512, or SOAP_SMD_SIGN_ECDSA_SHA1/224/256/384/512
@param[in] key is the HMAC secret key or DSA/RSA/ECDSA private EVP_PKEY
@param[in] keylen is the HMAC key length
@param[in] cert points to the X509 certificate
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_sign_saml1(struct soap *soap, saml1__AssertionType *assertion, int alg, const void *key, int keylen, X509 *cert)
{
  _wsse__Security *security = soap_wsse_Security(soap);
  soap_mode omode = soap->omode;
  short version = soap->version;
  int err = SOAP_OK;
  if (!assertion)
    return SOAP_OK;
  soap_set_omode(soap, SOAP_XML_CANONICAL | SOAP_XML_CANONICAL_NA);
  soap_clr_omode(soap, SOAP_XML_INDENT);
  soap->version = 0;
  soap->encodingStyle = NULL;
  if (!assertion->AssertionID)
    assertion->AssertionID = soap_strdup(soap, soap_rand_uuid(soap, "_"));
  assertion->wsu__Id = assertion->AssertionID;
  if (soap_wsse_sign(soap, alg, key, keylen)
   || soap_wsse_sign_only(soap, assertion->AssertionID)
   || soap_begin_count(soap))
  {
    err = soap->error;
  }
  else
  {
    soap_strcpy(soap->id, sizeof(soap->id), assertion->AssertionID);
    soap->event = SOAP_SEC_BEGIN;
    if (soap_out_saml1__AssertionType(soap, "saml1:Assertion", 0, assertion, "")
     || soap_end_count(soap)
     || soap_wsse_add_KeyInfo_X509Certificate(soap, cert))
      err = soap->error;
  }
  assertion->ds__Signature = soap_wsse_Signature(soap);
  soap_wsse_delete_Security(soap);
  soap->header->wsse__Security = security;
  soap->omode = omode;
  soap->version = version;
  return err;
}

/******************************************************************************/

/**
@fn saml1__AssertionType *soap_wsse_get_saml1(struct soap *soap)
@brief Returns SAML 1.0 Assertion in the wsse:Security header block, if present.
@param soap context
@return pointer to assertion or NULL
*/
SOAP_FMAC1
saml1__AssertionType *
SOAP_FMAC2
soap_wsse_get_saml1(struct soap *soap)
{
  _wsse__Security *security = soap_wsse_Security(soap);
  if (security)
    return security->saml1__Assertion;
  return NULL;
}

/******************************************************************************/

/**
@fn int soap_wsse_verify_saml1(struct soap *soap, saml1__AssertionType *assertion)
@brief Verifies the SAML 1.0 Assertion with its enveloped signature, requires soap->cafile to be set.
@param soap context
@param assertion SAML 1.0 assertion to verify
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_verify_saml1(struct soap *soap, saml1__AssertionType *assertion)
{
  struct soap_dom_element *elt = soap->dom;
  soap_mode omode = soap->omode;
  short version = soap->version;
  int err = SOAP_OK;
  _ds__Signature *signature = assertion->ds__Signature;
  assertion->ds__Signature = NULL;
  soap_set_omode(soap, SOAP_XML_DOM | SOAP_XML_CANONICAL_NA);
  soap_clr_omode(soap, SOAP_XML_INDENT);
  soap->version = 0;
  soap->encodingStyle = NULL;
  soap->dom = NULL;
  if (soap_begin_send(soap)
   || soap_put_saml1__AssertionType(soap, assertion, "saml1:Assertion", "")
   || soap_end_send(soap)
   || soap_wsse_verify_with_signature(soap, signature))
    err = soap->error;
  soap->version = version;
  soap->omode = omode;
  soap->dom = elt;
  assertion->ds__Signature = signature;
  return err;
}

#endif

/******************************************************************************\
 *
 *      wsse:Security/saml2:Assertion
 *
\******************************************************************************/

#ifdef SOAP_NAMESPACE_OF_saml2

/**
@fn saml2__AssertionType *soap_wsse_add_saml2(struct soap *soap, const char *wsuId)
@brief Adds SAML 2.0 Assertion to the wsse:Security header block, default initialized with wsu:Id set to enable signing of the assertion.
@param soap context
@param[in] wsuId string with unique ID value for signing or NULL to omit signing
@return pointer to assertion or NULL when failed to allocate
*/
SOAP_FMAC1
saml2__AssertionType *
SOAP_FMAC2
soap_wsse_add_saml2(struct soap *soap, const char *wsuId)
{
  saml2__AssertionType *assertion = NULL;
  _wsse__Security *security = soap_wsse_add_Security(soap);
  if (security)
  {
    assertion = (saml2__AssertionType*)soap_malloc(soap, sizeof(saml2__AssertionType));
    if (assertion)
    {
      soap_default_saml2__AssertionType(soap, assertion);
      assertion->Version = (char*)"2.0";
      assertion->wsu__Id = soap_strdup(soap, wsuId);
    }
  }
  return assertion;
}

/******************************************************************************/

/**
@fn int soap_wsse_sign_saml2(struct soap *soap, saml2__Assertion assertion, int alg, const void *key, int keylen, X509 *cert)
@brief Signs a SAML 2.0 assertion.
@param soap context
@param[in] alg is the signature algorithm, such as SOAP_SMD_HMAC_SHA1/224/256/384/512, SOAP_SMD_SIGN_DSA_SHA1/256, SOAP_SMD_SIGN_RSA_SHA1/224/256/384/512, or SOAP_SMD_SIGN_ECDSA_SHA1/224/256/384/512
@param[in] key is the HMAC secret key or DSA/RSA/ECDSA private EVP_PKEY
@param[in] keylen is the HMAC key length
@param[in] cert points to the X509 certificate
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_sign_saml2(struct soap *soap, saml2__AssertionType *assertion, int alg, const void *key, int keylen, X509 *cert)
{
  _wsse__Security *security = soap_wsse_Security(soap);
  soap_mode omode = soap->omode;
  short version = soap->version;
  int err = SOAP_OK;
  if (!assertion)
    return SOAP_OK;
  soap_set_omode(soap, SOAP_XML_CANONICAL | SOAP_XML_CANONICAL_NA);
  soap_clr_omode(soap, SOAP_XML_INDENT);
  soap->version = 0;
  soap->encodingStyle = NULL;
  if (!assertion->ID)
    assertion->ID = soap_strdup(soap, soap_rand_uuid(soap, "_"));
  if (soap_wsse_sign(soap, alg, key, keylen)
   || soap_wsse_sign_only(soap, assertion->ID)
   || soap_begin_count(soap))
  {
    err = soap->error;
  }
  else
  {
    soap_strcpy(soap->id, sizeof(soap->id), assertion->ID);
    soap->event = SOAP_SEC_BEGIN;
    if (soap_out_saml2__AssertionType(soap, "saml2:Assertion", 0, assertion, "")
     || soap_end_count(soap)
     || soap_wsse_add_KeyInfo_X509Certificate(soap, cert))
      err = soap->error;
  }
  assertion->ds__Signature = soap_wsse_Signature(soap);
  soap_wsse_delete_Security(soap);
  soap->header->wsse__Security = security;
  soap->omode = omode;
  soap->version = version;
  return err;
}

/******************************************************************************/

/**
@fn saml2__AssertionType *soap_wsse_get_saml2(struct soap *soap)
@brief Returns SAML 2.0 Assertion in the wsse:Security header block, if present.
@param soap context
@return pointer to assertion or NULL
*/
SOAP_FMAC1
saml2__AssertionType *
SOAP_FMAC2
soap_wsse_get_saml2(struct soap *soap)
{
  _wsse__Security *security = soap_wsse_Security(soap);
  if (security)
    return security->saml2__Assertion;
  return NULL;
}

/******************************************************************************/

/**
@fn int soap_wsse_verify_saml2(struct soap *soap, saml2__AssertionType *assertion)
@brief Verifies the SAML 2.0 Assertion with its enveloped signature, requires soap->cafile to be set.
@param soap context
@param assertion SAML 2.0 assertion to verify
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_verify_saml2(struct soap *soap, saml2__AssertionType *assertion)
{
  struct soap_dom_element *elt = soap->dom;
  soap_mode omode = soap->omode;
  short version = soap->version;
  int err = SOAP_OK;
  _ds__Signature *signature = assertion->ds__Signature;
  assertion->ds__Signature = NULL;
  soap_set_omode(soap, SOAP_XML_DOM | SOAP_XML_CANONICAL_NA);
  soap_clr_omode(soap, SOAP_XML_INDENT);
  soap->version = 0;
  soap->encodingStyle = NULL;
  soap->dom = NULL;
  if (soap_begin_send(soap)
   || soap_put_saml2__AssertionType(soap, assertion, "saml2:Assertion", "")
   || soap_end_send(soap)
   || soap_wsse_verify_with_signature(soap, signature))
    err = soap->error;
  soap->version = version;
  soap->omode = omode;
  soap->dom = elt;
  assertion->ds__Signature = signature;
  return err;
}

#endif

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
  DBGFUN2("soap_wsse_fault", "fault=%s", code ? code : "(null)", "detail=%s", detail ? detail : "(null)");
  /* remove incorrect or incomplete Security header */
  soap_wsse_delete_Security(soap);
  /* populate the SOAP Fault as per WS-Security spec */
  /* detail = NULL; */ /* uncomment when detail text is to be omitted from SOAP Fault messages */
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
 *      Digest authentication session management
 *
\******************************************************************************/

/**
@fn static int soap_wsse_session_verify(struct soap *soap, const char hash[SOAP_SMD_SHA1_SIZE], const char *created, const char *nonce)
@brief Verifies and updates the digest, nonce, and creation time against the digest authentication session database to prevent replay attacks.
@param soap context
@param[in] hash binary digest value of PasswordDigest
@param[in] created string
@param[in] nonce string (base64)
@return SOAP_OK or SOAP_FAULT
*/
static int
soap_wsse_session_verify(struct soap *soap, const char hash[SOAP_SMD_SHA1_SIZE], const char *created, const char *nonce)
{
  struct soap_wsse_session *session;
  time_t expired, now = time(NULL);
  DBGFUN("soap_wsse_session_verify");
  soap_s2dateTime(soap, created, &expired);
  /* creation time in the future? */
  if (expired > now + SOAP_WSSE_CLKSKEW)
    return soap_wsse_fault(soap, wsse__FailedAuthentication, "Authorization request in future");
  expired += SOAP_WSSE_NONCETIME;
  /* expired? */
  if (expired <= now)
    return soap_wsse_fault(soap, wsse__FailedAuthentication, "Authentication expired");
  /* purge expired messages, but don't do this all the time to improve efficiency */
  if (now % 10 == 0)
    soap_wsse_session_cleanup(soap);
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Verifying session nonce=%s\n", nonce));
  /* enter mutex to check and update session */
  MUTEX_LOCK(soap_wsse_session_lock);
  for (session = soap_wsse_session; session; session = session->next)
  {
    if (!memcmp(session->hash, hash, SOAP_SMD_SHA1_SIZE) && !strcmp(session->nonce, nonce))
      break;
  }
  /* if not found, allocate new session data */
  if (!session)
  {
    size_t l = strlen(nonce);
    session = (struct soap_wsse_session*)malloc(sizeof(struct soap_wsse_session) + l);
    if (session)
    {
      session->next = soap_wsse_session;
      session->expired = expired;
      (void)soap_memcpy((void*)session->hash, sizeof(session->hash), (const void*)hash, SOAP_SMD_SHA1_SIZE);
      soap_strcpy(session->nonce, l + 1, nonce); /* nonce[0..l] allocated as nonce[1] + l bytes to fit the last \0 */
      soap_wsse_session = session;
    }
    session = NULL;
  }
  /* exit mutex */
  MUTEX_UNLOCK(soap_wsse_session_lock);
  /* if replay attack, return non-descript failure */
  if (session)
    return soap_wsse_fault(soap, wsse__FailedAuthentication, NULL);
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn static void soap_wsse_session_cleanup(struct soap *soap)
@brief Removes expired authentication data from the digest authentication session database.
@param soap context
*/
static void
soap_wsse_session_cleanup(struct soap *soap)
{
  struct soap_wsse_session **session;
  time_t now = time(NULL);
  DBGFUN("soap_wsse_session_cleanup");
  /* enter mutex to purge expired session data */
  MUTEX_LOCK(soap_wsse_session_lock);
  session = &soap_wsse_session;
  while (*session)
  {
    if ((*session)->expired < now)
    {
      struct soap_wsse_session *p = *session;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Deleting session nonce=%s\n", p->nonce));
      *session = p->next;
      free(p);
    }
    else
      session = &(*session)->next;
  }
  /* exit mutex */
  MUTEX_UNLOCK(soap_wsse_session_lock);
}

/******************************************************************************\
 *
 *      Calculate SHA1(created, nonce, password) digest
 *
\******************************************************************************/

/**
@fn static void calc_digest(struct soap *soap, const char *created, const char *nonce, int noncelen, const char *password, char hash[SOAP_SMD_SHA1_SIZE])
@brief Calculates digest value SHA1(created, nonce, password).
@param soap context
@param[in] created string (XSD dateTime format)
@param[in] nonce value
@param[in] noncelen length of nonce value
@param[in] password string
@param[out] hash SHA1 digest
*/
static void
calc_digest(struct soap *soap, const char *created, const char *nonce, int noncelen, const char *password, char hash[SOAP_SMD_SHA1_SIZE])
{
  struct soap_smd_data context;
  /* use smdevp engine */
  soap_smd_init(soap, &context, SOAP_SMD_DGST_SHA1, NULL, 0);
  soap_smd_update(soap, &context, nonce, noncelen);
  soap_smd_update(soap, &context, created, strlen(created));
  soap_smd_update(soap, &context, password, strlen(password));
  soap_smd_final(soap, &context, hash, NULL);
}

/******************************************************************************\
 *
 *      Calculate randomized hex nonce
 *
\******************************************************************************/

/**
@fn static void soap_wsse_rand_nonce(char *nonce, size_t noncelen)
@brief Calculates randomized nonce.
@param[out] nonce value [0..noncelen-1]
@param[in] noncelen length of nonce must be multiple of 4
*/
SOAP_FMAC1
void
SOAP_FMAC2
soap_wsse_rand_nonce(char *nonce, size_t noncelen)
{
  size_t i;
  soap_int32 r = (soap_int32)time(NULL);
  (void)soap_memcpy((void*)nonce, 4, (const void*)&r, 4);
  for (i = 4; i < noncelen; i += 4)
  {
    r = soap_random;
    (void)soap_memcpy((void*)(nonce + i), 4, (const void*)&r, 4);
  }
}

/******************************************************************************\
 *
 *      WS-SecureConversation P_MD5, P_SHA1 and P_SHA256
 *
\******************************************************************************/

/**
@fn int soap_pmd5(struct soap *soap, const char *hmac_key, size_t hmac_key_len, const char *secret, size_t secretlen, char *pmd5, size_t pmd5len)
@brief Computes PMD5(hmac_key[0..hmac_key_len-1], secret[0..secretlen-1], pmd5[0..pmd5len-1]).
@param soap context
@param[in] hmac_key HMAC key (client secret) in raw bytes
@param[in] hmac_key_len HMAC key length
@param[in] secret seed (server secret) raw bytes
@param[in] secretlen number of bytes
@param[out] pmd5 points to pmd5 raw bytes to fill with result
@param[in] pmd5len number of bytes to fill pmd5
@return SOAP_OK or SOAP_EOM

To compute PMD5 with base64 input and output a base64 encoded pmd5[0..pmd5len-1]:
@code
    int pmd5len = 32; // or greater
    int n, m;
    const char *client_secret = soap_base642s(soap, client_secret_base64, NULL, 0, &n);
    const char *server_secret = soap_base642s(soap, server_secret_base64, NULL, 0, &m);
    char pmd5[pmd5len];
    char *pmd5_base64;
    if (soap_pmd5(soap, client_secret, n, server_secret, m, pmd5, pmd5len))
      .. error // insufficient memory
    pmd5_base64 = soap_s2base64(soap, (unsigned char*)pmd5, NULL, pmd5len);
@endcode
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_pmd5(struct soap *soap, const char *hmac_key, size_t hmac_key_len, const char *secret, size_t secretlen, char *pmd5, size_t pmd5len)
{
  char HA[SOAP_SMD_MD5_SIZE];
  char temp[SOAP_SMD_MD5_SIZE];
  return soap_p_hash(soap, hmac_key, hmac_key_len, secret, secretlen, SOAP_SMD_HMAC_MD5, HA, SOAP_SMD_MD5_SIZE, temp, pmd5, pmd5len);
}

/**
@fn int soap_psha1(struct soap *soap, const char *hmac_key, size_t hmac_key_len, const char *secret, size_t secretlen, char *psha1, size_t psha1len)
@brief Computes PSHA1(hmac_key[0..hmac_key_len-1], secret[0..secretlen-1], psha1[0..psha1len-1]).
@param soap context
@param[in] hmac_key HMAC key (client secret) in raw bytes
@param[in] hmac_key_len HMAC key length
@param[in] secret seed (server secret)
@param[in] secretlen number of bytes
@param[out] psha1 points to psha1 raw bytes to fill with result
@param[in] psha1len number of bytes to fill psha1
@return SOAP_OK or SOAP_EOM

To compute PSHA1 with base64 input and output a base64 encoded psha1[0..psha1len-1]:
@code
    int psha1len = 32; // or greater
    int n, m;
    const char *client_secret = soap_base642s(soap, client_secret_base64, NULL, 0, &n);
    const char *server_secret = soap_base642s(soap, server_secret_base64, NULL, 0, &m);
    char psha1[psha1len];
    char *psha1_base64;
    if (soap_psha1(soap, client_secret, n, server_secret, m, psha1, psha1len))
      .. error // insufficient memory
    psha1_base64 = soap_s2base64(soap, (unsigned char*)psha1, NULL, psha1len);
@endcode
*/

SOAP_FMAC1
int
SOAP_FMAC2
soap_psha1(struct soap *soap, const char *hmac_key, size_t hmac_key_len, const char *secret, size_t secretlen, char *psha1, size_t psha1len)
{
  char HA[SOAP_SMD_SHA1_SIZE];
  char temp[SOAP_SMD_SHA1_SIZE];
  return soap_p_hash(soap, hmac_key, hmac_key_len, secret, secretlen, SOAP_SMD_HMAC_SHA1, HA, SOAP_SMD_SHA1_SIZE, temp, psha1, psha1len);
}

/**
@fn int soap_psha256(struct soap *soap, const char *hmac_key, size_t hmac_key_len, const char *secret, size_t secretlen, char *psha256, size_t psha256len)
@brief Computes PSHA256(hmac_key[0..hmac_key_len-1], secret[0..secretlen-1], psha256[0..psha256len-1]).
@param soap context
@param[in] hmac_key HMAC key (client secret) in raw bytes
@param[in] hmac_key_len HMAC key length
@param[in] secret seed (server secret) raw bytes
@param[in] secretlen number of bytes
@param[out] psha256 points to psha256 raw bytes to fill with result
@param[in] psha256len number of bytes to fill psha256
@return SOAP_OK or SOAP_EOM

To compute PSHA256 with base64 input and output a base64 encoded psha256[0..psha256len-1]:
@code
    int psha256len = 32; // or greater
    int n, m;
    const char *client_secret = soap_base642s(soap, client_secret_base64, NULL, 0, &n);
    const char *server_secret = soap_base642s(soap, server_secret_base64, NULL, 0, &m);
    char psha256[psha256len];
    char *psha256_base64;
    if (soap_psha256(soap, client_secret, n, server_secret, m, psha256, psha256len))
      .. error // insufficient memory
    psha256_base64 = soap_s2base64(soap, (unsigned char*)psha256, NULL, psha256len);
@endcode
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_psha256(struct soap *soap, const char *hmac_key, size_t hmac_key_len, const char *secret, size_t secretlen, char *psha256, size_t psha256len)
{
  char HA[SOAP_SMD_SHA256_SIZE];
  char temp[SOAP_SMD_SHA256_SIZE];
  return soap_p_hash(soap, hmac_key, hmac_key_len, secret, secretlen, SOAP_SMD_HMAC_SHA256, HA, SOAP_SMD_SHA256_SIZE, temp, psha256, psha256len);
}

/**
@fn static int soap_p_hash(struct soap *soap, const char *hmac_key, size_t hmac_key_len, const char *secret, size_t secretlen, int alg, char HA[], size_t HA_len, char temp[], char *phash, size_t phashlen)
@brief Computes hash(hmac_key[0..hmac_key_len-1], secret[0..secretlen-1], HA[0..HA_len-1]) given algorithm alg.
@param soap context
@param[in] hmac_key HMAC key (client secret) raw bytes
@param[in] hmac_key_len HMAC key length
@param[in] secret seed (server secret) raw bytes
@param[in] secretlen number of bytes
@param[in] alg hash algorithm
@param HA buffer to contain hash (internally used)
@param HA_len buffer length to contain hash (internally used)
@param temp buffer to contain hash (internally used)
@param[out] phash points to phash raw bytes to fill with result
@param[in] phashlen number of bytes to fill phash
@return SOAP_OK or SOAP_EOM
*/
static int soap_p_hash(struct soap *soap, const char *hmac_key, size_t hmac_key_len, const char *secret, size_t secretlen, int alg, char HA[], size_t HA_len, char temp[], char *phash, size_t phashlen)
{
  size_t i;
  char *buffer = (char*)SOAP_MALLOC(soap, HA_len + secretlen);
  if (!buffer)
    return soap->error = SOAP_EOM;
  i = 0;
  while (i < phashlen)
  {
    struct soap_smd_data context;
    size_t j;
    soap_smd_init(soap, &context, alg, (void*)hmac_key, hmac_key_len);
    if (i == 0)
      soap_smd_update(soap, &context, secret, secretlen);
    else
      soap_smd_update(soap, &context, HA, HA_len);
    soap_smd_final(soap, &context, HA, NULL);
    (void)soap_memcpy((void*)buffer, HA_len + secretlen, (void*)HA, HA_len);
    (void)soap_memcpy((void*)(buffer + HA_len), secretlen, (void*)secret, secretlen);
    soap_smd_init(soap, &context, alg, (void*)hmac_key, hmac_key_len);
    soap_smd_update(soap, &context, buffer, HA_len + secretlen);
    soap_smd_final(soap, &context, temp, NULL);
    j = 0;
    while (j < HA_len && i < phashlen)
      phash[i++] = temp[j++];
  }
  SOAP_FREE(soap, buffer);
  return SOAP_OK;
}

/******************************************************************************\
 *
 *      Plugin registry functions
 *
\******************************************************************************/

/**
@fn int soap_wsse(struct soap *soap, struct soap_plugin *p, void *arg)
@brief Plugin registry function, used with soap_register_plugin.
@param soap context
@param[in,out] p plugin created in registry
@param[in] arg passed from soap_register_plugin_arg is an optional security token handler callback
@return SOAP_OK
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse(struct soap *soap, struct soap_plugin *p, void *arg)
{
  DBGFUN("soap_wsse");
  p->id = soap_wsse_id;
  p->data = (void*)SOAP_MALLOC(soap, sizeof(struct soap_wsse_data));
  p->fcopy = soap_wsse_copy;
  p->fdelete = soap_wsse_delete;
  if (!p->data)
    return SOAP_EOM;
  if (soap_wsse_init(soap, (struct soap_wsse_data*)p->data, (const void *(*)(struct soap*, int*, const char*, const unsigned char*, int, int*))arg))
  {
    SOAP_FREE(soap, p->data);
    return SOAP_EOM;
  }
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_init(struct soap *soap, struct soap_wsse_data *data, const void *(*arg)(struct soap*, int*, const char*, const unsigned char *keyid, int keyidlen, int*))
@brief Initializes plugin data.
@param soap context
@param[in,out] data plugin data
@param arg security token handler callback
@return SOAP_OK
*/
static int
soap_wsse_init(struct soap *soap, struct soap_wsse_data *data, const void *(*arg)(struct soap*, int *alg, const char *keyname, const unsigned char *keyid, int keyidlen, int *keylen))
{
  DBGFUN("soap_wsse_init");
  if (!data)
    return soap->error = SOAP_EOM;
  data->sigid = NULL;
  data->encid = NULL;
  data->prefixlist = NULL;
  data->sign_alg = SOAP_SMD_NONE;
  data->sign_key = NULL;
  data->sign_keylen = 0;
  data->vrfy_alg = SOAP_SMD_NONE;
  data->vrfy_key = NULL;
  data->vrfy_keylen = 0;
  data->enco_alg = SOAP_MEC_NONE;
  data->enco_keyname = NULL;
  data->enco_key = NULL;
  data->enco_keylen = 0;
  data->deco_alg = SOAP_MEC_NONE;
  data->deco_key = NULL;
  data->deco_keylen = 0;
  data->digest = NULL;
  data->fpreparesend = NULL;
  data->fpreparefinalsend = NULL;
  data->fpreparefinalrecv = NULL;
  data->mec = NULL;
  data->store = NULL;
  data->security_token_handler = arg;
  soap->feltbegin = soap_wsse_element_begin_in; 
  soap->feltendin = soap_wsse_element_end_in;
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src)
@brief Copies plugin data to localize plugin data for threads.
@param soap context
@param[out] dst target plugin
@param[in] src source plugin
@return SOAP_OK
*/
static int
soap_wsse_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src)
{
  DBGFUN("soap_wsse_copy");
  *dst = *src;
  dst->data = (void*)SOAP_MALLOC(soap, sizeof(struct soap_wsse_data));
  return soap_wsse_init(soap, (struct soap_wsse_data*)dst->data, ((struct soap_wsse_data*)src->data)->security_token_handler);
}

/******************************************************************************/

/**
@fn void soap_wsse_delete(struct soap *soap, struct soap_plugin *p)
@brief Deletes plugin data.
@param soap context
@param[in,out] p plugin
@return SOAP_OK
*/
static void
soap_wsse_delete(struct soap *soap, struct soap_plugin *p)
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  (void)p;
  DBGFUN("soap_wsse_delete");
  if (data)
  {
    if (data->prefixlist)
      SOAP_FREE(soap, data->prefixlist);
    soap_wsse_preparecleanup(soap, data);
    if (data->mec)
    {
      soap_mec_cleanup(soap, data->mec);
      SOAP_FREE(soap, data->mec);
      data->mec = NULL;
    }
    if (data->store)
    {
      X509_STORE_free(data->store);
      data->store = NULL;
    }
    SOAP_FREE(soap, data);
  }
}

/******************************************************************************\
 *
 *      Plugin-specific functions
 *
\******************************************************************************/

/**
@fn soap_wsse_set_security_token_handler(struct soap *soap, const void *(*callback)(struct soap*, int *alg, const char *keyname, const unsigned char *keyid, int keyidlen, int *keylen))
@brief Sets the security token handler callback that is optionaly used to retrieve keys for signature verification and decryption.
@param soap context
@param[in] callback function pointer
@return SOAP_OK
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_set_security_token_handler(struct soap *soap, const void *(*callback)(struct soap*, int *alg, const char *keyname, const unsigned char *keyid, int keyidlen, int *keylen))
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  DBGFUN("soap_wsse_set_security_token_handler");
  if (data)
    data->security_token_handler = callback;
  return SOAP_OK;
}

/**
@fn int soap_wsse_set_wsu_id(struct soap *soap, const char *tags)
@brief Sets the elements that are to be extended with wsu:Id attributes. The wsu:Id attribute values are set to the string value of the tag's QName by replacing colons with hyphens to produce an xsd:ID value.
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

/**
@fn int soap_wsse_set_InclusiveNamespaces(struct soap *soap, const char *prefixlist)
@brief Sets the C14N InclusiveNamespaces Prefix List property.
@param soap context
@param[in] prefixlist string of space-separated namespace prefixes, or NULL to remove
@return SOAP_OK
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_set_InclusiveNamespaces(struct soap *soap, const char *prefixlist)
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  DBGFUN1("soap_wsse_set_InclusiveNamespaces", "prefixlist=%s", prefixlist?prefixlist:"(null)");
  if (data)
  {
    char *s = NULL;
    if (data->prefixlist)
      SOAP_FREE(soap, data->prefixlist);
    if (prefixlist)
    {
      size_t l = strlen(prefixlist);
      if (l + 1 < l || (SOAP_MAXALLOCSIZE > 0 && l > SOAP_MAXALLOCSIZE))
        return soap->error = SOAP_EOM;
      s = (char*)SOAP_MALLOC(soap, l + 1);
      if (!s)
        return soap->error = SOAP_EOM;
      soap_strcpy(s, l + 1, prefixlist);
    }
    data->prefixlist = s;
  }
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_sign_only(struct soap *soap, const char *tags)
@brief Filters only the specified wsu:Id names for signing. Can be used with soap_wsse_set_wsu_id() and if so should use the element tag names.
@param soap context
@param[in] tags string of space-separated qualified and unqualified tag names
@return SOAP_OK
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_sign_only(struct soap *soap, const char *tags)
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  DBGFUN1("soap_wsse_sign_only", "tags=%s", tags?tags:"(null)");
  if (data)
    data->sigid = soap_wsse_ids(soap, tags, '-');
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn static char* soap_wsse_ids(struct soap *soap, const char *tags, int sub)
@brief converts tag name(s) to id name(s).
@param soap context
@param[in] tags string of space-separated (un)qualified tag names
@param[in] sub replacement character for ':'
@return string of ids
*/
static char *
soap_wsse_ids(struct soap *soap, const char *tags, int sub)
{
  char *s, *t;
  s = t = soap_strdup(soap, tags);
  while (s && *s)
  {
    if (*s == ':')
      *s = sub;
    s++;
  }
  return t;
}

/******************************************************************************/

/**
@fn int soap_wsse_sign(struct soap *soap, int alg, const void *key, int keylen)
@brief Uses the wsse plugin to sign all wsu:Id attributed elements.
@param soap context
@param[in] alg is the signature algorithm, such as SOAP_SMD_HMAC_SHA1/224/256/384/512, SOAP_SMD_SIGN_DSA_SHA1/256, SOAP_SMD_SIGN_RSA_SHA1/224/256/384/512, or SOAP_SMD_SIGN_ECDSA_SHA1/224/256/384/512
@param[in] key is the HMAC secret key or DSA/RSA/ECDSA private EVP_PKEY
@param[in] keylen is the HMAC key length
@return SOAP_OK or fault

This function does not actually sign the message, but initiates the plugin's
signature algorithm to sign the message upon message transfer.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_sign(struct soap *soap, int alg, const void *key, int keylen)
{
  struct soap_wsse_digest *digest, *next;
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  DBGFUN1("soap_wsse_sign", "alg=%x", alg);
  if (!data)
    return soap_set_receiver_error(soap, "soap_wsse_sign", "Plugin not registered", SOAP_PLUGIN_ERROR);
  if (!alg || !key)
    return soap_wsse_fault(soap, wsse__InvalidSecurity, "Invalid signature algorithm or key");
  soap_wsse_add_Security(soap);
  /* store alg and key in plugin data */
  data->sign_alg = alg;
  data->sign_key = key;
  data->sign_keylen = keylen;
  /* save and set the plugin's callbacks to preprocess outbound messages */
  if (soap->fpreparesend != soap_wsse_preparesend)
  {
    data->fpreparesend = soap->fpreparesend;
    soap->fpreparesend = soap_wsse_preparesend;
  }
  if (soap->fpreparefinalsend != soap_wsse_preparefinalsend)
  {
    data->fpreparefinalsend = soap->fpreparefinalsend;
    soap->fpreparefinalsend = soap_wsse_preparefinalsend;
  }
  /* support HTTP compression only with HTTP chunking to allow signing XML */
  if ((soap->omode & SOAP_ENC_ZLIB))
    soap->omode = (soap->omode & ~SOAP_IO) | SOAP_IO_CHUNK;
  else if ((soap->omode & SOAP_IO) == SOAP_IO_STORE) /* no store buffering (or else fpreparesend not working) */
    soap->omode = (soap->omode & ~SOAP_IO) | SOAP_IO_BUFFER;
  soap->omode &= ~SOAP_XML_DOM;
  /* cleanup the digest data */
  for (digest = data->digest; digest; digest = next)
  {
    next = digest->next;
    SOAP_FREE(soap, digest);
  }
  data->digest = NULL;
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_sign_body(struct soap *soap, int alg, const void *key, int keylen)
@brief Uses the wsse plugin to sign all wsu:Id attributed elements, including the SOAP Body (by adding a wsu:Id="Body" attribute).
@param soap context
@param[in] alg is the signature algorithm, such as SOAP_SMD_HMAC_SHA1/224/256/384/512, SOAP_SMD_SIGN_DSA_SHA1/256, SOAP_SMD_SIGN_RSA_SHA1/224/256/384/512, or SOAP_SMD_SIGN_ECDSA_SHA1/224/256/384/512
@param[in] key is the HMAC secret key or DSA/RSA/ECDSA private EVP_PKEY
@param[in] keylen is the HMAC key length
@return SOAP_OK or fault

This function does not actually sign the message, but initiates the plugin's
signature algorithm to sign the message upon message transfer.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_sign_body(struct soap *soap, int alg, const void *key, int keylen)
{
  DBGFUN1("soap_wsse_sign_body", "alg=%x", alg);
  soap_wsse_sign_only(soap, NULL);
  soap->omode |= SOAP_SEC_WSUID;
  return soap_wsse_sign(soap, alg, key, keylen);
}

/******************************************************************************/

/**
@fn int soap_wsse_verify_init(struct soap *soap)
@brief Uses the wsse plugin to initiate the verification of the signature and SignedInfo Reference digests.
@param soap context
@return SOAP_OK

This function does not actually verify the message, but initiates the plugin's
data to store the message in a DOM to verify the signature. The signature and
digests in the DOM must be verified manually.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_verify_init(struct soap *soap)
{
  DBGFUN("soap_wsse_verify_init");
  /* deserialize inbound message to DOM */
  soap->imode |= SOAP_XML_DOM;
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_verify_auto(struct soap *soap, int alg, const void *key, size_t keylen)
@brief Uses the wsse plugin to initiate the automatic verification of the signature and SignedInfo Reference digests.
@param soap context
@param[in] alg to verify signature if signature has no secret or public key, use SOAP_SMD_NONE to omit
@param[in] key is HMAC key or a DSA/RSA/ECDSA EVP_PKEY or NULL
@param[in] keylen is HMAC key length or 0
@return SOAP_OK

This function does not actually verify the message, but initiates the plugin's
algorithm to store the message in a DOM to automatically verify the signature
and digests. If the message does not contain a key to verify the signature,
the alg, key, and keylen parameters are used. It is important that the X509
certificate used to verify the signature, which requires soap->cafile and/or
soap->capath to be set.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_verify_auto(struct soap *soap, int alg, const void *key, size_t keylen)
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  DBGFUN("soap_wsse_verify_auto");
  if (!data)
    return soap_set_receiver_error(soap, "soap_wsse_verify_auto", "Plugin not registered", SOAP_PLUGIN_ERROR);
  data->vrfy_alg = alg;
  data->vrfy_key = key;
  data->vrfy_keylen = keylen;
  if (soap->fpreparefinalrecv != soap_wsse_preparefinalrecv)
  {
    data->fpreparefinalrecv = soap->fpreparefinalrecv;
    soap->fpreparefinalrecv = soap_wsse_preparefinalrecv; 
  }
  return soap_wsse_verify_init(soap);
}

/******************************************************************************/

/**
@fn int soap_wsse_verify_done(struct soap *soap)
@brief Terminates the automatic verification of signatures.
@param soap context
@return SOAP_OK
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_verify_done(struct soap *soap)
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  DBGFUN("soap_wsse_verify_done");
  if (!data)
    return soap_set_receiver_error(soap, "soap_wsse_verify_done", "Plugin not registered", SOAP_PLUGIN_ERROR);
  soap->imode &= ~SOAP_XML_DOM;
  soap->omode &= ~SOAP_SEC_WSUID;
  if (soap->fpreparefinalrecv == soap_wsse_preparefinalrecv)
    soap->fpreparefinalrecv = data->fpreparefinalrecv;
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn size_t soap_wsse_verify_element(struct soap *soap, const char *URI, const char *tag)
@brief Post-checks the presence of signed element(s), returns the number of matching elements signed or zero when none found or if one or more matching elements are not signed. Does not verify the signature of these elements, which is done with @ref soap_wsse_verify_auto.
@param soap context
@param URI namespace of element(s)
@param tag name of element(s)
@return number of matching elements that are signed or 0 if no matching elements found or if one or more non-signed matching elements were found (0 is returned to prevent signature wrapping attacks).

This function does not actually verify the signature of each element, but
checks whether the elements are signed and thus their integrity is preserved.
Signed element nesting rules are obeyed, so if the matching element is a
descendent of a signed element, it is signed as well.  Thus, the verification
process follows nesting rules.

Client should call this function after invocation. Services should call this
function inside a service operation. This function traverses the entire DOM, so
performance is determined by the size of a message.

To check the SOAP Body (either using SOAP 1.1 or 1.2), @ref soap_wsse_verify_element(soap, namespaces[0].ns, "Body"). To check whether the Timestamp was signed (assuming it is present and message expiration checked with @ref soap_wsse_verify_Timestamp), use @ref soap_wsse_verify_element(soap, "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd", "Timestamp").

@note
For future releases, XPath queries (or forms of these) will be considered.
*/
SOAP_FMAC1
size_t
SOAP_FMAC2
soap_wsse_verify_element(struct soap *soap, const char *URI, const char *tag)
{
  ds__SignedInfoType *signedInfo = soap_wsse_SignedInfo(soap);
  size_t count = 0;
  DBGFUN("soap_wsse_verify_element");
  if (signedInfo && soap->dom)
  {
    struct soap_dom_element *elt;
    /* traverse the DOM */
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "SignedInfo and DOM found\n"));
    elt = soap->dom;
    while (elt)
    {
      /* find wsu:Id or ds:Id and check for Id in signedInfo */
      int ok = 0;
      struct soap_dom_attribute *att;
      for (att = elt->atts; att; att = att->next)
      {
        if (att->name
         && att->nstr
         && (!strcmp(att->nstr, wsu_URI) || !strcmp(att->nstr, ds_URI))
         && (!strcmp(att->name, "Id") || !soap_tag_cmp(att->name, "*:Id")))
        {
          /* Id attribute found, search Id value in ds:Reference/@URI */
          int i;
          for (i = 0; i < signedInfo->__sizeReference; i++)
          {
            ds__ReferenceType *reference = signedInfo->Reference[i];
            if (reference->URI && *reference->URI == '#' && !strcmp(reference->URI + 1, att->text))
            {
              ok = 1;
              break;
            }
          }
          if (ok)
            break;
        }
      }
      /* the current element is signed, count this and the matching nested */
      if (ok)
      {
        count += soap_wsse_verify_nested(soap, elt, URI, tag);
        /* go to next sibling or back up */
        if (elt->next)
          elt = elt->next;
        else
        {
          do elt = elt->prnt;
          while (elt && !elt->next);
          if (elt)
            elt = elt->next;
        }
      }
      else if (elt->name && ((!elt->nstr && !URI) || (elt->nstr && URI && !strcmp(elt->nstr, URI))))
      {
        const char *s = strchr(elt->name, ':');
        if (s)
          s++;
        else
          s = elt->name;
        /* found an unsigned matching element */
        if (!strcmp(s, tag))
          return 0;
        elt = soap_dom_next_element(elt, NULL);
      }
      else
      {
        elt = soap_dom_next_element(elt, NULL);
      }
    }
  }
  return count;
}

/******************************************************************************/

/**
@fn size_t soap_wsse_verify_nested(struct soap *soap, struct soap_dom_element *dom, const char *URI, const char *tag)
@brief Counts signed matching elements from the dom node and down.
@param soap context
@param dom node to check and down
@param URI namespace of element(s)
@param tag name of element(s)
@return number of matching elements.
*/
static size_t
soap_wsse_verify_nested(struct soap *soap, struct soap_dom_element *dom, const char *URI, const char *tag)
{
  size_t count = 0;
  /* search the DOM node and descendants for matching elements */
  struct soap_dom_element *elt = dom;
  for (elt = dom; elt && elt != dom->next && elt != dom->prnt; elt = soap_dom_next_element(elt, NULL))
  {
    if (elt->name && ((!elt->nstr && !URI) || (elt->nstr && URI && !strcmp(elt->nstr, URI))))
    {
      const char *s = strchr(elt->name, ':');
      if (s)
        s++;
      else
        s = elt->name;
      /* found element? */
      if (!strcmp(s, tag))
        count++;
    }
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Element '%s' (\"%s\":%s) is signed\n", elt->name, elt->nstr, elt->name));
  }
  return count;
}

/******************************************************************************/

/**
@fn int soap_wsse_verify_body(struct soap *soap)
@brief Post-checks the presence of signed SOAP Body. Does not verify the signature of the Body, which is done with @ref soap_wsse_verify_auto.
@param soap context
@return SOAP_OK (signed) or SOAP_FAULT

This function does not actually verify the signature of the Body. It checks
whether the Body is signed and thus its integrity is preserved. Clients should
call this function after invocation. Services should call this function inside
a service operation. This function traverses the entire DOM, so performance is
determined by the size of a message.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_verify_body(struct soap *soap)
{
  const char *ns = NULL;
  /* Are we using SOAP 1.1 or 1.2? Check first row of namespace table */
  if (soap->local_namespaces)
  {
    if (soap->local_namespaces->out)
      ns = soap->local_namespaces->out;
    else if (soap->local_namespaces->ns)
      ns = soap->local_namespaces->ns;
  }
  /* If we don't know if we're using SOAP 1.1 or 1.2, then assume it is 1.2 */
  if (!ns)
    ns = "http://www.w3.org/2003/05/soap-envelope";
  if (soap_wsse_verify_element(soap, ns, "Body") == 1)
    return SOAP_OK;
  return soap_wsse_sender_fault(soap, "Message body not signed", NULL);
}

/******************************************************************************/

/**
@fn int soap_wsse_verify_with_signature(struct soap *soap, const ds__Signature *signature)
@brief Verifies XML in DOM of a message that was parsed, using the provided signature, assuming dsig non-WS-Security usage scenarios, requires DOM of the XML message which is created automatically with WS-Security enabled.
@param soap context
@param signature points to signature structure
@return SOAP_OK (signed and verified) or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_verify_with_signature(struct soap *soap, _ds__Signature *signature)
{
  _wsse__Security *new_security, *security;
  int err;
  if (!signature)
    return soap_wsse_fault(soap, wsse__FailedCheck, "Signature required");
  if (!soap->dom)
    return soap_wsse_fault(soap, wsse__FailedCheck, "XML DOM of signed message required");
  /* save wsse:Security/ds:Signature when present */
  security = soap_wsse_Security(soap);
  /* set the header wsse:Security/ds:Signature to be used internally */
  soap_header(soap);
  soap->header->wsse__Security = NULL;
  new_security = soap_wsse_add_Security(soap);
  if (!new_security)
    return soap->error = SOAP_EOM;
  new_security->ds__Signature = signature;
  err = soap_wsse_verify_Signature(soap);
  /* restore wsse:Security/ds:Signature when present */
  soap->header->wsse__Security = security;
  return err;
}

/******************************************************************************/

/**
@fn int soap_wsse_encrypt_body(struct soap *soap, int alg, const void *key, int keylen)
@brief Initiates the encryption of the SOAP Body. The algorithm should be SOAP_MEC_ENC_DES_CBC etc. for symmetric encryption. Use soap_wsse_add_EncryptedKey for public key encryption.
@param soap context
@param[in] alg the encryption algorithm, should be SOAP_MEC_ENC_DES_CBC etc.
@param[in] key a certificate with public key for encryption, a DES CBC 160-bit key or AES key
@param[in] keylen the symmetric encryption key length, 20 bytes for a DES CBC 160-bit key or larger for AES key
@return SOAP_OK or error code

This function initiates the encryption of the SOAP Body using an RSA public key
or a symmetric shared secret key. No WS-Security EncryptedKey header will be
set. Use soap_wsse_add_EncryptedKey instead for public key encryption.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_encrypt_body(struct soap *soap, int alg, const void *key, int keylen)
{
  struct soap_wsse_data *data;
  DBGFUN1("soap_wsse_encrypt_body", "alg=%x", alg);
  data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  if (!data)
    return soap_set_receiver_error(soap, "soap_wsse_encrypt_body", "Plugin not registered", SOAP_PLUGIN_ERROR);
  data->encid = NULL;
  soap->omode |= SOAP_SEC_WSUID;
  soap_wsse_add_DataReferenceURI(soap, "#SOAP-ENV_Body");
  return soap_wsse_encrypt(soap, alg, key, keylen);
}

/******************************************************************************/

/**
@fn int soap_wsse_encrypt_only(struct soap *soap, int alg, const void *key, int keylen, const char *tags)
@brief Initiates the encryption of XML elements specified in the tags string. Should be used in combination with soap_wsse_set_wsu_id to set wsu:Id for given XML element tags. The algorithm should be SOAP_MEC_ENC_DES_CBC etc. for symmetric encryption. Use soap_wsse_add_EncryptedKey_encrypt_only for public key encryption.
@param soap context
@param[in] alg the encryption algorithm, should be SOAP_MEC_ENC_DES_CBC etc.
@param[in] key a certificate with public key for encryption, a DES CBC 160-bit key or AES key
@param[in] keylen the symmetric encryption key length, 20 bytes for a DES CBC 160-bit key
@param[in] tags string of space-separated qualified and unqualified tag names
@return SOAP_OK or error code

This function initiates the encryption using an RSA public key or a symmetric
shared secret key. No WS-Security EncryptedKey header will be set. Use
soap_wsse_add_EncryptedKey instead for public key encryption.

@warning
Use @ref soap_wsse_add_EncryptedKey_encrypt_only only in combination with
@ref soap_wsse_set_wsu_id with the tag names of the elements to be encrypted.
OTHERWISE THE GIVEN XML ELEMENTS ARE NOT ENCRYPTED AND WILL BE SENT IN THE
CLEAR.

@warning
The elements identified with @ref soap_wsse_set_wsu_id to encrypt MUST occur
EXACTLY ONCE in the SOAP Body.

@warning
Encryption/decryption of elements with simple content (CDATA content) IS NOT
SUPPORTED. This means that elements you want to encrypt with this function must
have complex content. That is, only encrypt elements with sub elements or
encrypt the entire SOAP Body.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_encrypt_only(struct soap *soap, int alg, const void *key, int keylen, const char *tags)
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  DBGFUN2("soap_wsse_encrypt_only", "alg=%x", alg, "tags=%s", tags?tags:"(null)");
  if (!data)
    return soap_set_receiver_error(soap, "soap_wsse_encrypt_only", "Plugin not registered", SOAP_PLUGIN_ERROR);
  data->encid = soap_strdup(soap, tags);
  if (tags)
  {
    char *s, *t;
    size_t l = strlen(tags);
    /* make space to insert # to each id converted from a tag name */
    t = (char*)soap_malloc(soap, l + 2);
    if (!t)
      return soap->error = SOAP_EOM;
    *t = '#';
    soap_strcpy(t + 1, l + 1, tags);
    s = soap_wsse_ids(soap, t, '_');
    if (!s)
      return soap->error = SOAP_EOM;
    s++;
    for (;;)
    {
      t = strchr(s, ' ');
      if (t)
        *t = '\0';
      *--s = '#';
      if (soap_wsse_add_DataReferenceURI(soap, s))
        return soap->error;
      if (!t)
        break;
      s = t + 1;
      while (*s == ' ')
        s++;
    }
  }
  return soap_wsse_encrypt(soap, alg, key, keylen);
}

/******************************************************************************/

/**
@fn int soap_wsse_encrypt(struct soap *soap, int alg, const void *key, int keylen)
@brief Start encryption. This function is supposed to be used internally only. The algorithm should be SOAP_MEC_ENC_DES_CBC etc. for symmetric encryption. Use soap_wsse_add_EncryptedKey for public key.
encryption.
@param soap context
@param[in] alg the encryption algorithm, should be SOAP_MEC_ENC_DES_CBC etc.
@param[in] key a certificate with public key for encryption, a DES CBC 160-bit key or AES key
@param[in] keylen the symmetric encryption key length, 20 bytes for a DES CBC 160-bit key
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_encrypt(struct soap *soap, int alg, const void *key, int keylen)
{
  struct soap_wsse_data *data;
  DBGFUN1("soap_wsse_encrypt", "alg=%x", alg);
  data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  if (!data)
    return soap_set_receiver_error(soap, "soap_wsse_encrypt", "Plugin not registered", SOAP_PLUGIN_ERROR);
  if (!alg || !key)
    return soap_wsse_fault(soap, wsse__UnsupportedAlgorithm, "An unsupported signature or encryption algorithm was used");
  if (alg & SOAP_MEC_ENV)
    return soap_wsse_add_EncryptedKey(soap, alg, NULL, (X509*)key, NULL, NULL, NULL);
  /* store alg and key in plugin data */
  data->enco_alg = (alg | SOAP_MEC_ENC);
  data->enco_key = key;
  data->enco_keylen = keylen;
  if (data->mec)
    soap_mec_cleanup(soap, data->mec);
  else
    data->mec = (struct soap_mec_data*)SOAP_MALLOC(soap, sizeof(struct soap_mec_data));
  if (soap_mec_begin(soap, data->mec, alg, NULL, (unsigned char*)key, &keylen))
    return soap->error;
  soap->omode &= ~SOAP_XML_DOM;
  soap->feltbegout = soap_wsse_element_begin_out;
  soap->feltendout = soap_wsse_element_end_out;
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_decrypt_auto(struct soap *soap, int alg, const void *key, int keylen)
@brief Start automatic decryption when needed using the specified key. This function should be used just once. The algorithm should be SOAP_MEC_ENV_DEC_DES_CBC etc. for public key encryption/decryption and SOAP_MEC_DEC_DES_CBC etc. for symmetric shared secret keys.
@param soap context
@param[in] alg the decryption algorithm, 
@param[in] key a persistent decryption key for the algorithm, a private RSA key or a shared symmetric secret key
@param[in] keylen use 0 for public-key encryption/decryption
or the shared secret decryption key length, 20 bytes for a DES CBC 160-bit key
@return SOAP_OK or error code

This function can be called once before multiple messages are received with
WS-Security encrypted content, where only one key is used for encryption
(public key or shared secret key). The default decryption key is set. If
multiple decryption keys should be used, do NOT use this function but set the
security_token_handler callback of the wsse plugin. See @ref wsse_9_2. Use a
NULL key to remove the default decryption key.
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_decrypt_auto(struct soap *soap, int alg, const void *key, int keylen)
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  DBGFUN1("soap_wsse_decrypt_auto", "alg=%x", alg);
  if (!data)
    return soap_set_receiver_error(soap, "soap_wsse_decrypt_auto", "Plugin not registered", SOAP_PLUGIN_ERROR);
  /* store alg and key in plugin data */
  data->deco_alg = (alg & ~SOAP_MEC_ENC);
  /* TODO should add? data->enco_alg = (alg & ~(SOAP_MEC_ENC|SOAP_MEC_ENV)); */
  data->deco_key = key;
  data->deco_keylen = keylen;
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_encrypt_begin(struct soap *soap, const char *id, int alg, const char *URI, const char *keyname, const unsigned char *key, const char *type)
@brief Emit XML encryption tags and start encryption of the XML element content.
@param soap context
@param[in] id string for the EncryptedData element Id attribute
@param[in] alg algorithm used, or SOAP_MEC_NONE to ignore
@param[in] URI string for the encrypted element wsu:Id attribute
@param[in] keyname optional subject key name
@param[in] key optional DES/AES key for encryption (to override the current key)
@param[in] type of encryption, either xenc_elementURI ("http://www.w3.org/2001/04/xmlenc#Element") or xenc_contentURI ("http://www.w3.org/2001/04/xmlenc#Content")
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_encrypt_begin(struct soap *soap, const char *id, int alg, const char *URI, const char *keyname, const unsigned char *key, const char *type)
{
  int event;
  const char *algURI = NULL;
  (void)URI;
  DBGFUN("soap_wsse_encrypt_begin");
  if ((soap->mode & SOAP_IO_LENGTH) && ((soap->mode & SOAP_IO) == SOAP_IO_CHUNK || (soap->mode & SOAP_IO) == SOAP_IO_STORE))
    return SOAP_OK;
  /* disable digest */
  event = soap->event;
  soap->event = 0;
  if (soap_set_attr(soap, "Id", id, 1)
   || soap_set_attr(soap, "Type", type, 1)
   || soap_element(soap, "xenc:EncryptedData", 0, NULL)
   || soap_element_start_end_out(soap, NULL))
    return soap->error;
  switch (alg & SOAP_MEC_MASK & ~SOAP_MEC_ENV)
  {
    case SOAP_MEC_ENC_DES_CBC:
      algURI = xenc_3desURI;
      break;
    case SOAP_MEC_ENC_AES128_CBC:
      algURI = xenc_aes128cbcURI;
      break;
    case SOAP_MEC_ENC_AES192_CBC:
      algURI = xenc_aes192cbcURI;
      break;
    case SOAP_MEC_ENC_AES256_CBC:
      algURI = xenc_aes256cbcURI;
      break;
    case SOAP_MEC_ENC_AES512_CBC:
      algURI = xenc_aes512cbcURI;
      break;
    case SOAP_MEC_ENC_AES128_GCM:
      algURI = xenc_aes128gcmURI;
      break;
    case SOAP_MEC_ENC_AES192_GCM:
      algURI = xenc_aes192gcmURI;
      break;
    case SOAP_MEC_ENC_AES256_GCM:
      algURI = xenc_aes256gcmURI;
      break;
    case SOAP_MEC_ENC_AES512_GCM:
      algURI = xenc_aes512gcmURI;
      break;
  }
  if (algURI)
  {
    if (soap_set_attr(soap, "Algorithm", algURI, 1)
     || soap_element(soap, "xenc:EncryptionMethod", 0, NULL)
     || soap_element_start_end_out(soap, "xenc:EncryptionMethod"))
      return soap->error;
  }
#if 0 /* deprecated */
  if (URI)
  {
    if (soap_element(soap, "ds:KeyInfo", 0, NULL)
     || soap_element_start_end_out(soap, NULL)
     || soap_element(soap, "wsse:SecurityTokenReference", 0, NULL)
     || soap_element_start_end_out(soap, NULL)
     || soap_set_attr(soap, "URI", URI, 1)
     || soap_element(soap, "wsse:Reference", 0, NULL)
     || soap_element_start_end_out(soap, NULL)
     || soap_element_end_out(soap, "wsse:Reference")
     || soap_element_end_out(soap, "wsse:SecurityTokenReference")
     || soap_element_end_out(soap, "ds:KeyInfo"))
      return soap->error;
  }
  else
#endif
  if (keyname)
  {
    if (soap_element(soap, "ds:KeyInfo", 0, NULL)
     || soap_element_start_end_out(soap, NULL)
     || soap_element(soap, "ds:KeyName", 0, NULL)
     || soap_element_start_end_out(soap, NULL)
     || soap_string_out(soap, keyname, 0)
     || soap_element_end_out(soap, "ds:KeyName")
     || soap_element_end_out(soap, "ds:KeyInfo"))
      return soap->error;
  }
  if (soap_element(soap, "xenc:CipherData", 0, NULL)
   || soap_element_start_end_out(soap, NULL)
   || soap_element(soap, "xenc:CipherValue", 0, NULL)
   || soap_element_start_end_out(soap, NULL))
    return soap->error;
  /* re-enable digest */
  soap->event = event;
  /* adjust level, hiding xenc elements */
  soap->level -= 3;
  return soap_mec_start(soap, key);
}

/******************************************************************************/

/**
@fn int soap_wsse_encrypt_end(struct soap *soap)
@brief Emit XML encryption end tags and stop encryption of the XML element content.
@param soap context
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_encrypt_end(struct soap *soap)
{
  int event;
  DBGFUN("soap_wsse_encrypt_end");
  if ((soap->mode & SOAP_IO_LENGTH) && ((soap->mode & SOAP_IO) == SOAP_IO_CHUNK || (soap->mode & SOAP_IO) == SOAP_IO_STORE))
    return SOAP_OK;
  /* disable digest */
  event = soap->event;
  soap->event = 0;
  /* adjust level, hiding xenc elements */
  soap->level += 3;
  /* base64 body, no indent */
  soap->body = 1;
  if (soap_mec_stop(soap)
   || soap_element_end_out(soap, "xenc:CipherValue")
   || soap_element_end_out(soap, "xenc:CipherData")
   || soap_element_end_out(soap, "xenc:EncryptedData"))
    return soap->error;
  /* re-enable digest */
  soap->event = event;
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_decrypt_begin(struct soap *soap)
@brief Check for XML encryption tags and start decryption of the XML element content. If the KeyInfo element is present, the security_token_handler callback will be used to obtain a decryption key based on the key name.
@param soap context
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_decrypt_begin(struct soap *soap)
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  struct ds__KeyInfoType info;
  struct xenc__EncryptionMethodType meth;
  int alg = data->deco_alg;
  const unsigned char *keyid = NULL;
  int keyidlen = 0;
  unsigned char *key = NULL;
  int keylen = data->deco_keylen;
  DBGFUN("soap_wsse_decrypt_begin");
  if (!data)
    return soap_set_receiver_error(soap, "soap_wsse_decrypt_begin", "Plugin not registered", SOAP_PLUGIN_ERROR);
  if (soap_element_begin_in(soap, "xenc:EncryptedData", 0, NULL))
    return soap->error;
  /* TODO: use Type attribute? */
  soap_default_xenc__EncryptionMethodType(soap, &meth);
  if (soap_in_xenc__EncryptionMethodType(soap, "xenc:EncryptionMethod", &meth, NULL))
  {
    if (meth.Algorithm)
    {
      if (!strcmp(meth.Algorithm, xenc_3desURI))
        alg = (alg & (~SOAP_MEC_MASK | SOAP_MEC_ENV)) | SOAP_MEC_DEC_DES_CBC;
      else if (!strcmp(meth.Algorithm, xenc_aes128cbcURI))
        alg = (alg & (~SOAP_MEC_MASK | SOAP_MEC_ENV)) | SOAP_MEC_DEC_AES128_CBC;
      else if (!strcmp(meth.Algorithm, xenc_aes192cbcURI))
        alg = (alg & (~SOAP_MEC_MASK | SOAP_MEC_ENV)) | SOAP_MEC_DEC_AES192_CBC;
      else if (!strcmp(meth.Algorithm, xenc_aes256cbcURI))
        alg = (alg & (~SOAP_MEC_MASK | SOAP_MEC_ENV)) | SOAP_MEC_DEC_AES256_CBC;
      else if (!strcmp(meth.Algorithm, xenc_aes512cbcURI))
        alg = (alg & (~SOAP_MEC_MASK | SOAP_MEC_ENV)) | SOAP_MEC_DEC_AES512_CBC;
      else if (!strcmp(meth.Algorithm, xenc_aes128gcmURI))
        alg = (alg & (~SOAP_MEC_MASK | SOAP_MEC_ENV)) | SOAP_MEC_DEC_AES128_GCM;
      else if (!strcmp(meth.Algorithm, xenc_aes192gcmURI))
        alg = (alg & (~SOAP_MEC_MASK | SOAP_MEC_ENV)) | SOAP_MEC_DEC_AES192_GCM;
      else if (!strcmp(meth.Algorithm, xenc_aes256gcmURI))
        alg = (alg & (~SOAP_MEC_MASK | SOAP_MEC_ENV)) | SOAP_MEC_DEC_AES256_GCM;
      else if (!strcmp(meth.Algorithm, xenc_aes512gcmURI))
        alg = (alg & (~SOAP_MEC_MASK | SOAP_MEC_ENV)) | SOAP_MEC_DEC_AES512_GCM;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "EncryptionMethod alg=%x\n", alg));
    }
  }
  if (soap_in_ds__KeyInfoType(soap, "ds:KeyInfo", &info, NULL))
  {
    if (data->security_token_handler)
    {
      keyid = soap_wsse_get_KeyInfo_SecurityTokenReferenceKeyIdentifier(soap, &info, &keyidlen);
      if (keyid || info.KeyName)
      {
        /* retrieve key from token handler callback */
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Getting shared secret key '%s' through security_token_handler callback\n", info.KeyName));
        key = (unsigned char*)data->security_token_handler(soap, &alg, info.KeyName, keyid, keyidlen, &keylen);
        if (key)
        {
          if (data->mec)
            soap_mec_cleanup(soap, data->mec);
          else
            data->mec = (struct soap_mec_data*)SOAP_MALLOC(soap, sizeof(struct soap_mec_data));
          if (soap_mec_begin(soap, data->mec, alg, NULL, (unsigned char*)key, &keylen))
            return soap_wsse_fault(soap, wsse__FailedCheck, NULL);
        }
      }
    }
  }
  if (soap_element_begin_in(soap, "xenc:CipherData", 0, NULL)
   || soap_element_begin_in(soap, "xenc:CipherValue", 0, NULL))
    return soap->error;
  /* if no default shared key is set for symmetric decryption, get it */
  if (!(alg & SOAP_MEC_ENV))
  {
    if (alg != data->deco_alg)
    {
      if (data->security_token_handler)
      {
        /* retrieve key from token handler callback */
        DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Getting shared secret key for alg=%x through security_token_handler callback\n", alg));
        data->deco_key = (const unsigned char*)data->security_token_handler(soap, &alg, NULL, keyid, keyidlen, &keylen);
      }
    }
    if (!key && keylen)
      key = (unsigned char*)data->deco_key;
    if (key)
    {
      if (data->mec)
        soap_mec_cleanup(soap, data->mec);
      else
        data->mec = (struct soap_mec_data*)SOAP_MALLOC(soap, sizeof(struct soap_mec_data));
      if (soap_mec_begin(soap, data->mec, alg, NULL, (unsigned char*)key, &keylen))
        return soap_wsse_fault(soap, wsse__FailedCheck, NULL);
    }
  }
  DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Key=%p/%p keylen=%d alg=%x\n", key, data->deco_key, keylen, alg));
  data->deco_alg = alg;
  data->deco_keylen = keylen;
  if (soap_mec_start_alg(soap, alg, key))
    return soap_wsse_fault(soap, wsse__FailedCheck, NULL);
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_decrypt_end(struct soap *soap)
@brief Check for XML encryption ending tags and stop decryption of the XML element content.
@param soap context
@return SOAP_OK or error code
*/
SOAP_FMAC1
int
SOAP_FMAC2
soap_wsse_decrypt_end(struct soap *soap)
{
  DBGFUN("soap_wsse_decrypt_end");
  if (soap_mec_stop(soap)
   || soap_element_end_in(soap, "xenc:CipherValue")
   || soap_element_end_in(soap, "xenc:CipherData")
   || soap_element_end_in(soap, "xenc:EncryptedData"))
    return soap_wsse_fault(soap, wsse__FailedCheck, NULL);
  return SOAP_OK;
}

/******************************************************************************\
 *
 *      Callbacks registered by plugin
 *
\******************************************************************************/

/**
@fn int soap_wsse_element_begin_in(struct soap *soap, const char *tag)
@brief This callback is invoked as soon as a starting tag of an element is received by the XML parser.
@param soap context
@param[in] tag name of the element parsed
@return SOAP_OK or error code
*/
static int
soap_wsse_element_begin_in(struct soap *soap, const char *tag)
{
  /* make sure we always have a header allocated */
  if (soap->part == SOAP_IN_ENVELOPE)
    soap_header(soap);
  else if (!soap_match_tag(soap, tag, "xenc:EncryptedData"))
  {
    struct soap_dom_element **elt, *dom = soap->dom;
    /* temporarily disable DOM */
    soap->dom = NULL;
    /* parse encryption tags */
    if (soap_wsse_decrypt_begin(soap))
      return soap->error;
    /* re-enable dom and adjust DOM tree to skip encryption elements */
    soap->dom = dom->prnt;
    /* remove EncryptedData element from DOM */
    for (elt = &soap->dom->elts; *elt && (*elt)->next; elt = &(*elt)->next)
      continue;
    if (*elt)
      *elt = NULL;
    /* adjust nesting level */
    if (soap->level > 3)
      soap->level -= 3;
    DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Decryption started, parsing encrypted XML\n"));
    soap->event = SOAP_SEC_DECRYPT;
    return soap_peek_element(soap);
  }
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_element_end_in(struct soap *soap, const char *tag1, const char *tag2)
@brief This callback is invoked as soon as an ending tag of an element is received by the XML parser.
@param soap context
@param[in] tag1 name of the element parsed
@param[in] tag2 name of the element that was expected by the parser's state, or NULL
@return SOAP_OK or error code
*/
static int
soap_wsse_element_end_in(struct soap *soap, const char *tag1, const char *tag2)
{
  if (soap->event == SOAP_SEC_DECRYPT
   && soap->dom
   && soap->dom->elts
   && !soap_match_tag(soap, tag1, ":CipherValue"))
  {
    struct soap_dom_element *dom = soap->dom->elts;
    soap->event = 0;
    /* temporarily disable DOM */
    soap->dom = NULL;
    /* adjust nesting level */
    soap->level += 3;
    /* parse ending tags */
    if (soap_mec_stop(soap)
     || soap_element_end_in(soap, ":CipherData")
     || soap_element_end_in(soap, ":EncryptedData"))
      return soap_wsse_fault(soap, wsse__FailedCheck, NULL);
    /* adjust DOM tree to skip encryption elements */
    while (dom->next)
      dom = dom->next;
    /* remove the old indent before ending tag */
    dom->tail = NULL;
    /* re-enable DOM */
    soap->dom = dom;
    if (soap_element_end_in(soap, tag2))
      return soap->error;
  }
  else if (!soap_match_tag(soap, tag1, ":EncryptedKey"))
  {
    return soap_wsse_verify_EncryptedKey(soap);
  }
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_element_begin_out(struct soap *soap, const char *tag, int id, const char *type)
@brief This callback is invoked as soon as a starting tag of an element is to be sent by the XML generator.
@param soap context
@param[in] tag name of the element
@param[in] id of the element or 0
@param[in] type xsi:type of the element or NULL
@return SOAP_OK or error code
*/
static int
soap_wsse_element_begin_out(struct soap *soap, const char *tag, int id, const char *type)
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  if (data && (!data->encid || soap_tagsearch(data->encid, tag)))
  {
    char *URI = NULL;
#if 0 /* deprecated */
    _wsse__Security *security = soap_wsse_Security(soap);
    if (security && security->xenc__EncryptedKey && security->xenc__EncryptedKey->Id)
    {
      const char *Id = security->xenc__EncryptedKey->Id;
      size_t l = strlen(Id);
      URI = (char*)soap_malloc(soap, l + 2);
      if (!URI)
        return soap->error = SOAP_EOM;
      *URI = '#';
      soap_strcpy(URI + 1, l + 1, Id);
    }
#endif
    if (tag && !strcmp(tag, "SOAP-ENV:Body"))
    {
      if (soap_element(soap, tag, id, type)
       || soap_element_start_end_out(soap, NULL))
        return soap->error;
      return soap_wsse_encrypt_begin(soap, "SOAP-ENV_Body", data->enco_alg, URI, data->enco_keyname, NULL, xenc_contentURI);
    }
    if (data->encid)
    {
      struct soap_attribute *tp, *tq, *tr = soap->attributes; /* preserve attribute lists */
      int err;
      soap->attributes = NULL;
      err = soap_wsse_encrypt_begin(soap, soap_wsse_ids(soap, tag, '_'), data->enco_alg, URI, data->enco_keyname, NULL, xenc_elementURI);
      for (tp = soap->attributes; tp; tp = tq)
      { tq = tp->next;
        if (tp->value)
          SOAP_FREE(soap, tp->value);
        SOAP_FREE(soap, tp);
      }
      soap->attributes = tr;
      if (err)
        return err;
    }
  }
  if (soap_element(soap, tag, id, type)
   || soap_element_start_end_out(soap, NULL))
    return soap->error;
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_element_end_out(struct soap *soap, const char *tag)
@brief This callback is invoked as soon as an ending tag of an element is to be sent by the XML generator.
@param soap context
@param[in] tag name of the element
@return SOAP_OK or error code
*/
static int
soap_wsse_element_end_out(struct soap *soap, const char *tag)
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  if (soap->level <= 1 && !(soap->mode & SOAP_IO_LENGTH))
  {
    soap->feltbegout = NULL;
    soap->feltendout = NULL;
  }
  if (data && (!data->encid || soap_tagsearch(data->encid, tag)))
  {
    if (tag && !strcmp(tag, "SOAP-ENV:Body"))
    {
      if (soap_wsse_encrypt_end(soap))
        return soap->error;
    }
    else if (data->encid)
    {
      if (soap_element_end(soap, tag))
        return soap->error;
      return soap_wsse_encrypt_end(soap);
    }
  }
  return soap_element_end(soap, tag);
}

/******************************************************************************/

/**
@fn int soap_wsse_preparesend(struct soap *soap, const char *buf, size_t len)
@brief Takes a piece of the XML message (tokenized) to compute digest.
@param soap context
@param[in] buf string (XML "tokenized") to be sent
@param[in] len buf length
@return SOAP_OK or fault

This callback is invoked to analyze a message (usually during the HTTP content
length phase).

@note
Nested elements with wsu:Id attributes cannot be individually signed.
*/
static int
soap_wsse_preparesend(struct soap *soap, const char *buf, size_t len)
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  DBGFUN("soap_wsse_preparesend");
  if (!data)
    return SOAP_PLUGIN_ERROR;
  soap->c14ninclude = data->prefixlist;
  /* the gSOAP engine signals the start of a wsu:Id element */
  if (soap->event == SOAP_SEC_BEGIN)
  {
    int alg;
    /* start new digest or continue? */
    if (data->digest && !data->digest->done)
    {
      soap->event = SOAP_SEC_SIGN;
    }
    else if (!data->sigid || soap_tagsearch(data->sigid, soap->id))
    {
      /* initialize smdevp engine */
      struct soap_wsse_digest *digest;
      size_t l = strlen(soap->id);
      soap->event = SOAP_SEC_SIGN;
      digest = (struct soap_wsse_digest*)SOAP_MALLOC(soap, sizeof(struct soap_wsse_digest) + l + 1);
      if (!digest)
        return soap->error = SOAP_EOM;
      digest->next = data->digest;
      digest->done = 0;
      digest->level = soap->level;
      /* digest hash strength is same as signature strength */
      alg = (SOAP_SMD_DGST | (data->sign_alg & SOAP_SMD_HASH));
      soap_smd_init(soap, &digest->smd, alg, NULL, 0);
      memset((void*)digest->hash, 0, sizeof(digest->hash));
      digest->id[0] = '#';
      soap_strcpy(digest->id + 1, l + 1, soap->id);
      data->digest = digest;
      /* omit indent for indented XML (next time around, we will catch '<') */
      if (*buf != '<')
        goto end;
    }
  }
  if (soap->event == SOAP_SEC_SIGN)
  {
    /* update smdevp engine */
    if (data->digest && !data->digest->done)
    {
      soap_smd_update(soap, &data->digest->smd, buf, len);
      if (soap->level < data->digest->level)
      {
        soap->event = 0;
        soap_smd_final(soap, &data->digest->smd, (char*)data->digest->hash, NULL);
        data->digest->done = 1;
      }
    }
  }
end:
  if (data->fpreparesend)
    return data->fpreparesend(soap, buf, len);
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn int soap_wsse_preparefinalsend(struct soap *soap)
@brief Collects the digests of all the wsu:Id elements and populates the SignedInfo.
@param soap context
@return SOAP_OK or fault

This callback is invoked just before the message is sent.
*/
static int
soap_wsse_preparefinalsend(struct soap *soap)
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  DBGFUN("soap_wsse_preparefinalsend");
  if (!data)
    return SOAP_PLUGIN_ERROR;
  if (data->digest)
  {
    ds__SignatureType *signature = soap_wsse_Signature(soap);
    struct soap_wsse_digest *digest;
    const char *transform = NULL;
    int alg, signature_added = 0;
    /* if message is canonicalized populate transform element accordingly */
    if ((soap->mode & SOAP_XML_CANONICAL))
    {
      if (soap->c14ninclude && *soap->c14ninclude == '*')
        transform = c14n_URI;
      else
        transform = exc_c14n_URI;
    }
    /* to increase the message length counter we need to emit the Signature,
       SignedInfo and SignatureValue elements. However, this does not work if
       we already populated the wsse:Signature with SignedInfo and should never
       happen! */
    if (!signature)
    {
      signature = soap_wsse_add_Signature(soap);
      signature_added = 1;
    }
    else if (signature->SignedInfo)
    {
      return soap_set_receiver_error(soap, "wsse error", "Cannot use soap_wsse_sign with populated SignedInfo", SOAP_SSL_ERROR);
    }
    /* digest hash strength is same as signature strength */
    alg = (SOAP_SMD_DGST | (data->sign_alg & SOAP_SMD_HASH));
    /* add the SignedInfo/Reference elements for each digest */
    for (digest = data->digest; digest; digest = digest->next)
      if (soap_wsse_add_SignedInfo_Reference(soap, digest->id, digest->level, transform, data->prefixlist, alg, (char*)digest->hash))
        return soap->error;
    /* then compute the signature and add it */
    if (soap_wsse_add_SignatureValue(soap, data->sign_alg, data->sign_key, data->sign_keylen))
      return soap->error;
    /* Reset the callbacks and cleanup digests */
    soap_wsse_preparecleanup(soap, data);
    /* if non-chunked or stored message, adjust HTTP content length */
    if ((soap->mode & SOAP_IO) != SOAP_IO_CHUNK && (soap->mode & SOAP_IO) != SOAP_IO_STORE)
    {
      /* the code below ensures we increase the HTTP length counter */
      short part = soap->part;
      soap->part = SOAP_IN_HEADER; /* header encoding rules (literal) */
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "Counting the size of additional SOAP Header elements, mode=0x%x\n", soap->mode));
      if (signature_added)
      {
        const char *c14ninclude = soap->c14ninclude;
        soap->c14ninclude = NULL;
        soap->level = 3; /* indent level for XML Signature */
        if ((soap->mode & SOAP_XML_CANONICAL))
        {
          soap->ns = 0; /* need namespaces for canonicalization */
          if ((soap->mode & SOAP_XML_INDENT))
            soap->count += 4; /* correction for soap->ns = 0: add \n+indent */
        }
        soap_out_ds__SignatureType(soap, "ds:Signature", 0, signature, NULL);
        soap->c14ninclude = c14ninclude;
      }
      else
      {
        const char *c14ninclude = soap->c14ninclude;
        const char *c14nexclude = soap->c14nexclude;
        soap->c14nexclude = "ds xsi"; /* don't add xmlns:ds or xmlns:xsi to count msg len */
        soap->level = 4; /* indent level for XML SignedInfo */
        if ((soap->mode & SOAP_XML_CANONICAL))
        {
          soap->ns = 0; /* need namespaces for canonicalization */
          soap->c14ninclude = NULL; /* but do not render inclusive namespaces */
          if ((soap->mode & SOAP_XML_INDENT))
            soap->count += 5; /* correction for soap->ns = 0: add \n+indent */
          if ((soap->mode & SOAP_XML_DEFAULTNS))
            soap->count -= 2*(9 + strlen(ds_URI)); /* correct for xmlns="http://www.w3.org/2000/09/xmldsig#" added to SignedInfo and ds:SignatureValue */
        }
        soap_out_ds__SignedInfoType(soap, "ds:SignedInfo", 0, signature->SignedInfo, NULL);
        soap_out__ds__SignatureValue(soap, "ds:SignatureValue", 0, &signature->SignatureValue, NULL);
        soap->c14ninclude = c14ninclude;
        soap->c14nexclude = c14nexclude;
      }
      soap->part = part;
    }
  }
  else /* Reset the callbacks and cleanup digests */
  {
    soap_wsse_preparecleanup(soap, data);
  }
  if (soap->fpreparefinalsend)
    return soap->fpreparefinalsend(soap);
  return SOAP_OK;
}

/******************************************************************************/

/**
@fn void soap_wsse_preparecleanup(struct soap *soap, struct soap_wsse_data *data)
@brief Restores engine state.
@param soap context
@param[in,out] data plugin data
*/
static void
soap_wsse_preparecleanup(struct soap *soap, struct soap_wsse_data *data)
{
  struct soap_wsse_digest *digest, *next;
  DBGFUN("soap_wsse_preparecleanup");
  data->sign_alg = SOAP_SMD_NONE;
  data->sign_key = NULL;
  data->sign_keylen = 0;
  if (soap->fpreparesend == soap_wsse_preparesend)
    soap->fpreparesend = data->fpreparesend;
  if (soap->fpreparefinalsend == soap_wsse_preparefinalsend)
    soap->fpreparefinalsend = data->fpreparefinalsend;
  data->fpreparesend = NULL;
  data->fpreparefinalsend = NULL;
  for (digest = data->digest; digest; digest = next)
  {
    next = digest->next;
    SOAP_FREE(soap, digest);
  }
  data->digest = NULL;
}

/******************************************************************************/

/**
@fn int soap_wsse_preparefinalrecv(struct soap *soap)
@brief Verify signature and SignedInfo digests initiated with soap_wsse_verify_auto.
@param soap context
@return SOAP_OK or fault
@see soap_wsse_verify_auto

This callback is invoked immediately after a message was received.
*/
static int
soap_wsse_preparefinalrecv(struct soap *soap)
{
  struct soap_wsse_data *data = (struct soap_wsse_data*)soap_lookup_plugin(soap, soap_wsse_id);
  soap->omode &= ~SOAP_SEC_WSUID;
  data->sigid = NULL; /* so we must set again before next send */
  data->encid = NULL; /* so we must set again before next send */
  DBGFUN("soap_wsse_preparefinalrecv");
  if (!data)
    return SOAP_PLUGIN_ERROR;
  if (data->deco_alg != SOAP_MEC_NONE && data->mec)
    if (soap_mec_end(soap, data->mec))
      return soap_wsse_fault(soap, wsse__FailedCheck, NULL);
  data->deco_alg = SOAP_MEC_NONE;
  if (soap_wsse_verify_Signature(soap))
    return soap->error;
  if (data->fpreparefinalrecv && data->fpreparefinalrecv != soap_wsse_preparefinalrecv)
    return data->fpreparefinalrecv(soap);
  return SOAP_OK;
}

/******************************************************************************/

#ifdef __cplusplus
}
#endif

