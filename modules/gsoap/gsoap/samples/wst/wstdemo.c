/*
        wstdemo.c

        WS-Trust demo application.

gSOAP XML Web services tools
Copyright (C) 2000-2020, Robert van Engelen, Genivia Inc., All Rights Reserved.
This part of the software is released under one of the following licenses:
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

This application demonstrates the wst WS-Trust extensible framework.

Compile in C:

soapcpp2 -c -I import wstdemo.h
cc -DSKIP_CERTIFICATE_VERIFICATION -DWITH_DOM -DWITH_OPENSSL -DWITH_GZIP -o wstdemo wstdemo.c stdsoap2.c dom.c wstapi.c wsaapi.c wsseapi.c smdevp.c mecevp.c soapC.c soapClient.c soapServer.c -lcrypto -lssl -lz

Compile in C++ (assuming .c files treated as .cpp files):

soapcpp2 -I import wstdemo.h
c++ -DSKIP_CERTIFICATE_VERIFICATION -DWITH_DOM -DWITH_OPENSSL -DWITH_GZIP -o wstdemo wstdemo.c stdsoap2.cpp dom.cpp wstapi.c wsaapi.c wsseapi.c smdevp.c mecevp.c soapC.cpp soapClient.cpp soapServer.cpp -lcrypto -lssl -lz

Other required files:

server.pem              server private key and certificate (do not distrubute)
servercert.pem          server public certificate for public distribution
cacert.pem              root CA certificate for public distribution

Note:

The wsse.h, wsu.h, ds.h, xenc.h c14n.h files are located in 'import'.
The smdevp.*, mecevp.*, wstapi.*, wsaapi.*, and wsseapi.* files are located in 'plugin'.

Usage: wstdemo cinsz [port]

with options:

c enable chunked HTTP
i indent XML
n canonicalize XML (recommended!)
s server (stand-alone)
z enable compression

For example, to generate a SAML 2.0 request message and store it in file 'wstdemo.xml':

./wstdemo n > wstdemo.xml < /dev/null

To parse and verify this request message:

./wstdemo ns < wstdemo.xml

To run a stand-alone server:

./wstdemo ns 8080

And invoking it with a client:

./wstdemo n 8080

To test multiple calls using HTTP keep-alive, add a single digit number of runs
at the end of the options:

./wstdemo n4 8080

*/

#include "wstapi.h"
#include "wsaapi.h"
#include "wsseapi.h"
#include "wst.nsmap"

static int ssl_verify(int ok, X509_STORE_CTX *store);

#define SERVER          "http://localhost:8080"
#define PORT            8080
#define APPLIESTO       "mycompany.com"
#define USERNAME        "Username"
#define PASSWORD        "Password"

EVP_PKEY *rsa_privk = NULL;

/* server-side settings before soap_serve() to verify signatures and decrypt messages */
static int set_verify_decrypt_auto(struct soap *soap)
{
  /* verify signatures and enable decryption */
  if (soap_wsse_verify_auto(soap, SOAP_SMD_NONE, NULL, 0)
   || soap_wsse_decrypt_auto(soap, SOAP_MEC_ENV_ENC_AES256_CBC, rsa_privk, 0))
    return soap->error;
  return SOAP_OK;
}

int main(int argc, char **argv)
{
  int server = 0;
  int port = 0;
  int runs = 1;
  struct soap *soap = soap_new();
  /* register wsa plugin (optional, only if the client requires WS-Addressing rerouted messaging) */
  soap_register_plugin(soap, soap_wsa);
  /* register wsse plugin */
  soap_register_plugin(soap, soap_wsse);
  /* options */
  if (argc >= 2)
  {
    if (strchr(argv[1], 's'))
      server = 1;
    if (strchr(argv[1], 'c'))
      soap_set_omode(soap, SOAP_IO_CHUNK);
    else if (strchr(argv[1], 'y'))
      soap_set_omode(soap, SOAP_IO_STORE);
    if (strchr(argv[1], 'i'))
      soap_set_omode(soap, SOAP_XML_INDENT);
    if (strchr(argv[1], 'n'))
      soap_set_omode(soap, SOAP_XML_CANONICAL);
    if (strchr(argv[1], 'z'))
      soap_set_mode(soap, SOAP_ENC_ZLIB);
    if (isdigit(argv[1][strlen(argv[1])-1]))
    {
      runs = argv[1][strlen(argv[1])-1] - '0';
      soap_set_mode(soap, SOAP_IO_KEEPALIVE);
    }
  }
  /* port argument */
  if (argc >= 3)
    port = atoi(argv[2]);
  if (server)
  {
    FILE *fd;
    /* Get private key, certificate, and public key */
    fd = fopen("server.pem", "r");
    if (!fd)
      return soap_receiver_fault(soap, "Could not read server.pem", NULL);
    rsa_privk = PEM_read_PrivateKey(fd, NULL, NULL, (void*)"password");
    fclose(fd);
    if (!rsa_privk)
        return soap_receiver_fault(soap, "Could not read private RSA key from server.pem", NULL);
    /* HTTPS settings */
    if (soap_ssl_server_context(soap, SOAP_SSL_DEFAULT, "server.pem", "password", "cacert.pem", NULL, NULL, NULL, NULL))
    {
      soap_print_fault(soap, stderr);
      exit(1);
    }
    /* HTTPS and SAML certificate verification callback */
    soap->fsslverify = ssl_verify;
    if (port)
    {
      /* bind port */
      if (!soap_valid_socket(soap_bind(soap, NULL, port, 100)))
      {
        soap_print_fault(soap, stderr);
        exit(1);
      }
      printf("Server started at port %d\n", port);
      /* HTTP keep-alive is not trivial to support when we need to set soap_wsse_verify_auto and soap_wsse_decrypt_auto */
      if (soap->mode & SOAP_IO_KEEPALIVE)
      {
	/* set the serverloop callback to call soap_wsse_verify_auto and soap_wsse_decrypt_auto etc. for each next iteration */
	soap->fserveloop = set_verify_decrypt_auto;
      }
      while (soap_valid_socket(soap_accept(soap)))
      {
        fprintf(stderr, "Accepting connection from IP %d.%d.%d.%d\n", (int)(soap->ip>>24)&0xFF, (int)(soap->ip>>16)&0xFF, (int)(soap->ip>>8)&0xFF, (int)soap->ip&0xFF);
        /* init verify/decrypt and serve request */
        if (set_verify_decrypt_auto(soap)
         || soap_serve(soap))
        {
          soap_wsse_delete_Security(soap);
          soap_print_fault(soap, stderr);
          soap_print_fault_location(soap, stderr);
        }
        soap_destroy(soap);
        soap_end(soap);
      }
      soap_print_fault(soap, stderr);
      exit(1);
    }
    else /* CGI-style server serving messages over stdin/out */
    {
      /* init verify/decrypt and serve request */
      if (set_verify_decrypt_auto(soap)
       || soap_serve(soap))
      {
        soap_wsse_delete_Security(soap);
        soap_print_fault(soap, stderr);
        soap_print_fault_location(soap, stderr);
      }
      soap_destroy(soap);
      soap_end(soap);
    }
  }
  else /* client */
  {
    X509 *cert = NULL;
    FILE *fd;
    int soapver = 2; /* SOAP 1.2 */
    char endpoint[80];
    const char *to = endpoint;
    const char *appliesto = APPLIESTO;
    const char *username = USERNAME;
    const char *password = PASSWORD;
    int i;
    /* HTTPS settings */
    if (soap_ssl_client_context(soap, SOAP_SSL_DEFAULT, NULL, NULL, "cacert.pem", NULL, NULL))
    {
      soap_print_fault(soap, stderr);
      exit(1);
    }
    /* HTTPS and SAML certificate verification callback */
    soap->fsslverify = ssl_verify;
    /* Get private key, certificate, and public key */
    fd = fopen("server.pem", "r");
    if (!fd)
      return soap_receiver_fault(soap, "Could not read server.pem", NULL);
    rsa_privk = PEM_read_PrivateKey(fd, NULL, NULL, (void*)"password");
    fclose(fd);
    if (!rsa_privk)
      return soap_receiver_fault(soap, "Could not read private RSA key from server.pem", NULL);
    fd = fopen("servercert.pem", "r");
    if (!fd)
      return soap_receiver_fault(soap, "Could not read servercert.pem", NULL);
    cert = PEM_read_X509(fd, NULL, NULL, NULL);
    fclose(fd);
    if (!cert)
      soap_receiver_fault(soap, "Could not read certificate from servercert.pem", NULL);
    /* client sends messages to stdout or to a server port */
    if (port)
      (SOAP_SNPRINTF(endpoint, sizeof(endpoint), 37), "http://localhost:%d", port);
    else
      soap_strcpy(endpoint, sizeof(endpoint), "http://");
    /* run one or more times */
    for (i = 0; i < runs; ++i)
    {
      saml2__AssertionType *saml2;
      /* sign the request message */
      soap_wsse_add_BinarySecurityTokenX509(soap, "X509Token", cert);
      soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(soap, "#X509Token");
      soap_wsse_sign_body(soap, SOAP_SMD_SIGN_RSA_SHA256, rsa_privk, 0);
      /* init verify/decrypt of responses and make the call to request a SAML 2.0 token */
      if (set_verify_decrypt_auto(soap)
       || soap_wst_request_saml_token(soap, to, soapver, appliesto, username, password, NULL, &saml2))
      {
        soap_print_fault(soap, stderr);
      }
      else if (i == 0)
      {
        /* display some of the assertion information */
        if (saml2)
        {
          int i;
          fprintf(stderr, "SAML 2.0 received and verified OK:\n");
          for (i = 0; i < saml2->__size_AssertionType; i++)
          {
            if (saml2->__union_AssertionType[i].saml2__Statement)
            {
              /* left out SAML statements from being displayed */
            }
            if (saml2->__union_AssertionType[i].saml2__AuthnStatement)
            {
              if (saml2->__union_AssertionType[i].saml2__AuthnStatement->saml2__AuthnContext)
              {
                if (saml2->__union_AssertionType[i].saml2__AuthnStatement->saml2__AuthnContext->saml2__AuthnContextClassRef)
                  fprintf(stderr, "AuthnStatement Context: %s\n", saml2->__union_AssertionType[i].saml2__AuthnStatement->saml2__AuthnContext->saml2__AuthnContextClassRef);
              }
            }
            if (saml2->__union_AssertionType[i].saml2__AuthzDecisionStatement)
            {
              /* left out SAML decision statements from being displayed */
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
                    fprintf(stderr, "Type: %s\nValue: %s\n", saml2->__union_AssertionType[i].saml2__AttributeStatement->__union_AttributeStatementType[j].saml2__Attribute->Name, saml2->__union_AssertionType[i].saml2__AttributeStatement->__union_AttributeStatementType[j].saml2__Attribute->saml2__AttributeValue[k]);
                }
              }
            }
          }
          if (saml2->saml2__Conditions)
          {
            if (saml2->saml2__Conditions->NotBefore)
              fprintf(stderr, "Not before %s\n", soap_xsd__dateTime2s(soap, *saml2->saml2__Conditions->NotBefore));
            if (saml2->saml2__Conditions->NotOnOrAfter)
              fprintf(stderr, "Not on or after %s\n", soap_xsd__dateTime2s(soap, *saml2->saml2__Conditions->NotOnOrAfter));
          }
          fprintf(stderr, "\nAssertion data:\n\n");
          /* add SOAP_XML_CANONICAL flag to apply canonicalization, otherwise plain XML */
          soap_set_omode(soap, SOAP_XML_INDENT);
	  soap->sendfd = 2; /* send to fd=2 (stdout) */
          soap_write_saml2__AssertionType(soap, saml2);
          fprintf(stderr, "\n\nOK\n\n");
        }
        else
        {
          fprintf(stderr, "No SAML 2.0 received!\n");
        }
      }
      soap_destroy(soap);
      soap_end(soap);
    }
  }
  soap_free(soap);
}

/******************************************************************************\
 *
 *      Server-side WS-Trust Operations
 *
\******************************************************************************/

SOAP_FMAC5
int
SOAP_FMAC6
__wst__RequestSecurityToken(struct soap *soap, struct wst__RequestSecurityTokenType *request, struct wst__RequestSecurityTokenResponseType *response)
{ 
  DBGFUN("__wst__RequestSecurityToken");
  /* Check WS-Addressing */
  if (soap_wsa_check(soap))
    return soap->error;
  /* Check if RequestSecurityToken was signed (as required) */
  if (soap_wsse_verify_element(soap, SOAP_NAMESPACE_OF_wst, "RequestSecurityToken") == 0)
  {
    soap_wsse_delete_Security(soap);
    return soap_sender_fault_subcode(soap, "wst:InvalidRequest", "The request was invalid or malformed", NULL);
  }
  /* Check request */
  if (!request
   || !request->TokenType
   || !request->RequestType
   || strcmp(request->RequestType, SOAP_NAMESPACE_OF_wst "/Issue"))
    return soap_sender_fault_subcode(soap, "wst:InvalidRequest", "The request was invalid or malformed", NULL);
  /* Request SAML 2.0 token? */
  if (!strcmp(request->TokenType, "urn:oasis:names:tc:SAML:2.0:assertion"))
  {
    saml2__AssertionType *assertion;
    X509 *cert = NULL;
    char buf[1024];
    FILE *fd;
    time_t now = time(NULL), expires = now + 60*60; /* one hour */
    const char *appliesto = NULL;
    const char *username;
    /* Check Username and Password */
    username = soap_wsse_get_Username(soap);
    if (!username)
    {
      soap_wsse_delete_Security(soap);
      return soap->error;
    }
    if (strcmp(username, USERNAME) || soap_wsse_verify_Password(soap, PASSWORD))
    {
      soap_wsse_delete_Security(soap);
      return soap_wsse_fault(soap, wsse__FailedAuthentication, "Username authentication required");
    }
    soap_wsse_delete_Security(soap);
    /* Check AppliesTo */
    if (request->wsp__AppliesTo && request->wsp__AppliesTo->SOAP_WSA(EndpointReference))
      appliesto = request->wsp__AppliesTo->SOAP_WSA(EndpointReference)->Address;
    if (!appliesto || strcmp(appliesto, APPLIESTO))
      return soap_sender_fault_subcode(soap, "wst:InvalidRequest", "The request was invalid or malformed", NULL);
    if (strcmp(request->KeyType, SOAP_NAMESPACE_OF_wst "/Bearer"))
      return soap_sender_fault_subcode(soap, "wst:InvalidRequest", "The request was invalid or malformed", request->KeyType);
    /* Get private key, certificate, and public key */
    fd = fopen("server.pem", "r");
    if (!fd)
      return soap_receiver_fault(soap, "Could not read server.pem", NULL);
    rsa_privk = PEM_read_PrivateKey(fd, NULL, NULL, (void*)"password");
    fclose(fd);
    if (!rsa_privk)
        return soap_receiver_fault(soap, "Could not read private RSA key from server.pem", NULL);
    fd = fopen("servercert.pem", "r");
    if (!fd)
      return soap_receiver_fault(soap, "Could not read servercert.pem", NULL);
    cert = PEM_read_X509(fd, NULL, NULL, NULL);
    fclose(fd);
    if (!cert)
      soap_receiver_fault(soap, "Could not read certificate from servercert.pem", NULL);
    /* Create SAML 2.0 Assertion */
    assertion = (saml2__AssertionType*)soap_malloc(soap, sizeof(saml2__AssertionType));
    if (!assertion)
      return soap->error = SOAP_EOM;
    soap_default_saml2__AssertionType(soap, assertion);
    assertion->Version = (char*)"2.0";
    assertion->IssueInstant.tv_sec = now;
    assertion->IssueInstant.tv_usec = 0;
    /* Issuer = certificate issuer */
    assertion->saml2__Issuer = (struct saml2__NameIDType*)soap_malloc(soap, sizeof(struct saml2__NameIDType));
    if (!assertion->saml2__Issuer)
      return soap->error = SOAP_EOM;
    soap_default_saml2__NameIDType(soap, assertion->saml2__Issuer);
    X509_NAME_oneline(X509_get_issuer_name(cert), buf, sizeof(buf)-1);
    assertion->saml2__Issuer->__item = soap_strdup(soap, buf);
    /* Conditions */
    assertion->saml2__Conditions = (struct saml2__ConditionsType*)soap_malloc(soap, sizeof(struct saml2__ConditionsType));
    if (!assertion->saml2__Conditions)
      return soap->error = SOAP_EOM;
    soap_default_saml2__ConditionsType(soap, assertion->saml2__Conditions);
    assertion->saml2__Conditions->NotBefore = (xsd__dateTime*)soap_malloc(soap, sizeof(xsd__dateTime));
    if (!assertion->saml2__Conditions->NotBefore)
      return soap->error = SOAP_EOM;
    assertion->saml2__Conditions->NotBefore->tv_sec = now;
    assertion->saml2__Conditions->NotBefore->tv_usec = 0;
    assertion->saml2__Conditions->NotOnOrAfter = (xsd__dateTime*)soap_malloc(soap, sizeof(xsd__dateTime));
    if (!assertion->saml2__Conditions->NotOnOrAfter)
      return soap->error = SOAP_EOM;
    assertion->saml2__Conditions->NotOnOrAfter->tv_sec = expires;
    assertion->saml2__Conditions->NotOnOrAfter->tv_usec = 0;
    /* Conditions/AudienceRestriction = wsp:AppliesTo */
    assertion->saml2__Conditions->__size_ConditionsType = 1;
    assertion->saml2__Conditions->__union_ConditionsType = (struct __saml2__union_ConditionsType*)soap_malloc(soap, sizeof(struct __saml2__union_ConditionsType));
    if (!assertion->saml2__Conditions->__union_ConditionsType)
      return soap->error = SOAP_EOM;
    soap_default___saml2__union_ConditionsType(soap, assertion->saml2__Conditions->__union_ConditionsType);
    assertion->saml2__Conditions->__union_ConditionsType->saml2__AudienceRestriction = (struct saml2__AudienceRestrictionType*)soap_malloc(soap, sizeof(struct saml2__AudienceRestrictionType));
    if (!assertion->saml2__Conditions->__union_ConditionsType->saml2__AudienceRestriction)
      return soap->error = SOAP_EOM;
    soap_default_saml2__AudienceRestrictionType(soap, assertion->saml2__Conditions->__union_ConditionsType->saml2__AudienceRestriction);
    assertion->saml2__Conditions->__union_ConditionsType->saml2__AudienceRestriction->__sizeAudience = 1;
    assertion->saml2__Conditions->__union_ConditionsType->saml2__AudienceRestriction->saml2__Audience = &request->wsp__AppliesTo->SOAP_WSA(EndpointReference)->Address;
    /* AuthnStatement and AttributeStatement */
    assertion->__size_AssertionType = 2;
    assertion->__union_AssertionType = (struct __saml2__union_AssertionType*)soap_malloc(soap, 2 * sizeof(struct __saml2__union_AssertionType));
    if (!assertion->__union_AssertionType)
      return soap->error = SOAP_EOM;
    soap_default___saml2__union_AssertionType(soap, &assertion->__union_AssertionType[0]);
    soap_default___saml2__union_AssertionType(soap, &assertion->__union_AssertionType[1]);
    assertion->__union_AssertionType[0].saml2__AuthnStatement = (struct saml2__AuthnStatementType*)soap_malloc(soap, sizeof(struct saml2__AuthnStatementType));
    if (!assertion->__union_AssertionType[0].saml2__AuthnStatement)
      return soap->error = SOAP_EOM;
    soap_default_saml2__AuthnStatementType(soap, assertion->__union_AssertionType[0].saml2__AuthnStatement);
    assertion->__union_AssertionType[0].saml2__AuthnStatement->AuthnInstant.tv_sec = now;
    assertion->__union_AssertionType[0].saml2__AuthnStatement->AuthnInstant.tv_usec = now;
    assertion->__union_AssertionType[0].saml2__AuthnStatement->saml2__AuthnContext = (struct saml2__AuthnContextType*)soap_malloc(soap, sizeof(struct saml2__AuthnContextType));
    if (!assertion->__union_AssertionType[0].saml2__AuthnStatement->saml2__AuthnContext)
      return soap->error = SOAP_EOM;
    soap_default_saml2__AuthnContextType(soap, assertion->__union_AssertionType[0].saml2__AuthnStatement->saml2__AuthnContext);
    assertion->__union_AssertionType[0].saml2__AuthnStatement->saml2__AuthnContext->saml2__AuthnContextClassRef = (char*)"urn:oasis:names:tc:SAML:2.0:ac:classes:PasswordProtectedTransport";
    assertion->__union_AssertionType[1].saml2__AttributeStatement = (struct saml2__AttributeStatementType*)soap_malloc(soap, sizeof(struct saml2__AttributeStatementType));
    if (!assertion->__union_AssertionType[1].saml2__AttributeStatement)
      return soap->error = SOAP_EOM;
    soap_default_saml2__AttributeStatementType(soap, assertion->__union_AssertionType[1].saml2__AttributeStatement);
    assertion->__union_AssertionType[1].saml2__AttributeStatement->__size_AttributeStatementType = 1;
    assertion->__union_AssertionType[1].saml2__AttributeStatement->__union_AttributeStatementType = (struct __saml2__union_AttributeStatementType*)soap_malloc(soap, sizeof(struct __saml2__union_AttributeStatementType));
    if (!assertion->__union_AssertionType[1].saml2__AttributeStatement->__union_AttributeStatementType)
      return soap->error = SOAP_EOM;
    soap_default___saml2__union_AttributeStatementType(soap, assertion->__union_AssertionType[1].saml2__AttributeStatement->__union_AttributeStatementType);
    assertion->__union_AssertionType[1].saml2__AttributeStatement->__union_AttributeStatementType->saml2__Attribute = (struct saml2__AttributeType*)soap_malloc(soap, sizeof(struct saml2__AttributeType));
    if (!assertion->__union_AssertionType[1].saml2__AttributeStatement->__union_AttributeStatementType->saml2__Attribute)
      return soap->error = SOAP_EOM;
    soap_default_saml2__AttributeType(soap, assertion->__union_AssertionType[1].saml2__AttributeStatement->__union_AttributeStatementType->saml2__Attribute);
    assertion->__union_AssertionType[1].saml2__AttributeStatement->__union_AttributeStatementType->saml2__Attribute->Name = (char*)username;

    /* more logic can go here to populate SAML 2.0 assertion subject, conditions, statements, and attributes */

    /* sign the SAML token */
    if (soap_wsse_sign_saml2(soap, assertion, SOAP_SMD_SIGN_RSA_SHA256, rsa_privk, 0, cert))
      return soap->error;

    /* this part if for testing the validity of the SAML token only and can be omitted */
    /* assumes soap->cacert = "cacert.pem"; */ /* already set with soap_ssl_server_context() */
    if (soap_wsse_verify_saml2(soap, assertion))
      return soap_receiver_fault(soap, "Cannot sign SAML token (invalid key or certificate)", NULL);

    response->RequestedSecurityToken = (wst__RequestedSecurityTokenType*)soap_malloc(soap, sizeof(wst__RequestedSecurityTokenType));
    if (!response->RequestedSecurityToken)
      return soap->error = SOAP_EOM;
    soap_default_wst__RequestedSecurityTokenType(soap, response->RequestedSecurityToken);
    response->RequestedSecurityToken->saml2__Assertion = assertion;
    /* sign */
    if (soap_wsse_add_BinarySecurityTokenX509(soap, "X509Token", cert)
     || soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(soap, "#X509Token")
     || soap_wsse_sign_body(soap, SOAP_SMD_SIGN_RSA_SHA256, rsa_privk, 0))
    {
      soap_wsse_delete_Security(soap);
      return soap->error;
    }
  }

  /* other WS-Trust logic goes here to populate RSTR */

  response->Context = request->Context;
  response->TokenType = request->TokenType;
  response->RequestType = request->RequestType;
  response->KeyType = request->KeyType;
  return soap_wsa_reply(soap, NULL, soap_wst_rstr_action);
}

SOAP_FMAC5
int
SOAP_FMAC6
__wst__RequestSecurityTokenResponse(struct soap *soap, struct wst__RequestSecurityTokenResponseType *request, struct wst__RequestSecurityTokenResponseCollectionType *response)
{ 
  (void)soap;
  (void)request;
  (void)response;

  /* service logic goes here to populate RSTRC */

  return soap_wsa_reply(soap, NULL, soap_wst_rstrc_action);
}

SOAP_FMAC5
int
SOAP_FMAC6
__wst__RequestSecurityTokenCollection(struct soap *soap, struct wst__RequestSecurityTokenCollectionType *request, struct wst__RequestSecurityTokenResponseCollectionType *response)
{ 
  (void)soap;
  (void)request;
  (void)response;

  /* service logic goes here to populate RSTRC */

  return soap_wsa_reply(soap, NULL, soap_wst_rstrc_action);
}

/******************************************************************************\
 *
 *      WS-Addressing relayed SOAP-ENV:Fault Handler for FaultTo Server
 *
\******************************************************************************/

SOAP_FMAC5
int
SOAP_FMAC6
SOAP_ENV__Fault(struct soap *soap, char *faultcode, char *faultstring, char *faultactor, struct SOAP_ENV__Detail *detail, struct SOAP_ENV__Code *SOAP_ENV__Code, struct SOAP_ENV__Reason *SOAP_ENV__Reason, char *SOAP_ENV__Node, char *SOAP_ENV__Role, struct SOAP_ENV__Detail *SOAP_ENV__Detail)
{
  /* populate the fault struct from the operation arguments to print it */
  soap_fault(soap);
  /* SOAP 1.1 */
  soap->fault->faultcode = faultcode;
  soap->fault->faultstring = faultstring;
  soap->fault->faultactor = faultactor;
  soap->fault->detail = detail;
  /* SOAP 1.2 */
  soap->fault->SOAP_ENV__Code = SOAP_ENV__Code;
  soap->fault->SOAP_ENV__Reason = SOAP_ENV__Reason;
  soap->fault->SOAP_ENV__Node = SOAP_ENV__Node;
  soap->fault->SOAP_ENV__Role = SOAP_ENV__Role;
  soap->fault->SOAP_ENV__Detail = SOAP_ENV__Detail;
  /* set error */
  soap->error = SOAP_FAULT;
  return soap_send_empty_response(soap, SOAP_OK); /* HTTP 202 Accepted */
}

/******************************************************************************\
 *
 *      Callbacks
 *
\******************************************************************************/

static int ssl_verify(int ok, X509_STORE_CTX *store)
{
#ifdef SKIP_CERTIFICATE_VERIFICATION
  ok = 1;
  (void)store;
#else
  /* HTTPS and SAML certificate verification, return 0 when fails 1 when ok */
  if (!ok)
  {
    char buf[1024];
    int err = X509_STORE_CTX_get_error(store);
    X509 *cert = X509_STORE_CTX_get_current_cert(store);
    switch (err)
    {
      /* these may be tolerable exceptions:
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
      */
      default:
        fprintf(stderr, "SSL verify error %d or warning with certificate at depth %d: %s\n", err, X509_STORE_CTX_get_error_depth(store), X509_verify_cert_error_string(err));
        X509_NAME_oneline(X509_get_issuer_name(cert), buf, sizeof(buf));
        fprintf(stderr, "  certificate issuer:  %s\n", buf);
        X509_NAME_oneline(X509_get_subject_name(cert), buf, sizeof(buf));
        fprintf(stderr, "  certificate subject: %s\n", buf);
    }
  }
#endif
  return ok;
}

