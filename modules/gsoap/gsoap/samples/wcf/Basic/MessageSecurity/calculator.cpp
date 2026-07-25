/*
        calculator.cpp

        WCF BasicMessageSecurity demo

        See the README.txt

gSOAP XML Web services tools
Copyright (C) 2000-2016, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

This program is released under the GPL with the additional exemption that
compiling, linking, and/or using OpenSSL is allowed.
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

#include "soapBasicHttpBinding_USCOREICalculatorService.h"
#include "soapBasicHttpBinding_USCOREICalculatorProxy.h"
#include "BasicHttpBinding_USCOREICalculator.nsmap"

#include "wsseapi.h"                    /* gsoap/plugin/wsseapi.h */

/* user options */

#define ENCRYPTION                      /* enable encryption in addition to signing */
/* #define UNENCRYPTED_SIGNATURE */     /* disable encryption of signature */
/* #define CERTVERIFY */                /* enable verification of inbound certs, see ssl_verify() */

const char *endpoint = "http://10.0.1.5:8000/ServiceModelSamples/service"; // Set to the service endpoint

X509     *srv_cert   = NULL, *clt_cert   = NULL;
EVP_PKEY *srv_privk  = NULL, *clt_privk  = NULL;
EVP_PKEY *srv_pubk   = NULL, *clt_pubk   = NULL;
char      srv_subject[256],   clt_subject[256];
char      srv_issuer[256],    clt_issuer[256];
char     *srv_serial = NULL, *clt_serial = NULL;

static int chk_security(struct soap *soap);
static int set_srv_security(struct soap *soap);
static int set_clt_security(struct soap *soap);
static const void *token_handler(struct soap *soap, int *alg, const char *keyname, const unsigned char *keyid, int keyidlen, int *keylen);
static int ssl_verify(int, X509_STORE_CTX*);

/* primitive type managed allocator */
template<class T> T * soap_make(struct soap *soap, T val = T())
{
  T *p = (T*)soap_malloc(soap, sizeof(T));
  *p = val;
  return p;
}

int main(int argc, char **argv)
{
  FILE *fd;
  BIGNUM *bn = BN_new();

  /* init OpenSSL before any OpenSSL and crypto operations */
  soap_ssl_init();

  /* server's RSA private key for signing */
  if ((fd = fopen("serverWCF.pem", "r")))
  {
    srv_privk = PEM_read_PrivateKey(fd, NULL, NULL, (void*)"password");
    fclose(fd);
    if (!srv_privk)
    {
      fprintf(stderr, "Could not read private RSA key from server.pem\n");
      exit(1);
    }
  }
  else
  {
    fprintf(stderr, "Could not read server.pem\n");
  }

  /* server's certificate with public key for encryption and signature verification */
  if ((fd = fopen("servercertWCF.pem", "r")))
  {
    srv_cert = PEM_read_X509(fd, NULL, NULL, NULL);
    fclose(fd);
    if (!srv_cert)
    {
      fprintf(stderr, "Could not read certificate from clientcert.pem\n");
      exit(1);
    }
  }
  else
  {
    fprintf(stderr, "Could not read server.pem\n");
  }

  /* server's RSA public key from the certificate */
  srv_pubk = X509_get_pubkey(srv_cert);
  if (!srv_pubk)
  {
    fprintf(stderr, "Could not get public key from certificate\n");
    exit(1);
  }

  /* server's certificate subject, issuer and serial number */
  X509_NAME_oneline(X509_get_subject_name(srv_cert), srv_subject, sizeof(srv_subject));
  X509_NAME_oneline(X509_get_issuer_name(srv_cert), srv_issuer, sizeof(srv_issuer));
  ASN1_INTEGER_to_BN(X509_get_serialNumber(srv_cert), bn);
  srv_serial = BN_bn2dec(bn);

  fprintf(stderr, "Server certificate issuer = \"%s\"\n", srv_issuer);
  fprintf(stderr, "Server certificate serial = %s (%s)\n", BN_bn2hex(bn), srv_serial);

  /* client's RSA private key for signing */
  if ((fd = fopen("clientWCF.pem", "r")))
  {
    clt_privk = PEM_read_PrivateKey(fd, NULL, NULL, (void*)"password");
    fclose(fd);
    if (!clt_privk)
    {
      fprintf(stderr, "Could not read private RSA key from server.pem\n");
      exit(1);
    }
  }
  else
    fprintf(stderr, "Could not read server.pem\n");

  /* client's certificate with public key for encryption and signature verification */
  if ((fd = fopen("clientcertWCF.pem", "r")))
  {
    clt_cert = PEM_read_X509(fd, NULL, NULL, NULL);
    fclose(fd);
    if (!clt_cert)
    {
      fprintf(stderr, "Could not read certificate from clientcert.pem\n");
      exit(1);
    }
  }
  else
    fprintf(stderr, "Could not read server.pem\n");

  /* client's RSA public key from the certificate */
  clt_pubk = X509_get_pubkey(clt_cert);
  if (!clt_pubk)
  {
    fprintf(stderr, "Could not get public key from certificate\n");
    exit(1);
  }

  /* client's certificate subject, issuer and serial number */
  X509_NAME_oneline(X509_get_subject_name(clt_cert), clt_subject, sizeof(clt_subject));
  X509_NAME_oneline(X509_get_issuer_name(clt_cert), clt_issuer, sizeof(clt_issuer));
  ASN1_INTEGER_to_BN(X509_get_serialNumber(clt_cert), bn);
  clt_serial = BN_bn2dec(bn);

  fprintf(stderr, "Client certificate issuer = \"%s\"\n", clt_issuer);
  fprintf(stderr, "Client certificate serial = %s (%s)\n", BN_bn2hex(bn), clt_serial);

  OPENSSL_free(bn);

  if (argc >= 2)
  {
    // Service sample application
    int port = atoi(argv[1]);

    BasicHttpBinding_USCOREICalculatorService service(SOAP_XML_CANONICAL);

    soap_register_plugin_arg(service.soap, soap_wsse, (void*)token_handler);

    service.soap->send_timeout = service.soap->recv_timeout = 10; // 10 sec

    /* need cacert to verify certificates */
    service.soap->cafile = "clientcertWCF.pem";
    service.soap->fsslverify = ssl_verify;

    /* immediately reuse the port when restarting the service */
    service.soap->bind_flags = SO_REUSEADDR;

    if (!soap_valid_socket(service.bind(NULL, port, 100)))
    {
      service.soap_stream_fault(std::cerr);
      exit(1);
    }

    std::cerr << "Server running" << std::endl;

    for (;;)
    {
      soap_wsse_verify_auto(service.soap, SOAP_WSSE_IGNORE_EXTRA_REFS, NULL, 0);
      soap_wsse_decrypt_auto(service.soap, SOAP_MEC_ENV_DEC_AES256_CBC, srv_privk, 0);

      if (!soap_valid_socket(service.accept()) || service.serve())
        service.soap_stream_fault(std::cerr);

      service.destroy();
    }
  }
  else
  {
    // Client sample application
    BasicHttpBinding_USCOREICalculatorProxy proxy(endpoint, SOAP_XML_CANONICAL);

    soap_register_plugin_arg(proxy.soap, soap_wsse, (void*)token_handler);

    proxy.soap->send_timeout = proxy.soap->recv_timeout = 10; // 10 sec

    /* need cacert to verify certificates */
    proxy.soap->cafile = "servercertWCF.pem";
    proxy.soap->fsslverify = ssl_verify;

    double n1 = 3.14, n2 = 1.41;

    if (set_clt_security(proxy.soap))
    {
      proxy.soap_stream_fault(std::cerr);
    }
    else
    { 
      _mssamm__Add areq;
      _mssamm__AddResponse ares;
      areq.n1 = &n1;
      areq.n2 = &n2;
      if (proxy.Add(&areq, &ares) == SOAP_OK && ares.AddResult && chk_security(proxy.soap) == SOAP_OK)
        printf("Add(%g, %g) = %g\n", *areq.n1, *areq.n2, *ares.AddResult);
      else
        proxy.soap_stream_fault(std::cerr);
      proxy.destroy();
    }

    if (set_clt_security(proxy.soap))
    {
      proxy.soap_stream_fault(std::cerr);
    }
    else
    { 
      _mssamm__Subtract sreq;
      _mssamm__SubtractResponse sres;
      sreq.n1 = &n1;
      sreq.n2 = &n2;
      if (proxy.Subtract(&sreq, &sres) == SOAP_OK && sres.SubtractResult && chk_security(proxy.soap) == SOAP_OK)
        printf("Subtract(%g, %g) = %g\n", *sreq.n1, *sreq.n2, *sres.SubtractResult);
      else
        proxy.soap_stream_fault(std::cerr);
      proxy.destroy();
    }

    if (set_clt_security(proxy.soap))
    {
      proxy.soap_stream_fault(std::cerr);
    }
    else
    { 
      _mssamm__Multiply mreq;
      _mssamm__MultiplyResponse mres;
      mreq.n1 = &n1;
      mreq.n2 = &n2;
      if (proxy.Multiply(&mreq, &mres) == SOAP_OK && mres.MultiplyResult && chk_security(proxy.soap) == SOAP_OK)
        printf("Multiply(%g, %g) = %g\n", *mreq.n1, *mreq.n2, *mres.MultiplyResult);
      else
        proxy.soap_stream_fault(std::cerr);
      proxy.destroy();
    }

    if (set_clt_security(proxy.soap))
    {
      proxy.soap_stream_fault(std::cerr);
    }
    else
    { 
      _mssamm__Divide dreq;
      _mssamm__DivideResponse dres;
      dreq.n1 = &n1;
      dreq.n2 = &n2;
      if (proxy.Divide(&dreq, &dres) == SOAP_OK && dres.DivideResult && chk_security(proxy.soap) == SOAP_OK)
        printf("Divide(%g, %g) = %g\n", *dreq.n1, *dreq.n2, *dres.DivideResult);
      else
        proxy.soap_stream_fault(std::cerr);
      proxy.destroy();
    }

    proxy.destroy();
  }

  OPENSSL_free(clt_serial);
  OPENSSL_free(srv_serial);

  return 0;
}

/******************************************************************************\
 *
 *      Check and set security
 *
\******************************************************************************/

/* Check WS-Security properties of a message */
static int chk_security(struct soap *soap)
{
  X509 *cert = soap_wsse_get_KeyInfo_SecurityTokenReferenceX509(soap);
  if (cert)
  {
    char buf[1024];
    /* if certificate is included, we may want to check if the certificate is known to us */
    X509_NAME_oneline(X509_get_subject_name(cert), buf, sizeof(buf));
    if (!strstr(buf, srv_subject) && !strstr(buf, clt_subject))
    {
      fprintf(stderr, "Warning: certificate from %s is unknown\n", buf);

      strncat(buf, ": unrecognized subject name", sizeof(buf)-strlen(buf)-1);
      buf[sizeof(buf)-1] = '\0';

      return soap_wsse_fault(soap, wsse__InvalidSecurityToken, buf);
    }
  }

  /* Valid timestamp required */
  if (soap_wsse_verify_Timestamp(soap))
  {
    soap_wsse_delete_Security(soap);
    return soap->error;
  }

  /* Timestamp must be signed */
  if (soap_wsse_verify_element(soap, "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd", "Timestamp") == 0)
  {
    soap_wsse_delete_Security(soap);
    return soap_wsse_sender_fault(soap, "Timestamp not signed", NULL);
  }

  /* Body must be signed */
  if (soap_wsse_verify_body(soap))
  {
    soap_wsse_delete_Security(soap);
    return soap->error;
  }

  soap_wsse_delete_Security(soap);

  return SOAP_OK;
}

/* Set WS-Security server-side properties after an operation */
static int set_srv_security(struct soap *soap)
{
  if (soap_wsse_add_Timestamp(soap, "Time", 60) // 1 min to expire
   || soap_wsse_add_BinarySecurityTokenX509(soap, "X509Token", srv_cert)
   || soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(soap, "#X509Token")
   || soap_wsse_sign_body(soap, SOAP_SMD_SIGN_RSA_SHA1, srv_privk, 0)
#ifdef ENCRYPTION
#ifdef UNENCRYPTED_SIGNATURE
   || soap_wsse_add_EncryptedKey(soap, SOAP_MEC_ENV_ENC_AES256_CBC, NULL, clt_cert, NULL, clt_issuer+1, clt_serial)
#else
   || soap_wsse_add_EncryptedKey_encrypt_only(soap, SOAP_MEC_ENV_ENC_AES256_CBC, NULL, clt_cert, NULL, clt_issuer+1, clt_serial, "ds:Signature SOAP-ENV:Body")
#endif
#endif
    )
    return soap->error;

  return SOAP_OK;
}

/* Set WS-Security client-side properties before an operation */
static int set_clt_security(struct soap *soap)
{
  // soap_wsse_set_wsu_id(soap, "ds:Signature"); // to encrypt ds:Signature
  if (soap_wsse_add_Timestamp(soap, "Time", 60) // 1 min to expire
   || soap_wsse_add_BinarySecurityTokenX509(soap, "X509Token", clt_cert)
   || soap_wsse_add_KeyInfo_SecurityTokenReferenceX509(soap, "#X509Token")
   || soap_wsse_sign_body(soap, SOAP_SMD_SIGN_RSA_SHA1, clt_privk, 0)
   || soap_wsse_verify_auto(soap, SOAP_WSSE_IGNORE_EXTRA_REFS, NULL, 0)
#ifdef ENCRYPTION
#ifdef UNENCRYPTED_SIGNATURE
   || soap_wsse_add_EncryptedKey(soap, SOAP_MEC_ENV_ENC_AES256_CBC, NULL, srv_cert, NULL, srv_issuer+1, srv_serial)
#else
   || soap_wsse_add_EncryptedKey_encrypt_only(soap, SOAP_MEC_ENV_ENC_AES256_CBC, NULL, srv_cert, NULL, srv_issuer+1, srv_serial, "ds:Signature SOAP-ENV:Body")
#endif
   || soap_wsse_decrypt_auto(soap, SOAP_MEC_ENV_DEC_AES256_CBC, clt_privk, 0)
#endif
   )
    return soap->error;

  return SOAP_OK;
}

/******************************************************************************\
 *
 *      Service operations
 *
\******************************************************************************/

int BasicHttpBinding_USCOREICalculatorService::Add(_mssamm__Add *Add, _mssamm__AddResponse *AddResponse)
{
  double *res = soap_make<double>(soap);

  if (chk_security(soap))
    return soap->error;

  if (Add && Add->n1 && Add->n2)
    *res = *Add->n1 + *Add->n2;
  else
    return soap_sender_fault(soap, "Invalid data", NULL);
  AddResponse->AddResult = res;

  return set_srv_security(soap);
}

int BasicHttpBinding_USCOREICalculatorService::Subtract(_mssamm__Subtract *Subtract, _mssamm__SubtractResponse *SubtractResponse)
{
  double *res = soap_make<double>(soap);

  if (chk_security(soap))
    return soap->error;

  if (Subtract && Subtract->n1 && Subtract->n2)
    *res = *Subtract->n1 - *Subtract->n2;
  else
    return soap_sender_fault(soap, "Invalid data", NULL);
  SubtractResponse->SubtractResult = res;

  return set_srv_security(soap);
}

int BasicHttpBinding_USCOREICalculatorService::Multiply(_mssamm__Multiply *Multiply, _mssamm__MultiplyResponse *MultiplyResponse)
{
  double *res = soap_make<double>(soap);

  if (chk_security(soap))
    return soap->error;

  if (Multiply && Multiply->n1 && Multiply->n2)
    *res = *Multiply->n1 * *Multiply->n2;
  else
    return soap_sender_fault(soap, "Invalid data", NULL);
  MultiplyResponse->MultiplyResult = res;

  return set_srv_security(soap);
}

int BasicHttpBinding_USCOREICalculatorService::Divide(_mssamm__Divide *Divide, _mssamm__DivideResponse *DivideResponse)
{
  double *res = soap_make<double>(soap);

  if (chk_security(soap))
    return soap->error;

  if (Divide && Divide->n1 && Divide->n2 && *Divide->n2 != 0.0)
    *res = *Divide->n1 / *Divide->n2;
  else
    return soap_sender_fault(soap, "Invalid data", NULL);
  DivideResponse->DivideResult = res;

  return set_srv_security(soap);
}

/******************************************************************************\
 *
 *      WSSE Token Handler
 *
\******************************************************************************/

static char hmac_key[16] = /* Dummy HMAC key for signature test */
{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };

static char des_key[20] = /* Dummy 160-bit triple DES key for encryption test */
{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 };

static const void *token_handler(struct soap *soap, int *alg, const char *keyname, const unsigned char *keyid, int keyidlen, int *keylen)
{
  const char *ctxId;
  struct ds__X509IssuerSerialType *issuer;
  switch (*alg)
  {
    case SOAP_SMD_VRFY_DSA_SHA1:
    case SOAP_SMD_VRFY_DSA_SHA256:
    case SOAP_SMD_VRFY_RSA_SHA1:
    case SOAP_SMD_VRFY_RSA_SHA224:
    case SOAP_SMD_VRFY_RSA_SHA256:
    case SOAP_SMD_VRFY_RSA_SHA384:
    case SOAP_SMD_VRFY_RSA_SHA512:
      /* Signature verification requires a certificate with a public key */
      if (keyname && !strcmp(keyname, "Client"))
        return (const void*)clt_cert;
      if (keyname && !strcmp(keyname, "Server"))
        return (const void*)srv_cert;
      /* Use KeyInfo/SecurityTokenReference/X509Data */
      issuer = soap_wsse_get_KeyInfo_SecurityTokenReferenceX509Data(soap);
      if (issuer)
      {
        if (!strcmp(issuer->X509IssuerName, "CN=Root Agency")
         && !strcmp(issuer->X509SerialNumber, "-79441640260855276448009124614332182350"))
          return (const void*)clt_cert;
        if (!strcmp(issuer->X509IssuerName, "CN=Root Agency")
         && !strcmp(issuer->X509SerialNumber, "-154614137026298720537726749139640359303"))
          return (const void*)srv_cert;
      }
      break;
    case SOAP_SMD_HMAC_SHA1:
    case SOAP_SMD_HMAC_SHA224:
    case SOAP_SMD_HMAC_SHA256:
    case SOAP_SMD_HMAC_SHA384:
    case SOAP_SMD_HMAC_SHA512:
      /* HMAC digests requires a shared secret key */
      /* WS-SecureConversation: get & check context token ID */
      if ((keyname && !strcmp(keyname, "Shared"))
       || ((ctxId = soap_wsse_get_SecurityContextToken(soap)) && !strcmp(ctxId, "Context")))
      {
        *keylen = sizeof(hmac_key);
        return (const void*)hmac_key; /* signature verification with secret key */
      }
      break;
    case SOAP_MEC_ENV_DEC_DES_CBC:
    case SOAP_MEC_ENV_DEC_AES128_CBC:
    case SOAP_MEC_ENV_DEC_AES192_CBC:
    case SOAP_MEC_ENV_DEC_AES256_CBC:
    case SOAP_MEC_ENV_DEC_AES512_CBC:
      /* Envelope decryption requires a private key */
      /* Use KeyInfo/SecurityTokenReference/X509Data */
      issuer = soap_wsse_get_KeyInfo_SecurityTokenReferenceX509Data(soap);
      if (issuer)
      {
        if (!strcmp(issuer->X509IssuerName, "CN=Root Agency")
         && !strcmp(issuer->X509SerialNumber, "-79441640260855276448009124614332182350"))
          return (const void*)clt_cert;
        if (!strcmp(issuer->X509IssuerName, "CN=Root Agency")
         && !strcmp(issuer->X509SerialNumber, "-154614137026298720537726749139640359303"))
          return (const void*)srv_cert;
      }
      break;
      if (keyname && !strcmp(keyname, "Client"))
        return (const void*)clt_privk;
      if (keyname && !strcmp(keyname, "Server"))
        return (const void*)srv_privk;
      break;
    case SOAP_MEC_DEC_DES_CBC:
    case SOAP_MEC_DEC_AES128_CBC:
    case SOAP_MEC_DEC_AES192_CBC:
    case SOAP_MEC_DEC_AES256_CBC:
    case SOAP_MEC_DEC_AES512_CBC:
      /* Decryption requires a shared secret key */
      /* WS-SecureConversation: get & check context token ID */
      if ((keyname && !strcmp(keyname, "Shared"))
       || ((ctxId = soap_wsse_get_SecurityContextToken(soap)) && !strcmp(ctxId, "Context")))
      {
        *keylen = sizeof(des_key);
        return (const void*)des_key; /* decryption with secret key */
      }
  }
  fprintf(stderr, "Warning: token handler returned NULL key\n");
  return NULL; /* fail */
}

/******************************************************************************\
 *
 *      Optional certificate verification
 *
\******************************************************************************/

static int ssl_verify(int ok, X509_STORE_CTX *store)
{ /* put certificate verification here, return 0 to fail, 1 to force success */
#ifdef CERTVERIFY
  if (!ok)
  {
    char buf[1024];
    int err = X509_STORE_CTX_get_error(store);
    X509 *cert = X509_STORE_CTX_get_current_cert(store);
    /* accept self-signed certificates and certificates out of date - modify as you see fit */
    switch (err)
    {
      case X509_V_ERR_CERT_NOT_YET_VALID:
      case X509_V_ERR_CERT_HAS_EXPIRED:
      case X509_V_ERR_DEPTH_ZERO_SELF_SIGNED_CERT:
      case X509_V_ERR_SELF_SIGNED_CERT_IN_CHAIN:
      case X509_V_ERR_UNABLE_TO_GET_CRL:
      case X509_V_ERR_CRL_NOT_YET_VALID:
      case X509_V_ERR_CRL_HAS_EXPIRED:
        /* override with ok = 1 */
        X509_STORE_CTX_set_error(store, X509_V_OK);
        ok = 1;
        break;
      default:
        /* print errors to stderr */
        fprintf(stderr, "SSL verify error %d or warning with certificate at depth %d: %s\n", err, X509_STORE_CTX_get_error_depth(store), X509_verify_cert_error_string(err));
        X509_NAME_oneline(X509_get_issuer_name(cert), buf, sizeof(buf));
        fprintf(stderr, "  certificate issuer:  %s\n", buf);
        X509_NAME_oneline(X509_get_subject_name(cert), buf, sizeof(buf));
        fprintf(stderr, "  certificate subject: %s\n", buf);
    }
  }
  return ok;
#else
  /* Note: return 1 to try to continue, but unsafe progress will be terminated by OpenSSL */
  return 1;
#endif
}
