/*
	ssl_setup.c / ssl_setup.cpp

        See instructions below.

gSOAP XML Web services tools
Copyright (C) 2000-2019, Robert van Engelen, Genivia Inc., All Rights Reserved.
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
Copyright (C) 2000-2010, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

/*
   Easy TLS/SSL setup with soap_ssl_client_setup() to replace soap_ssl_client_context():
   - automatically uses certificates stored in Unix/Linux common locations
   - automatically uses Windows system certificate store
   - when -DWITH_WININET is defined, uses the gSOAP WinInet plugin with WinInet system certificate store
   - when -DWITH_CURL is defined, uses the gSOAP CURL plugin with CURL certificate store

   Requires:
   - compilation with -DWITH_OPENSSL or -DWITH_GNUTLS
   - compilation with -DWITH_WININET when the gSOAP WinInet plugin is used
   - compilation with -DWITH_CURL when the gSOAP CURL plugin is used

   Example usage:

   #include "thread_setup.h" // CRYPTO_thread_setup, CRYPTO_thread_cleanup
   #include "ssl_setup.h"    // soap_ssl_client_setup, soap_ssl_client_cleanup

   int main()
   {
     // OpenSSL versions prior to 1.1.0 require mutex locks
     if (CRYPTO_thread_setup())
       exit(EXIT_FAILURE);

     struct soap *soap = soap_new();

     if (soap_ssl_client_setup(
         soap,
         SOAP_SSL_DEFAULT, // authenticate servers
         NULL,             // a keyfile is required only when client must authenticate to server, NULL otherwise
         NULL,             // password to read the key file (not used with GNUTLS), NULL otherwise
         NULL,             // optional file name of certificates PEM file, NULL to search certificate stores
         NULL              // optional path to certificates PEM files, NULL to search certificate stores
     ))
     {
       soap_print_fault(soap, stderr);
       exit(EXIT_FAILURE);
     }

     ... // rest of the code to run the client making https and http calls
     ... // use soap_copy(soap) to create copies of the context with this setup
     ... // which is efficient and cheap to execute, or use soap_new() and
     ... // soap_ssl_client_setup() again to setup contexts but this is more
     ... // expensive since it searches for certificate files again
 
     CRYPTO_thread_cleanup();
  }
*/

#include "stdsoap2.h"

#if defined(WIN32) || defined(_WIN32) || defined(_WIN64)
#if (defined(WITH_OPENSSL) || defined(WITH_GNUTLS)) && !defined(WITH_WININET) && !defined(GSOAP_WIN_WININET) && !defined(WITH_CURL)
#include <openssl/opensslv.h>
#endif
#ifndef SOAP_CLIENT_OS_WINDOWS
#define SOAP_CLIENT_OS_WINDOWS
#endif
#endif

#if (defined(WITH_OPENSSL) || defined(WITH_GNUTLS)) && !defined(SOAP_CLIENT_OS_WINDOWS) && !defined(WITH_WININET) && !defined(GSOAP_WIN_WININET) && !defined(WITH_CURL)

#include <sys/stat.h> 

/* from curl configure or https://serverfault.com/questions/62496/ssl-certificate-location-on-unix-linux */
static const char *list_candidate_cacert[] = 
{
  "/etc/ssl/certs/ca-certificates.crt",
  "/etc/pki/tls/certs/ca-bundle.crt",
  "/etc/ssl/cert.pem",
  "/etc/pki/tls/cacert.pem",
  "/etc/ssl/ca-bundle.pem",
  "/etc/pki/ca-trust/extracted/pem/tls-ca-bundle.pem",
  "/usr/share/ssl/certs/ca-bundle.crt",
  "/usr/local/share/certs/ca-root-nss.crt",
  "/usr/local/etc/openssl/cert.pem",
  NULL
};

const char *search_ssl_cacert_default_file()
{
  int i = 0;
  for (;;)
  {
    const char *fname = list_candidate_cacert[i++];
    if (!fname) 
      break;
    if (access(fname, F_OK) != -1)
      return fname;
  }
  return NULL;
}

const char *list_candidate_capath[] =
{
  "/etc/ssl/certs",               // SLES10/SLES11
  "/system/etc/security/cacerts", // Android
  "/usr/local/share/certs",       // FreeBSD
  "/etc/pki/tls/certs",           // Fedora/RHEL
  "/etc/openssl/certs",           // NetBSD
  "/var/ssl/certs",               // AIX
  NULL
};

const char *search_ssl_cacert_default_path()
{
  int i = 0;
  for (;;)
  {
    struct stat statbuf;
    const char *fdir = list_candidate_capath[i++];
    if (!fdir)
      break;
    if (stat(fdir, &statbuf) != -1 && S_ISDIR(statbuf.st_mode))
      return fdir;
  }
  return NULL;
}

#else

const char *search_ssl_cacert_default_file()
{
  return NULL;
}

const char *search_ssl_cacert_default_path()
{
  return NULL;
}

#endif

#if (defined(WITH_OPENSSL) || defined(WITH_GNUTLS)) && defined(SOAP_CLIENT_OS_WINDOWS) && !defined(WITH_WININET) && !defined(GSOAP_WIN_WININET) && !defined(WITH_CURL)

#include <windows.h>
#include <Wincrypt.h>
#pragma comment(lib, "crypt32.lib")

/* from https://stackoverflow.com/questions/9507184/can-openssl-on-windows-use-the-system-certificate-store/15451831 */
/* from https://github.com/d3x0r/SACK/blob/master/src/netlib/ssl_layer.c#L1037 */
static bool Add_InStore_from_Windows_Store(X509_STORE *store, LPCWSTR szSubsystemProtocol)
{
  HCERTSTORE hStore;
  PCCERT_CONTEXT pContext = NULL;
  X509 *x509;

  hStore = CertOpenSystemStoreW((HCRYPTPROV_LEGACY)NULL, szSubsystemProtocol);

  if (!hStore)
  {
    /* lprintf("FATAL, CANNOT OPEN ROOT STORE"); */
    return false;
  }

  while ((pContext = CertEnumCertificatesInStore(hStore, pContext)))
  {
    /* uncomment this block if you want to see the certificates as pop ups:
       #include "cryptuiapi.h"
       #pragma comment (lib, "cryptui.lib")
       CryptUIDlgViewContext(CERT_STORE_CERTIFICATE_CONTEXT, pContext, NULL, NULL, 0, NULL); 
    */

    const unsigned char *encoded_cert = (const unsigned char*)pContext->pbCertEncoded;
    x509 = d2i_X509(NULL, &encoded_cert, pContext->cbCertEncoded);

    if (x509)
    {
      X509_STORE_add_cert(store, x509);
      X509_free(x509);
    }
  }

  CertFreeCertificateContext(pContext);
  CertCloseStore(hStore, 0);

  return true;
}
 
#endif

#if (defined(WITH_OPENSSL) || defined(WITH_GNUTLS)) && !defined(WITH_WININET) && !defined(GSOAP_WIN_WININET) && !defined(WITH_CURL)
int soap_ssl_client_setup(
    struct soap *soap,    /* the context */
    unsigned short flags, /* SOAP_SSL_DEFAULT, SOAP_SSL_NO_AUTHENTICATION etc */
    const char *keyfile,  /* required only when client must authenticate to server, NULL otherwise */
    const char *password, /* password to read the key file (not used with GNUTLS), NULL otherwise */
    const char *cacert,   /* optionally assign file name of certificates PEM file, NULL to search certificate stores */
    const char *capath)   /* optionally assign path to certificates PEM files, NULL to search certificate stores */
{

  if ((flags & SOAP_SSL_REQUIRE_SERVER_AUTHENTICATION) && !cacert && !capath)
  {
    cacert = search_ssl_cacert_default_file();
    if (!cacert)
      capath = search_ssl_cacert_default_path();
  }

  if (soap_ssl_client_context(soap,
    flags,
    keyfile,  /* required only when client must authenticate to server */
    password, /* password to read the key file (not used with GNUTLS) */
    cacert,   /* cacert file to store trusted certificates */
    capath,   /* capath to directory with trusted certificates */
    NULL      /* if randfile!=NULL: use a file with random data to seed randomness */
  ))
  {
    return soap->error;
  }

#if defined(SOAP_CLIENT_OS_WINDOWS)
  if ((flags & SOAP_SSL_REQUIRE_SERVER_AUTHENTICATION) && !cacert && !capath)
  {
    X509_STORE *store = SSL_CTX_get_cert_store(soap->ctx);

    /* only ROOT appears to be useful */
    Add_InStore_from_Windows_Store(store, L"ROOT");
    /* Add_InStore_from_Windows_Store(store, L"CA"); */
    /* Add_InStore_from_Windows_Store(store, L"MY"); */
  }
#endif

  return SOAP_OK;
}
#else
int soap_ssl_client_setup(
    struct soap *,
    unsigned short,
    const char *,
    const char *,
    const char *,
    const char *)
{
  return SOAP_OK;
}
#endif
