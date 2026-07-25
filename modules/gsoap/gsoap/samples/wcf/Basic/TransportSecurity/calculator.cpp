/*
	calculator.cpp

	WCF BasicTransportSecurity Binding demo

	See the README.txt

gSOAP XML Web services tools
Copyright (C) 2000-2012, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

#include "threads.h"		/* gsoap/plugin/threads.h */

int CRYPTO_thread_setup();
void CRYPTO_thread_cleanup();

const char *URI = "https://10.0.1.5:8000/ServiceModelSamples/service"; // the service URI

int main(int argc, char **argv)
{
  if (CRYPTO_thread_setup())
  {
    std::cerr << "Cannot setup thread mutex for OpenSSL" << std::endl;
    exit(1);
  }

  if (argc >= 2)
  {
    // Service sample application
    int port = atoi(argv[1]);

    BasicHttpBinding_USCOREICalculatorService service(SOAP_XML_INDENT);

    service.soap->send_timeout = service.soap->recv_timeout = 10; // 10 sec

    /*
    The supplied server certificate "server.pem" assumes that the server is
    running on 'localhost', so clients can only connect from the same host when
    verifying the server's certificate.
    To verify the certificates of third-party services, they must provide a
    certificate issued by Verisign or another trusted CA. At the client-side,
    the capath parameter should point to a directory that contains these
    trusted (root) certificates or the cafile parameter should refer to one
    file will all certificates. To help you out, the supplied "cacerts.pem"
    file contains the certificates issued by various CAs. You should use this
    file for the cafile parameter instead of "cacert.pem" to connect to trusted
    servers. Note that the client may fail to connect if the server's
    credentials have problems (e.g. expired).
    Note 1: the password and capath are not used with GNUTLS
    Note 2: setting capath may not work on Windows.
    */
    if (soap_ssl_server_context(service.soap,
      SOAP_SSL_DEFAULT,	/* use SOAP_SSL_REQUIRE_CLIENT_AUTHENTICATION to verify clients: client must provide a key file e.g. "client.pem" and "password" */
      "server.pem",	/* keyfile (cert+key): see SSL docs to create this file */
      "password",	/* password to read the private key in the key file */
      "cacert.pem",	/* optional cacert file to store trusted certificates (to authenticate clients) */
      NULL,		/* optional capath */
      "dh512.pem",	/* DH file name or DH param key len bits (e.g. "1024"), if NULL use RSA 2048 bits (SOAP_SSL_RSA_BITS) */
      NULL,		/* if randfile!=NULL: use a file with random data to seed randomness */ 
      NULL		/* optional server identification for SSL session cache (unique server name, e.g. use argv[0]) */
    ))
    {
      service.soap_stream_fault(std::cerr);
      exit(1);
    }

    std::cerr << "Server running" << std::endl;

    for (;;)
    {
      service.ssl_run(port);
      service.soap_stream_fault(std::cerr);
    }
  }
  else
  {
    // Client sample application
    BasicHttpBinding_USCOREICalculatorProxy proxy(URI, SOAP_XML_INDENT);

    proxy.soap->send_timeout = proxy.soap->recv_timeout = 10; // 10 sec

    if (soap_ssl_client_context(proxy.soap,
      SOAP_SSL_DEFAULT | SOAP_SSL_SKIP_HOST_CHECK,
      NULL,               /* keyfile (cert+key): required only when client must authenticate to server (see SSL docs to create this file) */
      NULL,               /* password to read the keyfile */
      "cacert.pem",       /* optional cacert file to store trusted certificates, use cacerts.pem for all public certificates issued by common CAs */
      NULL,               /* optional capath to directory with trusted certificates */
      NULL                /* optional: use a file with random data to seed randomness */ 
    ))
    {
      proxy.soap_stream_fault(std::cerr);
      exit(1);
    }

    double n1 = 3.14, n2 = 1.41;

    _mssamt__Add areq;
    _mssamt__AddResponse ares;
    areq.n1 = &n1;
    areq.n2 = &n2;
    if (proxy.Add(&areq, &ares) == SOAP_OK && ares.AddResult)
      printf("Add(%g, %g) = %g\n", *areq.n1, *areq.n2, *ares.AddResult);
    else
      proxy.soap_stream_fault(std::cerr);
    proxy.destroy();

    _mssamt__Subtract sreq;
    _mssamt__SubtractResponse sres;
    sreq.n1 = &n1;
    sreq.n2 = &n2;
    if (proxy.Subtract(&sreq, &sres) == SOAP_OK && sres.SubtractResult)
      printf("Subtract(%g, %g) = %g\n", *sreq.n1, *sreq.n2, *sres.SubtractResult);
    else
      proxy.soap_stream_fault(std::cerr);
    proxy.destroy();

    _mssamt__Multiply mreq;
    _mssamt__MultiplyResponse mres;
    mreq.n1 = &n1;
    mreq.n2 = &n2;
    if (proxy.Multiply(&mreq, &mres) == SOAP_OK && mres.MultiplyResult)
      printf("Multiply(%g, %g) = %g\n", *mreq.n1, *mreq.n2, *mres.MultiplyResult);
    else
      proxy.soap_stream_fault(std::cerr);
    proxy.destroy();

    _mssamt__Divide dreq;
    _mssamt__DivideResponse dres;
    dreq.n1 = &n1;
    dreq.n2 = &n2;
    if (proxy.Divide(&dreq, &dres) == SOAP_OK && dres.DivideResult)
      printf("Divide(%g, %g) = %g\n", *dreq.n1, *dreq.n2, *dres.DivideResult);
    else
      proxy.soap_stream_fault(std::cerr);
    proxy.destroy();
  }

  CRYPTO_thread_cleanup();

  return 0;
}

/******************************************************************************\
 *
 *	Service operations
 *
\******************************************************************************/

int BasicHttpBinding_USCOREICalculatorService::Add(_mssamt__Add *Add, _mssamt__AddResponse *AddResponse)
{
  double *res = (double*)soap_malloc(soap, sizeof(double));

  if (Add && Add->n1 && Add->n2)
    *res = *Add->n1 + *Add->n2;
  else
    return soap_sender_fault(soap, "Invalid data", NULL);
  AddResponse->AddResult = res;

  return SOAP_OK;
}

int BasicHttpBinding_USCOREICalculatorService::Subtract(_mssamt__Subtract *Subtract, _mssamt__SubtractResponse *SubtractResponse)
{
  double *res = (double*)soap_malloc(soap, sizeof(double));

  if (Subtract && Subtract->n1 && Subtract->n2)
    *res = *Subtract->n1 - *Subtract->n2;
  else
    return soap_sender_fault(soap, "Invalid data", NULL);
  SubtractResponse->SubtractResult = res;

  return SOAP_OK;
}

int BasicHttpBinding_USCOREICalculatorService::Multiply(_mssamt__Multiply *Multiply, _mssamt__MultiplyResponse *MultiplyResponse)
{
  double *res = (double*)soap_malloc(soap, sizeof(double));

  if (Multiply && Multiply->n1 && Multiply->n2)
    *res = *Multiply->n1 * *Multiply->n2;
  else
    return soap_sender_fault(soap, "Invalid data", NULL);
  MultiplyResponse->MultiplyResult = res;

  return SOAP_OK;
}

int BasicHttpBinding_USCOREICalculatorService::Divide(_mssamt__Divide *Divide, _mssamt__DivideResponse *DivideResponse)
{
  double *res = (double*)soap_malloc(soap, sizeof(double));

  if (Divide && Divide->n1 && Divide->n2 && *Divide->n2 != 0.0)
    *res = *Divide->n1 / *Divide->n2;
  else
    return soap_sender_fault(soap, "Invalid data", NULL);
  DivideResponse->DivideResult = res;

  return SOAP_OK;
}

/******************************************************************************\
 *
 *	OpenSSL
 *
\******************************************************************************/

#ifdef WITH_OPENSSL

struct CRYPTO_dynlock_value
{ MUTEX_TYPE mutex;
};

static MUTEX_TYPE *mutex_buf;

static struct CRYPTO_dynlock_value *dyn_create_function(const char *file, int line)
{ struct CRYPTO_dynlock_value *value;
  value = (struct CRYPTO_dynlock_value*)malloc(sizeof(struct CRYPTO_dynlock_value));
  if (value)
    MUTEX_SETUP(value->mutex);
  return value;
}

static void dyn_lock_function(int mode, struct CRYPTO_dynlock_value *l, const char *file, int line)
{ if (mode & CRYPTO_LOCK)
    MUTEX_LOCK(l->mutex);
  else
    MUTEX_UNLOCK(l->mutex);
}

static void dyn_destroy_function(struct CRYPTO_dynlock_value *l, const char *file, int line)
{ MUTEX_CLEANUP(l->mutex);
  free(l);
}

void locking_function(int mode, int n, const char *file, int line)
{ if (mode & CRYPTO_LOCK)
    MUTEX_LOCK(mutex_buf[n]);
  else
    MUTEX_UNLOCK(mutex_buf[n]);
}

unsigned long id_function()
{ return (unsigned long)THREAD_ID;
}

int CRYPTO_thread_setup()
{ int i;
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
{ int i;
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

#else

/* OpenSSL not used, e.g. GNUTLS is used */

int CRYPTO_thread_setup()
{ return SOAP_OK;
}

void CRYPTO_thread_cleanup()
{ }

#endif

