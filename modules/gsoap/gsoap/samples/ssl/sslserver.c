/*
	sslserver.c

	Example stand-alone SSL-secure gSOAP Web service.

        Build steps:

        soapcpp2 -c ssl.h
        cc -o sslserver sslserver.c soapC.c soapServer.c stdsoap2.c thread_setup.c

	SSL-enabled services use the gSOAP SSL interface. See sslclient.c and
	sslserver.c for example code with instructions and the gSOAP
	documentation more details.

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2008, Robert van Engelen, Genivia, Inc. All Rights Reserved.
This software is released under one of the following two licenses:
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
*/

#include "soapH.h"
#include "ssl.nsmap"
#include "threads.h"		/* gsoap/plugin/threads.h */
#include <signal.h>		/* defines SIGPIPE */

void sigpipe_handle(int x) { }

void *process_request(struct soap*);

int CRYPTO_thread_setup();
void CRYPTO_thread_cleanup();

/******************************************************************************\
 *
 *	Main
 *
\******************************************************************************/

int main()
{
  SOAP_SOCKET m;
  THREAD_TYPE tid;
  struct soap soap, *tsoap;
  /* init gsoap context */
  soap_init(&soap);
#if defined(WITH_OPENSSL)
  /* Uncomment to call this first before all else if SSL is initialized elsewhere, e.g. in application code */
  /* soap_ssl_noinit(); */
  /* Init SSL before any threads are started (do this just once) */
  soap_ssl_init();
  /* Need SIGPIPE handler on Unix/Linux systems to catch broken pipes: */
  signal(SIGPIPE, sigpipe_handle);
  /* set up SSL locks (not needed for OpenSSL 1.1.0 and greater) */
  if (CRYPTO_thread_setup())
  {
    fprintf(stderr, "Cannot setup thread mutex for OpenSSL\n");
    exit(EXIT_FAILURE);
  }
  /* The supplied server certificate "server.pem" assumes that the server is
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
  if (soap_ssl_server_context(&soap,
    SOAP_SSL_DEFAULT,	/* use SOAP_SSL_REQUIRE_CLIENT_AUTHENTICATION to verify clients: client must provide a key file e.g. "client.pem" and "password" */
    "server.pem",	/* keyfile (cert+key): see README.txt to create this file */
    "password",		/* password to read the private key in the key file */
    "cacert.pem",	/* cacert file to store trusted certificates (to authenticate clients), see README.txt */
    NULL,		/* capath */
    NULL,       	/* DH file name (e.g. "dh2048.pem") or DH param key len bits in string (e.g. "2048"), if NULL then RSA with 2048 bits is used instead (bits defined by SOAP_SSL_RSA_BITS) */
    NULL,		/* if randfile!=NULL: use a file with random data to seed randomness */ 
    "sslserver"		/* server identification for SSL session cache (unique server name, e.g. use argv[0]) */
  )
  )
  {
    soap_print_fault(&soap, stderr);
    exit(EXIT_FAILURE);
  }
  /* enable CRL, may need SOAP_SSL_ALLOW_EXPIRED_CERTIFICATE when certs have no CRL
  if (soap_ssl_crl(&soap, ""))
  {
    soap_print_fault(&soap, stderr);
    exit(EXIT_FAILURE);
  }
  */
#endif
  soap.accept_timeout = 60;	/* server times out after 1 minute inactivity */
  soap.send_timeout = soap.recv_timeout = 5;	/* max I/O idle time is 5 seconds */
  m = soap_bind(&soap, NULL, 18081, 100);
  if (!soap_valid_socket(m))
  {
    soap_print_fault(&soap, stderr);
    exit(EXIT_FAILURE);
  }
  fprintf(stderr, "Bind successful: socket = %d\n", m);
  for (;;)
  {
    SOAP_SOCKET s = soap_accept(&soap);
    if (!soap_valid_socket(s))
    {
      if (soap.errnum)
        soap_print_fault(&soap, stderr);
      else
        fprintf(stderr, "Server timed out (timeout set to %d seconds)\n", soap.accept_timeout);
      break;
    }
    fprintf(stderr, "Socket %d connection from IP %d.%d.%d.%d\n", s, (int)(soap.ip>>24)&0xFF, (int)(soap.ip>>16)&0xFF, (int)(soap.ip>>8)&0xFF, (int)soap.ip&0xFF);
    tsoap = soap_copy(&soap);
    if (!tsoap)
    {
      soap_closesock(&soap);
      continue;
    }
    while (THREAD_CREATE(&tid, (void*(*)(void*))&process_request, tsoap))
      sleep(1);
  }
  soap_destroy(&soap);
  soap_end(&soap);
  soap_done(&soap); /* MUST call before CRYPTO_thread_cleanup */
  CRYPTO_thread_cleanup();
  return 0;
} 

void *process_request(struct soap *soap)
{
  THREAD_DETACH(THREAD_ID);
#if defined(WITH_OPENSSL)
  if (soap_ssl_accept(soap) != SOAP_OK)
  {
    /* when soap_ssl_accept() fails, socket is closed and SSL data reset */
    soap_print_fault(soap, stderr);
    fprintf(stderr, "SSL request failed, continue with next...\n");
  }
  else
#endif
  {
    soap_serve(soap);
    fprintf(stderr, "SSL request served, continue with next...\n");
  }
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  return NULL;
}

/******************************************************************************\
 *
 *	Service methods
 *
\******************************************************************************/

int ns__add(struct soap *soap, double a, double b, double *result)
{
  (void)soap;
  *result = a + b;
  return SOAP_OK;
} 

/******************************************************************************\
 *
 *	OpenSSL
 *
\******************************************************************************/

#if defined(WITH_OPENSSL) && OPENSSL_VERSION_NUMBER < 0x10100000L

struct CRYPTO_dynlock_value
{
  MUTEX_TYPE mutex;
};

static MUTEX_TYPE *mutex_buf = NULL;

static struct CRYPTO_dynlock_value *dyn_create_function(const char *file, int line)
{
  struct CRYPTO_dynlock_value *value;
  (void)file; (void)line;
  value = (struct CRYPTO_dynlock_value*)OPENSSL_malloc(sizeof(struct CRYPTO_dynlock_value));
  if (value)
    MUTEX_SETUP(value->mutex);
  return value;
}

static void dyn_lock_function(int mode, struct CRYPTO_dynlock_value *l, const char *file, int line)
{
  (void)file; (void)line;
  if (mode & CRYPTO_LOCK)
    MUTEX_LOCK(l->mutex);
  else
    MUTEX_UNLOCK(l->mutex);
}

static void dyn_destroy_function(struct CRYPTO_dynlock_value *l, const char *file, int line)
{
  (void)file; (void)line;
  MUTEX_CLEANUP(l->mutex);
  OPENSSL_free(l);
}

static void locking_function(int mode, int n, const char *file, int line)
{
  (void)file; (void)line;
  if (mode & CRYPTO_LOCK)
    MUTEX_LOCK(mutex_buf[n]);
  else
    MUTEX_UNLOCK(mutex_buf[n]);
}

static unsigned long id_function()
{
  return (unsigned long)THREAD_ID;
}

int CRYPTO_thread_setup()
{
  int i;
  mutex_buf = (MUTEX_TYPE*)OPENSSL_malloc(CRYPTO_num_locks() * sizeof(MUTEX_TYPE));
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
  OPENSSL_free(mutex_buf);
  mutex_buf = NULL;
}

#else

/* OpenSSL not used or OpenSSL prior to 1.1.0 */

int CRYPTO_thread_setup()
{
  return SOAP_OK;
}

void CRYPTO_thread_cleanup()
{ }

#endif
