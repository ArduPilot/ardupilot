/*
	wsrmdemo.c

	WS-ReliableMessaging 1.1 and WS-Addressing demo service and client.

	See the wsrmdemo.h header file for build and usage details.
	See gsoap/doc/wsrm for the WS-RM plugin user guide.

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

/*
	This code auto-configures depending on the folling flags:

	-D WITH_UDP:     enables UDP, disables multithreaded servicing
	-D WITH_OPENSSL: enables SSL transfers (transfer security)

	To use WS-ReliableMessaging 1.0, change wsrmdemo.h to import the 2005
	protocol, then rerun soapcpp2 and compile:

	#import "wsrm5.h"
*/


#include "soapH.h"
#include "wsrmdemo.nsmap"

#include <unistd.h>	/* defines _POSIX_THREADS if pthreads are available */
#if defined(_POSIX_THREADS) || defined(_SC_THREADS)
#include <threads.h>	/* from plugin/threads.h */
#endif

#include "wsaapi.h"  /* from plugin/wsaapi.h */
#include "wsrmapi.h" /* from plugin/wsrmapi.h */

/*
	HTTP Basic Authentication is used by default, which is only secure over
	HTTPS. Use the HTTP Digest Authentication plugin for secure auth over
	HTTP by enabling this line:

#include "httpda.h"

	Then compile and link with plugin/httpda.c and plugin/md5evp.c, Compile
	with -DWITH_OPENSSL to use SSL.
*/

/******************************************************************************\
 *
 *	Common Constants
 *
\******************************************************************************/

#if defined(WITH_UDP)
# define HTTP_TARGET "soap.udp:" /* Test WS-ReliableMessaging with SOAP-over-UDP */
#elif defined(WITH_OPENSSL)
# define HTTP_TARGET "https:" /* Test HTTPS */
#else
# define HTTP_TARGET "http:"
#endif

#if defined(WITH_OPENSSL) || defined(HTTPDA_H)
# define HTTP_USERID "foo"
# define HTTP_PASSWD "bar"
# define HTTP_REALM  "svc"
#endif

#if defined(HTTP_USERID) && defined(HTTP_PASSWD)
const char *userid = HTTP_USERID;
const char *passwd = HTTP_PASSWD;
#else
const char *userid = NULL;
const char *passwd = NULL;
#endif

/* the matching SOAP/WSA actions, defined in wsdldemo.h service definitions */
const char *RequestAction  = "urn:wsrmdemo/wsrmdemoPort/wsrmdemo";
const char *ResponseAction = "urn:wsrmdemo/wsrmdemoPort/wsrmdemoResponse";

const char *serverURI = HTTP_TARGET"//localhost:8000";
const char *clientURI = HTTP_TARGET"//localhost:8001";
int serverPort = 8000;
int clientPort = 8001;

/******************************************************************************\
 *
 *	Main Implements a Server and Client
 *
\******************************************************************************/

int callback_poll(struct soap *soap, int timeout);
void *callback_server(void *soap);

void *process_request(void *soap);

int CRYPTO_thread_setup();
void CRYPTO_thread_cleanup();

int main(int argc, char **argv)
{
  struct soap *soap = soap_new1(SOAP_XML_INDENT | SOAP_XML_STRICT);
#ifdef HTTPDA_H
  /* Use HTTP Digest Authentication, see gsoap/doc/httpda/html/index.html */
  struct http_da_info info;
  soap_register_plugin(soap, http_da);
  http_da_save(soap, &info, HTTP_REALM, HTTP_USERID, HTTP_PASSWD);
#endif
  soap_register_plugin(soap, soap_wsa);
  soap_register_plugin(soap, soap_wsrm);
  if (argc < 2) /* no args: server */
  { soap_set_mode(soap, SOAP_IO_KEEPALIVE);
#if !defined(WITH_UDP) && defined(THREADS_H)
    THREAD_TYPE tid;
#endif
    /* TCP/IP and UDP timeouts + = seconds, - = microsecond*/
    soap->send_timeout = soap->recv_timeout = 5;
#if defined(WITH_UDP)
    /* UDP recv timeout is short to limit blocking time */
    soap->recv_timeout = -200000;
    /* set UDP server, pure XML w/o HTTP headers */
    soap_set_mode(soap, SOAP_IO_UDP);
#elif defined(WITH_OPENSSL)
    /* set up SSL locks (not needed for OpenSSL 1.1.0 and greater) */
    if (CRYPTO_thread_setup())
    { fprintf(stderr, "Cannot setup thread mutex for OpenSSL\n");
      exit(1);
    }
    /* WS-Addressing server uses client-side sends, so init SSL for client+server: */
    if (soap_ssl_server_context(soap,
      SOAP_SSL_DEFAULT,
      "server.pem",	/* keyfile (server) */
      "password",	/* password to read the key file (server) */
      "cacert.pem",	/* cacert file to store trusted certificates (client) */
      NULL,		/* optional capath */
      NULL, 		/* DH file name or DH param key len bits, NULL: RSA */
      NULL,		/* file with random data to seed randomness */ 
      NULL		/* unique server identification for SSL session cache */
    ))
    { soap_print_fault(soap, stderr);
      exit(1);
    }
#endif

    soap->bind_flags = SO_REUSEADDR;	/* allow immediate port rebinding */
    if (!soap_valid_socket(soap_bind(soap, NULL, serverPort, 100)))
    { soap_print_fault(soap, stderr);
      exit(1);
    }

    printf("**** Server Running\n");

    soap->accept_timeout = -100000; /* 100ms timeout: do not block on accept */
    for (;;)
    { /* TCP accept (for UDP simply returns current socket) */
#if !defined(WITH_UDP) && defined(THREADS_H)
      struct soap *tsoap;
#endif
      if (!soap_valid_socket(soap_accept(soap)))
      { if (soap->errnum)
          soap_print_fault(soap, stderr);
        else /* timeout, send acknowledgements to all peers */
	{ soap_wsrm_pulse(soap, -10000); /* 10 ms */
#ifdef WITH_UDP
	  sleep(1); /* optional, needed for UDP: accept() ruturns immediately */
#endif
	}
        continue;
      }

      /* do not spawn threads for UDP, since accept() is a no-op for UDP */
#if !defined(WITH_UDP) && defined(THREADS_H)
      tsoap = soap_copy(soap);
      while (THREAD_CREATE(&tid, (void*(*)(void*))process_request, (void*)tsoap))
        sleep(1);
#else
#if !defined(WITH_UDP) && defined(WITH_OPENSSL)
      /* SSL accept */
      if (soap_ssl_accept(soap))
      { soap_print_fault(soap, stderr);
        fprintf(stderr, "SSL request failed, continue with next call...\n");
        soap_destroy(soap);
        soap_end(soap);
        continue;
      }
#endif
      if (soap_serve(soap) && soap->error != SOAP_STOP && soap->error != SOAP_EOF)
        soap_print_fault(soap, stderr);
      else if (soap->error != SOAP_EOF || soap->errnum)
        printf("\n**** Request served\n");
      soap_wsrm_dump(soap, stdout);
      soap_destroy(soap);
      soap_end(soap);
#endif
    }
    soap_destroy(soap);
    soap_end(soap);
    soap_done(soap); /* MUST call before CRYPTO_thread_cleanup */
    CRYPTO_thread_cleanup();
  }
  else /* client */
  { xsd__duration expires = 30000; /* 30000 ms to expire */
    soap_wsrm_sequence_handle seq;
    struct ns__wsrmdemoResponse res;
    int retry;
    const char *replyto = NULL;
    struct soap *callback = NULL;
    int duplex = 0; /* duplex mode */
    int server = 0; /* use callback server with duplex mode */
    int alive = 0; /* keep-alive callback port */
    const char *to = NULL; /* service endpoint */
#ifdef THREADS_H
    THREAD_TYPE tid;
#endif

    /* TCP/IP and UDP timeouts + = seconds, - = microsecond*/
    soap->send_timeout = soap->recv_timeout = 5;

#if defined(WITH_UDP)
    /* UDP recv timeout is short to limit blocking time */
    soap->recv_timeout = -200000;
    /* "soap.udp:" endpoint specifies SOAP-over-UDP and should always be used for ReplyTo and FaultTo if these use UDP */
    /* When the endpoint is an IP with a UDP destination, it is important to set UDP: */
    soap_set_mode(soap, SOAP_IO_UDP);
#elif defined(WITH_OPENSSL)
    /* set up SSL locks (not needed for OpenSSL 1.1.0 and greater) */
    CRYPTO_thread_setup();
    if (soap_ssl_client_context(soap,
      SOAP_SSL_DEFAULT | SOAP_SSL_SKIP_HOST_CHECK,
      NULL, 		/* optional keyfile to authenticate to server */
      NULL, 		/* password to read the keyfile */
      "cacert.pem",	/* cacert file to store trusted certificates */
      NULL,		/* capath to directory with trusted certificates */
      NULL		/* optional file with random data to seed randomness */ 
    ))
    { soap_print_fault(soap, stderr);
      exit(1);
    }
#endif

    /* Command-line options:
       d duplex mode with dual channels
       s callback server for duplex mode (default is callback polling)
    */
    if (argc >= 3)
    { if (strchr(argv[2], 's'))
      { duplex = 1;
        server = 1;
      }
      else if (strchr(argv[2], 'd'))
      { duplex = 1;
#ifdef WITH_OPENSSL
        fprintf(stderr, "Warning: SSL requires a callback server for duplex mode, because the client waits for the HTTP Accepted response while server is sending a message to the callback: deadlock will occur\n");
#endif
      }
      if (strchr(argv[2], 'a'))
        alive = 1;
    }

    if (alive)
      soap_set_mode(soap, SOAP_IO_KEEPALIVE);

    if (duplex)
    { /* set up the callback for duplex communication */
      replyto = clientURI;
      callback = soap_new1(SOAP_XML_INDENT | SOAP_XML_STRICT);
      if (alive)
        soap_set_mode(callback, SOAP_IO_KEEPALIVE);
      soap_register_plugin(callback, soap_wsa);
      soap_register_plugin(callback, soap_wsrm);
#if defined(WITH_UDP)
      /* "soap.udp:" endpoint specifies SOAP-over-UDP and should always be used for ReplyTo and FaultTo if these use UDP */
      /* When the endpoint is an IP with a UDP destination, it is important to set UDP: */
      soap_set_mode(callback, SOAP_IO_UDP);
#elif defined(WITH_OPENSSL)
      if (soap_ssl_server_context(callback,
        SOAP_SSL_DEFAULT,
        "client.pem",	/* keyfile (client) */
        "password",	/* password to read the key file (server) */
        "cacert.pem",	/* cacert file to store trusted certificates (client) */
        NULL,		/* optional capath */
        NULL, 		/* DH file name or DH param key len bits, NULL: RSA */
        NULL,		/* file with random data to seed randomness */ 
        NULL		/* unique server identification for SSL session cache */
      ))
      { soap_print_fault(soap, stderr);
        exit(1);
      }
#endif
      callback->bind_flags = SO_REUSEADDR;	/* allow immediate port rebinding */
      if (!soap_valid_socket(soap_bind(callback, NULL, clientPort, 100)))
      { soap_print_fault(soap, stderr);
        exit(1);
      }
    }

#ifdef THREADS_H
    if (server)
      THREAD_CREATE(&tid, (void*(*)(void*))callback_server, (void*)callback);
#endif

    printf("\n**** Creating a Sequence\n");

#ifdef HTTPDA_H
    /* Digest Auth */
    http_da_restore(soap, &info);
#else
    /* Basic Auth */
    soap->userid = userid; soap->passwd = passwd;
#endif

    if (soap_wsrm_create_offer(soap, serverURI, replyto, NULL, expires, DiscardEntireSequence, NULL, &seq))
    { if (soap->error == 202)
        printf("\n**** Create request was accepted\n");
      else if (!duplex) /* if not duplex, error is fatal */
      { soap_print_fault(soap, stderr);
        soap_wsrm_seq_free(soap, seq);
        return soap->error;
      }
    }

    if (duplex)
    { for (retry = 10; retry && !soap_wsrm_seq_created(soap, seq); retry--)
      { if (server)
          sleep(1); /* wait for callback server to receive response */
	else if (callback_poll(callback, 1)) /* poll for 1 sec */
          return callback->error;
      }
      if (!retry)
      { fprintf(stderr, "CANNOT CREATE SEQUENCE - SERVER NOT RESPONDING\n");
        exit(1);
      }
    }

    soap_wsrm_dump(soap, stdout);

    printf("\n**** Sending first message\n");

#ifdef HTTPDA_H
    /* Digest Auth */
    http_da_restore(soap, &info);
#else
    /* Basic Auth */
    soap->userid = userid; soap->passwd = passwd;
#endif
    /* this shows how to use a retry loop to improve message delivery */
    /* UDP may timeout when no UDP response message is sent by the server */
    if (soap_wsrm_request(soap, seq, soap_wsa_rand_uuid(soap), RequestAction))
    { soap_print_fault(soap, stderr);
      return soap->error;
    }
    while ((to = soap_wsrm_to(seq)) != NULL && soap_call_ns__wsrmdemo(soap, to, RequestAction, "First Message", &res))
    { if (soap->error == 202)
      { printf("\n**** Request was accepted\n");
        break;
      }
      else if (soap->error == SOAP_NO_TAG) /* empty <Body> */
      { printf("\n**** Request was accepted, acks received\n");
        break;
      }
      else if (duplex) /* duplex: we opt not to retry (could poll first) */
        break;
      soap_print_fault(soap, stderr);
      /* only continue if retry is recommended */
      if (soap_wsrm_check_retry(soap, seq))
        break;
      /* wait a second to give network a chance to recover */
      printf("\n**** Transmission failed: retrying after 1 second...\n");
      sleep(1);
    }

    if (soap->error == SOAP_OK || soap->error == 202)
      printf("\n**** Response OK\n");

    soap_wsrm_dump(soap, stdout);

    if (duplex)
    { if (server)
        sleep(1);
      else if (callback_poll(callback, -200000)) /* poll for 200 ms */
        return callback->error;
    }

    printf("\n**** Sending second message, requesting acks\n");

#ifdef HTTPDA_H
    /* Digest Auth */
    http_da_restore(soap, &info);
#else
    /* Basic Auth */
    soap->userid = userid; soap->passwd = passwd;
#endif
    /* here we just send the message without retry loop */
    /* UDP may timeout when no UDP response message is sent by the server */
    to = soap_wsrm_to(seq);
    if (to)
    { if (soap_wsrm_request_acks(soap, seq, soap_wsa_rand_uuid(soap), RequestAction))
      { soap_print_fault(soap, stderr);
        return soap->error;
      }
      if (soap_call_ns__wsrmdemo(soap, to, RequestAction, (char*)"Second Message", &res))
      { if (soap->error == 202)
          printf("\n**** Request was accepted\n");
        else if (soap->error == SOAP_NO_TAG) /* empty <Body> */
          printf("\n**** Request was accepted, acks received\n");
        else if (!duplex)
          soap_print_fault(soap, stderr);
      }
      else
        printf("\n**** Response OK\n");

      if (duplex)
      { if (server)
          sleep(1);
        else if (callback_poll(callback, -200000)) /* poll for 200 ms */
          return callback->error;
      }
    }
    /* check for any non-acked messages, and resend these */
    if (soap_wsrm_nack(seq))
    { printf("\n**** Resending "SOAP_ULONG_FORMAT" Non-Acked Messages\n", soap_wsrm_nack(seq));
      soap_wsrm_resend(soap, seq, 0, 0); /* 0 0 means full range of msg nums of non-acked message to resend */
      if (duplex)
      { if (server)
          sleep(1);
        else if (callback_poll(callback, 1)) /* poll for 1 sec */
          return callback->error;
      }
    }

    soap_wsrm_dump(soap, stdout);

    printf("\n**** Sending third message\n");

#ifdef HTTPDA_H
    /* Digest Auth */
    http_da_restore(soap, &info);
#else
    /* Basic Auth */
    soap->userid = userid; soap->passwd = passwd;
#endif
    /* here we show how to just send the message without retry loop */
    /* UDP may timeout when no UDP response message is sent by the server */
    to = soap_wsrm_to(seq);
    if (to)
    { if (soap_wsrm_request(soap, seq, soap_wsa_rand_uuid(soap), RequestAction))
      { soap_print_fault(soap, stderr);
        return soap->error;
      }
      if (soap_call_ns__wsrmdemo(soap, to, RequestAction, argv[1], &res))
      { if (soap->error == 202)
          printf("\n**** Request was accepted\n");
        else if (soap->error == SOAP_NO_TAG) /* empty <Body> */
          printf("\n**** Request was accepted, acks received\n");
        else if (!duplex)
          soap_print_fault(soap, stderr);
      }
      else
        printf("\n**** Response OK\n");

      if (duplex)
      { if (server)
          sleep(1); /* allows to get messages before closing */
        else if (callback_poll(callback, -200000)) /* poll for 200 ms */
          return callback->error;
      }

      soap_wsrm_dump(soap, stdout);
    }

    printf("\n**** Closing the Sequence\n");

#ifdef HTTPDA_H
    /* Digest Auth */
    http_da_restore(soap, &info);
#else
    /* Basic Auth */
    soap->userid = userid; soap->passwd = passwd;
#endif
    /* close the sequence */
    if (soap_wsrm_close(soap, seq, soap_wsa_rand_uuid(soap)))
    { if (soap->error == 202)
        printf("\n**** Close request was accepted\n");
      else if (!duplex)
      { soap_print_fault(soap, stderr);
        soap_wsrm_seq_free(soap, seq);
        return soap->error;
      }
    }

    if (duplex)
    { if (server)
        sleep(1); /* still accept messages after close */
      else if (callback_poll(callback, 1)) /* poll for 1 sec */
        return callback->error;
    }

    soap_wsrm_dump(soap, stdout);

    /* Resend messages marked as non-acked (as an option) */
    for (retry = 2; retry && soap_wsrm_nack(seq); retry--)
    {
      printf("\n**** Resending "SOAP_ULONG_FORMAT" Non-Acked Messages\n", soap_wsrm_nack(seq));
      soap_wsrm_resend(soap, seq, 0, 0); /* 0 0 means full range of msg nums */

      if (duplex)
      { if (server)
          sleep(1);
	else if (callback_poll(callback, 1)) /* poll for 1 sec */
          return callback->error;
      }
    }

    printf("\n**** Terminating the Sequence\n");

#ifdef HTTPDA_H
    /* Digest Auth */
    http_da_restore(soap, &info);
#else
    /* Basic Auth */
    soap->userid = userid; soap->passwd = passwd;
#endif
    /* termination fails if the server did not get all messages */
    if (soap_wsrm_terminate(soap, seq, soap_wsa_rand_uuid(soap)))
    { if (soap->error == 202)
        printf("\n**** Terminate request was accepted\n");
      else if (!duplex || soap->error != SOAP_EOF)
        soap_print_fault(soap, stderr);
    }

    if (duplex)
    { if (server)
        sleep(1);
      else if (callback_poll(callback, 1)) /* poll for 1 sec */
        return callback->error;
    }

    soap_wsrm_dump(soap, stdout);

#ifdef THREADS_H
    if (server)
      THREAD_JOIN(tid);
#endif

    /* delete the sequence */
    soap_wsrm_seq_free(soap, seq);

    /* delete the callback */
    if (callback)
    { soap_destroy(callback);
      soap_end(callback);
      soap_free(callback);
    }

    /* cleanup deserialized data, allowed at any time in/after sequence */
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap); /* MUST call before CRYPTO_thread_cleanup */
    CRYPTO_thread_cleanup();
  }
  return 0;
}

/******************************************************************************\
 *
 *	Threaded Server
 *
\******************************************************************************/

#ifdef THREADS_H
void *process_request(void *ctx)
{
  struct soap *soap = (struct soap*)ctx;

  pthread_detach(pthread_self());
#if !defined(WITH_UDP) && defined(WITH_OPENSSL)
  /* SSL accept */
  if (soap_ssl_accept(soap))
  { soap_print_fault(soap, stderr);
    fprintf(stderr, "SSL request failed\n");
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
    return NULL;
  }
#endif
  if (soap_serve(soap) && soap->error != SOAP_STOP && soap->error != SOAP_EOF)
    soap_print_fault(soap, stderr);
  else if (soap->error == SOAP_EOF && soap->errnum == 0)
    printf("\n**** Thread timed out\n");
  else
  { printf("\n**** Request served by thread\n");
    soap_wsrm_dump(soap, stdout);
  }
  soap_destroy(soap);
  soap_end(soap);
  soap_free(soap);
  return NULL;
}
#endif

/******************************************************************************\
 *
 *	Optional Client-Side Threaded Callback Server
 *
\******************************************************************************/

#ifdef THREADS_H
void *callback_server(void *ctx)
{
  struct soap *soap = (struct soap*)ctx;

  soap->accept_timeout = 10; /* server quits after 10 seconds of inactivity */
  soap->recv_timeout = 10;
  soap->send_timeout = 1; /* 1 sec I/O timeout */

  printf("\n**** Callback Server Running\n");

  while (soap_valid_socket(soap_accept(soap)))
  {
#if !defined(WITH_UDP) && defined(WITH_OPENSSL)
    if (soap_ssl_accept(soap))
    { soap_print_fault(soap, stderr);
      fprintf(stderr, "SSL request failed, continue with next call...\n");
      continue;
    }
#endif
    if (soap_serve(soap) && soap->error != SOAP_STOP && soap->error != SOAP_EOF)
      soap_print_fault(soap, stderr);
    else if (soap->error == SOAP_EOF && !soap->errnum)
      continue; /* messaging IO timed out */
    else
      soap_wsrm_dump(soap, stdout);
  }

  printf("\n**** Callback Server Terminated\n");

  return NULL;
}
#endif

/******************************************************************************\
 *
 *	Client-Side Callback Polling
 *
\******************************************************************************/

int callback_poll(struct soap *soap, int timeout)
{
  int poll = 5; /* 5 poll cycles max, ensures we make progress */

  soap->accept_timeout = timeout;
  soap->recv_timeout = timeout;
  soap->send_timeout = 1; /* 1 sec I/O timeout */

  printf("\n**** Callback Polling\n");

  while (poll-- && soap_valid_socket(soap_accept(soap)))
  {
#if !defined(WITH_UDP) && defined(WITH_OPENSSL)
    if (soap_ssl_accept(soap))
    { soap_print_fault(soap, stderr);
      fprintf(stderr, "SSL request failed, continue with next call...\n");
      continue;
    }
#endif
    if (soap_serve(soap) && soap->error != SOAP_STOP)
    { if (soap->error == SOAP_EOF) /* timed out */
        return SOAP_OK;
      soap_print_fault(soap, stderr);
      return soap->error;
    }
    soap_wsrm_dump(soap, stdout);
  }

  return SOAP_OK;
}

/******************************************************************************\
 *
 *	Service Operations
 *
\******************************************************************************/

int ns__wsrmdemo(struct soap *soap, char *in, struct ns__wsrmdemoResponse *result)
{
  /* check Basic/Digest Auth, when enabled */
  if (userid && passwd)
  { if (!soap->userid
     || !soap->passwd
     || strcmp(userid, soap->userid)
     || strcmp(passwd, soap->passwd))
    {
#ifdef HTTPDA_H
      soap->authrealm = HTTP_REALM;
#endif
      return 401; /* HTTP Unauthorized */
    }
  }

  /* check for WS-RM/WSA and set WS-RM/WSA return headers */
  if (soap_wsrm_check(soap))
    return soap->error;

  printf("\n**** Received Request \"%s\"\n", in?in:"(null)");

  /* simulate a fatal server error, which is possibly relayed */
  /* for fatal errors that terminate the sequence, we must call soap_wsrm_sender_fault() before soap_wsrm_check() */
  if (in && !strcmp(in, "error"))
  { /* this is fatal, we terminate the sequence */
    int err;
    soap_wsrm_sequence_handle seq = soap_wsrm_seq(soap);
    err = soap_wsrm_error(soap, seq, wsrm__SequenceTerminated);
    soap_wsrm_seq_release(soap, seq);
    printf("\n**** Simulating Server Operation Fatal Error\n");
    return err;
  }

  /* simulate a non-fatal user-defined error, which is can be relayed */
  if (in && !strcmp(in, "fault"))
  { printf("\n**** Simulating Server Operation Fault\n");
    return soap_wsrm_sender_fault(soap, "The demo service wsrmdemo() operation returned a fault", NULL);
  }

  result->out = in;

#if 0
  /* to just respond without requesting acks for response messages: */
  return soap_wsrm_reply(soap, soap_wsa_rand_uuid(soap), ResponseAction);
#else
  /* to request acks for response messages (only when WS-RM "create offer" was made): */
  return soap_wsrm_reply_request_acks(soap, soap_wsa_rand_uuid(soap), ResponseAction);
#endif
}

/******************************************************************************\
 *
 *	Client-Side Callback Operation to Receive Service Responses
 *
\******************************************************************************/

int ns__wsrmdemoResponse(struct soap *soap, char *out)
{
  /* check Basic Auth, when enabled */
  if (userid && passwd)
  { if (!soap->userid
     || !soap->passwd
     || strcmp(userid, soap->userid)
     || strcmp(passwd, soap->passwd))
      return soap_send_empty_response(soap, 401); /* HTTP Unauthorized */
  }

  /* check for WS-RM/WSA and set WS-RM/WSA return headers */
  if (soap_wsrm_check(soap))
    return soap->error;

  printf("\n**** Received Response \"%s\"\n", out?out:"(null)");

  return soap_send_empty_response(soap, 202); /* HTTP 202 Accepted */
}

/******************************************************************************\
 *
 *	Relayed SOAP-ENV:Fault Handler for FaultTo Server
 *
\******************************************************************************/

int SOAP_ENV__Fault(struct soap *soap, 
        _QName			 faultcode,		// SOAP 1.1
        char			*faultstring,		// SOAP 1.1
        char			*faultactor,		// SOAP 1.1
        struct SOAP_ENV__Detail	*detail,		// SOAP 1.1
        struct SOAP_ENV__Code	*Code,			// SOAP 1.2
        struct SOAP_ENV__Reason	*Reason,		// SOAP 1.2
        char			*Node,			// SOAP 1.2
        char			*Role,			// SOAP 1.2
        struct SOAP_ENV__Detail	*Detail			// SOAP 1.2
)
{ 
  soap_send_empty_response(soap, 202); /* HTTP 202 Accepted */

  /* populate the fault struct from the operation arguments to print it */
  soap_fault(soap);
  /* SOAP 1.1 */
  soap->fault->faultcode = faultcode;
  soap->fault->faultstring = faultstring;
  soap->fault->faultactor = faultactor;
  soap->fault->detail = detail;
  /* SOAP 1.2 */
  soap->fault->SOAP_ENV__Code = Code;
  soap->fault->SOAP_ENV__Reason = Reason;
  soap->fault->SOAP_ENV__Node = Node;
  soap->fault->SOAP_ENV__Role = Role;
  soap->fault->SOAP_ENV__Detail = Detail;

  /* set error and display */
  soap->error = SOAP_FAULT;
  printf("\n**** Received Fault:\n");
  soap_print_fault(soap, stdout);

  if (!detail)
    detail = Detail;
  if (detail && detail->__type == SOAP_TYPE__wsrm__Identifier)
  {
    /* the sequence id is in the Fault Detail __type and fault members */
    char *id = (char*)detail->fault;

    /* we opt to treat all faults fatal, so let's terminate the sequence */
    soap_wsrm_sequence_handle seq = soap_wsrm_seq_lookup_id(soap, id);
    if (seq)
    {
      soap_wsrm_error(soap, seq, wsrm__SequenceTerminated);
      soap_wsrm_seq_release(soap, seq);
      return soap->error;
    }
  }

  return SOAP_OK;
}

/******************************************************************************\
 *
 *	OpenSSL
 *
\******************************************************************************/

#if defined(WITH_OPENSSL) && OPENSSL_VERSION_NUMBER < 0x10100000L

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

/* OpenSSL not used or OpenSSL prior to 1.1.0 */

int CRYPTO_thread_setup()
{ return SOAP_OK;
}

void CRYPTO_thread_cleanup()
{ }

#endif

