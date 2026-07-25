/*
   All OpenSSL versions prior to 1.1.0 require mutex locks to be explicitly set
   up to work safely with multi-threaded applications

   At the start of your program call:

     soap_ssl_init();
     CRYPTO_thread_setup();

   At the end of your program call:

     CRYPTO_thread_cleanup();

   Compile together with gsoap/plugin/threads.c

   The CRYPTO_thread_setup() and CRYPTO_thread_cleanup() are effective for
   OpenSSL versions prior to 1.1.0 but otherwise have no effect.
*/

#ifndef THREAD_SETUP_H
#define THREAD_SETUP_H

int CRYPTO_thread_setup();
void CRYPTO_thread_cleanup();

#endif
