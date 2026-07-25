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

#include "threads.h" /* gsoap/plugin/threads.h */

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
