/*
  wrapper around new for C++ to ensure we always get zero filled memory
 */

#include <AP_HAL/AP_HAL.h>
#include <stdlib.h>
#include <new>
#include <AP_InternalError/AP_InternalError.h>

/*
  globally override new and delete to ensure that we always start with
  zero memory. This ensures consistent behaviour.

  Note that new comes in multiple different variants. When new is used
  without std::nothrow the compiler is free to assume it will not fail
  as it assumes exceptions are enabled. This makes code like this
  unsafe when using -fno-exceptions:

    a = new b;
    if (a == nullptr) {
      handle_error()
    }

  the compiler may remove the error handling. With g++ you can use
  -fcheck-new to avoid this, but on clang++ the compiler accepts
  -fcheck-new as a valid flag, but doesn't implement it, and may elide
  the error checking. That makes using clang++ unsafe with
  -fno-exceptions if you ever call new without std::nothrow.

  To avoid this we define NEW_NOTHROW as new(std::nothrow) and use it
  everywhere in ArduPilot, then we catch any missing cases with both
  an internal error and with a check of the elf for the symbols we
  want to avoid
*/

/*
  variant for new(std::nothrow), which is all that should be used in
  ArduPilot
 */
void * operator new(size_t size, std::nothrow_t const &nothrow)
{
    if (size < 1) {
        size = 1;
    }
    return(calloc(size, 1));
}

void * operator new[](size_t size, std::nothrow_t const &nothrow)
{
    if (size < 1) {
        size = 1;
    }
    return(calloc(size, 1));
}

/*
  These variants are for new without std::nothrow. We don't want to ever
  use this from ArduPilot code
 */
void * operator new(size_t size)
{
    if (size < 1) {
        size = 1;
    }
    return(calloc(size, 1));
}


void * operator new[](size_t size)
{
    if (size < 1) {
        size = 1;
    }
    return(calloc(size, 1));
}

void operator delete(void *p)
{
    if (p) free(p);
}

void operator delete[](void * ptr)
{
    if (ptr) free(ptr);
}

#if defined(CYGWIN_BUILD) && CONFIG_HAL_BOARD == HAL_BOARD_SITL
/*
  wrapper around malloc to ensure all memory is initialised as zero
  cygwin needs to wrap _malloc_r
 */
#undef _malloc_r
extern "C" {
    void *__wrap__malloc_r(_reent *r, size_t size);
    void *__real__malloc_r(_reent *r, size_t size);
    void *_malloc_r(_reent *r, size_t size);
}
void *__wrap__malloc_r(_reent *r, size_t size)
{
    void *ret = __real__malloc_r(r, size);
    if (ret != nullptr) {
        memset(ret, 0, size);
    }
    return ret;
}
void *_malloc_r(_reent *x, size_t size)
{
    void *ret = __real__malloc_r(x, size);
    if (ret != nullptr) {
        memset(ret, 0, size);
    }
    return ret;
}

#elif CONFIG_HAL_BOARD != HAL_BOARD_CHIBIOS && CONFIG_HAL_BOARD != HAL_BOARD_QURT
/*
  wrapper around malloc to ensure all memory is initialised as zero
  ChibiOS and QURT have their own wrappers
 */
extern "C" {
    void *__wrap_malloc(size_t size);
    void *__real_malloc(size_t size);
}
void *__wrap_malloc(size_t size)
{
    void *ret = __real_malloc(size);
    if (ret != nullptr) {
        memset(ret, 0, size);
    }
    return ret;
}
#endif
