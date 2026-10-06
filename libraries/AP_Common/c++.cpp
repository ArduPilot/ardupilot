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

// intercept malloc to ensure memory so allocated is zeroed. this is rather
// nasty hack that hopefully does not live long! if it breaks again, that's it!
// this is completely unsupported!

// __imp_malloc is a pointer to a function that will be called to actually
// do the malloc work. we can't replace it (or malloc itself) at build time as
// that will cause cygwin to disable its own malloc. we therefore use a
// constructor function that runs before main but after cygwin's disable check
// to set the pointer to our own malloc which calls back into cygwin's allocator
// through calloc to actually do the work and the zeroing.

#include <stdio.h> // perror
#include <sys/mman.h> // mprotect and constants

extern "C" void *__imp_malloc; // the pointer of our affection

static void *do_the_malloc(size_t size) { // replacement for malloc
    // allocate zeroed using calloc (malloc would of course recurse infinitely)
    return calloc(1, size);
}

// called before main to set __imp_malloc. priority of 101 guarantees execution
// before C++ constructors.
__attribute__((constructor(101))) static void hack_in_malloc() {
    // __imp_malloc is in .text which is read-only so make it read-write first

    // compute address of its memory page as mprotect mandates page alignment
    size_t page_size = (size_t)sysconf(_SC_PAGESIZE);
    void *page_base = (void *)((size_t)&__imp_malloc & (~(page_size-1)));

    // make it writable and executable as we (unlikely) may be executing near it
    int res = mprotect(page_base, page_size, PROT_READ|PROT_WRITE|PROT_EXEC);
    if (res) {
        perror("hack_in_malloc() mprotect writable");
    }

    *(void **)&__imp_malloc = (void *)&do_the_malloc; // set the pointer now

    // put the page back to read-only (and let execution keep happening)
    res = mprotect(page_base, page_size, PROT_READ|PROT_EXEC);
    if (res) {
        perror("hack_in_malloc() mprotect readable");
    }
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

/*
  custom realloc which is guaranteed to zero newly-allocated memory
  (assuming malloc does). marked as WEAK so tests can override it.

  * always frees ptr (and returns nullptr) if new_size is 0
  * allocates if ptr is nullptr and new_size is not 0
  * else allocates new_size and copies over the smaller of old_size and new_size
    and frees ptr (old_size can be <= actual original requested size)
  * returns nullptr if allocation fails and leaves ptr and its data unchanged
*/
void * WEAK mem_realloc(void *ptr, size_t old_size, size_t new_size)
{
    if (new_size == 0) {
        free(ptr);
        return nullptr;
    }

    if (ptr == nullptr) {
        return malloc(new_size);
    }

    void *new_ptr = malloc(new_size);
    if (new_ptr != nullptr) {
        size_t copy_size = new_size > old_size ? old_size : new_size;
        memcpy(new_ptr, ptr, copy_size);
        free(ptr);
    }

    return new_ptr;
}
