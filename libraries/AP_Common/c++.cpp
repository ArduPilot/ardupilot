// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// C++ runtime support not provided by Arduino
//
// Note: use new/delete with caution.  The heap is small and
// easily fragmented.

#include <AP_HAL/AP_HAL.h>
#include <stdlib.h>

/*
  globally override new and delete to ensure that we always start with
  zero memory. This ensures consistent behaviour.
 */
void * operator new(size_t size)
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

void * operator new[](size_t size)
{
    if (size < 1) {
        size = 1;
    }
    return(calloc(size, 1));
}

void operator delete[](void * ptr)
{
    if (ptr) free(ptr);
}


#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

__extension__ typedef int __guard __attribute__((mode (__DI__)));

int __cxa_guard_acquire(__guard *g)
{
    return !*(char *)(g);
};

void __cxa_guard_release (__guard *g){
    *(char *)g = 1;
};

void __cxa_guard_abort (__guard *) {
};

#endif // CONFIG_HAL_BOARD

