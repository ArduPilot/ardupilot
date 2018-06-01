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
  zero memory. This ensures consistent behaviour. Note that
  initialising all members of all C++ classes separately takes a lot
  of flash space. On APM1/APM2 it would mean we wouldn't fit on the
  board at all.
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


#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

// Conflicts with libmaple wirish/cxxabi-compat.cpp
#if CONFIG_HAL_BOARD != HAL_BOARD_FLYMAPLE
extern "C" void __cxa_pure_virtual(){
    while (1){}
}
#endif

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

