// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// C++ runtime support not provided by Arduino
//
// Note: use new/delete with caution.  The heap is small and
// easily fragmented.

#include <AP_HAL.h>
#include <stdlib.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_SMACCM || CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

void * operator new(size_t size)
{
    return(calloc(size, 1));
}

void operator delete(void *p)
{
    if (p) free(p);
}

// Conflicts with libmaple wirish/cxxabi-compat.cpp
#if CONFIG_HAL_BOARD != HAL_BOARD_FLYMAPLE
extern "C" void __cxa_pure_virtual(){
    while (1){}
}
#endif

void * operator new[](size_t size)
{
    return(calloc(size, 1));
}

void operator delete[](void * ptr)
{
    if (ptr) free(ptr);
}

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

