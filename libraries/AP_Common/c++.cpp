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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsized-deallocation"
void operator delete(void *p)
{
    if (p) free(p);
}
#pragma GCC diagnostic pop

void * operator new[](size_t size)
{
    if (size < 1) {
        size = 1;
    }
    return(calloc(size, 1));
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsized-deallocation"
void operator delete[](void * ptr)
{
    if (ptr) free(ptr);
}
#pragma GCC diagnostic pop
