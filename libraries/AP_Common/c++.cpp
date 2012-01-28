// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// C++ runtime support not provided by Arduino
//
// Note: use new/delete with caution.  The heap is small and
// easily fragmented.
//

#include <stdlib.h>
#include "c++.h"
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

void * operator new(size_t size)
{
#ifdef AP_DISPLAYMEM
    displayMemory();
#endif
    return(calloc(size, 1));
}

void operator delete(void *p)
{
    if (p) free(p);
}

extern "C" void __cxa_pure_virtual()
{
    while (1)
    {
        Serial.println("Error: pure virtual call");
        delay(1000);
    }
}

void * operator new[](size_t size)
{
#ifdef AP_DISPLAYMEM
    displayMemory();
#endif
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

void __cxa_guard_release (__guard *g)
{
    *(char *)g = 1;
};

void __cxa_guard_abort (__guard *) {};

// free memory
extern unsigned int __bss_end;
extern void *__brkval;

void displayMemory()
{
    static int minMemFree=0;
    if (minMemFree<=0 || freeMemory()<minMemFree) {
        minMemFree = freeMemory();
        Serial.print("bytes free: ");
        Serial.println(minMemFree);
    }
}

int freeMemory()
{
    int free_memory;

    if ((intptr_t)__brkval == 0)
        free_memory = ((intptr_t)&free_memory) - ((intptr_t)&__bss_end);
    else
        free_memory = ((intptr_t)&free_memory) - ((intptr_t)__brkval);

    return free_memory;
}
