#pragma once 

#include <reent.h>
#include <stdbool.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <hal_types.h>


// placement new is missed somehow

inline void * operator new(size_t size, caddr_t ptr)
{
    memset(ptr,0,size);
    return ptr;
}
