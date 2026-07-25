
#ifndef CPPUTEST_STD_C_LIB_DISABLED

#include "AllocLetTestFree.h"
#include <stdlib.h>
#include <memory.h>

typedef struct AllocLetTestFreeStruct
{
    int placeHolderForHiddenStructElements;
} AllocLetTestFreeStruct;

AllocLetTestFree AllocLetTestFree_Create(void)
{
	size_t count = 1;
    AllocLetTestFree self = calloc(count, sizeof(AllocLetTestFreeStruct));
    return self;
}

void AllocLetTestFree_Destroy(AllocLetTestFree self)
{
	(void)self;
}

#endif
