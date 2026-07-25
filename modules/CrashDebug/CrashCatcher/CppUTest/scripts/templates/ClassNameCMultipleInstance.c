#include "ClassName.h"
#include <stdlib.h>
#include <memory.h>

typedef struct ClassNameStruct
{
    int placeHolderForHiddenStructElements;
} ClassNameStruct;

ClassName ClassName_Create(void)
{
     ClassName self = calloc(1, sizeof(ClassNameStruct));
     return self;
}

void ClassName_Destroy(ClassName self)
{
    free(self);
}


