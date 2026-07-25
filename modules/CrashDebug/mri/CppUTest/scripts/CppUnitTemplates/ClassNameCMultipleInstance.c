#include "ClassName.h"
#include <stdlib.h>
#include <memory.h>

//static local variables
typedef struct _ClassName
{
    int placeHolderForHiddenStructElements;
};

ClassName* ClassName_Create(void)
{
     ClassName* self = malloc(sizeof(ClassName));
     memset(self, 0, sizeof(ClassName));
     return self;
}

void ClassName_Destroy(ClassName* self)
{
    free(self);
}


