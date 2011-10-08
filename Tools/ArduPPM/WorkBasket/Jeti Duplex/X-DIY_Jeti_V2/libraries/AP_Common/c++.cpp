// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// C++ runtime support not provided by Arduino
//
// Note: use new/delete with caution.  The heap is small and
// easily fragmented.
//

#include <stdlib.h>

void * operator new(size_t size) 
{
	return(calloc(size, 1));
}

void operator delete(void *p) 
{
	if (p)
		free(p);
}
