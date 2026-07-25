#include "AllocationInCFile.h"
#include "CppUTest/MemoryLeakDetectorMallocMacros.h"
#include "CppUTest/StandardCLibrary.h"

/* This file is for simulating overloads of malloc */

char* mallocAllocation()
{
	return (char*) malloc(10UL);
}

void freeAllocation(void* memory)
{
	free(memory);
}

#undef free

void freeAllocationWithoutMacro(void* memory)
{
	free(memory);
}

