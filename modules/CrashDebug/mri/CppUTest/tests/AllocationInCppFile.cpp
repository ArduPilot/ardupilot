/* This file is for emulating allocations in a C++ file.
 * It is used simulating the use of the memory leak detector on production code in C++
 */
#undef new
#include "CppUTest/MemoryLeakDetectorNewMacros.h"

char* newAllocation()
{
	return new char;
}

char* newArrayAllocation()
{
	return new char[100];
}

#undef new

char* newAllocationWithoutMacro()
{
	return new char;
}

char* newArrayAllocationWithoutMacro()
{
	return new char[100];
}
