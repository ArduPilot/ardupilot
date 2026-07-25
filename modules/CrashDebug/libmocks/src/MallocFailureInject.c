/*  Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/* Module for injecting allocation failures into realloc() calls from code under test. */


static void* defaultMalloc(size_t size);
static void* defaultRealloc(void* ptr, size_t size);
static void  defaultFree(void* ptr);

void* (*hook_malloc)(size_t size) = defaultMalloc;
void* (*hook_realloc)(void* ptr, size_t size) = defaultRealloc;
void  (*hook_free)(void* ptr) = defaultFree;

unsigned int   g_allocationToFail = 0;


/* Mallocs default to this routine so that the memory leak detection macros from the CppUTest code will be used. */
static void* defaultMalloc(size_t size)
{
    return malloc(size);
}

static void* defaultRealloc(void* ptr, size_t size)
{
    return realloc(ptr, size);
}

static void defaultFree(void* ptr)
{
    free(ptr);
}

static int shouldThisAllocationBeFailed(void)
{
    return g_allocationToFail && 0 == --g_allocationToFail;
}

static void* mock_malloc(size_t size)
{
    if (shouldThisAllocationBeFailed())
        return NULL;
    else
        return malloc(size);
}

static void* mock_realloc(void* ptr, size_t size)
{
    if (shouldThisAllocationBeFailed())
        return NULL;
    else
        return realloc(ptr, size);
}


/********************/
/* Public routines. */
/********************/
void MallocFailureInject_FailAllocation(unsigned int allocationToFail)
{
    hook_malloc = mock_malloc;
    hook_realloc = mock_realloc;
    g_allocationToFail = allocationToFail;
}

void MallocFailureInject_Restore(void)
{
    hook_realloc = defaultRealloc;
    hook_malloc = defaultMalloc;
}
