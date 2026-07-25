/*  Copyright (C) 2012  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

// Include headers from C modules under test.
extern "C"
{
#include "MallocFailureInject.h"
}

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"


TEST_GROUP(MallocFailureInject)
{
    void setup()
    {
    }

    void teardown()
    {
        MallocFailureInject_Restore();
    }

    void reallocShouldFail(void)
    {
        void* pTest = hook_realloc(NULL, 10);
        POINTERS_EQUAL(NULL, pTest);
    }

    void reallocShouldPass(void)
    {
        void* pTest = hook_realloc(NULL, 10);
        CHECK(pTest != NULL);
        free(pTest);
    }
};


TEST(MallocFailureInject, FailFirstMalloc)
{
    MallocFailureInject_FailAllocation(1);
    void* pTest = hook_malloc(10);
    POINTERS_EQUAL(NULL, pTest);
}

TEST(MallocFailureInject, FailSecondMalloc)
{
    MallocFailureInject_FailAllocation(2);

    void* pFirstMalloc = hook_malloc(10);
    CHECK(NULL != pFirstMalloc);

    void* pSecondMalloc = hook_malloc(20);
    POINTERS_EQUAL(NULL, pSecondMalloc);

    free(pFirstMalloc);
}

TEST(MallocFailureInject, FailFirstRealloc)
{
    MallocFailureInject_FailAllocation(1);
    reallocShouldFail();
}

TEST(MallocFailureInject, FailSecondReallocOutOfThree)
{
    MallocFailureInject_FailAllocation(2);
    reallocShouldPass();
    reallocShouldFail();
    reallocShouldPass();
}

TEST(MallocFailureInject, VerifyDefaultsSucceed)
{
    void* pAlloc = hook_malloc(1);
    pAlloc = hook_realloc(pAlloc, 2);
    hook_free(pAlloc);
}
