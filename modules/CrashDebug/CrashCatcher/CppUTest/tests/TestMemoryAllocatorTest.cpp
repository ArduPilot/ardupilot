/*
 * Copyright (c) 2007, Michael Feathers, James Grenning and Bas Vodde
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE EARLIER MENTIONED AUTHORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "CppUTest/TestHarness.h"
#include "CppUTest/TestMemoryAllocator.h"
#include "CppUTest/PlatformSpecificFunctions.h"

TEST_GROUP(TestMemoryAllocatorTest)
{
	TestMemoryAllocator* allocator;

	void setup()
	{
		allocator = NULL;
	}

	void teardown()
	{
		if (allocator) delete allocator;
	}
};

TEST(TestMemoryAllocatorTest, SetCurrentNewAllocator)
{
	allocator = new TestMemoryAllocator("new allocator for test");
	setCurrentNewAllocator(allocator);
	POINTERS_EQUAL(allocator, getCurrentNewAllocator());
	setCurrentNewAllocatorToDefault();
	POINTERS_EQUAL(defaultNewAllocator(), getCurrentNewAllocator());
}

TEST(TestMemoryAllocatorTest, SetCurrentNewArrayAllocator)
{
	allocator = new TestMemoryAllocator("new array allocator for test");
	setCurrentNewArrayAllocator(allocator);
	POINTERS_EQUAL(allocator, getCurrentNewArrayAllocator());
	setCurrentNewArrayAllocatorToDefault();
	POINTERS_EQUAL(defaultNewArrayAllocator(), getCurrentNewArrayAllocator());
}

TEST(TestMemoryAllocatorTest, SetCurrentMallocAllocator)
{
	allocator = new TestMemoryAllocator("malloc_allocator");
	setCurrentMallocAllocator(allocator);
	POINTERS_EQUAL(allocator, getCurrentMallocAllocator());
	setCurrentMallocAllocatorToDefault();
	POINTERS_EQUAL(defaultMallocAllocator(), getCurrentMallocAllocator());
}

TEST(TestMemoryAllocatorTest, MemoryAllocation)
{
	allocator = new TestMemoryAllocator();
	allocator->free_memory(allocator->alloc_memory(100, "file", 1), "file", 1);
}

TEST(TestMemoryAllocatorTest, MallocNames)
{
	STRCMP_EQUAL("Standard Malloc Allocator", defaultMallocAllocator()->name());
	STRCMP_EQUAL("malloc", defaultMallocAllocator()->alloc_name());
	STRCMP_EQUAL("free", defaultMallocAllocator()->free_name());
}

TEST(TestMemoryAllocatorTest, NewNames)
{
	STRCMP_EQUAL("Standard New Allocator", defaultNewAllocator()->name());
	STRCMP_EQUAL("new", defaultNewAllocator()->alloc_name());
	STRCMP_EQUAL("delete", defaultNewAllocator()->free_name());
}

TEST(TestMemoryAllocatorTest, NewArrayNames)
{
	STRCMP_EQUAL("Standard New [] Allocator", defaultNewArrayAllocator()->name());
	STRCMP_EQUAL("new []", defaultNewArrayAllocator()->alloc_name());
	STRCMP_EQUAL("delete []", defaultNewArrayAllocator()->free_name());
}

TEST(TestMemoryAllocatorTest, NullUnknownAllocation)
{
	allocator = new NullUnknownAllocator;
	allocator->free_memory(allocator->alloc_memory(100, "file", 1), "file", 1);
}

TEST(TestMemoryAllocatorTest, NullUnknownNames)
{
	allocator = new NullUnknownAllocator;
	STRCMP_EQUAL("Null Allocator", allocator->name());
	STRCMP_EQUAL("unknown", allocator->alloc_name());
	STRCMP_EQUAL("unknown", allocator->free_name());
}
