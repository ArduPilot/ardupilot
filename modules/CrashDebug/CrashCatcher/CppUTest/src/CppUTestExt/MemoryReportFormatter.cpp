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
#include "CppUTestExt/MemoryReportAllocator.h"
#include "CppUTestExt/MemoryReportFormatter.h"

NormalMemoryReportFormatter::NormalMemoryReportFormatter()
{
}

NormalMemoryReportFormatter::~NormalMemoryReportFormatter()
{
}

void NormalMemoryReportFormatter::report_test_start(TestResult* result, UtestShell& test)
{
	result->print(StringFromFormat("TEST(%s, %s)\n", test.getGroup().asCharString(), test.getName().asCharString()).asCharString());
}

void NormalMemoryReportFormatter::report_test_end(TestResult* result, UtestShell& test)
{
	result->print(StringFromFormat("ENDTEST(%s, %s)\n", test.getGroup().asCharString(), test.getName().asCharString()).asCharString());
}

void NormalMemoryReportFormatter::report_alloc_memory(TestResult* result, TestMemoryAllocator* allocator, size_t size, char* memory, const char* file, int line)
{
	result->print(StringFromFormat("\tAllocation using %s of size: %d pointer: %p at %s:%d\n", allocator->alloc_name(), size, memory, file, line).asCharString());
}

void NormalMemoryReportFormatter::report_free_memory(TestResult* result, TestMemoryAllocator* allocator, char* memory, const char* file, int line)
{
	result->print(StringFromFormat("\tDeallocation using %s of pointer: %p at %s:%d\n", allocator->free_name(),  memory, file, line).asCharString());
}

void NormalMemoryReportFormatter::report_testgroup_start(TestResult* result, UtestShell& test)
{
	const size_t line_size = 80;

	SimpleString groupName = StringFromFormat("TEST GROUP(%s)", test.getGroup().asCharString());
	size_t beginPos = (line_size/2) - (groupName.size()/2);

	SimpleString line("-", beginPos);
	line += groupName;
	line += SimpleString("-", line_size - line.size());
	line += "\n";
	result->print(line.asCharString());
}
