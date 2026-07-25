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
#include "CppUTest/TestOutput.h"
#include "CppUTestExt/MemoryReportAllocator.h"
#include "CppUTestExt/MemoryReportFormatter.h"

#define TESTOUPUT_EQUAL(a) STRCMP_EQUAL_LOCATION(a, testOutput.getOutput().asCharString(), __FILE__, __LINE__);
#define TESTOUPUT_CONTAINS(a) STRCMP_CONTAINS_LOCATION(a, testOutput.getOutput().asCharString(), __FILE__, __LINE__);

TEST_GROUP(NormalMemoryReportFormatter)
{
	char* memory01;

	StringBufferTestOutput testOutput;
	TestResult* testResult;
	NormalMemoryReportFormatter formatter;

	void setup()
	{
		memory01 = (char*) 0x01;
		testResult = new TestResult(testOutput);
	}

	void teardown()
	{
		delete testResult;
	}
};


TEST(NormalMemoryReportFormatter, mallocCreatesAnMallocCall)
{
	formatter.report_alloc_memory(testResult, defaultMallocAllocator(), 10, memory01, "file", 9);
	TESTOUPUT_EQUAL(StringFromFormat("\tAllocation using malloc of size: 10 pointer: %p at file:9\n", memory01).asCharString());
}

TEST(NormalMemoryReportFormatter, freeCreatesAnFreeCall)
{
	formatter.report_free_memory(testResult, defaultMallocAllocator(), memory01, "boo", 6);
	TESTOUPUT_EQUAL(StringFromFormat("\tDeallocation using free of pointer: %p at boo:6\n", memory01).asCharString());
}

TEST(NormalMemoryReportFormatter, testStarts)
{
	UtestShell test("groupName", "TestName", "file", 1);
	formatter.report_test_start(testResult, test);
	TESTOUPUT_EQUAL("TEST(groupName, TestName)\n");
}

TEST(NormalMemoryReportFormatter, testEnds)
{
	UtestShell test("groupName", "TestName", "file", 1);
	formatter.report_test_end(testResult, test);
	TESTOUPUT_EQUAL("ENDTEST(groupName, TestName)\n");
}

TEST(NormalMemoryReportFormatter, testGroupStarts)
{
	UtestShell test("groupName", "TestName", "file", 1);
	formatter.report_testgroup_start(testResult, test);
	TESTOUPUT_EQUAL("------------------------------TEST GROUP(groupName)-----------------------------\n");
}
