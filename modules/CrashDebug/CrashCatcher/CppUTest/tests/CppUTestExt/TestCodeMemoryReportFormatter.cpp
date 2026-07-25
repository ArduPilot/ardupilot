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
#include "CppUTestExt/CodeMemoryReportFormatter.h"

#define TESTOUPUT_EQUAL(a) STRCMP_EQUAL_LOCATION(a, testOutput.getOutput().asCharString(), __FILE__, __LINE__);
#define TESTOUPUT_CONTAINS(a) STRCMP_CONTAINS_LOCATION(a, testOutput.getOutput().asCharString(), __FILE__, __LINE__);

TEST_GROUP(CodeMemoryReportFormatter)
{
	TestMemoryAllocator* cAllocator;
	TestMemoryAllocator* newAllocator;
	TestMemoryAllocator* newArrayAllocator;
	char* memory01;
	char* memory02;

	StringBufferTestOutput testOutput;
	TestResult* testResult;
	CodeMemoryReportFormatter* formatter;

	void setup()
	{
		cAllocator = defaultMallocAllocator();
		newAllocator = defaultNewAllocator();
		newArrayAllocator= defaultNewArrayAllocator();
		memory01 = (char*) 0x01;
		memory02 = (char*) 0x02;

		formatter = new CodeMemoryReportFormatter(cAllocator);
		testResult = new TestResult(testOutput);
	}

	void teardown()
	{
		delete testResult;
		delete formatter;
	}
};


TEST(CodeMemoryReportFormatter, mallocCreatesAnMallocCall)
{
	formatter->report_alloc_memory(testResult, cAllocator, 10, memory01, "file", 9);
	TESTOUPUT_EQUAL("\tvoid* file_9_1 = malloc(10);\n");
}

TEST(CodeMemoryReportFormatter, freeCreatesAnFreeCall)
{
	formatter->report_alloc_memory(testResult, cAllocator, 10, memory01, "file", 9);
	testOutput.flush();
	formatter->report_free_memory(testResult, cAllocator, memory01, "boo", 6);
	TESTOUPUT_EQUAL("\tfree(file_9_1); /* at boo:6 */\n");
}

TEST(CodeMemoryReportFormatter, twoMallocAndTwoFree)
{
	formatter->report_alloc_memory(testResult, cAllocator, 10, memory01, "file", 2);
	formatter->report_alloc_memory(testResult, cAllocator, 10, memory02, "boo", 4);
	testOutput.flush();
	formatter->report_free_memory(testResult, cAllocator, memory01, "foo", 6);
	formatter->report_free_memory(testResult, cAllocator, memory02, "bar", 8);
	TESTOUPUT_CONTAINS("\tfree(file_2_1); /* at foo:6 */\n");
	TESTOUPUT_CONTAINS("\tfree(boo_4_1); /* at bar:8 */\n");
}

TEST(CodeMemoryReportFormatter, variableNamesShouldNotContainSlahses)
{
	formatter->report_alloc_memory(testResult, cAllocator, 10, memory01, "dir/file", 2);
	TESTOUPUT_CONTAINS("\tvoid* file_2");
}

TEST(CodeMemoryReportFormatter, variableNamesShouldNotContainDotButUseUnderscore)
{
	formatter->report_alloc_memory(testResult, cAllocator, 10, memory01, "foo.cpp", 2);
	TESTOUPUT_CONTAINS("foo_cpp");
}

TEST(CodeMemoryReportFormatter, newArrayAllocatorGeneratesNewArrayCode)
{
	formatter->report_alloc_memory(testResult, newArrayAllocator, 10, memory01, "file", 8);
	TESTOUPUT_CONTAINS("char* file_8_1 = new char[10]; /* using new [] */");
}

TEST(CodeMemoryReportFormatter, newArrayGeneratesNewCode)
{
	formatter->report_alloc_memory(testResult, newAllocator, 6, memory01, "file", 4);
	TESTOUPUT_CONTAINS("new char[6]; /* using new */");
}

TEST(CodeMemoryReportFormatter, NewAllocatorGeneratesDeleteCode)
{
	formatter->report_alloc_memory(testResult, newAllocator, 10, memory01, "file", 8);
	testOutput.flush();
	formatter->report_free_memory(testResult, newAllocator, memory01, "boo", 4);
	TESTOUPUT_CONTAINS("delete [] file_8_1; /* using delete at boo:4 */");
}

TEST(CodeMemoryReportFormatter, DeleteNullWorksFine)
{
	formatter->report_free_memory(testResult, newAllocator, NULL, "boo", 4);
	TESTOUPUT_CONTAINS("delete [] NULL; /* using delete at boo:4 */");
}

TEST(CodeMemoryReportFormatter, NewArrayAllocatorGeneratesDeleteArrayCode)
{
	formatter->report_alloc_memory(testResult, newArrayAllocator, 10, memory01, "file", 8);
	testOutput.flush();
	formatter->report_free_memory(testResult, newArrayAllocator, memory01, "boo", 4);
	TESTOUPUT_CONTAINS("delete [] file_8_1; /* using delete [] at boo:4 */");
}

TEST(CodeMemoryReportFormatter, allocationUsingMallocOnTheSameLineDoesntGenerateTheSameVariableTwice)
{
	formatter->report_alloc_memory(testResult, cAllocator, 10, memory01, "file", 8);
	testOutput.flush();
	formatter->report_alloc_memory(testResult, cAllocator, 10, memory02, "file", 8);
	CHECK(testOutput.getOutput().contains("2"));
}

TEST(CodeMemoryReportFormatter, allocationUsingNewcOnTheSameLineDoesntGenerateTheSameVariableTwice)
{
	formatter->report_alloc_memory(testResult, newAllocator, 10, memory01, "file", 8);
	testOutput.flush();
	formatter->report_alloc_memory(testResult, newAllocator, 10, memory01, "file", 8);
	CHECK(testOutput.getOutput().contains("2"));
}

TEST(CodeMemoryReportFormatter, allocationUsingNewcOnTheSameLineDoesntGenerateVariableTwiceExceptWhenInANewTest)
{
	formatter->report_alloc_memory(testResult, newAllocator, 10, memory01, "file", 8);
	formatter->report_test_start(testResult, *UtestShell::getCurrent());
	testOutput.flush();
	formatter->report_alloc_memory(testResult, newAllocator, 10, memory01, "file", 8);
	CHECK(testOutput.getOutput().contains("char*"));
}

TEST(CodeMemoryReportFormatter, testStartGeneratesTESTcode)
{
	UtestShell test("groupName", "testName", "fileName", 1);
	formatter->report_test_start(testResult, test);
	TESTOUPUT_EQUAL("*/\nTEST(groupName_memoryReport, testName)\n{ /* at fileName:1 */\n");
}

TEST(CodeMemoryReportFormatter, testEndGeneratesTESTcode)
{
	UtestShell test("groupName", "testName", "fileName", 1);
	formatter->report_test_end(testResult, test);
	TESTOUPUT_EQUAL("}/*");
}

TEST(CodeMemoryReportFormatter, TestGroupGeneratesTestGroupCode)
{
	UtestShell test("groupName", "testName", "fileName", 1);
	formatter->report_testgroup_start(testResult, test);
	TESTOUPUT_EQUAL("*/TEST_GROUP(groupName_memoryReport)\n{\n};\n/*");
}

// TODO: do!
/* Dealloc without alloc */
/* Remove the ugly comments by controlling the output! */
/* Write tests for the variable name lengths */
