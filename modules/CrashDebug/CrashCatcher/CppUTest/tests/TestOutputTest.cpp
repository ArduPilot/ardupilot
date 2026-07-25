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
#include "CppUTest/TestResult.h"
#include "CppUTest/PlatformSpecificFunctions.h"

static long millisTime;

static long MockGetPlatformSpecificTimeInMillis()
{
	return millisTime;
}

TEST_GROUP(TestOutput)
{
	TestOutput* printer;
	StringBufferTestOutput* mock;
	UtestShell* tst;
	TestFailure *f;
	TestFailure *f2;
	TestFailure *f3;
	TestResult* result;

	void setup()
	{
		mock = new StringBufferTestOutput();
		printer = mock;
		tst = new UtestShell("group", "test", "file", 10);
		f = new TestFailure(tst, "failfile", 20, "message");
		f2 = new TestFailure(tst, "file", 20, "message");
		f3 = new TestFailure(tst, "file", 2, "message");
		result = new TestResult(*mock);
		result->setTotalExecutionTime(10);
		millisTime = 0;
		SetPlatformSpecificTimeInMillisMethod(MockGetPlatformSpecificTimeInMillis);
		TestOutput::setWorkingEnvironment(TestOutput::eclipse);

	}
	void teardown()
	{
		TestOutput::setWorkingEnvironment(TestOutput::detectEnvironment);
		delete printer;
		delete tst;
		delete f;
		delete f2;
		delete f3;
		delete result;
		SetPlatformSpecificTimeInMillisMethod(0);
	}
};

TEST(TestOutput, PrintConstCharStar)
{
	printer->print("hello");
	printer->print("hello\n");
	STRCMP_EQUAL("hellohello\n", mock->getOutput().asCharString());
}

TEST(TestOutput, PrintLong)
{
	printer->print(1234);
	STRCMP_EQUAL("1234", mock->getOutput().asCharString());
}

TEST(TestOutput, PrintDouble)
{
	printer->printDouble(12.34);
	STRCMP_EQUAL("12.34", mock->getOutput().asCharString());
}

TEST(TestOutput, StreamOperators)
{
	*printer << "n=" << 1234;
	STRCMP_EQUAL("n=1234", mock->getOutput().asCharString());
}

TEST(TestOutput, PrintTestEnded)
{
	printer->printCurrentTestEnded(*result);
	STRCMP_EQUAL(".", mock->getOutput().asCharString());
}

TEST(TestOutput, PrintTestALot)
{
	for (int i = 0; i < 60; ++i) {
		printer->printCurrentTestEnded(*result);
	}
	STRCMP_EQUAL("..................................................\n..........", mock->getOutput().asCharString());
}

TEST(TestOutput, SetProgressIndicator)
{
	result->setProgressIndicator(".");
	printer->printCurrentTestEnded(*result);
	result->setProgressIndicator("!");
	printer->printCurrentTestEnded(*result);
	result->setProgressIndicator(".");
	printer->printCurrentTestEnded(*result);

	STRCMP_EQUAL(".!.", mock->getOutput().asCharString());
}

TEST(TestOutput, PrintTestVerboseStarted)
{
	mock->verbose();
	printer->printCurrentTestStarted(*tst);
	STRCMP_EQUAL("TEST(group, test)", mock->getOutput().asCharString());
}

TEST(TestOutput, PrintTestVerboseEnded)
{
	mock->verbose();
	result->currentTestStarted(tst);
	millisTime = 5;
	result->currentTestEnded(tst);
	STRCMP_EQUAL("TEST(group, test) - 5 ms\n", mock->getOutput().asCharString());
}

TEST(TestOutput, PrintTestRun)
{
	printer->printTestRun(2, 3);
	STRCMP_EQUAL("Test run 2 of 3\n", mock->getOutput().asCharString());
}

TEST(TestOutput, PrintTestRunOnlyOne)
{
	printer->printTestRun(1, 1);
	STRCMP_EQUAL("", mock->getOutput().asCharString());
}

TEST(TestOutput, PrintWithFailureInSameFile)
{
	printer->print(*f2);
	STRCMP_EQUAL("\nfile:20: error: Failure in TEST(group, test)\n\tmessage\n\n", mock->getOutput().asCharString());
}

TEST(TestOutput, PrintFailureWithFailInDifferentFile)
{
	printer->print(*f);
	const char* expected =
			"\nfile:10: error: Failure in TEST(group, test)"
			"\nfailfile:20: error:\n\tmessage\n\n";
	STRCMP_EQUAL(expected, mock->getOutput().asCharString());
}

TEST(TestOutput, PrintFailureWithFailInHelper)
{
	printer->print(*f3);
	const char* expected =
			"\nfile:10: error: Failure in TEST(group, test)"
			"\nfile:2: error:\n\tmessage\n\n";
	STRCMP_EQUAL(expected, mock->getOutput().asCharString());
}

TEST(TestOutput, PrintInVisualStudioFormat)
{
	TestOutput::setWorkingEnvironment(TestOutput::vistualStudio);
	printer->print(*f3);
	const char* expected =
			"\nfile(10): error: Failure in TEST(group, test)"
			"\nfile(2): error:\n\tmessage\n\n";
	STRCMP_EQUAL(expected, mock->getOutput().asCharString());
}

TEST(TestOutput, PrintTestStarts)
{
	printer->printTestsStarted();
	STRCMP_EQUAL("", mock->getOutput().asCharString());
}

TEST(TestOutput, printTestsEnded)
{
	result->countTest();
	result->countCheck();
	result->countIgnored();
	result->countIgnored();
	result->countRun();
	result->countRun();
	result->countRun();
	printer->printTestsEnded(*result);
	STRCMP_EQUAL("\nOK (1 tests, 3 ran, 1 checks, 2 ignored, 0 filtered out, 10 ms)\n\n", mock->getOutput().asCharString());
}

TEST(TestOutput, printTestsEndedWithFailures)
{
	result->addFailure(*f);
	printer->flush();
	printer->printTestsEnded(*result);
	STRCMP_EQUAL("\nErrors (1 failures, 0 tests, 0 ran, 0 checks, 0 ignored, 0 filtered out, 10 ms)\n\n", mock->getOutput().asCharString());
}
