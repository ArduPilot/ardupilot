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

namespace
{
const int failLineNumber = 2;
const char* failFileName = "fail.cpp";
}

static double zero = 0.0;
static const double not_a_number = zero / zero;

TEST_GROUP(TestFailure)
{
	UtestShell* test;
	StringBufferTestOutput* printer;

	void setup()
	{
		test = new NullTestShell(failFileName, failLineNumber-1);
		printer = new StringBufferTestOutput();
	}
	void teardown()
	{
		delete test;
		delete printer;
	}
	;
};
#define FAILURE_EQUAL(a, b) STRCMP_EQUAL_LOCATION(a, b.getMessage().asCharString(), __FILE__, __LINE__)

TEST(TestFailure, CreateFailure)
{
	TestFailure f1(test, failFileName, failLineNumber, "the failure message");
	TestFailure f2(test, "the failure message");
	TestFailure f3(test, failFileName, failLineNumber);
}

TEST(TestFailure, GetTestFileAndLineFromFailure)
{
	TestFailure f1(test, failFileName, failLineNumber, "the failure message");
	STRCMP_EQUAL(failFileName, f1.getTestFileName().asCharString());
	LONGS_EQUAL(1, f1.getTestLineNumber());
}

TEST(TestFailure, CreatePassingEqualsFailure)
{
	EqualsFailure f(test, failFileName, failLineNumber, "expected", "actual");
	FAILURE_EQUAL("expected <expected>\n\tbut was  <actual>", f);
}

TEST(TestFailure, EqualsFailureWithNullAsActual)
{
	EqualsFailure f(test, failFileName, failLineNumber, "expected", NULL);
	FAILURE_EQUAL("expected <expected>\n\tbut was  <(null)>", f);
}

TEST(TestFailure, EqualsFailureWithNullAsExpected)
{
	EqualsFailure f(test, failFileName, failLineNumber, NULL, "actual");
	FAILURE_EQUAL("expected <(null)>\n\tbut was  <actual>", f);
}

TEST(TestFailure, CheckEqualFailure)
{
	CheckEqualFailure f(test, failFileName, failLineNumber, "expected", "actual");
	FAILURE_EQUAL("expected <expected>\n"
			      "\tbut was  <actual>\n"
		          "\tdifference starts at position 0 at: <          actual    >\n"
		          "\t                                               ^", f);
}

TEST(TestFailure, CheckFailure)
{
	CheckFailure f(test, failFileName, failLineNumber, "CHECK", "chk");
	FAILURE_EQUAL("CHECK(chk) failed", f);
}

TEST(TestFailure, FailFailure)
{
	FailFailure f(test, failFileName, failLineNumber, "chk");
	FAILURE_EQUAL("chk", f);
}

TEST(TestFailure, LongsEqualFailure)
{
	LongsEqualFailure f(test, failFileName, failLineNumber, 1, 2);
	FAILURE_EQUAL("expected <1 0x1>\n\tbut was  <2 0x2>", f);
}

TEST(TestFailure, StringsEqualFailure)
{
	StringEqualFailure f(test, failFileName, failLineNumber, "abc", "abd");
	FAILURE_EQUAL("expected <abc>\n"
			    "\tbut was  <abd>\n"
			    "\tdifference starts at position 2 at: <        abd         >\n"
			    "\t                                               ^", f);
}

TEST(TestFailure, StringsEqualFailureAtTheEnd)
{
	StringEqualFailure f(test, failFileName, failLineNumber, "abc", "ab");
	FAILURE_EQUAL("expected <abc>\n"
			    "\tbut was  <ab>\n"
			    "\tdifference starts at position 2 at: <        ab          >\n"
			    "\t                                               ^", f);
}

TEST(TestFailure, StringsEqualFailureNewVariantAtTheEnd)
{
	StringEqualFailure f(test, failFileName, failLineNumber, "EndOfALongerString", "EndOfALongerStrinG");
	FAILURE_EQUAL("expected <EndOfALongerString>\n"
			    "\tbut was  <EndOfALongerStrinG>\n"
			    "\tdifference starts at position 17 at: <ongerStrinG         >\n"
			    "\t                                                ^", f);
}

TEST(TestFailure, StringsEqualFailureWithNewLinesAndTabs)
{
	StringEqualFailure f(test, failFileName, failLineNumber,
			"StringWith\t\nDifferentString",
			"StringWith\t\ndifferentString");

	FAILURE_EQUAL("expected <StringWith\t\nDifferentString>\n"
			    "\tbut was  <StringWith\t\ndifferentString>\n"
			    "\tdifference starts at position 12 at: <ringWith\t\ndifferentS>\n"
			    "\t                                              \t\n^", f);
}

TEST(TestFailure, StringsEqualFailureInTheMiddle)
{
	StringEqualFailure f(test, failFileName, failLineNumber, "aa", "ab");
	FAILURE_EQUAL("expected <aa>\n"
			    "\tbut was  <ab>\n"
			    "\tdifference starts at position 1 at: <         ab         >\n"
			    "\t                                               ^", f);
}


TEST(TestFailure, StringsEqualFailureAtTheBeginning)
{
	StringEqualFailure f(test, failFileName, failLineNumber, "aaa", "bbb");
	FAILURE_EQUAL("expected <aaa>\n"
			    "\tbut was  <bbb>\n"
			    "\tdifference starts at position 0 at: <          bbb       >\n"
			    "\t                                               ^", f);
}

TEST(TestFailure, StringsEqualNoCaseFailure)
{
	StringEqualNoCaseFailure f(test, failFileName, failLineNumber, "ABC", "abd");
	FAILURE_EQUAL("expected <ABC>\n"
			    "\tbut was  <abd>\n"
			    "\tdifference starts at position 2 at: <        abd         >\n"
			    "\t                                               ^", f);
}

TEST(TestFailure, StringsEqualNoCaseFailure2)
{
	StringEqualNoCaseFailure f(test, failFileName, failLineNumber, "ac", "AB");
	FAILURE_EQUAL("expected <ac>\n"
			    "\tbut was  <AB>\n"
			    "\tdifference starts at position 1 at: <         AB         >\n"
			    "\t                                               ^", f);
}

TEST(TestFailure, DoublesEqualNormal)
{
	DoublesEqualFailure f(test, failFileName, failLineNumber, 1.0, 2.0, 3.0);
	FAILURE_EQUAL("expected <1>\n"
			    "\tbut was  <2> threshold used was <3>", f);
}

TEST(TestFailure, DoublesEqualExpectedIsNaN)
{
	DoublesEqualFailure f(test, failFileName, failLineNumber, not_a_number, 2.0, 3.0);
	FAILURE_EQUAL("expected <Nan - Not a number>\n"
			    "\tbut was  <2> threshold used was <3>\n"
			    "\tCannot make comparisons with Nan", f);
}

TEST(TestFailure, DoublesEqualActualIsNaN)
{
	DoublesEqualFailure f(test, failFileName, failLineNumber, 1.0, not_a_number, 3.0);
	FAILURE_EQUAL("expected <1>\n"
			    "\tbut was  <Nan - Not a number> threshold used was <3>\n"
			    "\tCannot make comparisons with Nan", f);
}

TEST(TestFailure, DoublesEqualThresholdIsNaN)
{
	DoublesEqualFailure f(test, failFileName, failLineNumber, 1.0, 2.0, not_a_number);
	FAILURE_EQUAL("expected <1>\n"
			    "\tbut was  <2> threshold used was <Nan - Not a number>\n"
			    "\tCannot make comparisons with Nan", f);
}
