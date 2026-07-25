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
#include "CppUTest/TestRegistry.h"
#include "CppUTest/TestTestingFixture.h"

static bool afterCheck;

TEST_GROUP(UtestShell)
{
	TestTestingFixture* fixture;
	void setup()
	{
		fixture = new TestTestingFixture();
		afterCheck = false;
	}
	void teardown()
	{
		delete fixture;
	}

	void testFailureWith(void(*method)())
	{
		fixture->setTestFunction(method);
		fixture->runAllTests();
		LONGS_EQUAL(1, fixture->getFailureCount());
		CHECK(!afterCheck);
	}

	void testFailureWithMethodShouldContain(void(*method)(), const char * expected)
	{
		fixture->setTestFunction(method);
		fixture->runAllTests();
		fixture->assertPrintContains(expected);
		LONGS_EQUAL(1, fixture->getFailureCount());
		CHECK(!afterCheck);
	}

};

static void _passMethod()
{
	CHECK(true);
	afterCheck = true;
}

static void _passPrint()
{
	UT_PRINT("Hello World!");
}

static void _passPrintF()
{
	UT_PRINT(StringFromFormat("Hello %s %d", "World!", 2009));
}

static void _failMethod()
{
	FAIL("This test fails");
	afterCheck = true;
}

static void _failMethodFAIL_TEST()
{
	FAIL_TEST("This test fails");
	afterCheck = true;
}

static void _failMethodCHECK()
{
	CHECK(false);
	afterCheck = true;
}

static void _failMethodCHECK_TRUE()
{
	CHECK_TRUE(false);
	afterCheck = true;
}

static void _failMethodCHECK_FALSE()
{
	CHECK_FALSE(true);
	afterCheck = true;
}

static void _failMethodCHECK_EQUAL()
{
	CHECK_EQUAL(1, 2);
	afterCheck = true;
}

static void _failMethodSTRCMP_CONTAINS()
{
	STRCMP_CONTAINS("hello", "world");
	afterCheck = true;
}

static void _failMethodSTRCMP_NOCASE_CONTAINS()
{
	STRCMP_NOCASE_CONTAINS("hello", "WORLD");
	afterCheck = true;
}

static void _failMethodLONGS_EQUAL()
{
	LONGS_EQUAL(1, 0xff);
	afterCheck = true;
}
static void _failMethodBYTES_EQUAL()
{
	BYTES_EQUAL('a', 'b');
	afterCheck = true;
}

static void _failMethodPOINTERS_EQUAL()
{
	POINTERS_EQUAL((void*)0xa5a5, (void*)0xf0f0);
	afterCheck = true;
}

static void _failMethodDOUBLES_EQUAL()
{
	DOUBLES_EQUAL(0.12, 44.1, 0.3);
	afterCheck = true;
}

TEST(UtestShell, FailurePrintsSomething)
{
	testFailureWith(_failMethod);
	fixture->assertPrintContains(__FILE__);
	fixture->assertPrintContains("This test fails");
}

TEST(UtestShell, FailureWithFailTest)
{
	testFailureWith(_failMethodFAIL_TEST);
}

TEST(UtestShell, FailurePrintHexOutputForLongInts)
{
	testFailureWith(_failMethodLONGS_EQUAL);
	fixture->assertPrintContains("expected <  1 0x01>");
	fixture->assertPrintContains("but was  <255 0xff>");
}

TEST(UtestShell, FailurePrintHexOutputForPointers)
{
	testFailureWith(_failMethodPOINTERS_EQUAL);
	fixture->assertPrintContains("expected <0xa5a5>");
	fixture->assertPrintContains("but was  <0xf0f0>");
}

TEST(UtestShell, FailureWithDOUBLES_EQUAL)
{
	testFailureWith(_failMethodDOUBLES_EQUAL);
}

#include "CppUTest/PlatformSpecificFunctions.h"

TEST(UtestShell, compareDoubles)
{
	double zero = 0.0;
	double not_a_number = zero / zero;
	CHECK(doubles_equal(1.0, 1.001, 0.01));
	CHECK(!doubles_equal(not_a_number, 1.001, 0.01));
	CHECK(!doubles_equal(1.0, not_a_number, 0.01));
	CHECK(!doubles_equal(1.0, 1.001, not_a_number));
	CHECK(!doubles_equal(1.0, 1.1, 0.05));

	double a = 1.2345678;
	CHECK(doubles_equal(a, a, 0.000000001));


}


TEST(UtestShell, FailureWithCHECK)
{
	testFailureWith(_failMethodCHECK);
}

TEST(UtestShell, FailureWithCHECK_TRUE)
{
	testFailureWith(_failMethodCHECK_TRUE);
	fixture->assertPrintContains("CHECK_TRUE");
}

TEST(UtestShell, FailureWithCHECK_FALSE)
{
	testFailureWith(_failMethodCHECK_FALSE);
	fixture->assertPrintContains("CHECK_FALSE");
}

TEST(UtestShell, FailureWithCHECK_EQUAL)
{
	testFailureWith(_failMethodCHECK_EQUAL);
}

TEST(UtestShell, FailureWithSTRCMP_CONTAINS)
{
	testFailureWith(_failMethodSTRCMP_CONTAINS);
}

TEST(UtestShell, FailureWithSTRCMP_NOCASE_CONTAINS)
{
	testFailureWith(_failMethodSTRCMP_NOCASE_CONTAINS);
}

TEST(UtestShell, FailureWithBYTES_EQUAL)
{
	testFailureWith(_failMethodBYTES_EQUAL);
}

TEST(UtestShell, SuccessPrintsNothing)
{
	fixture->setTestFunction(_passMethod);
	fixture->runAllTests();
	LONGS_EQUAL(0, fixture->getFailureCount());
	fixture->assertPrintContains(".\nOK (1 tests");
	CHECK(afterCheck);
}

TEST(UtestShell, PrintPrintsWhateverPrintPrints)
{
	fixture->setTestFunction(_passPrint);
	fixture->runAllTests();
	LONGS_EQUAL(0, fixture->getFailureCount());
	fixture->assertPrintContains("Hello World!");
	fixture->assertPrintContains(__FILE__);
}

TEST(UtestShell, PrintPrintsPrintf)
{
	fixture->setTestFunction(_passPrintF);
	fixture->runAllTests();
	LONGS_EQUAL(0, fixture->getFailureCount());
	fixture->assertPrintContains("Hello World! 2009");
}

TEST(UtestShell, allMacros)
{
	CHECK(0 == 0);
	LONGS_EQUAL(1,1);
	BYTES_EQUAL(0xab,0xab);
	CHECK_EQUAL(100,100);
	STRCMP_EQUAL("THIS", "THIS");
	STRCMP_CONTAINS("THIS", "THISTHAT");
	STRCMP_NOCASE_EQUAL("this", "THIS");
	STRCMP_NOCASE_CONTAINS("this", "THISTHAT");
	DOUBLES_EQUAL(1.0, 1.0, .01);
	POINTERS_EQUAL(this, this);
}

static int functionThatReturnsAValue()
{
	CHECK(0 == 0);
	LONGS_EQUAL(1,1);
	BYTES_EQUAL(0xab,0xab);
	CHECK_EQUAL(100,100);
	STRCMP_EQUAL("THIS", "THIS");
	DOUBLES_EQUAL(1.0, 1.0, .01);
	POINTERS_EQUAL(0, 0);
	return 0;
}

TEST(UtestShell, allMacrosFromFunctionThatReturnsAValue)
{
	functionThatReturnsAValue();
}

TEST(UtestShell, AssertsActLikeStatements)
{
	if (fixture != 0) CHECK(true)
	else CHECK(false)

	if (fixture != 0) CHECK_EQUAL(true, true)
	else CHECK_EQUAL(false, false)

	if (fixture != 0) STRCMP_EQUAL("", "")
	else STRCMP_EQUAL("", " ")

	if (fixture != 0)
	STRCMP_CONTAINS("con", "contains")
	else
	STRCMP_CONTAINS("hello", "world")

	if (fixture != 0)
	LONGS_EQUAL(1, 1)
	else
	LONGS_EQUAL(1, 0)

	if (fixture != 0)
	DOUBLES_EQUAL(1, 1, 0.01)
	else
	DOUBLES_EQUAL(1, 0, 0.01)

	if (false)
	FAIL("")
	else CHECK(true);;

	if (true) ;
	else
	FAIL("")
}

IGNORE_TEST(UtestShell, IgnoreTestSupportsAllMacros)
{
	CHECK(true);
	CHECK_EQUAL(true, true);
	STRCMP_EQUAL("", "");
	LONGS_EQUAL(1, 1);
	DOUBLES_EQUAL(1, 1, 0.01);
	FAIL("");
}

IGNORE_TEST(UtestShell, IgnoreTestAccessingFixture)
{
	CHECK(fixture != 0);
}

TEST(UtestShell, MacrosUsedInSetup)
{
	IGNORE_ALL_LEAKS_IN_TEST();
	fixture->setSetup(_failMethod);
	fixture->setTestFunction(_passMethod);
	fixture->runAllTests();
	LONGS_EQUAL(1, fixture->getFailureCount());
}

TEST(UtestShell, MacrosUsedInTearDown)
{
	IGNORE_ALL_LEAKS_IN_TEST();
	fixture->setTeardown(_failMethod);
	fixture->setTestFunction(_passMethod);
	fixture->runAllTests();
	LONGS_EQUAL(1, fixture->getFailureCount());
}

static int teardownCalled = 0;

static void _teardownMethod()
{
	teardownCalled++;
}

TEST(UtestShell, TeardownCalledAfterTestFailure)
{
	teardownCalled = 0;
	IGNORE_ALL_LEAKS_IN_TEST();
	fixture->setTeardown(_teardownMethod);
	fixture->setTestFunction(_failMethod);
	fixture->runAllTests();
	LONGS_EQUAL(1, fixture->getFailureCount());
	LONGS_EQUAL(1, teardownCalled);
}

static int stopAfterFailure = 0;
static void _stopAfterFailureMethod()
{
	FAIL("fail");
	stopAfterFailure++;
}

TEST(UtestShell, TestStopsAfterTestFailure)
{
	IGNORE_ALL_LEAKS_IN_TEST();
	stopAfterFailure = 0;
	fixture->setTestFunction(_stopAfterFailureMethod);
	fixture->runAllTests();
	LONGS_EQUAL(1, fixture->getFailureCount());
	LONGS_EQUAL(0, stopAfterFailure);
}

TEST(UtestShell, TestStopsAfterSetupFailure)
{
	stopAfterFailure = 0;
	fixture->setSetup(_stopAfterFailureMethod);
	fixture->setTeardown(_stopAfterFailureMethod);
	fixture->setTestFunction(_failMethod);
	fixture->runAllTests();
	LONGS_EQUAL(2, fixture->getFailureCount());
	LONGS_EQUAL(0, stopAfterFailure);
}

static bool destructorWasCalledOnFailedTest = false;

class DestructorOughtToBeCalled
{
public:
	virtual ~DestructorOughtToBeCalled()
	{
		destructorWasCalledOnFailedTest = true;
	}
};

static void _destructorCalledForLocalObjects()
{
	DestructorOughtToBeCalled pleaseCallTheDestructor;
	destructorWasCalledOnFailedTest = false;
	FAIL("fail");
}

/* This test can only pass when we use exception handling instead of longjmp */
IGNORE_TEST(UtestShell, DestructorIsCalledForLocalObjectsWhenTheTestFails)
{
	fixture->setTestFunction(_destructorCalledForLocalObjects);
	fixture->runAllTests();
	CHECK(destructorWasCalledOnFailedTest);
}

TEST_BASE(MyOwnTest)
{
	MyOwnTest() :
		inTest(false)
	{
	}
	bool inTest;

	void setup()
	{
		CHECK(!inTest);
		inTest = true;
	}
	void teardown()
	{
		CHECK(inTest);
		inTest = false;
	}
};

TEST_GROUP_BASE(UtestMyOwn, MyOwnTest)
{
};

TEST(UtestMyOwn, test)
{
	CHECK(inTest);
}

class NullParameterTest: public UtestShell
{
};

TEST(UtestMyOwn, NullParameters)
{
	NullParameterTest nullTest; /* Bug fix tests for creating a test without a name, fix in SimpleString */
	TestRegistry* reg = TestRegistry::getCurrentRegistry();
	nullTest.shouldRun(reg->getGroupFilter(), reg->getNameFilter());
}

class AllocateAndDeallocateInConstructorAndDestructor
{
	char* memory_;
	char* morememory_;
public:
	AllocateAndDeallocateInConstructorAndDestructor()
	{
		memory_ = new char[100];
		morememory_ = NULL;
	}
	void allocateMoreMemory()
	{
		morememory_ = new char[123];
	}

	~AllocateAndDeallocateInConstructorAndDestructor()
	{
		delete [] memory_;
		delete [] morememory_;
	}
};

TEST_GROUP(CanHaveMemberVariablesInTestGroupThatAllocateMemoryWithoutCausingMemoryLeaks)
{
	AllocateAndDeallocateInConstructorAndDestructor dummy;
};

TEST(CanHaveMemberVariablesInTestGroupThatAllocateMemoryWithoutCausingMemoryLeaks, testInTestGroupName)
{
	dummy.allocateMoreMemory();
}


