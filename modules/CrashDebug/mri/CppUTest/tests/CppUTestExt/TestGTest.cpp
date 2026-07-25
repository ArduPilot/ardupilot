/*
 * Copyright (c) 2011, Michael Feathers, James Grenning and Bas Vodde
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

#ifdef CPPUTEST_USE_REAL_GTEST
#undef new
#endif

#include "gtest/gtest.h"

TEST(GTestSimpleTest, GTestEqual)
{
	EXPECT_EQ(1, 1);
}
TEST(GTestSimpleTest, GTestAssertEq)
{
	ASSERT_EQ(1, 1);
}

TEST(GTestSimpleTest, GTestExpectTrue)
{
	EXPECT_TRUE(true);
}

TEST(GTestSimpleTest, GTestAssertTrue)
{
	ASSERT_TRUE(true);
}

TEST(GTestSimpleTest, GTestExpectFalse)
{
	EXPECT_FALSE(false);
}

TEST(GTestSimpleTest, GTestExpectStreq)
{
	EXPECT_STREQ("hello world", "hello world");
}

#ifdef CPPUTEST_USE_REAL_GTEST

/* Death tests are IMHO not a good idea at all. But for compatibility reason, we'll support it */

static void crashMe ()
{
	fprintf(stderr, "Crash me!");
	*((int*) 0) = 10;
}

TEST(GTestSimpleTest, GTestDeathTest)
{
	ASSERT_DEATH(crashMe(), "Crash me!");
}

#endif

class GTestTestingFixtureTest : public testing::Test {
protected:
	bool setup_was_called;
	char* freed_during_teardown;

	void SetUp()
	{
		setup_was_called = true;
		freed_during_teardown = NULL;
	}

	void TearDown()
	{
		delete [] freed_during_teardown;
	}
};

TEST_F(GTestTestingFixtureTest, setupBeenCalled)
{
	EXPECT_TRUE(setup_was_called);
}

TEST_F(GTestTestingFixtureTest, teardownMustBeCalledOrElseThisWillLeak)
{
	freed_during_teardown = new char[100];
}

#undef TEST
#undef RUN_ALL_TESTS
#undef FAIL

#include "CppUTest/TestHarness.h"
#include "CppUTest/TestRegistry.h"
#include "CppUTest/TestOutput.h"
#include "CppUTest/TestTestingFixture.h"
#include "CppUTestExt/GTestConvertor.h"

#ifdef CPPUTEST_USE_REAL_GTEST

TEST_GROUP(GTestConvertor)
{
};

TEST(GTestConvertor, correctNumberOfTestCases)
{
	LONGS_EQUAL(2, ::testing::UnitTest::GetInstance()->total_test_case_count());
	CHECK(::testing::UnitTest::GetInstance()->GetTestCase(0));
	CHECK(::testing::UnitTest::GetInstance()->GetTestCase(1));
	CHECK(::testing::UnitTest::GetInstance()->GetTestCase(2) == NULL);
}

TEST(GTestConvertor, correctNumberOfTestsInTheTestCases)
{
	const ::testing::TestCase* firstTestCase = ::testing::UnitTest::GetInstance()->GetTestCase(0);
	const ::testing::TestCase* secondTestCase = ::testing::UnitTest::GetInstance()->GetTestCase(1);

	STRCMP_EQUAL("GTestSimpleTest", firstTestCase->name());
	STRCMP_EQUAL("GTestTestingFixtureTest", secondTestCase->name());
	LONGS_EQUAL(7, firstTestCase->total_test_count());
	LONGS_EQUAL(2, secondTestCase->total_test_count());
}

TEST(GTestConvertor, testsGetAddedToCurrentTestRegistry)
{
	TestTestingFixture fixture;
	TestRegistry::getCurrentRegistry()->unDoLastAddTest();

	GTestConvertor convertor(false);
	convertor.addAllGTestToTestRegistry();

	LONGS_EQUAL(9, TestRegistry::getCurrentRegistry()->countTests());
}

#endif

TEST_GROUP(gtest)
{
};

TEST(gtest, SimpleGoogleTestExists)
{
	TestRegistry* registry = TestRegistry::getCurrentRegistry();
	CHECK(registry->findTestWithName("GTestEqual"));
}

TEST(gtest, SimpleGoogleTestGroupExists)
{
	TestRegistry* registry = TestRegistry::getCurrentRegistry();
	CHECK(registry->findTestWithGroup("GTestSimpleTest"));
}

bool afterCheck;

static void _failMethodEXPECT_EQ()
{
	EXPECT_EQ(1, 2);
	afterCheck = true;
}

static void _failMethodASSERT_EQ()
{
	ASSERT_EQ(1, 2);
	afterCheck = true;
}

static void _failMethodEXPECT_TRUE()
{
	EXPECT_TRUE(false);
	afterCheck = true;
}

static void _failMethodASSERT_TRUE()
{
	ASSERT_TRUE(false);
	afterCheck = true;
}

static void _failMethodEXPECT_FALSE()
{
	EXPECT_FALSE(true);
	afterCheck = true;
}

static void _failMethodEXPECT_STREQ()
{
	EXPECT_STREQ("hello", "world");
	afterCheck = true;
}

TEST_GROUP(gtestMacros)
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

};

TEST(gtestMacros, EXPECT_EQFails)
{
	testFailureWith(_failMethodEXPECT_EQ);
}

TEST(gtestMacros, EXPECT_TRUEFails)
{
	testFailureWith(_failMethodEXPECT_TRUE);
}

TEST(gtestMacros, EXPECT_FALSEFails)
{
	testFailureWith(_failMethodEXPECT_FALSE);
}

TEST(gtestMacros, EXPECT_STREQFails)
{
	testFailureWith(_failMethodEXPECT_STREQ);
}

TEST(gtestMacros, ASSERT_EQFails)
{
	testFailureWith(_failMethodASSERT_EQ);
}

TEST(gtestMacros, ASSERT_TRUEFails)
{
	testFailureWith(_failMethodASSERT_TRUE);
}
