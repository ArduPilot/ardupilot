
static void (*real_one) ();
static void stub(){}

/* in CheatSheetTest.cpp */
#include "CppUTest/TestHarness.h"

/* Declare TestGroup with name CheatSheet */
TEST_GROUP(CheatSheet)
{
/* declare a setup method for the test group. Optional. */
	void setup ()
	{
/* Set method real_one to stub. Automatically restore in teardown */
		UT_PTR_SET(real_one, stub);
	}

/* Declare a teardown method for the test group. Optional */
	void teardown()
	{
	}
}; /* Do not forget semicolumn */

/* Declare one test within the test group */
TEST(CheatSheet, TestName)
{
	/* Check two longs are equal */
	LONGS_EQUAL(1, 1);

	/* Check a condition */
	CHECK(true == true);

	/* Check a string */
	STRCMP_EQUAL("HelloWorld", "HelloWorld");
}

