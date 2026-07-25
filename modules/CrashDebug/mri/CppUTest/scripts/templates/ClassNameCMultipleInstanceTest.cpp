extern "C"
{
#include "ClassName.h"
}

#include "CppUTest/TestHarness.h"

TEST_GROUP(ClassName)
{
	ClassName aClassName;

	void setup()
	{
		aClassName = ClassName_Create();
	}

	void teardown()
	{
		ClassName_Destroy(aClassName);
	}
};

TEST(ClassName, Create)
{
	FAIL("Start here");
}

