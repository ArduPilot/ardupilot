extern "C" {
#include "ClassName.h"
#include "MockIO.h"
}

#include "CppUTest/TestHarness.h"

TEST_GROUP(ClassName)
{
	void setup()
	{
		Reset_Mock_IO();
		ClassName_Create();
	}

	void teardown()
	{
		ClassName_Destroy();
		Assert_No_Unused_Expectations();
	}
};

TEST(ClassName, Create)
{
	FAIL("Start here");
}

