
/* Additional include from CppUTestExt */
#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

/* Stubbed out product code using linker, function pointer, or overriding */
int foo(const char* param_string, int param_int)
{
	/* Tell CppUTest Mocking what we mock. Also return recorded value */
	return mock().actualCall("Foo")
			.withParameter("param_string", param_string)
			.withParameter("param_int", param_int)
			.returnValue().getIntValue();
}

void bar(double param_double, const char* param_string)
{
	mock().actualCall("Bar")
		.withParameter("param_double", param_double)
		.withParameter("param_string", param_string);
}

/* Production code calls to the methods we stubbed */
void productionCodeFooCalls()
{
	int return_value;
	return_value = foo("value_string", 10);
	return_value = foo("value_string", 10);
}

void productionCodeBarCalls()
{
	bar(1.5, "more");
	bar(1.5, "more");
}

/* Actual test */
TEST_GROUP(MockCheatSheet)
{
	void teardown()
	{
		/* Check expectations. Alternatively use MockSupportPlugin */
		mock().checkExpectations();
	}
};

TEST(MockCheatSheet, foo)
{
	/* Record 2 calls to Foo. Return different values on each call */
	mock().expectOneCall("Foo")
		.withParameter("param_string", "value_string")
		.withParameter("param_int", 10)
		.andReturnValue(30);
	mock().expectOneCall("Foo")
		.ignoreOtherParameters()
		.andReturnValue(50);

	/* Call production code */
	productionCodeFooCalls();
}

TEST(MockCheatSheet, bar)
{
	/* Expect 2 calls on Bar. Check only one parameter */
	mock().expectNCalls(2, "Bar")
		.withParameter("param_double", 1.5)
		.ignoreOtherParameters();

	/* And the production code call */
	productionCodeBarCalls();
}

