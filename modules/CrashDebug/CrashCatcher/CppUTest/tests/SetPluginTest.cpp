#include "CppUTest/TestHarness.h"
#include "CppUTest/TestRegistry.h"
#include "CppUTest/TestOutput.h"
#include "CppUTest/TestPlugin.h"

void orig_func1()
{
}

void stub_func1()
{
}

void orig_func2()
{
}

void stub_func2()
{
}

void (*fp1)();
void (*fp2)();

TEST_GROUP(SetPointerPluginTest)
{
	SetPointerPlugin* plugin_;
	TestRegistry* myRegistry_;
	StringBufferTestOutput* output_;
	TestResult* result_;

	void setup()
	{
		myRegistry_ = new TestRegistry();
		plugin_ = new SetPointerPlugin("TestSetPlugin");
		myRegistry_->setCurrentRegistry(myRegistry_);
		myRegistry_->installPlugin(plugin_);
		output_ = new StringBufferTestOutput();
		result_ = new TestResult(*output_);
	}

	void teardown()
	{
		myRegistry_->setCurrentRegistry(0);
		delete myRegistry_;
		delete plugin_;
		delete output_;
		delete result_;
	}
};

class FunctionPointerUtest: public UtestShell
{
public:
	void setup()
	{
		UT_PTR_SET(fp1, stub_func1);
		UT_PTR_SET(fp2, stub_func2);
		UT_PTR_SET(fp2, stub_func2);
	}
	void testBody()
	{
		CHECK(fp1 == stub_func1);
		CHECK(fp2 == stub_func2);
	}
};

TEST(SetPointerPluginTest, installTwoFunctionPointer)
{
	FunctionPointerUtest *tst = new FunctionPointerUtest();
	;

	fp1 = orig_func1;
	fp2 = orig_func2;
	myRegistry_->addTest(tst);
	myRegistry_->runAllTests(*result_);
	CHECK(fp1 == orig_func1);
	CHECK(fp2 == orig_func2);
	LONGS_EQUAL(0, result_->getFailureCount());
	delete tst;
}

class MaxFunctionPointerUtest: public UtestShell
{
public:
	int numOfFpSets;
	MaxFunctionPointerUtest(int num) :
		numOfFpSets(num)
	{
	}
	;

	void setup()
	{
		for (int i = 0; i < numOfFpSets; ++i)
			UT_PTR_SET(fp1, stub_func1);
	}
};

IGNORE_TEST(SetPointerPluginTest, installTooMuchFunctionPointer)
{
	MaxFunctionPointerUtest *tst = new MaxFunctionPointerUtest(SetPointerPlugin::MAX_SET + 1);
	myRegistry_->addTest(tst);

	myRegistry_->runAllTests(*result_);

	LONGS_EQUAL(1, result_->getFailureCount());
	delete tst;
}

double orig_double = 3.0;
double* orig_double_ptr = &orig_double;
double stub_double = 4.0;

class SetDoublePointerUtest: public UtestShell
{
public:
	void setup()
	{
		UT_PTR_SET(orig_double_ptr, &stub_double);
	}
	void testBody()
	{
		CHECK(orig_double_ptr == &stub_double);
	}
};

TEST(SetPointerPluginTest, doublePointer)
{
	SetDoublePointerUtest *doubletst = new SetDoublePointerUtest();
	myRegistry_->addTest(doubletst);
	CHECK(orig_double_ptr == &orig_double);
	delete doubletst;
}
