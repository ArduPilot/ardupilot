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
#include "CppUTestExt/MockSupport.h"
#include "CppUTestExt/MockSupportPlugin.h"
#include "TestMockFailure.h"

TEST_GROUP(MockPlugin)
{
	UtestShell *test;
	StringBufferTestOutput *output;
	TestResult *result;
	MockExpectedFunctionsList *expectationsList;
	MockExpectedFunctionCall *call;

	MockSupportPlugin *plugin;

	void setup()
	{
		mock().setMockFailureReporter(MockFailureReporterForTest::getReporter());

		test = new UtestShell("group", "name", "file", 1);
		output = new StringBufferTestOutput;
		result = new TestResult(*output);
		expectationsList = new MockExpectedFunctionsList;
		call = new MockExpectedFunctionCall;
		expectationsList->addExpectedCall(call);
		plugin = new MockSupportPlugin;;
	}

	void teardown()
	{
		delete test;
		delete output;
		delete result;
		delete expectationsList;
		delete call;
		delete plugin;

		CHECK_NO_MOCK_FAILURE();
		mock().setMockFailureReporter(NULL);
	}
};

TEST(MockPlugin, checkExpectationsAndClearAtEnd)
{
	call->withName("foobar");
	MockExpectedCallsDidntHappenFailure expectedFailure(test, *expectationsList);

	mock().expectOneCall("foobar");

	plugin->postTestAction(*test, *result);

	STRCMP_CONTAINS(expectedFailure.getMessage().asCharString(), output->getOutput().asCharString())
	LONGS_EQUAL(0, mock().expectedCallsLeft());
//	clear makes sure there are no memory leaks.
}

TEST(MockPlugin, checkExpectationsWorksAlsoWithHierachicalObjects)
{
	call->withName("foobar").onObject((void*) 1);
	MockExpectedObjectDidntHappenFailure expectedFailure(test, "foobar", *expectationsList);

	mock("differentScope").expectOneCall("foobar").onObject((void*) 1);
	mock("differentScope").actualCall("foobar");

	plugin->postTestAction(*test, *result);

	STRCMP_CONTAINS(expectedFailure.getMessage().asCharString(), output->getOutput().asCharString())
}

class DummyComparator : public MockNamedValueComparator
{
public:
	bool isEqual(void* object1, void* object2)
	{
		return object1 == object2;
	}
	SimpleString valueToString(void*)
	{
		return "string";
	}

};

TEST(MockPlugin, installComparatorRecordsTheComparatorButNotInstallsItYet)
{
	DummyComparator comparator;
	plugin->installComparator("myType", comparator);
	mock().expectOneCall("foo").withParameterOfType("myType", "name", &comparator);
	mock().actualCall("foo").withParameterOfType("myType", "name", &comparator);

	MockNoWayToCompareCustomTypeFailure failure(test, "myType");
	CHECK_EXPECTED_MOCK_FAILURE(failure);
}

TEST(MockPlugin, preTestActionWillEnableMultipleComparatorsToTheGlobalMockSupportSpace)
{
	DummyComparator comparator;
	DummyComparator comparator2;
	plugin->installComparator("myType", comparator);
	plugin->installComparator("myOtherType", comparator2);

	plugin->preTestAction(*test, *result);
	mock().expectOneCall("foo").withParameterOfType("myType", "name", &comparator);
	mock().expectOneCall("foo").withParameterOfType("myOtherType", "name", &comparator);
	mock().actualCall("foo").withParameterOfType("myType", "name", &comparator);
	mock().actualCall("foo").withParameterOfType("myOtherType", "name", &comparator);

	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
	LONGS_EQUAL(0, result->getFailureCount());
}
