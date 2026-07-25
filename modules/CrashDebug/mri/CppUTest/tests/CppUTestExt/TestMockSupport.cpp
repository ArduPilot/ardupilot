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
#include "CppUTestExt/MockSupport.h"
#include "CppUTestExt/MockExpectedFunctionCall.h"
#include "CppUTestExt/MockFailure.h"
#include "TestMockFailure.h"

TEST_GROUP(MockSupportTest)
{
	MockExpectedFunctionsList *expectationsList;

	void setup()
	{
		mock().setMockFailureReporter(MockFailureReporterForTest::getReporter());
		expectationsList = new MockExpectedFunctionsList;
	}

	void teardown()
	{
		mock().checkExpectations();
		CHECK_NO_MOCK_FAILURE();
		expectationsList->deleteAllExpectationsAndClearList();
		delete expectationsList;
		mock().setMockFailureReporter(NULL);
	}

	MockExpectedFunctionCall* addFunctionToExpectationsList(const SimpleString& name)
	{
		MockExpectedFunctionCall* newCall = new MockExpectedFunctionCall;
		newCall->withName(name);
		expectationsList->addExpectedCall(newCall);
		return newCall;
	}

	MockExpectedFunctionCall* addFunctionToExpectationsList(const SimpleString& name, int order)
	{
		MockExpectedFunctionCall* newCall = new MockExpectedFunctionCall;
		newCall->withName(name);
		newCall->withCallOrder(order);
		expectationsList->addExpectedCall(newCall);
		return newCall;
	}

};

TEST(MockSupportTest, clear)
{
	mock().expectOneCall("func");
	mock().clear();
	CHECK(! mock().expectedCallsLeft());
}

TEST(MockSupportTest, checkExpectationsDoesntFail)
{
	mock().checkExpectations();
}

TEST(MockSupportTest, checkExpectationsClearsTheExpectations)
{
	addFunctionToExpectationsList("foobar");
	MockExpectedCallsDidntHappenFailure expectedFailure(mockFailureTest(), *expectationsList);

	mock().expectOneCall("foobar");
	mock().checkExpectations();

	CHECK(! mock().expectedCallsLeft());
	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, exceptACallThatHappens)
{
	mock().expectOneCall("func");
	mock().actualCall("func");
	CHECK(! mock().expectedCallsLeft());
}

TEST(MockSupportTest, exceptACallInceasesExpectedCallsLeft)
{
	mock().expectOneCall("func");
	CHECK(mock().expectedCallsLeft());
	mock().clear();
}

TEST(MockSupportTest, unexpectedCallHappened)
{
	MockUnexpectedCallHappenedFailure expectedFailure(mockFailureTest(), "func", *expectationsList);

	mock().actualCall("func");

	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, ignoreOtherCallsExceptForTheExpectedOne)
{
	mock().expectOneCall("foo");
	mock().ignoreOtherCalls();
	mock().actualCall("bar").withParameter("foo", 1);;

	CHECK_NO_MOCK_FAILURE();

	mock().clear();
}

TEST(MockSupportTest, ignoreOtherCallsDoesntIgnoreMultipleCallsOfTheSameFunction)
{
	mock().expectOneCall("foo");
	mock().ignoreOtherCalls();
	mock().actualCall("bar");
	mock().actualCall("foo");
	mock().actualCall("foo");

	addFunctionToExpectationsList("foo")->callWasMade(1);
	MockUnexpectedCallHappenedFailure expectedFailure(mockFailureTest(), "foo", *expectationsList);
	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, ignoreOtherStillFailsIfExpectedOneDidntHappen)
{
	mock().expectOneCall("foo");
	mock().ignoreOtherCalls();
	mock().checkExpectations();

	addFunctionToExpectationsList("foo");

	MockExpectedCallsDidntHappenFailure expectedFailure(mockFailureTest(), *expectationsList);
	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, expectMultipleCallsThatHappen)
{
	mock().expectOneCall("foo");
	mock().expectOneCall("foo");
	mock().actualCall("foo");
	mock().actualCall("foo");
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, strictOrderObserved)
{
	mock().strictOrder();
	mock().expectOneCall("foo1");
	mock().expectOneCall("foo2");
	mock().actualCall("foo1");
	mock().actualCall("foo2");
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, someStrictOrderObserved)
{
	mock().expectOneCall("foo3").withCallOrder(3);
	mock().expectOneCall("foo1");
	mock().expectOneCall("foo2");
	mock().actualCall("foo2");
	mock().actualCall("foo1");
	mock().actualCall("foo3");
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, strictOrderViolated)
{
	mock().strictOrder();
	addFunctionToExpectationsList("foo1", 1)->callWasMade(1);
	addFunctionToExpectationsList("foo1", 2)->callWasMade(3);
	addFunctionToExpectationsList("foo2", 3)->callWasMade(2);
	MockCallOrderFailure expectedFailure(mockFailureTest(), *expectationsList);
	mock().expectOneCall("foo1");
	mock().expectOneCall("foo1");
	mock().expectOneCall("foo2");
	mock().actualCall("foo1");
	mock().actualCall("foo2");
	mock().actualCall("foo1");
	mock().checkExpectations();
	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, strictOrderNotViolatedWithTwoMocks)
{
	mock("mock1").strictOrder();
	mock("mock2").strictOrder();
	mock("mock1").expectOneCall("foo1");
	mock("mock2").expectOneCall("foo2");

	mock("mock1").actualCall("foo1");
	mock("mock2").actualCall("foo2");

	mock("mock1").checkExpectations();
	mock("mock2").checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

IGNORE_TEST(MockSupportTest, strictOrderViolatedWithTwoMocks)
{
	//this test and scenario needs a decent failure message.
	mock("mock1").strictOrder();
	mock("mock2").strictOrder();
	mock("mock1").expectOneCall("foo1");
	mock("mock2").expectOneCall("foo2");

	mock("mock2").actualCall("foo2");
	mock("mock1").actualCall("foo1");

	mock("mock1").checkExpectations();
	mock("mock2").checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, usingNCalls)
{
	mock().strictOrder();
	mock().expectOneCall("foo1");
	mock().expectNCalls(2, "foo2");
	mock().expectOneCall("foo1");

	mock().actualCall("foo1");
	mock().actualCall("foo2");
	mock().actualCall("foo2");
	mock().actualCall("foo1");

	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, expectOneCallHoweverMultipleHappened)
{
	addFunctionToExpectationsList("foo")->callWasMade(1);
	addFunctionToExpectationsList("foo")->callWasMade(2);
	MockUnexpectedCallHappenedFailure expectedFailure(mockFailureTest(), "foo", *expectationsList);

	mock().expectOneCall("foo");
	mock().expectOneCall("foo");
	mock().actualCall("foo");
	mock().actualCall("foo");
	mock().actualCall("foo");

	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, expectOneIntegerParameterAndValue)
{
	mock().expectOneCall("foo").withParameter("parameter", 10);
	mock().actualCall("foo").withParameter("parameter", 10);
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, expectOneDoubleParameterAndValue)
{
	mock().expectOneCall("foo").withParameter("parameter", 1.0);
	mock().actualCall("foo").withParameter("parameter", 1.0);
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, expectOneStringParameterAndValue)
{
	mock().expectOneCall("foo").withParameter("parameter", "string");
	mock().actualCall("foo").withParameter("parameter", "string");
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, expectOnePointerParameterAndValue)
{
	mock().expectOneCall("foo").withParameter("parameter", (void*) 0x01);
	mock().actualCall("foo").withParameter("parameter", (void*) 0x01);
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, expectOneStringParameterAndValueFails)
{
	MockNamedValue parameter("parameter");
	parameter.setValue("different");
	addFunctionToExpectationsList("foo")->withParameter("parameter", "string");
	MockUnexpectedParameterFailure expectedFailure(mockFailureTest(), "foo", parameter, *expectationsList);

	mock().expectOneCall("foo").withParameter("parameter", "string");
	mock().actualCall("foo").withParameter("parameter", "different");

	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, expectOneIntegerParameterAndFailsDueToParameterName)
{
	MockNamedValue parameter("different");
	parameter.setValue(10);
	addFunctionToExpectationsList("foo")->withParameter("parameter", 10);
	MockUnexpectedParameterFailure expectedFailure(mockFailureTest(), "foo", parameter, *expectationsList);

	mock().expectOneCall("foo").withParameter("parameter", 10);
	mock().actualCall("foo").withParameter("different", 10);

	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, expectOneIntegerParameterAndFailsDueToValue)
{
	MockNamedValue parameter("parameter");
	parameter.setValue(8);
	addFunctionToExpectationsList("foo")->withParameter("parameter", 10);
	MockUnexpectedParameterFailure expectedFailure(mockFailureTest(), "foo", parameter, *expectationsList);

	mock().expectOneCall("foo").withParameter("parameter", 10);
	mock().actualCall("foo").withParameter("parameter", 8);

	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, expectOneIntegerParameterAndFailsDueToTypes)
{
	MockNamedValue parameter("parameter");
	parameter.setValue("heh");
	addFunctionToExpectationsList("foo")->withParameter("parameter", 10);
	MockUnexpectedParameterFailure expectedFailure(mockFailureTest(), "foo", parameter, *expectationsList);

	mock().expectOneCall("foo").withParameter("parameter", 10);
	mock().actualCall("foo").withParameter("parameter", "heh");

	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, expectMultipleCallsWithDifferentParametersThatHappenOutOfOrder)
{
	mock().expectOneCall("foo").withParameter("p1", 1);
	mock().expectOneCall("foo").withParameter("p1", 2);
	mock().actualCall("foo").withParameter("p1", 2);
	mock().actualCall("foo").withParameter("p1", 1);
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, expectMultipleCallsWithMultipleDifferentParametersThatHappenOutOfOrder)
{
	mock().expectOneCall("foo").withParameter("p1", 1).withParameter("p2", 2);
	mock().expectOneCall("foo").withParameter("p1", 1).withParameter("p2", 20);

	mock().actualCall("foo").withParameter("p1", 1).withParameter("p2", 20);
	mock().actualCall("foo").withParameter("p1", 1).withParameter("p2", 2);
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, twiceCalledWithSameParameters)
{
	mock().expectOneCall("foo").withParameter("p1", 1).withParameter("p2", 2);
	mock().expectOneCall("foo").withParameter("p1", 1).withParameter("p2", 2);
	mock().actualCall("foo").withParameter("p1", 1).withParameter("p2", 2);
	mock().actualCall("foo").withParameter("p1", 1).withParameter("p2", 2);
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, calledWithoutParameters)
{
	addFunctionToExpectationsList("foo")->withParameter("p1", 1);
	MockExpectedParameterDidntHappenFailure expectedFailure(mockFailureTest(), "foo", *expectationsList);

	mock().expectOneCall("foo").withParameter("p1", 1);
	mock().actualCall("foo");
	mock().checkExpectations();

	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);

}

TEST(MockSupportTest, ignoreOtherParameters)
{
	mock().expectOneCall("foo").withParameter("p1", 1).ignoreOtherParameters();
	mock().actualCall("foo").withParameter("p1", 1).withParameter("p2", 2);
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, ignoreOtherParametersButStillPassAll)
{
	mock().expectOneCall("foo").withParameter("p1", 1).ignoreOtherParameters();
	mock().actualCall("foo").withParameter("p1", 1);
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, ignoreOtherParametersButExpectedParameterDidntHappen)
{
	addFunctionToExpectationsList("foo")->withParameter("p1", 1).ignoreOtherParameters();
	MockExpectedParameterDidntHappenFailure expectedFailure(mockFailureTest(), "foo", *expectationsList);

	mock().expectOneCall("foo").withParameter("p1", 1).ignoreOtherParameters();
	mock().actualCall("foo").withParameter("p2", 2).withParameter("p3", 3).withParameter("p4", 4);
	mock().checkExpectations();
	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, ignoreOtherParametersMultipleCalls)
{
	mock().expectOneCall("foo").ignoreOtherParameters();
	mock().expectOneCall("foo").ignoreOtherParameters();
	mock().actualCall("foo").withParameter("p2", 2).withParameter("p3", 3).withParameter("p4", 4);
	LONGS_EQUAL(1, mock().expectedCallsLeft());
	mock().actualCall("foo").withParameter("p2", 2).withParameter("p3", 3).withParameter("p4", 4);

	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, ignoreOtherParametersMultipleCallsButOneDidntHappen)
{
	MockExpectedFunctionCall* call = addFunctionToExpectationsList("boo");
	call->ignoreOtherParameters();
	call->callWasMade(1);
	call->parametersWereIgnored();
	call->ignoreOtherParameters();
	addFunctionToExpectationsList("boo")->ignoreOtherParameters();
	mock().expectOneCall("boo").ignoreOtherParameters();
	mock().expectOneCall("boo").ignoreOtherParameters();
	mock().actualCall("boo");
	mock().checkExpectations();
	MockExpectedCallsDidntHappenFailure expectedFailure(mockFailureTest(), *expectationsList);
	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}


TEST(MockSupportTest, newCallStartsWhileNotAllParametersWerePassed)
{
	addFunctionToExpectationsList("foo")->withParameter("p1", 1);
	MockExpectedParameterDidntHappenFailure expectedFailure(mockFailureTest(), "foo", *expectationsList);

	mock().expectOneCall("foo").withParameter("p1", 1);
	mock().actualCall("foo");
	mock().actualCall("foo").withParameter("p1", 1);;

	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, threeExpectedAndActual)
{
	mock().expectOneCall("function1");
	mock().expectOneCall("function2");
	mock().expectOneCall("function3");
	mock().actualCall("function1");
	mock().actualCall("function2");
	mock().actualCall("function3");
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

class MyTypeForTesting
{
public:
	MyTypeForTesting(int val) : value(val){};
	int value;
};

class MyTypeForTestingComparator : public MockNamedValueComparator
{
public:
	virtual bool isEqual(void* object1, void* object2)
	{
		return ((MyTypeForTesting*)object1)->value == ((MyTypeForTesting*)object2)->value;
	}
	virtual SimpleString valueToString(void* object)
	{
		return StringFrom(((MyTypeForTesting*)object)->value);
	}
};


TEST(MockSupportTest, customObjectParameterFailsWhenNotHavingAComparisonRepository)
{
	MyTypeForTesting object(1);
	mock().expectOneCall("function").withParameterOfType("MyTypeForTesting", "parameterName", &object);
	mock().actualCall("function").withParameterOfType("MyTypeForTesting", "parameterName", &object);

	MockNoWayToCompareCustomTypeFailure expectedFailure(mockFailureTest(), "MyTypeForTesting");
	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, customObjectParameterSucceeds)
{
	MyTypeForTesting object(1);
	MyTypeForTestingComparator comparator;
	mock().installComparator("MyTypeForTesting", comparator);
	mock().expectOneCall("function").withParameterOfType("MyTypeForTesting", "parameterName", &object);
	mock().actualCall("function").withParameterOfType("MyTypeForTesting", "parameterName", &object);
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
	mock().removeAllComparators();
}

bool myTypeIsEqual(void* object1, void* object2)
{
	return ((MyTypeForTesting*)object1)->value == ((MyTypeForTesting*)object2)->value;
}

SimpleString myTypeValueToString(void* object)
{
	return StringFrom(((MyTypeForTesting*)object)->value);
}

TEST(MockSupportTest, customObjectWithFunctionComparator)
{
	MyTypeForTesting object(1);
	MockFunctionComparator comparator(myTypeIsEqual, myTypeValueToString);
	mock().installComparator("MyTypeForTesting", comparator);
	mock().expectOneCall("function").withParameterOfType("MyTypeForTesting", "parameterName", &object);
	mock().actualCall("function").withParameterOfType("MyTypeForTesting", "parameterName", &object);
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
	mock().removeAllComparators();
}

TEST(MockSupportTest, disableEnable)
{
	mock().disable();
	mock().expectOneCall("function");
	mock().actualCall("differenFunction");
	CHECK(! mock().expectedCallsLeft());
	mock().enable();
	mock().expectOneCall("function");
	CHECK(mock().expectedCallsLeft());
	mock().actualCall("function");
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, setDataForIntegerValues)
{
	mock().setData("data", 10);
	LONGS_EQUAL(10, mock().getData("data").getIntValue());
}

TEST(MockSupportTest, hasDataBeenSet)
{
	CHECK(!mock().hasData("data"));
	mock().setData("data", 10);
	CHECK(mock().hasData("data"));
}

TEST(MockSupportTest, dataCanBeChanged)
{
	mock().setData("data", 10);
	mock().setData("data", 15);
	LONGS_EQUAL(15, mock().getData("data").getIntValue());

}

TEST(MockSupportTest, uninitializedData)
{
	LONGS_EQUAL(0, mock().getData("nonexisting").getIntValue());
	STRCMP_EQUAL("int", mock().getData("nonexisting").getType().asCharString());
}

TEST(MockSupportTest, setMultipleData)
{
	mock().setData("data", 1);
	mock().setData("data2", 10);
	LONGS_EQUAL(1, mock().getData("data").getIntValue());
	LONGS_EQUAL(10, mock().getData("data2").getIntValue());
}

TEST(MockSupportTest, setDataString)
{
	mock().setData("data", "string");
	STRCMP_EQUAL("string", mock().getData("data").getStringValue());
}

TEST(MockSupportTest, setDataDouble)
{
	mock().setData("data", 1.0);
	DOUBLES_EQUAL(1.0, mock().getData("data").getDoubleValue(), 0.05);
}

TEST(MockSupportTest, setDataPointer)
{
	void * ptr = (void*) 0x001;
	mock().setData("data", ptr);
	POINTERS_EQUAL(ptr, mock().getData("data").getPointerValue());
}

TEST(MockSupportTest, setDataObject)
{
	void * ptr = (void*) 0x001;
	mock().setDataObject("data", "type", ptr);
	POINTERS_EQUAL(ptr, mock().getData("data").getObjectPointer());
	STRCMP_EQUAL("type", mock().getData("data").getType().asCharString());
}

TEST(MockSupportTest, getMockSupportScope)
{
	MockSupport* mock1 = mock().getMockSupportScope("name");
	MockSupport* mock2 = mock().getMockSupportScope("differentName");
	CHECK(!mock().hasData("name"));
	CHECK(mock1 != mock2);
	POINTERS_EQUAL(mock1, mock().getMockSupportScope("name"));
	CHECK(mock1 != &mock());
}

TEST(MockSupportTest, usingTwoMockSupportsByName)
{
	mock("first").expectOneCall("boo");
	LONGS_EQUAL(0, mock("other").expectedCallsLeft());
	LONGS_EQUAL(1, mock("first").expectedCallsLeft());
	mock("first").clear();
}

TEST(MockSupportTest, EnableDisableWorkHierarchically)
{
	mock("first");

	mock().disable();
	mock("first").expectOneCall("boo");
	LONGS_EQUAL(0, mock("first").expectedCallsLeft());

	mock().enable();
	mock("first").expectOneCall("boo");
	LONGS_EQUAL(1, mock("first").expectedCallsLeft());

	mock("first").clear();
}

TEST(MockSupportTest, EnableDisableWorkHierarchicallyWhenSupportIsDynamicallyCreated)
{
	mock().disable();
	mock("first").expectOneCall("boo");
	LONGS_EQUAL(0, mock("first").expectedCallsLeft());

	mock().enable();
	mock("second").expectOneCall("boo");
	LONGS_EQUAL(1, mock("second").expectedCallsLeft());

	mock().clear();
}

TEST(MockSupportTest, ExpectedCallsLeftWorksHierarchically)
{
	mock("first").expectOneCall("foobar");
	LONGS_EQUAL(1, mock().expectedCallsLeft());
	mock().clear();
}

TEST(MockSupportTest, checkExpectationsWorksHierarchically)
{
	mock("first").expectOneCall("foobar");
	mock("second").expectOneCall("helloworld");

	addFunctionToExpectationsList("foobar");
	addFunctionToExpectationsList("helloworld");
	MockExpectedCallsDidntHappenFailure expectedFailure(mockFailureTest(), *expectationsList);

	mock().checkExpectations();
	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, ignoreOtherCallsWorksHierarchically)
{
	mock("first");
	mock().ignoreOtherCalls();
	mock("first").actualCall("boo");
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, ignoreOtherCallsWorksHierarchicallyWhenDynamicallyCreated)
{
	mock().ignoreOtherCalls();
	mock("first").actualCall("boo");
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, checkExpectationsWorksHierarchicallyForLastCallNotFinished)
{
	mock("first").expectOneCall("foobar").withParameter("boo", 1);
	mock("first").actualCall("foobar");

	addFunctionToExpectationsList("foobar")->withParameter("boo", 1);
	MockExpectedParameterDidntHappenFailure expectedFailure(mockFailureTest(), "foobar", *expectationsList);

	mock().checkExpectations();
	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, reporterIsInheritedInHierarchicalMocks)
{
	mock("differentScope").actualCall("foobar");

	MockUnexpectedCallHappenedFailure expectedFailure(mockFailureTest(), "foobar", *expectationsList);
	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, installComparatorWorksHierarchicalOnBothExistingAndDynamicallyCreatedMockSupports)
{
	MyTypeForTesting object(1);
	MyTypeForTestingComparator comparator;

	mock("existing");
	mock().installComparator("MyTypeForTesting", comparator);
	mock("existing").expectOneCall("function").withParameterOfType("MyTypeForTesting", "parameterName", &object);
	mock("existing").actualCall("function").withParameterOfType("MyTypeForTesting", "parameterName", &object);
	mock("dynamic").expectOneCall("function").withParameterOfType("MyTypeForTesting", "parameterName", &object);
	mock("dynamic").actualCall("function").withParameterOfType("MyTypeForTesting", "parameterName", &object);

	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
	mock().removeAllComparators();
}

TEST(MockSupportTest, installComparatorsWorksHierarchical)
{
	MyTypeForTesting object(1);
	MyTypeForTestingComparator comparator;
	MockNamedValueComparatorRepository repos;
	repos.installComparator("MyTypeForTesting", comparator);

	mock("existing");
	mock().installComparators(repos);
	mock("existing").expectOneCall("function").withParameterOfType("MyTypeForTesting", "parameterName", &object);
	mock("existing").actualCall("function").withParameterOfType("MyTypeForTesting", "parameterName", &object);

	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
	mock().removeAllComparators();
}

TEST(MockSupportTest, removeComparatorsWorksHierachically)
{
	MyTypeForTesting object(1);
	MyTypeForTestingComparator comparator;

	mock("scope").installComparator("MyTypeForTesting", comparator);
	mock().removeAllComparators();
	mock("scope").expectOneCall("function").withParameterOfType("MyTypeForTesting", "parameterName", &object);
	mock("scope").actualCall("function").withParameterOfType("MyTypeForTesting", "parameterName", &object);

	MockNoWayToCompareCustomTypeFailure expectedFailure(mockFailureTest(), "MyTypeForTesting");
	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, hasReturnValue)
{
	CHECK(!mock().hasReturnValue());
	mock().expectOneCall("foo");
	CHECK(!mock().actualCall("foo").hasReturnValue());
	CHECK(!mock().hasReturnValue());

	mock().expectOneCall("foo2").andReturnValue(1);
	CHECK(mock().actualCall("foo2").hasReturnValue());
	CHECK(mock().hasReturnValue());
}

TEST(MockSupportTest, IntegerReturnValue)
{
	mock().expectOneCall("foo").andReturnValue(1);
	LONGS_EQUAL(1, mock().actualCall("foo").returnValue().getIntValue());
	LONGS_EQUAL(1, mock().returnValue().getIntValue());
	LONGS_EQUAL(1, mock().intReturnValue());
}

TEST(MockSupportTest, IntegerReturnValueSetsDifferentValues)
{
	mock().expectOneCall("foo").andReturnValue(1);
	mock().expectOneCall("foo").andReturnValue(2);

	LONGS_EQUAL(1, mock().actualCall("foo").returnValue().getIntValue());
	LONGS_EQUAL(1, mock().returnValue().getIntValue());
	LONGS_EQUAL(2, mock().actualCall("foo").returnValue().getIntValue());
	LONGS_EQUAL(2, mock().returnValue().getIntValue());

}

TEST(MockSupportTest, IntegerReturnValueSetsDifferentValuesWhileParametersAreIgnored)
{
	mock().expectOneCall("foo").withParameter("p1", 1).ignoreOtherParameters().andReturnValue(1);
	mock().expectOneCall("foo").withParameter("p1", 1).ignoreOtherParameters().andReturnValue(2);

	LONGS_EQUAL(1, mock().actualCall("foo").withParameter("p1", 1).returnValue().getIntValue());
	LONGS_EQUAL(1, mock().returnValue().getIntValue());
	LONGS_EQUAL(2, mock().actualCall("foo").withParameter("p1", 1).returnValue().getIntValue());
	LONGS_EQUAL(2, mock().returnValue().getIntValue());
}

TEST(MockSupportTest, MatchingReturnValueOnWhileSignature)
{
	mock().expectOneCall("foo").withParameter("p1", 1).andReturnValue(1);
	mock().expectOneCall("foo").withParameter("p1", 2).andReturnValue(2);
	mock().expectOneCall("foo").withParameter("p1", 3).andReturnValue(3);
	mock().expectOneCall("foo").ignoreOtherParameters().andReturnValue(4);

	LONGS_EQUAL(3, mock().actualCall("foo").withParameter("p1", 3).returnValue().getIntValue());
	LONGS_EQUAL(4, mock().actualCall("foo").withParameter("p1", 4).returnValue().getIntValue());
	LONGS_EQUAL(1, mock().actualCall("foo").withParameter("p1", 1).returnValue().getIntValue());
	LONGS_EQUAL(2, mock().actualCall("foo").withParameter("p1", 2).returnValue().getIntValue());
}

TEST(MockSupportTest, StringReturnValue)
{
	mock().expectOneCall("foo").andReturnValue("hello world");
	STRCMP_EQUAL("hello world", mock().actualCall("foo").returnValue().getStringValue());
	STRCMP_EQUAL("hello world", mock().stringReturnValue());
}

TEST(MockSupportTest, DoubleReturnValue)
{
	mock().expectOneCall("foo").andReturnValue(1.0);
	DOUBLES_EQUAL(1.0, mock().actualCall("foo").returnValue().getDoubleValue(), 0.05);
	DOUBLES_EQUAL(1.0, mock().doubleReturnValue(), 0.05);
}

TEST(MockSupportTest, PointerReturnValue)
{
	void* ptr = (void*) 0x001;
	mock().expectOneCall("foo").andReturnValue(ptr);
	POINTERS_EQUAL(ptr, mock().actualCall("foo").returnValue().getPointerValue());
	POINTERS_EQUAL(ptr, mock().pointerReturnValue());
}

TEST(MockSupportTest, OnObject)
{
	void* objectPtr = (void*) 0x001;
	mock().expectOneCall("boo").onObject(objectPtr);
	mock().actualCall("boo").onObject(objectPtr);
}

TEST(MockSupportTest, OnObjectFails)
{
	void* objectPtr = (void*) 0x001;
	void* objectPtr2 = (void*) 0x002;
	addFunctionToExpectationsList("boo")->onObject(objectPtr);

	mock().expectOneCall("boo").onObject(objectPtr);
	mock().actualCall("boo").onObject(objectPtr2);

	MockUnexpectedObjectFailure expectedFailure(mockFailureTest(), "boo", objectPtr2, *expectationsList);
	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, OnObjectExpectedButNotCalled)
{
	void* objectPtr = (void*) 0x001;
	addFunctionToExpectationsList("boo")->onObject(objectPtr);
	addFunctionToExpectationsList("boo")->onObject(objectPtr);

	mock().expectOneCall("boo").onObject(objectPtr);
	mock().expectOneCall("boo").onObject(objectPtr);
	mock().actualCall("boo");
	mock().actualCall("boo");

	MockExpectedObjectDidntHappenFailure expectedFailure(mockFailureTest(), "boo", *expectationsList);
	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
	mock().checkExpectations();
	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockSupportTest, expectMultipleCalls)
{
	mock().expectNCalls(2, "boo");
	mock().actualCall("boo");
	mock().actualCall("boo");
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, expectMultipleCallsWithParameters)
{
	mock().expectNCalls(2, "boo").withParameter("double", 1.0).withParameter("int", 1).withParameter("string", "string");
	mock().actualCall("boo").withParameter("double", 1.0).withParameter("int", 1).withParameter("string", "string");
	mock().actualCall("boo").withParameter("double", 1.0).withParameter("int", 1).withParameter("string", "string");
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, expectMultipleMultipleCallsWithParameters)
{
	mock().expectNCalls(2, "boo").withParameter("double", 1.0).ignoreOtherParameters();
	mock().expectNCalls(2, "boo").withParameter("double", 1.0).ignoreOtherParameters();
	mock().actualCall("boo").withParameter("double", 1.0).withParameter("int", 1).withParameter("string", "string");
	mock().actualCall("boo").withParameter("double", 1.0).withParameter("int", 1).withParameter("string", "string");
	mock().actualCall("boo").withParameter("double", 1.0).withParameter("int", 1).withParameter("string", "string");
	mock().actualCall("boo").withParameter("double", 1.0).withParameter("int", 1).withParameter("string", "string");
	mock().checkExpectations();
	CHECK_NO_MOCK_FAILURE();
}

TEST(MockSupportTest, whenCallingDisabledOrIgnoredActualCallsThenTheyDontReturnPreviousCallsValues)
{
	mock().expectOneCall("boo").ignoreOtherParameters().andReturnValue(10);
	mock().ignoreOtherCalls();
	mock().actualCall("boo");
	mock().actualCall("An Ignored Call");
	CHECK(!mock().hasReturnValue());
}

TEST(MockSupportTest, tracing)
{
	mock().tracing(true);

	mock().actualCall("boo").withParameter("double", 1.0).withParameter("int", 1).withParameter("string", "string");
	mock("scope").actualCall("foo").withParameter("double", 1.0).withParameter("int", 1).withParameter("string", "string");
	mock().checkExpectations();

	STRCMP_CONTAINS("boo", mock().getTraceOutput());
	STRCMP_CONTAINS("foo", mock().getTraceOutput());
}

TEST(MockSupportTest, shouldntFailTwice)
{
	mock().expectOneCall("foo");
	mock().actualCall("bar");
	mock().checkExpectations();
	LONGS_EQUAL(1, MockFailureReporterForTest::getReporter()->getAmountOfTestFailures());
	CLEAR_MOCK_FAILURE();
}


IGNORE_TEST(MockSupportTest, testForPerformanceProfiling)
{
	/* TO fix! */
	mock().expectNCalls(1000, "SimpleFunction");
	for (int i = 0; i < 1000; i++) {
		mock().actualCall("SimpleFunction");
	}

}
