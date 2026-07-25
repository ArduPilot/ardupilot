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
#include "CppUTestExt/MockExpectedFunctionCall.h"
#include "CppUTestExt/MockFailure.h"
#include "TestMockFailure.h"

class TypeForTestingExpectedFunctionCall
{
public:
	TypeForTestingExpectedFunctionCall(int val) : value(val) {};
	int value;
};


class TypeForTestingExpectedFunctionCallComparator : public MockNamedValueComparator
{
public:
	TypeForTestingExpectedFunctionCallComparator() {}
	virtual ~TypeForTestingExpectedFunctionCallComparator() {};

	virtual bool isEqual(void* object1, void* object2)
	{
		return ((TypeForTestingExpectedFunctionCall*)object1)->value == ((TypeForTestingExpectedFunctionCall*)object2)->value;
	}
	virtual SimpleString valueToString(void* object)
	{
		return StringFrom(((TypeForTestingExpectedFunctionCall*)object)->value);
	}

};

TEST_GROUP(MockNamedValueComparatorRepository)
{
	void teardown()
	{
		CHECK_NO_MOCK_FAILURE();
	}
};

TEST(MockNamedValueComparatorRepository, getComparatorForNonExistingName)
{
	MockNamedValueComparatorRepository repository;
	POINTERS_EQUAL(NULL, repository.getComparatorForType("typeName"));
}

TEST(MockNamedValueComparatorRepository, installComparator)
{
	TypeForTestingExpectedFunctionCallComparator comparator;
	MockNamedValueComparatorRepository repository;
	repository.installComparator("typeName", comparator);
	POINTERS_EQUAL(&comparator, repository.getComparatorForType("typeName"));
}

TEST(MockNamedValueComparatorRepository, installMultipleComparator)
{
	TypeForTestingExpectedFunctionCallComparator comparator1, comparator2, comparator3;
	MockNamedValueComparatorRepository repository;
	repository.installComparator("type1", comparator1);
	repository.installComparator("type2", comparator2);
	repository.installComparator("type3", comparator3);
	POINTERS_EQUAL(&comparator3, repository.getComparatorForType("type3"));
	POINTERS_EQUAL(&comparator2, repository.getComparatorForType("type2"));
	POINTERS_EQUAL(&comparator1, repository.getComparatorForType("type1"));
}

TEST_GROUP(MockExpectedFunctionCall)
{
	MockExpectedFunctionCall* call;
	void setup ()
	{
		call = new MockExpectedFunctionCall;
	}
	void teardown()
	{
		delete call;
		CHECK_NO_MOCK_FAILURE();
	}
};

TEST(MockExpectedFunctionCall, callWithoutParameterSetOrNotFound)
{
	STRCMP_EQUAL("", call->getParameterType("nonexisting").asCharString());
	LONGS_EQUAL(0, call->getParameter("nonexisting").getIntValue());
	CHECK(!call->hasParameterWithName("nonexisting"));
}

TEST(MockExpectedFunctionCall, callWithIntegerParameter)
{
	call->withParameter("integer", 1);
	STRCMP_EQUAL("int", call->getParameterType("integer").asCharString());
	LONGS_EQUAL(1, call->getParameter("integer").getIntValue());
	CHECK(call->hasParameterWithName("integer"));
}

TEST(MockExpectedFunctionCall, callWithDoubleParameter)
{
	call->withParameter("double", 1.2);
	STRCMP_EQUAL("double", call->getParameterType("double").asCharString());
	DOUBLES_EQUAL(1.2, call->getParameter("double").getDoubleValue(), 0.05);
}

TEST(MockExpectedFunctionCall, callWithStringParameter)
{
	call->withParameter("string", "hello world");
	STRCMP_EQUAL("char*", call->getParameterType("string").asCharString());
	STRCMP_EQUAL("hello world", call->getParameter("string").getStringValue());
}

TEST(MockExpectedFunctionCall, callWithPointerParameter)
{
	void* ptr = (void*) 0x123;
	call->withParameter("pointer", ptr);
	STRCMP_EQUAL("void*", call->getParameterType("pointer").asCharString());
	POINTERS_EQUAL(ptr, call->getParameter("pointer").getPointerValue());
}

TEST(MockExpectedFunctionCall, callWithObjectParameter)
{
	void* ptr = (void*) 0x123;
	call->withParameterOfType("class", "object", ptr);
	POINTERS_EQUAL(ptr, call->getParameter("object").getObjectPointer());
	STRCMP_EQUAL("class", call->getParameterType("object").asCharString());
}

TEST(MockExpectedFunctionCall, callWithObjectParameterUnequalComparison)
{
	TypeForTestingExpectedFunctionCall type(1), unequalType(2);
	MockNamedValue parameter ("name");
	parameter.setObjectPointer("type", &unequalType);
	call->withParameterOfType("type", "name", &type);
	CHECK (!call->hasParameter(parameter));
}

TEST(MockExpectedFunctionCall, callWithObjectParameterEqualComparisonButFailsWithoutRepository)
{
	TypeForTestingExpectedFunctionCall type(1), equalType(1);
	MockNamedValue parameter ("name");
	parameter.setObjectPointer("type", &equalType);
	call->withParameterOfType("type", "name", &type);
	CHECK (!call->hasParameter(parameter));
}

TEST(MockExpectedFunctionCall, callWithObjectParameterEqualComparisonButFailsWithoutComparator)
{
	MockNamedValueComparatorRepository repository;
	call->setComparatorRepository(&repository);

	TypeForTestingExpectedFunctionCall type(1), equalType(1);
	MockNamedValue parameter ("name");
	parameter.setObjectPointer("type", &equalType);
	call->withParameterOfType("type", "name", &type);
	CHECK (!call->hasParameter(parameter));
}

TEST(MockExpectedFunctionCall, callWithObjectParameterEqualComparison)
{
	TypeForTestingExpectedFunctionCallComparator comparator;
	MockNamedValueComparatorRepository repository;
	repository.installComparator("type", comparator);

	TypeForTestingExpectedFunctionCall type(1), equalType(1);
	MockNamedValue parameter ("name");
	parameter.setObjectPointer("type", &equalType);

	call->setComparatorRepository(&repository);
	call->withParameterOfType("type", "name", &type);
	CHECK (call->hasParameter(parameter));
}

TEST(MockExpectedFunctionCall, getParameterValueOfObjectType)
{
	TypeForTestingExpectedFunctionCallComparator comparator;
	MockNamedValueComparatorRepository repository;
	repository.installComparator("type", comparator);

	TypeForTestingExpectedFunctionCall type(1);
	call->setComparatorRepository(&repository);
	call->withParameterOfType("type", "name", &type);
	POINTERS_EQUAL(&type, call->getParameter("name").getObjectPointer());
	STRCMP_EQUAL("1", call->getParameterValueString("name").asCharString());
}

TEST(MockExpectedFunctionCall, getParameterValueOfObjectTypeWithoutRepository)
{
	TypeForTestingExpectedFunctionCall type(1);
	call->withParameterOfType("type", "name", &type);
	STRCMP_EQUAL("No comparator found for type: \"type\"", call->getParameterValueString("name").asCharString());
}

TEST(MockExpectedFunctionCall, getParameterValueOfObjectTypeWithoutComparator)
{
	TypeForTestingExpectedFunctionCall type(1);
	MockNamedValueComparatorRepository repository;
	call->setComparatorRepository(&repository);
	call->withParameterOfType("type", "name", &type);
	STRCMP_EQUAL("No comparator found for type: \"type\"", call->getParameterValueString("name").asCharString());
}


TEST(MockExpectedFunctionCall, callWithTwoIntegerParameter)
{
	call->withParameter("integer1", 1);
	call->withParameter("integer2", 2);
	STRCMP_EQUAL("int", call->getParameterType("integer1").asCharString());
	STRCMP_EQUAL("int", call->getParameterType("integer2").asCharString());
	LONGS_EQUAL(1, call->getParameter("integer1").getIntValue());
	LONGS_EQUAL(2, call->getParameter("integer2").getIntValue());
}

TEST(MockExpectedFunctionCall, callWithThreeDifferentParameter)
{
	call->withParameter("integer", 1);
	call->withParameter("string", "hello world");
	call->withParameter("double", 0.12);
	STRCMP_EQUAL("int", call->getParameterType("integer").asCharString());
	STRCMP_EQUAL("char*", call->getParameterType("string").asCharString());
	STRCMP_EQUAL("double", call->getParameterType("double").asCharString());
	LONGS_EQUAL(1, call->getParameter("integer").getIntValue());
	STRCMP_EQUAL("hello world", call->getParameter("string").getStringValue());
	DOUBLES_EQUAL(0.12, call->getParameter("double").getDoubleValue(), 0.05);
}

TEST(MockExpectedFunctionCall, withoutANameItsFulfilled)
{
	CHECK(call->isFulfilled());
}

TEST(MockExpectedFunctionCall, withANameItsNotFulfilled)
{
	call->withName("name");
	CHECK(!call->isFulfilled());
}

TEST(MockExpectedFunctionCall, afterSettingCallFulfilledItsFulFilled)
{
	call->withName("name");
	call->callWasMade(1);
	CHECK(call->isFulfilled());
}

TEST(MockExpectedFunctionCall, calledButNotWithParameterIsNotFulFilled)
{
	call->withName("name").withParameter("para", 1);
	call->callWasMade(1);
	CHECK(!call->isFulfilled());
}

TEST(MockExpectedFunctionCall, calledAndParametersAreFulfilled)
{
	call->withName("name").withParameter("para", 1);
	call->callWasMade(1);
	call->parameterWasPassed("para");
	CHECK(call->isFulfilled());
}

TEST(MockExpectedFunctionCall, calledButNotAllParametersAreFulfilled)
{
	call->withName("name").withParameter("para", 1).withParameter("two", 2);
	call->callWasMade(1);
	call->parameterWasPassed("para");
	CHECK(! call->isFulfilled());
}

TEST(MockExpectedFunctionCall, toStringForNoParameters)
{
	call->withName("name");
	STRCMP_EQUAL("name -> no parameters", call->callToString().asCharString());
}

TEST(MockExpectedFunctionCall, toStringForIgnoredParameters)
{
	call->withName("name");
	call->ignoreOtherParameters();
	STRCMP_EQUAL("name -> all parameters ignored", call->callToString().asCharString());
}

TEST(MockExpectedFunctionCall, toStringForMultipleParameters)
{
	call->withName("name");
	call->withParameter("string", "value");
	call->withParameter("integer", 10);
	STRCMP_EQUAL("name -> char* string: <value>, int integer: <10>", call->callToString().asCharString());
}

TEST(MockExpectedFunctionCall, toStringForParameterAndIgnored)
{
	call->withName("name");
	call->withParameter("string", "value");
	call->ignoreOtherParameters();
	STRCMP_EQUAL("name -> char* string: <value>, other parameters are ignored", call->callToString().asCharString());
}

TEST(MockExpectedFunctionCall, toStringForCallOrder)
{
	call->withName("name");
	call->withCallOrder(2);
	STRCMP_EQUAL("name -> expected call order: <2> -> no parameters", call->callToString().asCharString());
}

TEST(MockExpectedFunctionCall, callOrderIsNotFulfilledWithWrongOrder)
{
	call->withName("name");
	call->withCallOrder(2);
	call->callWasMade(1);
	CHECK(call->isFulfilled());
	CHECK(call->isOutOfOrder());
}

TEST(MockExpectedFunctionCall, callOrderIsFulfilled)
{
	call->withName("name");
	call->withCallOrder(1);
	call->callWasMade(1);
	CHECK(call->isFulfilled());
	CHECK_FALSE(call->isOutOfOrder());
}

