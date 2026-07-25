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
#include "CppUTestExt/MockExpectedFunctionsList.h"
#include "CppUTestExt/MockExpectedFunctionCall.h"
#include "CppUTestExt/MockFailure.h"
#include "TestMockFailure.h"

TEST_GROUP(MockExpectedFunctionsList)
{
	MockExpectedFunctionsList * list;
	MockExpectedFunctionCall* call1;
	MockExpectedFunctionCall* call2;
	MockExpectedFunctionCall* call3;
	MockExpectedFunctionCall* call4;
	void setup()
	{
		list = new MockExpectedFunctionsList;
		call1 = new MockExpectedFunctionCall;
		call2 = new MockExpectedFunctionCall;
		call3 = new MockExpectedFunctionCall;
		call4 = new MockExpectedFunctionCall;
		call1->withName("foo");
		call2->withName("bar");
		call3->withName("boo");
	}
	void teardown()
	{
		delete call1;
		delete call2;
		delete call3;
		delete call4;
		delete list;
		CHECK_NO_MOCK_FAILURE();
	}
};

TEST(MockExpectedFunctionsList, emptyList)
{
	CHECK(! list->hasUnfullfilledExpectations());
	CHECK(! list->hasFulfilledExpectations());
	LONGS_EQUAL(0, list->size());
}

TEST(MockExpectedFunctionsList, addingCalls)
{
	list->addExpectedCall(call1);
	list->addExpectedCall(call2);
	LONGS_EQUAL(2, list->size());
}

TEST(MockExpectedFunctionsList, listWithFulfilledExpectationHasNoUnfillfilledOnes)
{
	call1->callWasMade(1);
	call2->callWasMade(2);
	list->addExpectedCall(call1);
	list->addExpectedCall(call2);
	CHECK(! list->hasUnfullfilledExpectations());
}

TEST(MockExpectedFunctionsList, listWithFulfilledExpectationButOutOfOrder)
{
	call1->withCallOrder(1);
	call2->withCallOrder(2);
	list->addExpectedCall(call1);
	list->addExpectedCall(call2);
	call2->callWasMade(1);
	call1->callWasMade(2);
	CHECK(! list->hasUnfullfilledExpectations());
	CHECK(list->hasCallsOutOfOrder());
}

TEST(MockExpectedFunctionsList, listWithUnFulfilledExpectationHasNoUnfillfilledOnes)
{
	call1->callWasMade(1);
	call3->callWasMade(2);
	list->addExpectedCall(call1);
	list->addExpectedCall(call2);
	list->addExpectedCall(call3);
	CHECK(list->hasUnfullfilledExpectations());
}

TEST(MockExpectedFunctionsList, deleteAllExpectationsAndClearList)
{
	list->addExpectedCall(new MockExpectedFunctionCall);
	list->addExpectedCall(new MockExpectedFunctionCall);
	list->deleteAllExpectationsAndClearList();
}

TEST(MockExpectedFunctionsList, onlyKeepUnfulfilledExpectationsRelatedTo)
{
	call1->withName("relate");
	call2->withName("unrelate");
	call3->withName("relate");
	call3->callWasMade(1);
	list->addExpectedCall(call1);
	list->addExpectedCall(call2);
	list->addExpectedCall(call3);
	list->onlyKeepUnfulfilledExpectationsRelatedTo("relate");
	LONGS_EQUAL(1, list->size());
}

TEST(MockExpectedFunctionsList, removeAllExpectationsExceptThisThatRelateToTheWoleList)
{
	call1->withName("relate");
	call2->withName("relate");
	call3->withName("relate");
	list->addExpectedCall(call1);
	list->addExpectedCall(call2);
	list->addExpectedCall(call3);
	list->onlyKeepUnfulfilledExpectationsRelatedTo("unrelate");
	LONGS_EQUAL(0, list->size());
}

TEST(MockExpectedFunctionsList, removeAllExpectationsExceptThisThatRelateToFirstOne)
{
	call1->withName("relate");
	call2->withName("unrelate");
	list->addExpectedCall(call1);
	list->addExpectedCall(call2);
	list->onlyKeepUnfulfilledExpectationsRelatedTo("unrelate");
	LONGS_EQUAL(1, list->size());
}

TEST(MockExpectedFunctionsList, removeAllExpectationsExceptThisThatRelateToLastOne)
{
	call1->withName("unrelate");
	call2->withName("relate");
	list->addExpectedCall(call1);
	list->addExpectedCall(call2);
	list->onlyKeepUnfulfilledExpectationsRelatedTo("unrelate");
	LONGS_EQUAL(1, list->size());
}

TEST(MockExpectedFunctionsList, onlyKeepExpectationsWithParameterName)
{
	call1->withName("func").withParameter("param", 1);
	call2->withName("func").withParameter("diffname", 1);
	call3->withName("func").withParameter("diffname", 1);
	list->addExpectedCall(call1);
	list->addExpectedCall(call2);
	list->addExpectedCall(call3);
	list->onlyKeepExpectationsWithParameterName("diffname");
	LONGS_EQUAL(2, list->size());
}

TEST(MockExpectedFunctionsList, onlyKeepUnfulfilledExpectationsWithParameter)
{
	MockNamedValue parameter("diffname");
	parameter.setValue(1);
	call1->withName("func").withParameter("param", 1);
	call2->withName("func").withParameter("diffname", 1);
	call3->withName("func").withParameter("diffname", 1);
	call4->withName("func").withParameter("diffname", 2);
	call3->callWasMade(1);
	call3->parameterWasPassed("diffname");
	list->addExpectedCall(call1);
	list->addExpectedCall(call2);
	list->addExpectedCall(call3);
	list->addExpectedCall(call4);
	list->onlyKeepUnfulfilledExpectationsWithParameter(parameter);
	LONGS_EQUAL(1, list->size());
}

TEST(MockExpectedFunctionsList, addUnfilfilledExpectationsWithEmptyList)
{
	MockExpectedFunctionsList newList;
	newList.addUnfilfilledExpectations(*list);
	LONGS_EQUAL(0, newList.size());
}

TEST(MockExpectedFunctionsList, addUnfilfilledExpectationsMultipleUnfulfilledExpectations)
{
	call2->callWasMade(1);
	list->addExpectedCall(call1);
	list->addExpectedCall(call2);
	list->addExpectedCall(call3);
	MockExpectedFunctionsList newList;
	newList.addUnfilfilledExpectations(*list);
	LONGS_EQUAL(2, newList.size());
}

TEST(MockExpectedFunctionsList, amountOfExpectationsFor)
{
	call1->withName("foo");
	call2->withName("bar");
	list->addExpectedCall(call1);
	list->addExpectedCall(call2);
	LONGS_EQUAL(1, list->amountOfExpectationsFor("bar"));
}

TEST(MockExpectedFunctionsList, amountOfExpectationsForHasNone)
{
	call1->withName("foo");
	list->addExpectedCall(call1);
	LONGS_EQUAL(0, list->amountOfExpectationsFor("bar"));
}

TEST(MockExpectedFunctionsList, callToStringForUnfulfilledFunctions)
{
	call1->withName("foo");
	call2->withName("bar");
	call3->withName("blah");
	call3->callWasMade(1);

	list->addExpectedCall(call1);
	list->addExpectedCall(call2);
	list->addExpectedCall(call3);

	SimpleString expectedString;
	expectedString = StringFromFormat("%s\n%s", call1->callToString().asCharString(), call2->callToString().asCharString());
	STRCMP_EQUAL(expectedString.asCharString(), list->unfulfilledFunctionsToString().asCharString());
}

TEST(MockExpectedFunctionsList, callToStringForFulfilledFunctions)
{
	call1->withName("foo");
	call2->withName("bar");

	call2->callWasMade(1);
	call1->callWasMade(2);

	list->addExpectedCall(call1);
	list->addExpectedCall(call2);

	SimpleString expectedString;
	expectedString = StringFromFormat("%s\n%s", call2->callToString().asCharString(), call1->callToString().asCharString());
	STRCMP_EQUAL(expectedString.asCharString(), list->fulfilledFunctionsToString().asCharString());
}



TEST(MockExpectedFunctionsList, toStringOnEmptyList)
{
	STRCMP_EQUAL("<none>", list->unfulfilledFunctionsToString().asCharString());
}
