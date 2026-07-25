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
#include "CppUTestExt/MockActualFunctionCall.h"
#include "CppUTestExt/MockExpectedFunctionCall.h"
#include "CppUTestExt/MockExpectedFunctionsList.h"
#include "CppUTestExt/MockFailure.h"
#include "TestMockFailure.h"

TEST_GROUP(MockActualFunctionCall)
{
	MockExpectedFunctionsList* emptyList;
	MockExpectedFunctionsList* list;
	MockFailureReporter* reporter;

	void setup()
	{
		emptyList = new MockExpectedFunctionsList;
		list = new MockExpectedFunctionsList;
		reporter = MockFailureReporterForTest::getReporter();
	}

	void teardown()
	{
		CHECK_NO_MOCK_FAILURE();
		delete emptyList;
		delete list;
	}
};

TEST(MockActualFunctionCall, unExpectedCall)
{
	MockActualFunctionCall actualCall(1, reporter, *emptyList);
	actualCall.withName("unexpected");

	MockUnexpectedCallHappenedFailure expectedFailure(mockFailureTest(), "unexpected", *list);
	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockActualFunctionCall, unExpectedParameterName)
{
	MockExpectedFunctionCall call1;
	call1.withName("func");
	list->addExpectedCall(&call1);

	MockActualFunctionCall actualCall(1, reporter, *list);
	actualCall.withName("func").withParameter("integer", 1);

	MockNamedValue parameter("integer");
	parameter.setValue(1);

	MockUnexpectedParameterFailure expectedFailure(mockFailureTest(), "func", parameter, *list);
	CHECK_EXPECTED_MOCK_FAILURE(expectedFailure);
}

TEST(MockActualFunctionCall, multipleSameFunctionsExpectingAndHappenGradually)
{
	MockExpectedFunctionCall* call1 = new MockExpectedFunctionCall();
	MockExpectedFunctionCall* call2 = new MockExpectedFunctionCall();
	call1->withName("func");
	call2->withName("func");
	list->addExpectedCall(call1);
	list->addExpectedCall(call2);

	MockActualFunctionCall actualCall1(1, reporter, *list);
	MockActualFunctionCall actualCall2(2, reporter, *list);

	LONGS_EQUAL(2, list->amountOfUnfulfilledExpectations());
	actualCall1.withName("func");
	LONGS_EQUAL(1, list->amountOfUnfulfilledExpectations());
	actualCall2.withName("func");
	LONGS_EQUAL(0, list->amountOfUnfulfilledExpectations());

	list->deleteAllExpectationsAndClearList();
}
