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
#include "CppUTestExt/MockFailure.h"
#include "CppUTestExt/MockExpectedFunctionCall.h"
#include "CppUTestExt/MockExpectedFunctionsList.h"
#include "TestMockFailure.h"

TEST_GROUP(MockFailureTest)
{
	MockFailureReporter reporter;

	MockExpectedFunctionsList *list;
	MockExpectedFunctionCall* call1;
	MockExpectedFunctionCall* call2;
	MockExpectedFunctionCall* call3;

	void setup ()
	{
		list = new MockExpectedFunctionsList;
		call1 = new MockExpectedFunctionCall;
		call2 = new MockExpectedFunctionCall;
		call3 = new MockExpectedFunctionCall;
	}
	void teardown ()
	{
		delete list;
		delete call1;
		delete call2;
		delete call3;
		CHECK_NO_MOCK_FAILURE();
	}
	void addAllToList()
	{
		list->addExpectedCall(call1);
		list->addExpectedCall(call2);
		list->addExpectedCall(call3);
	}
};

TEST(MockFailureTest, noErrorFailureSomethingGoneWrong)
{
	MockFailure failure(UtestShell::getCurrent());
	STRCMP_EQUAL("Test failed with MockFailure without an error! Something went seriously wrong.", failure.getMessage().asCharString());
}

TEST(MockFailureTest, unexpectedCallHappened)
{
	MockUnexpectedCallHappenedFailure failure(UtestShell::getCurrent(), "foobar", *list);
	STRCMP_EQUAL("Mock Failure: Unexpected call to function: foobar\n"
				 "\tEXPECTED calls that did NOT happen:\n"
				 "\t\t<none>\n"
				 "\tACTUAL calls that did happen (in call order):\n"
				 "\t\t<none>", failure.getMessage().asCharString());
}

TEST(MockFailureTest, expectedCallDidNotHappen)
{
	call1->withName("foobar");
	call2->withName("world").withParameter("boo", 2).withParameter("hello", "world");
	call3->withName("haphaphap");
	call3->callWasMade(1);
	addAllToList();

	MockExpectedCallsDidntHappenFailure failure(UtestShell::getCurrent(), *list);
	STRCMP_EQUAL("Mock Failure: Expected call did not happen.\n"
				 "\tEXPECTED calls that did NOT happen:\n"
				 "\t\tfoobar -> no parameters\n"
				 "\t\tworld -> int boo: <2>, char* hello: <world>\n"
				 "\tACTUAL calls that did happen (in call order):\n"
				 "\t\thaphaphap -> no parameters", failure.getMessage().asCharString());
}

TEST(MockFailureTest, MockUnexpectedAdditionalCallFailure)
{
	call1->withName("bar");
	call1->callWasMade(1);
	list->addExpectedCall(call1);

	MockUnexpectedCallHappenedFailure failure(UtestShell::getCurrent(), "bar", *list);
	STRCMP_CONTAINS("Mock Failure: Unexpected additional (2th) call to function: bar\n\tEXPECTED", failure.getMessage().asCharString());
}

TEST(MockFailureTest, MockUnexpectedParameterFailure)
{
	call1->withName("foo").withParameter("boo", 2);
	call2->withName("foo").withParameter("boo", 10);
	call3->withName("unrelated");
	addAllToList();

	MockNamedValue actualParameter("bar");
	actualParameter.setValue(2);

	MockUnexpectedParameterFailure failure(UtestShell::getCurrent(), "foo", actualParameter, *list);
	STRCMP_EQUAL("Mock Failure: Unexpected parameter name to function \"foo\": bar\n"
			     "\tEXPECTED calls that DID NOT happen related to function: foo\n"
				 "\t\tfoo -> int boo: <2>\n"
				 "\t\tfoo -> int boo: <10>\n"
				 "\tACTUAL calls that DID happen related to function: foo\n"
				 "\t\t<none>\n"
			     "\tACTUAL unexpected parameter passed to function: foo\n"
			     "\t\tint bar: <2>", failure.getMessage().asCharString());
}

TEST(MockFailureTest, MockUnexpectedParameterValueFailure)
{
	call1->withName("foo").withParameter("boo", 2);
	call2->withName("foo").withParameter("boo", 10);
	call3->withName("unrelated");
	addAllToList();

	MockNamedValue actualParameter("boo");
	actualParameter.setValue(20);

	MockUnexpectedParameterFailure failure(UtestShell::getCurrent(), "foo", actualParameter, *list);
	STRCMP_EQUAL("Mock Failure: Unexpected parameter value to parameter \"boo\" to function \"foo\": <20>\n"
			     "\tEXPECTED calls that DID NOT happen related to function: foo\n"
				 "\t\tfoo -> int boo: <2>\n"
				 "\t\tfoo -> int boo: <10>\n"
				 "\tACTUAL calls that DID happen related to function: foo\n"
				 "\t\t<none>\n"
			     "\tACTUAL unexpected parameter passed to function: foo\n"
			     "\t\tint boo: <20>", failure.getMessage().asCharString());
}

TEST(MockFailureTest, MockExpectedParameterDidntHappenFailure)
{
	call1->withName("foo").withParameter("bar", 2).withParameter("boo", "str");
	call2->withName("foo").withParameter("bar", 10).withParameter("boo", "bleh");
	call2->callWasMade(1);
	call2->parameterWasPassed("bar");
	call2->parameterWasPassed("boo");
	call3->withName("unrelated");
	addAllToList();

	MockExpectedParameterDidntHappenFailure failure(UtestShell::getCurrent(), "foo", *list);
	STRCMP_EQUAL("Mock Failure: Expected parameter for function \"foo\" did not happen.\n"
			     "\tEXPECTED calls that DID NOT happen related to function: foo\n"
				 "\t\tfoo -> int bar: <2>, char* boo: <str>\n"
				 "\tACTUAL calls that DID happen related to function: foo\n"
				 "\t\tfoo -> int bar: <10>, char* boo: <bleh>\n"
				 "\tMISSING parameters that didn't happen:\n"
			     "\t\tint bar, char* boo", failure.getMessage().asCharString());
}

TEST(MockFailureTest, MockNoWayToCompareCustomTypeFailure)
{
	MockNoWayToCompareCustomTypeFailure failure(UtestShell::getCurrent(), "myType");
	STRCMP_EQUAL("MockFailure: No way to compare type <myType>. Please install a ParameterTypeComparator.", failure.getMessage().asCharString());
}

TEST(MockFailureTest, MockUnexpectedObjectFailure)
{
	call1->withName("foo").onObject((void*) 0x02);
	call2->withName("foo").onObject((void*) 0x03);
	call2->callWasMade(1);
	call2->wasPassedToObject();
	call3->withName("unrelated");
	addAllToList();

	MockUnexpectedObjectFailure failure(UtestShell::getCurrent(), "foo", (void*)0x1, *list);
	STRCMP_EQUAL(StringFromFormat (
			     "MockFailure: Function called on a unexpected object: foo\n"
			     "\tActual object for call has address: <%p>\n"
				 "\tEXPECTED calls that DID NOT happen related to function: foo\n"
				 "\t\t(object address: %p)::foo -> no parameters\n"
				 "\tACTUAL calls that DID happen related to function: foo\n"
				 "\t\t(object address: %p)::foo -> no parameters",
				 (void*) 0x01, (void*) 0x02, (void*) 0x03).asCharString(), failure.getMessage().asCharString());
}

TEST(MockFailureTest, MockExpectedObjectDidntHappenFailure)
{
	call1->withName("foo").onObject((void*) 0x02);
	call2->withName("foo").onObject((void*) 0x03);
	call2->callWasMade(1);
	call2->wasPassedToObject();
	call3->withName("unrelated");
	addAllToList();

	MockExpectedObjectDidntHappenFailure failure(UtestShell::getCurrent(), "foo", *list);
	STRCMP_EQUAL(StringFromFormat(
			     "Mock Failure: Expected call on object for function \"foo\" but it did not happen.\n"
			     "\tEXPECTED calls that DID NOT happen related to function: foo\n"
				 "\t\t(object address: %p)::foo -> no parameters\n"
				 "\tACTUAL calls that DID happen related to function: foo\n"
				 "\t\t(object address: %p)::foo -> no parameters",
				 (void*) 0x2, (void*) 0x3).asCharString(), failure.getMessage().asCharString());
}
