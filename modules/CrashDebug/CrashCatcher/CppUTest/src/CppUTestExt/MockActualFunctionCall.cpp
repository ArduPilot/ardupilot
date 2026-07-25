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
#include "CppUTestExt/MockExpectedFunctionsList.h"
#include "CppUTestExt/MockExpectedFunctionCall.h"
#include "CppUTestExt/MockFailure.h"

MockActualFunctionCall::MockActualFunctionCall(int callOrder, MockFailureReporter* reporter, const MockExpectedFunctionsList& allExpectations)
	: callOrder_(callOrder), reporter_(reporter), state_(CALL_SUCCEED), _fulfilledExpectation(NULL), allExpectations_(allExpectations)
{
	unfulfilledExpectations_.addUnfilfilledExpectations(allExpectations);
}

MockActualFunctionCall::~MockActualFunctionCall()
{
}

void MockActualFunctionCall::setMockFailureReporter(MockFailureReporter* reporter)
{
	reporter_ = reporter;
}


UtestShell* MockActualFunctionCall::getTest() const
{
	return reporter_->getTestToFail();
}

void MockActualFunctionCall::failTest(const MockFailure& failure)
{
	setState(CALL_FAILED);
	reporter_->failTest(failure);
}

void MockActualFunctionCall::finnalizeCallWhenFulfilled()
{
	if (unfulfilledExpectations_.hasFulfilledExpectations()) {
		_fulfilledExpectation = unfulfilledExpectations_.removeOneFulfilledExpectation();
		callHasSucceeded();
	}
}

void MockActualFunctionCall::callHasSucceeded()
{
	setState(CALL_SUCCEED);
	unfulfilledExpectations_.resetExpectations();
}

MockFunctionCall& MockActualFunctionCall::withName(const SimpleString& name)
{
	setName(name);
	setState(CALL_IN_PROGESS);

	unfulfilledExpectations_.onlyKeepUnfulfilledExpectationsRelatedTo(name);
	if (unfulfilledExpectations_.isEmpty()) {
		MockUnexpectedCallHappenedFailure failure(getTest(), name, allExpectations_);
		failTest(failure);
		return *this;
	}

	unfulfilledExpectations_.callWasMade(callOrder_);

	finnalizeCallWhenFulfilled();

	return *this;
}

MockFunctionCall& MockActualFunctionCall::withCallOrder(int)
{
	return *this;
}

void MockActualFunctionCall::checkActualParameter(const MockNamedValue& actualParameter)
{
	unfulfilledExpectations_.onlyKeepUnfulfilledExpectationsWithParameter(actualParameter);

	if (unfulfilledExpectations_.isEmpty()) {
		MockUnexpectedParameterFailure failure(getTest(), getName(), actualParameter, allExpectations_);
		failTest(failure);
		return;
	}

	unfulfilledExpectations_.parameterWasPassed(actualParameter.getName());
	finnalizeCallWhenFulfilled();
}

MockFunctionCall& MockActualFunctionCall::withParameter(const SimpleString& name, int value)
{
	MockNamedValue actualParameter(name);
	actualParameter.setValue(value);
	checkActualParameter(actualParameter);
	return *this;
}

MockFunctionCall& MockActualFunctionCall::withParameter(const SimpleString& name, double value)
{
	MockNamedValue actualParameter(name);
	actualParameter.setValue(value);
	checkActualParameter(actualParameter);
	return *this;
}

MockFunctionCall& MockActualFunctionCall::withParameter(const SimpleString& name, const char* value)
{
	MockNamedValue actualParameter(name);
	actualParameter.setValue(value);
	checkActualParameter(actualParameter);
	return *this;
}

MockFunctionCall& MockActualFunctionCall::withParameter(const SimpleString& name, void* value)
{
	MockNamedValue actualParameter(name);
	actualParameter.setValue(value);
	checkActualParameter(actualParameter);
	return *this;
}

MockFunctionCall& MockActualFunctionCall::withParameterOfType(const SimpleString& type, const SimpleString& name, void* value)
{
	if (getComparatorForType(type) == NULL) {
		MockNoWayToCompareCustomTypeFailure failure(getTest(), type);
		failTest(failure);
		return *this;
	}
	MockNamedValue actualParameter(name);
	actualParameter.setObjectPointer(type, value);
	actualParameter.setComparator(getComparatorForType(type));
	checkActualParameter(actualParameter);
	return *this;
}

bool MockActualFunctionCall::isFulfilled() const
{
	return state_ == CALL_SUCCEED;
}

bool MockActualFunctionCall::hasFailed() const
{
	return state_ == CALL_FAILED;
}

void MockActualFunctionCall::checkExpectations()
{
	if (state_ != CALL_IN_PROGESS) return;

	if (! unfulfilledExpectations_.hasUnfullfilledExpectations())
		FAIL("Actual call is in progress. Checking expectations. But no unfulfilled expectations. Cannot happen.")

	_fulfilledExpectation = unfulfilledExpectations_.removeOneFulfilledExpectationWithIgnoredParameters();
	if (_fulfilledExpectation) {
		callHasSucceeded();
		return;
	}

	if (unfulfilledExpectations_.hasUnfulfilledExpectationsBecauseOfMissingParameters()) {
		MockExpectedParameterDidntHappenFailure failure(getTest(), getName(), allExpectations_);
		failTest(failure);
	}
	else {
		MockExpectedObjectDidntHappenFailure failure(getTest(), getName(), allExpectations_);
		failTest(failure);
	}
}

const char* MockActualFunctionCall::stringFromState(ActualCallState state)
{
	switch (state) {
	case CALL_IN_PROGESS: return "In progress";
	case CALL_FAILED: return "Failed";
	case CALL_SUCCEED: return "Succeed";
	default: ;
	}
	return "No valid state info";
}

void MockActualFunctionCall::checkStateConsistency(ActualCallState oldState, ActualCallState newState)
{
	if (oldState == newState)
		FAIL(StringFromFormat("State change to the same state: %s.", stringFromState(newState)).asCharString());
	if (oldState == CALL_FAILED)
		FAIL("State was already failed. Cannot change state again.");
}

void MockActualFunctionCall::setState(ActualCallState state)
{
	checkStateConsistency(state_, state);
	state_ = state;
}

MockFunctionCall& MockActualFunctionCall::andReturnValue(int)
{
	FAIL("andReturnValue cannot be called on an ActualFunctionCall. Use returnValue instead to get the value.");
	return *this;
}

MockFunctionCall& MockActualFunctionCall::andReturnValue(const char*)
{
	FAIL("andReturnValue cannot be called on an ActualFunctionCall. Use returnValue instead to get the value.");
	return *this;
}

MockFunctionCall& MockActualFunctionCall::andReturnValue(double)
{
	FAIL("andReturnValue cannot be called on an ActualFunctionCall. Use returnValue instead to get the value.");
	return *this;
}

MockFunctionCall& MockActualFunctionCall::andReturnValue(void*)
{
	FAIL("andReturnValue cannot be called on an ActualFunctionCall. Use returnValue instead to get the value.");
	return *this;
}

MockNamedValue MockActualFunctionCall::returnValue()
{
	checkExpectations();
	if (_fulfilledExpectation)
		return _fulfilledExpectation->returnValue();
	return MockNamedValue("no return value");
}

bool MockActualFunctionCall::hasReturnValue()
{
	return ! returnValue().getName().isEmpty();
}

MockFunctionCall& MockActualFunctionCall::onObject(void* objectPtr)
{
	unfulfilledExpectations_.onlyKeepUnfulfilledExpectationsOnObject(objectPtr);

	if (unfulfilledExpectations_.isEmpty()) {
		MockUnexpectedObjectFailure failure(getTest(), getName(), objectPtr, allExpectations_);
		failTest(failure);
		return *this;
	}

	unfulfilledExpectations_.wasPassedToObject();

	finnalizeCallWhenFulfilled();
	return *this;
}

