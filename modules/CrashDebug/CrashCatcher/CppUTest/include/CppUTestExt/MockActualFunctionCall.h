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

#ifndef D_MockActualFunctionCall_h
#define D_MockActualFunctionCall_h

#include "CppUTestExt/MockFunctionCall.h"
#include "CppUTestExt/MockExpectedFunctionsList.h"

class MockFailureReporter;
class MockFailure;
class MockNamedValue;

class MockActualFunctionCall : public MockFunctionCall
{
public:
	MockActualFunctionCall(int callOrder, MockFailureReporter* reporter, const MockExpectedFunctionsList& expectations);
	virtual ~MockActualFunctionCall();

	virtual MockFunctionCall& withName(const SimpleString& name);
	virtual MockFunctionCall& withCallOrder(int);
	virtual MockFunctionCall& withParameter(const SimpleString& name, int value);
	virtual MockFunctionCall& withParameter(const SimpleString& name, double value);
	virtual MockFunctionCall& withParameter(const SimpleString& name, const char* value);
	virtual MockFunctionCall& withParameter(const SimpleString& name, void* value);
	virtual MockFunctionCall& withParameterOfType(const SimpleString& type, const SimpleString& name, void* value);

	virtual MockFunctionCall& andReturnValue(int value);
	virtual MockFunctionCall& andReturnValue(double value);
	virtual MockFunctionCall& andReturnValue(const char* value);
	virtual MockFunctionCall& andReturnValue(void* value);
	virtual bool hasReturnValue();
	virtual MockNamedValue returnValue();

	virtual MockFunctionCall& onObject(void* objectPtr);

	virtual bool isFulfilled() const;
	virtual bool hasFailed() const;

	virtual void checkExpectations();

	virtual void setMockFailureReporter(MockFailureReporter* reporter);
protected:
	virtual UtestShell* getTest() const;
	virtual void callHasSucceeded();
	virtual void finnalizeCallWhenFulfilled();
	virtual void failTest(const MockFailure& failure);
	virtual void checkActualParameter(const MockNamedValue& actualParameter);

	enum ActualCallState {
		CALL_IN_PROGESS,
		CALL_FAILED,
		CALL_SUCCEED
	};
	virtual const char* stringFromState(ActualCallState state);
	virtual void setState(ActualCallState state);
	virtual void checkStateConsistency(ActualCallState oldState, ActualCallState newState);

private:
	int callOrder_;
	MockFailureReporter* reporter_;

	ActualCallState state_;
	MockExpectedFunctionCall* _fulfilledExpectation;

	MockExpectedFunctionsList unfulfilledExpectations_;
	const MockExpectedFunctionsList& allExpectations_;
};

#endif
