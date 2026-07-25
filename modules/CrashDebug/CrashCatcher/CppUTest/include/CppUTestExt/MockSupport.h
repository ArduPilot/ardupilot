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

#ifndef D_MockSupport_h
#define D_MockSupport_h

#include "CppUTestExt/MockFailure.h"
#include "CppUTestExt/MockFunctionCall.h"
#include "CppUTestExt/MockExpectedFunctionCall.h"
#include "CppUTestExt/MockExpectedFunctionsList.h"

class UtestShell;
class MockSupport;

/* This allows access to "the global" mocking support for easier testing */
MockSupport& mock(const SimpleString& mockName = "");

class MockSupport
{
public:
	MockSupport();
	virtual ~MockSupport();

	virtual void strictOrder();
	virtual MockFunctionCall& expectOneCall(const SimpleString& functionName);
	virtual MockFunctionCall& expectNCalls(int amount, const SimpleString& functionName);
	virtual MockFunctionCall& actualCall(const SimpleString& functionName);
	virtual bool hasReturnValue();
	virtual MockNamedValue returnValue();
	virtual int intReturnValue();
	virtual const char* stringReturnValue();
	virtual double doubleReturnValue();
	virtual void* pointerReturnValue();

	bool hasData(const SimpleString& name);
	void setData(const SimpleString& name, int value);
	void setData(const SimpleString& name, const char* value);
	void setData(const SimpleString& name, double value);
	void setData(const SimpleString& name, void* value);
	void setDataObject(const SimpleString& name, const SimpleString& type, void* value);
	MockNamedValue getData(const SimpleString& name);

	MockSupport* getMockSupportScope(const SimpleString& name);

	const char* getTraceOutput();
	/*
	 * The following functions are recursively through the lower MockSupports scopes
	 * This means, if you do mock().disable() it will disable *all* mocking scopes, including mock("myScope").
  	 */

	virtual void disable();
    virtual void enable();
    virtual void tracing(bool enabled);
	virtual void ignoreOtherCalls();

    virtual void checkExpectations();
    virtual bool expectedCallsLeft();

    virtual void clear();
	virtual void setMockFailureReporter(MockFailureReporter* reporter);
	virtual void crashOnFailure();

	virtual void installComparator(const SimpleString& typeName, MockNamedValueComparator& comparator);
	virtual void installComparators(const MockNamedValueComparatorRepository& repository);
	virtual void removeAllComparators();

protected:
    virtual MockActualFunctionCall *createActualFunctionCall();
    virtual void failTest(MockFailure& failure);
private:
    static int callOrder_;
    static int expectedCallOrder_;
    bool strictOrdering_;
    MockFailureReporter *reporter_;
    MockFailureReporter defaultReporter_;
    MockExpectedFunctionsList expectations_;
    bool ignoreOtherCalls_;
    bool enabled_;
    MockActualFunctionCall *lastActualFunctionCall_;
	MockFunctionCallComposite compositeCalls_;
    MockNamedValueComparatorRepository comparatorRepository_;
    MockNamedValueList data_;

    bool tracing_;

    void checkExpectationsOfLastCall();
    bool wasLastCallFulfilled();
    void failTestWithUnexpectedCalls();
    void failTestWithOutOfOrderCalls();

	MockNamedValue* retrieveDataFromStore(const SimpleString& name);

	MockSupport* getMockSupport(MockNamedValueListNode* node);
};

#endif

