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

#ifndef D_MockExpectedFunctionCall_h
#define D_MockExpectedFunctionCall_h

#include "CppUTestExt/MockFunctionCall.h"
#include "CppUTestExt/MockNamedValue.h"

extern SimpleString StringFrom(const MockNamedValue& parameter);

class MockExpectedFunctionCall : public MockFunctionCall
{

public:
	MockExpectedFunctionCall();
	virtual ~MockExpectedFunctionCall();

	virtual MockFunctionCall& withName(const SimpleString& name);
	virtual MockFunctionCall& withCallOrder(int);
	virtual MockFunctionCall& withParameter(const SimpleString& name, int value);
	virtual MockFunctionCall& withParameter(const SimpleString& name, double value);
	virtual MockFunctionCall& withParameter(const SimpleString& name, const char* value);
	virtual MockFunctionCall& withParameter(const SimpleString& name, void* value);
	virtual MockFunctionCall& withParameterOfType(const SimpleString& typeName, const SimpleString& name, void* value);
	virtual MockFunctionCall& ignoreOtherParameters();

	virtual MockFunctionCall& andReturnValue(int value);
	virtual MockFunctionCall& andReturnValue(double value);
	virtual MockFunctionCall& andReturnValue(const char* value);
	virtual MockFunctionCall& andReturnValue(void* value);
	virtual bool hasReturnValue();
	virtual MockNamedValue returnValue();

	virtual MockFunctionCall& onObject(void* objectPtr);

	virtual MockNamedValue getParameter(const SimpleString& name);
	virtual SimpleString getParameterType(const SimpleString& name);
	virtual SimpleString getParameterValueString(const SimpleString& name);

	virtual bool hasParameterWithName(const SimpleString& name);
	virtual bool hasParameter(const MockNamedValue& parameter);
	virtual bool relatesTo(const SimpleString& functionName);
	virtual bool relatesToObject(void*objectPtr) const;

	virtual bool isFulfilled();
	virtual bool isFulfilledWithoutIgnoredParameters();
	virtual bool areParametersFulfilled();
	virtual bool areIgnoredParametersFulfilled();
	virtual bool isOutOfOrder() const;

	virtual void callWasMade(int callOrder);
	virtual void parameterWasPassed(const SimpleString& name);
	virtual void parametersWereIgnored();
	virtual void wasPassedToObject();
	virtual void resetExpectation();

	virtual SimpleString callToString();
	virtual SimpleString missingParametersToString();

	enum { NOT_CALLED_YET = -1, NO_EXPECTED_CALL_ORDER = -1};
	virtual int getCallOrder() const;
private:

	class MockExpectedFunctionParameter : public MockNamedValue
	{
	public:
		MockExpectedFunctionParameter(const SimpleString& name);
		void setFulfilled(bool b);
		bool isFulfilled() const;

	private:
		bool fulfilled_;
	};

	MockExpectedFunctionParameter* item(MockNamedValueListNode* node);

	bool ignoreOtherParameters_;
	bool parametersWereIgnored_;
	int callOrder_;
	int expectedCallOrder_;
	bool outOfOrder_;
	MockNamedValueList* parameters_;
	MockNamedValue returnValue_;
	void* objectPtr_;
	bool wasPassedToObject_;
};

#endif
