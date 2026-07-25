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

#ifndef D_MockExpectedFunctionsList_h
#define D_MockExpectedFunctionsList_h

class MockExpectedFunctionCall;
class MockNamedValue;

class MockExpectedFunctionsList
{

public:
	MockExpectedFunctionsList();
	virtual ~MockExpectedFunctionsList();
	virtual void deleteAllExpectationsAndClearList();

	virtual int size() const;
	virtual int amountOfExpectationsFor(const SimpleString& name) const;
	virtual int amountOfUnfulfilledExpectations() const;
	virtual bool hasUnfullfilledExpectations() const;
	virtual bool hasFulfilledExpectations() const;
	virtual bool hasUnfulfilledExpectationsBecauseOfMissingParameters() const;
	virtual bool hasExpectationWithName(const SimpleString& name) const;
	virtual bool hasCallsOutOfOrder() const;
	virtual bool isEmpty() const;

	virtual void addExpectedCall(MockExpectedFunctionCall* call);
	virtual void addExpectations(const MockExpectedFunctionsList& list);
	virtual void addExpectationsRelatedTo(const SimpleString& name, const MockExpectedFunctionsList& list);
	virtual void addUnfilfilledExpectations(const MockExpectedFunctionsList& list);

	virtual void onlyKeepExpectationsRelatedTo(const SimpleString& name);
	virtual void onlyKeepExpectationsWithParameter(const MockNamedValue& parameter);
	virtual void onlyKeepExpectationsWithParameterName(const SimpleString& name);
	virtual void onlyKeepExpectationsOnObject(void* objectPtr);
	virtual void onlyKeepUnfulfilledExpectations();
	virtual void onlyKeepUnfulfilledExpectationsRelatedTo(const SimpleString& name);
	virtual void onlyKeepUnfulfilledExpectationsWithParameter(const MockNamedValue& parameter);
	virtual void onlyKeepUnfulfilledExpectationsOnObject(void* objectPtr);

	virtual MockExpectedFunctionCall* removeOneFulfilledExpectation();
	virtual MockExpectedFunctionCall* removeOneFulfilledExpectationWithIgnoredParameters();

	virtual void resetExpectations();
	virtual void callWasMade(int callOrder);
	virtual void wasPassedToObject();
	virtual void parameterWasPassed(const SimpleString& parameterName);

	virtual SimpleString unfulfilledFunctionsToString(const SimpleString& linePrefix = "") const;
	virtual SimpleString fulfilledFunctionsToString(const SimpleString& linePrefix = "") const;
	virtual SimpleString missingParametersToString() const;

protected:
	virtual void pruneEmptyNodeFromList();

	class MockExpectedFunctionsListNode
	{
	public:
		MockExpectedFunctionCall* expectedCall_;

		MockExpectedFunctionsListNode* next_;
		MockExpectedFunctionsListNode(MockExpectedFunctionCall* expectedCall)
			: expectedCall_(expectedCall), next_(NULL) {};
	};

	virtual MockExpectedFunctionsListNode* findNodeWithCallOrderOf(int callOrder) const;
private:
	MockExpectedFunctionsListNode* head_;

	MockExpectedFunctionsList(const MockExpectedFunctionsList&);
};

#endif
