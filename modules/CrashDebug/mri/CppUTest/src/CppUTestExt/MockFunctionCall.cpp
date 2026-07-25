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
#include "CppUTestExt/MockFunctionCall.h"
#include "CppUTestExt/MockNamedValue.h"

MockFunctionCall::MockFunctionCall() : comparatorRepository_(NULL)
{
}

MockFunctionCall::~MockFunctionCall()
{
}

void MockFunctionCall::setComparatorRepository(MockNamedValueComparatorRepository* repository)
{
	comparatorRepository_ = repository;
}

void MockFunctionCall::setName(const SimpleString& name)
{
	functionName_ = name;
}

SimpleString MockFunctionCall::getName() const
{
	return functionName_;
}

MockNamedValueComparator* MockFunctionCall::getComparatorForType(const SimpleString& type) const
{
	if (comparatorRepository_)
		return comparatorRepository_->getComparatorForType(type);
	return NULL;
}

struct MockFunctionCallCompositeNode
{
	MockFunctionCallCompositeNode(MockFunctionCall& functionCall, MockFunctionCallCompositeNode* next) : next_(next), call_(functionCall){}

	MockFunctionCallCompositeNode* next_;
	MockFunctionCall& call_;
};

MockFunctionCallComposite::MockFunctionCallComposite() : head_(NULL)
{
}

MockFunctionCallComposite::~MockFunctionCallComposite()
{

}

void MockFunctionCallComposite::add(MockFunctionCall& call)
{
	head_ = new MockFunctionCallCompositeNode(call, head_);
}

void MockFunctionCallComposite::clear()
{
	while (head_) {
		MockFunctionCallCompositeNode* next = head_->next_;
		delete head_;
		head_ = next;
	}
}

MockFunctionCall& MockFunctionCallComposite::withName(const SimpleString& name)
{
	for (MockFunctionCallCompositeNode* node = head_; node != NULL; node = node->next_)
		node->call_.withName(name);
	return *this;
}

MockFunctionCall& MockFunctionCallComposite::withCallOrder(int)
{
	FAIL("withCallOrder not supported for CompositeCalls");
	return *this;
}

MockFunctionCall& MockFunctionCallComposite::withParameter(const SimpleString& name, int value)
{
	for (MockFunctionCallCompositeNode* node = head_; node != NULL; node = node->next_)
		node->call_.withParameter(name, value);
	return *this;
}

MockFunctionCall& MockFunctionCallComposite::withParameter(const SimpleString& name, double value)
{
	for (MockFunctionCallCompositeNode* node = head_; node != NULL; node = node->next_)
		node->call_.withParameter(name, value);
	return *this;

}

MockFunctionCall& MockFunctionCallComposite::withParameter(const SimpleString& name, const char* value)
{
	for (MockFunctionCallCompositeNode* node = head_; node != NULL; node = node->next_)
		node->call_.withParameter(name, value);
	return *this;
}

MockFunctionCall& MockFunctionCallComposite::withParameter(const SimpleString& name, void* value)
{
	for (MockFunctionCallCompositeNode* node = head_; node != NULL; node = node->next_)
		node->call_.withParameter(name, value);
	return *this;
}

MockFunctionCall& MockFunctionCallComposite::withParameterOfType(const SimpleString& typeName, const SimpleString& name, void* value)
{
	for (MockFunctionCallCompositeNode* node = head_; node != NULL; node = node->next_)
		node->call_.withParameterOfType(typeName, name, value);
	return *this;

}

MockFunctionCall& MockFunctionCallComposite::ignoreOtherParameters()
{
	for (MockFunctionCallCompositeNode* node = head_; node != NULL; node = node->next_)
		node->call_.ignoreOtherParameters();
	return *this;
}

MockFunctionCall& MockFunctionCallComposite::andReturnValue(int value)
{
	for (MockFunctionCallCompositeNode* node = head_; node != NULL; node = node->next_)
		node->call_.andReturnValue(value);
	return *this;
}

MockFunctionCall& MockFunctionCallComposite::andReturnValue(double value)
{
	for (MockFunctionCallCompositeNode* node = head_; node != NULL; node = node->next_)
		node->call_.andReturnValue(value);
	return *this;
}

MockFunctionCall& MockFunctionCallComposite::andReturnValue(const char* value)
{
	for (MockFunctionCallCompositeNode* node = head_; node != NULL; node = node->next_)
		node->call_.andReturnValue(value);
	return *this;
}

MockFunctionCall& MockFunctionCallComposite::andReturnValue(void* value)
{
	for (MockFunctionCallCompositeNode* node = head_; node != NULL; node = node->next_)
		node->call_.andReturnValue(value);
	return *this;
}

bool MockFunctionCallComposite::hasReturnValue()
{
	return head_->call_.hasReturnValue();
}

MockNamedValue MockFunctionCallComposite::returnValue()
{
	return head_->call_.returnValue();
}

MockFunctionCall& MockFunctionCallComposite::onObject(void* object)
{
	for (MockFunctionCallCompositeNode* node = head_; node != NULL; node = node->next_)
		node->call_.onObject(object);
	return *this;
}


MockFunctionCallTrace::MockFunctionCallTrace()
{
}

MockFunctionCallTrace::~MockFunctionCallTrace()
{
}

MockFunctionCall& MockFunctionCallTrace::withName(const SimpleString& name)
{
	traceBuffer_ += "\nFunction name: ";
	traceBuffer_ += name;
	return *this;
}

MockFunctionCall& MockFunctionCallTrace::withCallOrder(int callOrder)
{
	traceBuffer_ += "\nwithCallOrder: ";
	traceBuffer_ += StringFrom(callOrder);
	return *this;
}

MockFunctionCall& MockFunctionCallTrace::withParameter(const SimpleString& name, int value)
{
	traceBuffer_ += " ";
	traceBuffer_ += name;
	traceBuffer_ += ":";
	traceBuffer_ += StringFrom(value);
	return *this;
}

MockFunctionCall& MockFunctionCallTrace::withParameter(const SimpleString& name, double value)
{
	traceBuffer_ += " ";
	traceBuffer_ += name;
	traceBuffer_ += ":";
	traceBuffer_ += StringFrom(value);
	return *this;
}

MockFunctionCall& MockFunctionCallTrace::withParameter(const SimpleString& name, const char* value)
{
	traceBuffer_ += " ";
	traceBuffer_ += name;
	traceBuffer_ += ":";
	traceBuffer_ += StringFrom(value);
	return *this;
}

MockFunctionCall& MockFunctionCallTrace::withParameter(const SimpleString& name, void* value)
{
	traceBuffer_ += " ";
	traceBuffer_ += name;
	traceBuffer_ += ":";
	traceBuffer_ += StringFrom(value);
	return *this;
}

MockFunctionCall& MockFunctionCallTrace::withParameterOfType(const SimpleString& typeName, const SimpleString& name, void* value)
{
	traceBuffer_ += " ";
	traceBuffer_ += typeName;
	traceBuffer_ += " ";
	traceBuffer_ += name;
	traceBuffer_ += ":";
	traceBuffer_ += StringFrom(value);
	return *this;
}

MockFunctionCall& MockFunctionCallTrace::ignoreOtherParameters()
{
	return *this;
}

MockFunctionCall& MockFunctionCallTrace::andReturnValue(int)
{
	return *this;
}

MockFunctionCall& MockFunctionCallTrace::andReturnValue(double)
{
	return *this;
}

MockFunctionCall& MockFunctionCallTrace::andReturnValue(const char*)
{
	return *this;
}

MockFunctionCall& MockFunctionCallTrace::andReturnValue(void*)
{
	return *this;
}

bool MockFunctionCallTrace::hasReturnValue()
{
	return false;
}

MockNamedValue MockFunctionCallTrace::returnValue()
{
	return MockNamedValue("");
}

MockFunctionCall& MockFunctionCallTrace::onObject(void* objectPtr)
{
	traceBuffer_ += StringFrom(objectPtr);
	return *this;
}

void MockFunctionCallTrace::clear()
{
	traceBuffer_ = "";
}

const char* MockFunctionCallTrace::getTraceOutput()
{
	return traceBuffer_.asCharString();
}

MockFunctionCallTrace& MockFunctionCallTrace::instance()
{
	static MockFunctionCallTrace call;
	return call;
}


