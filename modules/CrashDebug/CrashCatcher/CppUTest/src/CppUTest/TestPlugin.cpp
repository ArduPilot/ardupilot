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
#include "CppUTest/TestPlugin.h"

TestPlugin::TestPlugin(const SimpleString& name) :
	next_(NullTestPlugin::instance()), name_(name), enabled_(true)
{
}

TestPlugin::TestPlugin(TestPlugin* next) :
	next_(next), name_("null")
{
}

TestPlugin::~TestPlugin()
{
}

TestPlugin* TestPlugin::addPlugin(TestPlugin* plugin)
{
	next_ = plugin;
	return this;
}

void TestPlugin::runAllPreTestAction(UtestShell& test, TestResult& result)
{
	if (enabled_) preTestAction(test, result);
	next_->runAllPreTestAction(test, result);
}

void TestPlugin::runAllPostTestAction(UtestShell& test, TestResult& result)
{
	next_ ->runAllPostTestAction(test, result);
	if (enabled_) postTestAction(test, result);
}

bool TestPlugin::parseAllArguments(int ac, char** av, int index)
{
	return parseAllArguments(ac, const_cast<const char**> (av), index);
}

bool TestPlugin::parseAllArguments(int ac, const char** av, int index)
{
	if (parseArguments(ac, av, index)) return true;
	if (next_) return next_->parseAllArguments(ac, av, index);
	return false;
}

const SimpleString& TestPlugin::getName()
{
	return name_;
}

TestPlugin* TestPlugin::getPluginByName(const SimpleString& name)
{
	if (name == name_) return this;
	if (next_) return next_->getPluginByName(name);
	return (next_);
}

TestPlugin* TestPlugin::getNext()
{
	return next_;
}
TestPlugin* TestPlugin::removePluginByName(const SimpleString& name)
{
	TestPlugin* removed = 0;
	if (next_ && next_->getName() == name) {
		removed = next_;
		next_ = next_->next_;
	}
	return removed;
}

void TestPlugin::disable()
{
	enabled_ = false;
}

void TestPlugin::enable()
{
	enabled_ = true;
}

bool TestPlugin::isEnabled()
{
	return enabled_;
}

struct cpputest_pair
{
	void **orig;
	void *orig_value;
};

//////// SetPlugin

static int pointerTableIndex;
static cpputest_pair setlist[SetPointerPlugin::MAX_SET];

SetPointerPlugin::SetPointerPlugin(const SimpleString& name) :
	TestPlugin(name)
{
	pointerTableIndex = 0;
}

SetPointerPlugin::~SetPointerPlugin()
{
}

void CppUTestStore(void**function, void*value)
{
	if (pointerTableIndex >= SetPointerPlugin::MAX_SET) {
		FAIL("Maximum number of function pointers installed!");
	}
	setlist[pointerTableIndex].orig_value = value;
	setlist[pointerTableIndex].orig = function;
	pointerTableIndex++;
}

void SetPointerPlugin::postTestAction(UtestShell& /*test*/, TestResult& /*result*/)
{
	for (int i = pointerTableIndex - 1; i >= 0; i--)
		*((void**) setlist[i].orig) = setlist[i].orig_value;
	pointerTableIndex = 0;
}

//////// NullPlugin

NullTestPlugin::NullTestPlugin() :
	TestPlugin(0)
{
}

NullTestPlugin* NullTestPlugin::instance()
{
	static NullTestPlugin _instance;
	return &_instance;
}

void NullTestPlugin::runAllPreTestAction(UtestShell&, TestResult&)
{
}

void NullTestPlugin::runAllPostTestAction(UtestShell&, TestResult&)
{
}
