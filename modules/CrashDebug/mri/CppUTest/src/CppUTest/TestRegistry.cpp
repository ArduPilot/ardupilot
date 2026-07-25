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
#include "CppUTest/TestRegistry.h"

TestRegistry::TestRegistry() :
	tests_(&NullTestShell::instance()), firstPlugin_(NullTestPlugin::instance()), runInSeperateProcess_(false)
{
}

TestRegistry::~TestRegistry()
{
}

void TestRegistry::addTest(UtestShell *test)
{
	tests_ = test->addTest(tests_);
}

void TestRegistry::runAllTests(TestResult& result)
{
	bool groupStart = true;

	result.testsStarted();
	for (UtestShell *test = tests_; !test->isNull(); test = test->getNext()) {
		if (runInSeperateProcess_) test->setRunInSeperateProcess();

		if (groupStart) {
			result.currentGroupStarted(test);
			groupStart = false;
		}

		result.setProgressIndicator(test->getProgressIndicator());
		result.countTest();
		if (testShouldRun(test, result)) {
			result.currentTestStarted(test);
			test->runOneTestWithPlugins(firstPlugin_, result);
			result.currentTestEnded(test);
		}

		if (endOfGroup(test)) {
			groupStart = true;
			result.currentGroupEnded(test);
		}
	}
	result.testsEnded();
}

bool TestRegistry::endOfGroup(UtestShell* test)
{
	return (test->isNull() || test->getGroup() != test->getNext()->getGroup());
}

int TestRegistry::countTests()
{
	return tests_->countTests();
}

TestRegistry* TestRegistry::currentRegistry_ = 0;

TestRegistry* TestRegistry::getCurrentRegistry()
{
	static TestRegistry registry;
	return (currentRegistry_ == 0) ? &registry : currentRegistry_;
}

void TestRegistry::setCurrentRegistry(TestRegistry* registry)
{
	currentRegistry_ = registry;
}

void TestRegistry::unDoLastAddTest()
{
	tests_ = tests_->getNext();

}

void TestRegistry::nameFilter(const TestFilter& f)
{
	nameFilter_ = f;
}

void TestRegistry::groupFilter(const TestFilter& f)
{
	groupFilter_ = f;
}

TestFilter TestRegistry::getGroupFilter()
{
	return groupFilter_;
}

TestFilter TestRegistry::getNameFilter()
{
	return nameFilter_;
}

void TestRegistry::setRunTestsInSeperateProcess()
{
	runInSeperateProcess_ = true;
}


bool TestRegistry::testShouldRun(UtestShell* test, TestResult& result)
{
	if (test->shouldRun(groupFilter_, nameFilter_)) return true;
	else {
		result.countFilteredOut();
		return false;
	}
}

void TestRegistry::resetPlugins()
{
	firstPlugin_ = NullTestPlugin::instance();
}

void TestRegistry::installPlugin(TestPlugin* plugin)
{
	firstPlugin_ = plugin->addPlugin(firstPlugin_);
}

TestPlugin* TestRegistry::getFirstPlugin()
{
	return firstPlugin_;
}

TestPlugin* TestRegistry::getPluginByName(const SimpleString& name)
{
	return firstPlugin_->getPluginByName(name);
}

void TestRegistry::removePluginByName(const SimpleString& name)
{
	if (firstPlugin_->removePluginByName(name) == firstPlugin_) firstPlugin_ = firstPlugin_->getNext();
	if (firstPlugin_->getName() == name) firstPlugin_ = firstPlugin_->getNext();
	firstPlugin_->removePluginByName(name);
}

UtestShell* TestRegistry::getFirstTest()
{
	return tests_;
}

UtestShell* TestRegistry::getLastTest()
{
	UtestShell* current = tests_;
	while (!current->getNext()->isNull())
		current = current->getNext();
	return current;
}

UtestShell* TestRegistry::getTestWithNext(UtestShell* test)
{
	UtestShell* current = tests_;
	while (!current->getNext()->isNull() && current->getNext() != test)
		current = current->getNext();
	return current;
}

UtestShell* TestRegistry::findTestWithName(const SimpleString& name)
{
	UtestShell* current = tests_;
	while (!current->isNull()) {
		if (current->getName() == name)
			return current;
		current = current->getNext();
	}
	return NULL;
}

UtestShell* TestRegistry::findTestWithGroup(const SimpleString& group)
{
	UtestShell* current = tests_;
	while (!current->isNull()) {
		if (current->getGroup() == group)
			return current;
		current = current->getNext();
	}
	return NULL;
}

