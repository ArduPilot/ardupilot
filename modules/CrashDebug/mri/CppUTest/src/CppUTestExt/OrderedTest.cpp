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
#include "CppUTestExt/OrderedTest.h"

OrderedTestShell* OrderedTestShell::_orderedTestsHead = 0;

OrderedTestShell::OrderedTestShell() :
	_nextOrderedTest(0)
{
}

OrderedTestShell::~OrderedTestShell()
{
}

int OrderedTestShell::getLevel()
{
	return _level;
}

void OrderedTestShell::setLevel(int level)
{
	_level = level;
}

void OrderedTestShell::setOrderedTestHead(OrderedTestShell* test)
{
	_orderedTestsHead = test;
}

OrderedTestShell* OrderedTestShell::getOrderedTestHead()
{
	return _orderedTestsHead;
}

bool OrderedTestShell::firstOrderedTest()
{
	return (getOrderedTestHead() == 0);
}

OrderedTestShell* OrderedTestShell::addOrderedTest(OrderedTestShell* test)
{
	UtestShell::addTest(test);
	_nextOrderedTest = test;
	return this;
}

void OrderedTestShell::addOrderedTestToHead(OrderedTestShell* test)
{
	TestRegistry *reg = TestRegistry::getCurrentRegistry();

	if (reg->getFirstTest()->isNull() || getOrderedTestHead()
			== reg->getFirstTest()) reg->addTest(test);
	else reg->getTestWithNext(getOrderedTestHead())->addTest(test);

	test->_nextOrderedTest = getOrderedTestHead();
	setOrderedTestHead(test);
}

OrderedTestShell* OrderedTestShell::getNextOrderedTest()
{
	return _nextOrderedTest;
}

OrderedTestInstaller::OrderedTestInstaller(OrderedTestShell& test,
		const char* groupName, const char* testName, const char* fileName,
		int lineNumber, int level)
{
	test.setTestName(testName);
	test.setGroupName(groupName);
	test.setFileName(fileName);
	test.setLineNumber(lineNumber);
	test.setLevel(level);

	if (OrderedTestShell::firstOrderedTest()) OrderedTestShell::addOrderedTestToHead(&test);
	else addOrderedTestInOrder(&test);
}

void OrderedTestInstaller::addOrderedTestInOrder(OrderedTestShell* test)
{
	if (test->getLevel() < OrderedTestShell::getOrderedTestHead()->getLevel()) OrderedTestShell::addOrderedTestToHead(
			test);
	else addOrderedTestInOrderNotAtHeadPosition(test);
}

void OrderedTestInstaller::addOrderedTestInOrderNotAtHeadPosition(
		OrderedTestShell* test)
{
	OrderedTestShell* current = OrderedTestShell::getOrderedTestHead();
	while (current->getNextOrderedTest()) {

		if (current->getNextOrderedTest()->getLevel() > test->getLevel()) {
			test->addOrderedTest(current->getNextOrderedTest());
			current->addOrderedTest(test);
			return;
		}
		current = current->getNextOrderedTest();
	}
	test->addOrderedTest(current->getNextOrderedTest());
	current->addOrderedTest(test);
}

OrderedTestInstaller::~OrderedTestInstaller()
{
}
