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
#include "CppUTest/TestOutput.h"

namespace
{
const int testLineNumber = 1;
}

class MockTest: public UtestShell
{
public:
	MockTest(const char* group = "Group") :
		UtestShell(group, "Name", "File", testLineNumber), hasRun_(false)
	{
	}
	virtual void runOneTestWithPlugins(TestPlugin*, TestResult&)
	{
		hasRun_ = true;
	}

	bool hasRun_;
};

class MockTestResult: public TestResult
{
public:

	int countTestsStarted;
	int countTestsEnded;
	int countCurrentTestStarted;
	int countCurrentTestEnded;
	int countCurrentGroupStarted;
	int countCurrentGroupEnded;

	MockTestResult(TestOutput& p) :
		TestResult(p)
	{
		resetCount();
	}

	virtual ~MockTestResult()
	{
	}

	void resetCount()
	{
		countTestsStarted = 0;
		countTestsEnded = 0;
		countCurrentTestStarted = 0;
		countCurrentTestEnded = 0;
		countCurrentGroupStarted = 0;
		countCurrentGroupEnded = 0;
	}

	virtual void testsStarted()
	{
		countTestsStarted++;
	}
	virtual void testsEnded()
	{
		countTestsEnded++;
	}
	virtual void currentTestStarted(UtestShell* /*test*/)
	{
		countCurrentTestStarted++;
	}
	virtual void currentTestEnded(UtestShell* /*test*/)
	{
		countCurrentTestEnded++;
	}
	virtual void currentGroupStarted(UtestShell* /*test*/)
	{
		countCurrentGroupStarted++;
	}
	virtual void currentGroupEnded(UtestShell* /*test*/)
	{
		countCurrentGroupEnded++;
	}

};

TEST_GROUP(TestRegistry)
{
	TestRegistry* myRegistry;
	StringBufferTestOutput* output;
	MockTest* test1;
	MockTest* test2;
	MockTest* test3;
	TestResult *result;
	MockTestResult *mockResult;
	void setup()
	{
		output = new StringBufferTestOutput();
		mockResult = new MockTestResult(*output);
		result = mockResult;
		test1 = new MockTest();
		test2 = new MockTest();
		test3 = new MockTest("group2");
		myRegistry = new TestRegistry();
		myRegistry->setCurrentRegistry(myRegistry);
	}

	void teardown()
	{
		myRegistry->setCurrentRegistry(0);
		delete myRegistry;
		delete test1;
		delete test2;
		delete test3;
		delete result;
		delete output;
	}

	void addAndRunAllTests()
	{
		myRegistry->addTest(test1);
		myRegistry->addTest(test2);
		myRegistry->addTest(test3);
		myRegistry->runAllTests(*result);
	}
};

TEST(TestRegistry, registryMyRegistryAndReset)
{
	CHECK(myRegistry->getCurrentRegistry() == myRegistry);
}

TEST(TestRegistry, emptyRegistryIsEmpty)
{
	CHECK(myRegistry->countTests() == 0);
}

TEST(TestRegistry, addOneTestIsNotEmpty)
{
	myRegistry->addTest(test1);
	CHECK(myRegistry->countTests() == 1);
}

TEST(TestRegistry, addOneTwoTests)
{
	myRegistry->addTest(test1);
	myRegistry->addTest(test2);
	CHECK(myRegistry->countTests() == 2);
}

TEST(TestRegistry, runTwoTests)
{
	myRegistry->addTest(test1);
	myRegistry->addTest(test2);
	CHECK(!test1->hasRun_);
	CHECK(!test2->hasRun_);
	myRegistry->runAllTests(*result);
	CHECK(test1->hasRun_);
	CHECK(test2->hasRun_);
}

TEST(TestRegistry, runTwoTestsCheckResultFunctionsCalled)
{
	myRegistry->addTest(test1);
	myRegistry->addTest(test2);
	myRegistry->runAllTests(*result);
	LONGS_EQUAL(1, mockResult->countTestsStarted);
	LONGS_EQUAL(1, mockResult->countTestsEnded);
	LONGS_EQUAL(1, mockResult->countCurrentGroupStarted);
	LONGS_EQUAL(1, mockResult->countCurrentGroupEnded);
	LONGS_EQUAL(2, mockResult->countCurrentTestStarted);
	LONGS_EQUAL(2, mockResult->countCurrentTestEnded);
}

TEST(TestRegistry, runThreeTestsandTwoGroupsCheckResultFunctionsCalled)
{
	addAndRunAllTests();
	LONGS_EQUAL(2, mockResult->countCurrentGroupStarted);
	LONGS_EQUAL(2, mockResult->countCurrentGroupEnded);
	LONGS_EQUAL(3, mockResult->countCurrentTestStarted);
	LONGS_EQUAL(3, mockResult->countCurrentTestEnded);
}

TEST(TestRegistry, unDoTest)
{
	myRegistry->addTest(test1);
	CHECK(myRegistry->countTests() == 1);
	myRegistry->unDoLastAddTest();
	CHECK(myRegistry->countTests() == 0);
}

TEST(TestRegistry, unDoButNoTest)
{
	CHECK(myRegistry->countTests() == 0);
	myRegistry->unDoLastAddTest();
	CHECK(myRegistry->countTests() == 0);
}

TEST(TestRegistry, reallyUndoLastTest)
{
	myRegistry->addTest(test1);
	myRegistry->addTest(test2);
	CHECK(myRegistry->countTests() == 2);
	myRegistry->unDoLastAddTest();
	CHECK(myRegistry->countTests() == 1);
	myRegistry->runAllTests(*result);
	CHECK(test1->hasRun_);
	CHECK(!test2->hasRun_);
}

TEST(TestRegistry, findTestWithNameDoesntExist)
{
	CHECK(myRegistry->findTestWithName("ThisTestDoesntExists") == NULL);
}

TEST(TestRegistry, findTestWithName)
{
	test1->setTestName("NameOfATestThatDoesExist");
	myRegistry->addTest(test1);
	CHECK(myRegistry->findTestWithName("NameOfATestThatDoesExist"));
}

TEST(TestRegistry, findTestWithGroupDoesntExist)
{
	CHECK(myRegistry->findTestWithGroup("ThisTestGroupDoesntExists") == NULL);
}

TEST(TestRegistry, findTestWithGroup)
{
	test1->setGroupName("GroupOfATestThatDoesExist");
	myRegistry->addTest(test1);
	CHECK(myRegistry->findTestWithGroup("GroupOfATestThatDoesExist"));
}

TEST(TestRegistry, nameFilterWorks)
{
	test1->setTestName("testname");
	test2->setTestName("noname");
	myRegistry->nameFilter("testname");
	addAndRunAllTests();
	CHECK(test1->hasRun_);
	CHECK(!test2->hasRun_);
}

TEST(TestRegistry, groupFilterWorks)
{
	test1->setGroupName("groupname");
	test2->setGroupName("noname");
	myRegistry->groupFilter("groupname");
	addAndRunAllTests();
	CHECK(test1->hasRun_);
	CHECK(!test2->hasRun_);
}

TEST(TestRegistry, runTestInSeperateProcess)
{
	myRegistry->setRunTestsInSeperateProcess();
	myRegistry->addTest(test1);
	myRegistry->runAllTests(*result);
	CHECK(test1->isRunInSeperateProcess());
}

