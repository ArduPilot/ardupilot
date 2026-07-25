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
#include "CppUTest/MemoryLeakWarningPlugin.h"
#include "CppUTest/MemoryLeakDetector.h"
#include "CppUTest/TestMemoryAllocator.h"
#include "CppUTest/TestTestingFixture.h"

static char* leak1;
static long* leak2;

class DummyReporter: public MemoryLeakFailure
{
public:
	virtual ~DummyReporter()
	{
	}
	virtual void fail(char* /*fail_string*/)
	{
	}
};

static MemoryLeakDetector* detector;
static MemoryLeakWarningPlugin* memPlugin;
static DummyReporter dummy;
static TestMemoryAllocator* allocator;

TEST_GROUP(MemoryLeakWarningTest)
{
	TestTestingFixture* fixture;

	void setup()
	{
		fixture = new TestTestingFixture();
		detector = new MemoryLeakDetector(&dummy);
		allocator = new TestMemoryAllocator;
		memPlugin = new MemoryLeakWarningPlugin("TestMemoryLeakWarningPlugin", detector);
		fixture->registry_->installPlugin(memPlugin);
		memPlugin->enable();

		leak1 = 0;
		leak2 = 0;
	}
	void teardown()
	{
		detector->deallocMemory(allocator, leak1);
		detector->deallocMemory(allocator, leak2);

		delete fixture;
		delete memPlugin;
		delete detector;
		delete allocator;
	}
};

void _testTwoLeaks()
{
	leak1 = detector->allocMemory(allocator, 10);
	leak2 = (long*) detector->allocMemory(allocator, 4);
}

TEST(MemoryLeakWarningTest, TwoLeaks)
{
	fixture->setTestFunction(_testTwoLeaks);
	fixture->runAllTests();
	LONGS_EQUAL(1, fixture->getFailureCount());
	fixture->assertPrintContains("Total number of leaks:  2");
}

void _testIgnore2()
{
	memPlugin->expectLeaksInTest(2);
	leak1 = detector->allocMemory(allocator, 10);
	leak2 = (long*) detector->allocMemory(allocator, 4);
}

TEST(MemoryLeakWarningTest, Ignore2)
{
	fixture->setTestFunction(_testIgnore2);
	fixture->runAllTests();
	LONGS_EQUAL(0, fixture->getFailureCount());
}

static void _failAndLeakMemory()
{
	leak1 = detector->allocMemory(allocator, 10);
	FAIL("");
}

TEST(MemoryLeakWarningTest, FailingTestDoesNotReportMemoryLeaks)
{
	fixture->setTestFunction(_failAndLeakMemory);
	fixture->runAllTests();
	LONGS_EQUAL(1, fixture->getFailureCount());
}

bool memoryLeakDetectorWasDeleted = false;
bool memoryLeakFailureWasDelete = false;

class DummyMemoryLeakDetector : public MemoryLeakDetector
{
public:
	DummyMemoryLeakDetector(MemoryLeakFailure* reporter) : MemoryLeakDetector(reporter) {}
	virtual ~DummyMemoryLeakDetector()
	{
		memoryLeakDetectorWasDeleted = true;
	}
};

class DummyMemoryLeakFailure : public MemoryLeakFailure
{
	virtual ~DummyMemoryLeakFailure()
	{
		memoryLeakFailureWasDelete = true;
	}
	virtual void fail(char*)
	{
	}
};


static bool cpputestHasCrashed;

TEST_GROUP(MemoryLeakWarningGlobalDetectorTest)
{
	MemoryLeakDetector* detector;
	MemoryLeakFailure* failureReporter;

	DummyMemoryLeakDetector * dummyDetector;
	MemoryLeakFailure* dummyReporter;

	static void crashMethod()
	{
		cpputestHasCrashed = true;
	}

	void setup()
	{
		detector = MemoryLeakWarningPlugin::getGlobalDetector();
		failureReporter = MemoryLeakWarningPlugin::getGlobalFailureReporter();

		memoryLeakDetectorWasDeleted = false;
		memoryLeakFailureWasDelete = false;

		MemoryLeakWarningPlugin::turnOffNewDeleteOverloads();

		dummyReporter = new DummyMemoryLeakFailure;
		dummyDetector = new DummyMemoryLeakDetector(dummyReporter);

		UtestShell::setCrashMethod(crashMethod);
		cpputestHasCrashed = false;
}

	void teardown()
	{
		if (!memoryLeakDetectorWasDeleted) delete dummyDetector;
		if (!memoryLeakFailureWasDelete) delete dummyReporter;

		MemoryLeakWarningPlugin::setGlobalDetector(detector, failureReporter);
		MemoryLeakWarningPlugin::turnOnNewDeleteOverloads();

		UtestShell::resetCrashMethod();

		setCurrentMallocAllocatorToDefault();
		setCurrentNewAllocatorToDefault();
		setCurrentNewArrayAllocatorToDefault();
	}
};

TEST(MemoryLeakWarningGlobalDetectorTest, turnOffNewOverloadsCausesNoAdditionalLeaks)
{
	int storedAmountOfLeaks = detector->totalMemoryLeaks(mem_leak_period_all);

	char* arrayMemory = new char[100];
	char* nonArrayMemory = new char;

	LONGS_EQUAL(storedAmountOfLeaks, detector->totalMemoryLeaks(mem_leak_period_all));

	delete [] arrayMemory;
	delete nonArrayMemory;
}

TEST(MemoryLeakWarningGlobalDetectorTest, destroyGlobalDetector)
{
	MemoryLeakWarningPlugin::setGlobalDetector(dummyDetector, dummyReporter);

	MemoryLeakWarningPlugin::destroyGlobalDetector();

	CHECK(memoryLeakDetectorWasDeleted);
	CHECK(memoryLeakFailureWasDelete);
}

TEST(MemoryLeakWarningGlobalDetectorTest, MemoryWarningPluginCanBeSetToDestroyTheGlobalDetector)
{
	MemoryLeakWarningPlugin* plugin = new MemoryLeakWarningPlugin("dummy");
	plugin->destroyGlobalDetectorAndTurnOffMemoryLeakDetectionInDestructor(true);
	MemoryLeakWarningPlugin::setGlobalDetector(dummyDetector, dummyReporter);

	delete plugin;

	CHECK(memoryLeakDetectorWasDeleted);
}

#ifndef CPPUTEST_MEM_LEAK_DETECTION_DISABLED

TEST(MemoryLeakWarningGlobalDetectorTest, crashOnLeakWithOperatorNew)
{
	MemoryLeakWarningPlugin::setGlobalDetector(dummyDetector, dummyReporter);

	MemoryLeakWarningPlugin::turnOnNewDeleteOverloads();

	crash_on_allocation_number(1);
	char* memory = new char[100];
	CHECK(cpputestHasCrashed);
	delete[] memory;

	MemoryLeakWarningPlugin::turnOffNewDeleteOverloads();

}

TEST(MemoryLeakWarningGlobalDetectorTest, crashOnLeakWithOperatorNewArray)
{
	MemoryLeakWarningPlugin::setGlobalDetector(dummyDetector, dummyReporter);

	MemoryLeakWarningPlugin::turnOnNewDeleteOverloads();

	crash_on_allocation_number(1);
	char* memory = new char;
	CHECK(cpputestHasCrashed);
	delete memory;

	MemoryLeakWarningPlugin::turnOffNewDeleteOverloads();
}

#endif

TEST(MemoryLeakWarningGlobalDetectorTest, crashOnLeakWithOperatorMalloc)
{
	MemoryLeakWarningPlugin::setGlobalDetector(dummyDetector, dummyReporter);

	crash_on_allocation_number(1);
	char* memory = (char*) cpputest_malloc(10);
	CHECK(cpputestHasCrashed);
	cpputest_free(memory);

	MemoryLeakWarningPlugin::turnOffNewDeleteOverloads();
}

