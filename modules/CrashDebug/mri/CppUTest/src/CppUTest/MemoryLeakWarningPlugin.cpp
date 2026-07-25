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
#include "CppUTest/MemoryLeakWarningPlugin.h"
#include "CppUTest/MemoryLeakDetector.h"
#include "CppUTest/TestMemoryAllocator.h"
#include "CppUTest/PlatformSpecificFunctions.h"

#if CPPUTEST_USE_MEM_LEAK_DETECTION
#undef new

#if CPPUTEST_USE_STD_CPP_LIB
#define UT_THROW_BAD_ALLOC_WHEN_NULL(memory) if (memory == NULL) throw std::bad_alloc();
#define UT_THROW(except) throw (except)
#define UT_THROW_EMPTY() throw ()
#else
#define UT_THROW_BAD_ALLOC_WHEN_NULL(memory)
#define UT_THROW(except)
#define UT_THROW_EMPTY()
#endif

static void* mem_leak_operator_new (size_t size) UT_THROW(std::bad_alloc)
{
	void* memory = MemoryLeakWarningPlugin::getGlobalDetector()->allocMemory(getCurrentNewAllocator(), size);
	UT_THROW_BAD_ALLOC_WHEN_NULL(memory);
	return memory;
}

static void* mem_leak_operator_new_debug (size_t size, const char* file, int line) UT_THROW(std::bad_alloc)
{
	void *memory = MemoryLeakWarningPlugin::getGlobalDetector()->allocMemory(getCurrentNewAllocator(), size, (char*) file, line);
	UT_THROW_BAD_ALLOC_WHEN_NULL(memory);
	return memory;
}

static void* mem_leak_operator_new_array (size_t size) UT_THROW(std::bad_alloc)
{
	void* memory = MemoryLeakWarningPlugin::getGlobalDetector()->allocMemory(getCurrentNewArrayAllocator(), size);
	UT_THROW_BAD_ALLOC_WHEN_NULL(memory);
	return memory;
}

static void* mem_leak_operator_new_array_debug (size_t size, const char* file, int line) UT_THROW(std::bad_alloc)
{
	void* memory = MemoryLeakWarningPlugin::getGlobalDetector()->allocMemory(getCurrentNewArrayAllocator(), size, (char*) file, line);
	UT_THROW_BAD_ALLOC_WHEN_NULL(memory);
	return memory;
}

static void mem_leak_operator_delete (void* mem) UT_THROW_EMPTY()
{
	MemoryLeakWarningPlugin::getGlobalDetector()->invalidateMemory((char*) mem);
	MemoryLeakWarningPlugin::getGlobalDetector()->deallocMemory(getCurrentNewAllocator(), (char*) mem);
}

static void mem_leak_operator_delete_array (void* mem) UT_THROW_EMPTY()
{
	MemoryLeakWarningPlugin::getGlobalDetector()->invalidateMemory((char*) mem);
	MemoryLeakWarningPlugin::getGlobalDetector()->deallocMemory(getCurrentNewArrayAllocator(), (char*) mem);
}

static void* normal_operator_new (size_t size) UT_THROW(std::bad_alloc)
{
	return PlatformSpecificMalloc(size);
}

static void* normal_operator_new_debug (size_t size, const char* /*file*/, int /*line*/) UT_THROW(std::bad_alloc)
{
	return PlatformSpecificMalloc(size);
}

static void* normal_operator_new_array (size_t size) UT_THROW(std::bad_alloc)
{
	return PlatformSpecificMalloc(size);
}

static void* normal_operator_new_array_debug (size_t size, const char* /*file*/, int /*line*/) UT_THROW(std::bad_alloc)
{
	return PlatformSpecificMalloc(size);
}

static void normal_operator_delete (void* mem) UT_THROW_EMPTY()
{
	PlatformSpecificFree(mem);
}

static void normal_operator_delete_array (void* mem) UT_THROW_EMPTY()
{
	PlatformSpecificFree(mem);
}

static void *(*operator_new_fptr)(size_t size) UT_THROW(std::bad_alloc) = mem_leak_operator_new;
static void *(*operator_new_debug_fptr)(size_t size, const char* file, int line) UT_THROW(std::bad_alloc) = mem_leak_operator_new_debug;
static void *(*operator_new_array_fptr)(size_t size) UT_THROW(std::bad_alloc) = mem_leak_operator_new_array;
static void *(*operator_new_array_debug_fptr)(size_t size, const char* file, int line) UT_THROW(std::bad_alloc) = mem_leak_operator_new_array_debug;
static void (*operator_delete_fptr)(void* mem) UT_THROW_EMPTY() = mem_leak_operator_delete;
static void (*operator_delete_array_fptr)(void* mem) UT_THROW_EMPTY() = mem_leak_operator_delete_array;

void* operator new(size_t size) UT_THROW(std::bad_alloc)
{
	return operator_new_fptr(size);
}

void* operator new(size_t size, const char* file, int line) UT_THROW(std::bad_alloc)
{
	return operator_new_debug_fptr(size, file, line);
}

void operator delete(void* mem) UT_THROW_EMPTY()
{
	operator_delete_fptr(mem);
}

void* operator new[](size_t size) UT_THROW(std::bad_alloc)
{
	return operator_new_array_fptr(size);
}

void* operator new [](size_t size, const char* file, int line) UT_THROW(std::bad_alloc)
{
	return operator_new_array_debug_fptr(size, file, line);
}

void operator delete[](void* mem) UT_THROW_EMPTY()
{
	 operator_delete_array_fptr(mem);
}

#endif

void MemoryLeakWarningPlugin::turnOffNewDeleteOverloads()
{
#if CPPUTEST_USE_MEM_LEAK_DETECTION
	operator_new_fptr = normal_operator_new;
	operator_new_debug_fptr = normal_operator_new_debug;
	operator_new_array_fptr = normal_operator_new_array;
	operator_new_array_debug_fptr = normal_operator_new_array_debug;
	operator_delete_fptr = normal_operator_delete;
	operator_delete_array_fptr = normal_operator_delete_array;
#endif
}

void MemoryLeakWarningPlugin::turnOnNewDeleteOverloads()
{
#if CPPUTEST_USE_MEM_LEAK_DETECTION
	operator_new_fptr = mem_leak_operator_new;
	operator_new_debug_fptr = mem_leak_operator_new_debug;
	operator_new_array_fptr = mem_leak_operator_new_array;
	operator_new_array_debug_fptr = mem_leak_operator_new_array_debug;
	operator_delete_fptr = mem_leak_operator_delete;
	operator_delete_array_fptr = mem_leak_operator_delete_array;
#endif
}

void crash_on_allocation_number(unsigned alloc_number)
{
	static CrashOnAllocationAllocator crashAllocator;
	crashAllocator.setNumberToCrashOn(alloc_number);
	setCurrentMallocAllocator(&crashAllocator);
	setCurrentNewAllocator(&crashAllocator);
	setCurrentNewArrayAllocator(&crashAllocator);
}

class MemoryLeakWarningReporter: public MemoryLeakFailure
{
public:
	virtual ~MemoryLeakWarningReporter()
	{
	}

	virtual void fail(char* fail_string)
	{
		FAIL(fail_string);
	}
};

static MemoryLeakFailure* globalReporter = 0;
static MemoryLeakDetector* globalDetector = 0;

MemoryLeakDetector* MemoryLeakWarningPlugin::getGlobalDetector()
{
	if (globalDetector == 0) {
		turnOffNewDeleteOverloads();

		globalReporter = new MemoryLeakWarningReporter;
		globalDetector = new MemoryLeakDetector(globalReporter);

		turnOnNewDeleteOverloads();
	}
	return globalDetector;
}

MemoryLeakFailure* MemoryLeakWarningPlugin::getGlobalFailureReporter()
{
	return globalReporter;
}

void MemoryLeakWarningPlugin::destroyGlobalDetectorAndTurnOffMemoryLeakDetectionInDestructor(bool des)
{
	destroyGlobalDetectorAndTurnOfMemoryLeakDetectionInDestructor_ = des;
}

void MemoryLeakWarningPlugin::setGlobalDetector(MemoryLeakDetector* detector, MemoryLeakFailure* reporter)
{
	globalDetector = detector;
	globalReporter = reporter;
}

void MemoryLeakWarningPlugin::destroyGlobalDetector()
{
	turnOffNewDeleteOverloads();
	delete globalDetector;
	delete globalReporter;
	globalDetector = NULL;
}


MemoryLeakWarningPlugin* MemoryLeakWarningPlugin::firstPlugin_ = 0;

MemoryLeakWarningPlugin* MemoryLeakWarningPlugin::getFirstPlugin()
{
	return firstPlugin_;
}

MemoryLeakDetector* MemoryLeakWarningPlugin::getMemoryLeakDetector()
{
	return memLeakDetector_;
}

void MemoryLeakWarningPlugin::ignoreAllLeaksInTest()
{
	ignoreAllWarnings_ = true;
}

void MemoryLeakWarningPlugin::expectLeaksInTest(int n)
{
	expectedLeaks_ = n;
}

MemoryLeakWarningPlugin::MemoryLeakWarningPlugin(const SimpleString& name, MemoryLeakDetector* localDetector) :
	TestPlugin(name), ignoreAllWarnings_(false), destroyGlobalDetectorAndTurnOfMemoryLeakDetectionInDestructor_(false), expectedLeaks_(0)
{
	if (firstPlugin_ == 0) firstPlugin_ = this;

	if (localDetector) memLeakDetector_ = localDetector;
	else memLeakDetector_ = getGlobalDetector();

	memLeakDetector_->enable();
}

MemoryLeakWarningPlugin::~MemoryLeakWarningPlugin()
{
	if (this == firstPlugin_) firstPlugin_ = 0;

	if (destroyGlobalDetectorAndTurnOfMemoryLeakDetectionInDestructor_) {
		MemoryLeakWarningPlugin::turnOffNewDeleteOverloads();
		MemoryLeakWarningPlugin::destroyGlobalDetector();
	}
}

void MemoryLeakWarningPlugin::preTestAction(UtestShell& /*test*/, TestResult& result)
{
	memLeakDetector_->startChecking();
	failureCount_ = result.getFailureCount();
}

void MemoryLeakWarningPlugin::postTestAction(UtestShell& test, TestResult& result)
{
	memLeakDetector_->stopChecking();
	int leaks = memLeakDetector_->totalMemoryLeaks(mem_leak_period_checking);

	if (!ignoreAllWarnings_ && expectedLeaks_ != leaks && failureCount_ == result.getFailureCount()) {
		TestFailure f(&test, memLeakDetector_->report(mem_leak_period_checking));
		result.addFailure(f);
	}
	memLeakDetector_->markCheckingPeriodLeaksAsNonCheckingPeriod();
	ignoreAllWarnings_ = false;
	expectedLeaks_ = 0;
}

const char* MemoryLeakWarningPlugin::FinalReport(int toBeDeletedLeaks)
{
	int leaks = memLeakDetector_->totalMemoryLeaks(mem_leak_period_enabled);
	if (leaks != toBeDeletedLeaks) return memLeakDetector_->report(mem_leak_period_enabled);
	return "";
}
