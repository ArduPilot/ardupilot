/*
 * Copyright (c) 2011, Michael Feathers, James Grenning and Bas Vodde
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

#ifdef CPPUTEST_USE_REAL_GTEST

#undef new

/* Enormous hack!
 *
 * This sucks enormously. We need to do two things in GTest that seem to not be possible without
 * this hack. Hopefully there is *another way*.
 *
 * We need to access the factory in the TestInfo in order to be able to create tests. The factory
 * is private and there seems to be no way to access it...
 *
 * We need to be able to call the Test SetUp and TearDown methods, but they are protected for
 * some reason. We can't subclass either as the tests are created with the TEST macro.
 *
 * If anyone knows how to get the above things done *without* these ugly #defines, let me know!
 *
 */

#define private public
#define protected public

#include "gtest/gtest.h"
#include "gtest/gtest-spi.h"
#include "gtest/gtest-death-test.h"
#include "gmock/gmock.h"

#ifdef ADD_FAILURE_AT
#define GTEST_VERSION_GTEST_1_6
#else
#define GTEST_VERSION_GTEST_1_5
#endif

/*
 * We really need some of its internals as they don't have a public interface.
 *
 */
#define GTEST_IMPLEMENTATION_ 1
#include "src/gtest-internal-inl.h"


#include "CppUTestExt/GTestConvertor.h"
#include "CppUTest/TestRegistry.h"
#include "CppUTest/TestFailure.h"
#include "CppUTest/TestResult.h"

/* Store some of the flags as we'll need to reset them each test to avoid leaking memory */

static ::testing::internal::String GTestFlagcolor;
static ::testing::internal::String GTestFlagfilter;
static ::testing::internal::String GTestFlagoutput;
static ::testing::internal::String GTestFlagdeath_test_style;
static ::testing::internal::String GTestFlaginternal_run_death_test;
#ifndef GTEST_VERSION_GTEST_1_5
static ::testing::internal::String GTestFlagstream_result_to;
#endif

static void storeValuesOfGTestFLags()
{
	GTestFlagcolor = ::testing::GTEST_FLAG(color);
	GTestFlagfilter = ::testing::GTEST_FLAG(filter);
	GTestFlagoutput = ::testing::GTEST_FLAG(output);
	GTestFlagdeath_test_style = ::testing::GTEST_FLAG(death_test_style);
	GTestFlaginternal_run_death_test = ::testing::internal::GTEST_FLAG(internal_run_death_test);
	#ifndef GTEST_VERSION_GTEST_1_5
	GTestFlagstream_result_to = ::testing::GTEST_FLAG(stream_result_to);
	#endif
}

static void resetValuesOfGTestFlags()
{
	::testing::GTEST_FLAG(color) = GTestFlagcolor;
	::testing::GTEST_FLAG(filter) = GTestFlagfilter;
	::testing::GTEST_FLAG(output) = GTestFlagoutput;
	::testing::GTEST_FLAG(death_test_style) = GTestFlagdeath_test_style;
	::testing::internal::GTEST_FLAG(internal_run_death_test) = GTestFlaginternal_run_death_test;
	#ifndef GTEST_VERSION_GTEST_1_5
	::testing::GTEST_FLAG(stream_result_to) = GTestFlagstream_result_to;
	#endif
}

void setGTestFLagValuesToNULLToAvoidMemoryLeaks()
{
	::testing::GTEST_FLAG(color) = NULL;
	::testing::GTEST_FLAG(filter) = NULL;
	::testing::GTEST_FLAG(output) = NULL;
	::testing::GTEST_FLAG(death_test_style) = NULL;
	::testing::internal::GTEST_FLAG(internal_run_death_test) = NULL;
	#ifndef GTEST_VERSION_GTEST_1_5
	::testing::GTEST_FLAG(stream_result_to) = NULL;
	#endif
}

/* Left a global to avoid a header dependency to gtest.h */

class GTestDummyResultReporter : public ::testing::ScopedFakeTestPartResultReporter
{
public:
	GTestDummyResultReporter () : ::testing::ScopedFakeTestPartResultReporter(INTERCEPT_ALL_THREADS, NULL) {}
	virtual void ReportTestPartResult(const ::testing::TestPartResult& /*result*/) {}
};

class GTestResultReporter : public ::testing::ScopedFakeTestPartResultReporter
{
public:
	GTestResultReporter () : ::testing::ScopedFakeTestPartResultReporter(INTERCEPT_ALL_THREADS, NULL) {}

	virtual void ReportTestPartResult(const ::testing::TestPartResult& result)
	{
		FailFailure failure(UtestShell::getCurrent(), result.file_name(), result.line_number(), result.message());
		UtestShell::getCurrent()->getTestResult()->addFailure(failure);

		/*
		 * When using GMock, it throws an exception fromt he destructor leaving
		 * the system in an unstable state.
		 * Therefore, when the test fails because of failed gmock expectation
		 * then don't throw the exception, but let it return. Usually this should
		 * already be at the end of the test, so it doesn't matter much
		 */
		if (!SimpleString(result.message()).contains("Actual: never called") &&
			!SimpleString(result.message()).contains("Actual function call count doesn't match"))
			throw CppUTestFailedException();
	}
};

GTest::GTest(::testing::TestInfo* testinfo, GTest* next) : testinfo_(testinfo), next_(next)
{
	setGroupName(testinfo->test_case_name());
	setTestName(testinfo->name());
}

void GTest::setup()
{
	resetValuesOfGTestFlags();

	#ifdef GTEST_VERSION_GTEST_1_5
	test_ = testinfo_->impl()->factory_->CreateTest();
#else
	test_ = testinfo_->factory_->CreateTest();
#endif

	::testing::UnitTest::GetInstance()->impl()->set_current_test_info(testinfo_);
	try {
		test_->SetUp();
	}
	catch (CppUTestFailedException& ex)
	{
	}
}

void GTest::teardown()
{
	try {
		test_->TearDown();
	}
	catch (CppUTestFailedException& ex)
	{
	}
	::testing::UnitTest::GetInstance()->impl()->set_current_test_info(NULL);
	delete test_;

	setGTestFLagValuesToNULLToAvoidMemoryLeaks();
	::testing::internal::DeathTest::set_last_death_test_message(NULL);
}

void GTest::testBody()
{
	try {
		test_->TestBody();
	}
	catch (CppUTestFailedException& ex)
	{
	}
}

GTest* GTest::nextGTest()
{
	return next_;
}

void GTestConvertor::simulateGTestFailureToPreAllocateAllTheThreadLocalData()
{
	GTestDummyResultReporter *dummyReporter = new GTestDummyResultReporter();
	ASSERT_TRUE(false);
	delete dummyReporter;
}

GTestConvertor::GTestConvertor(bool shouldSimulateFailureAtCreationToAllocateThreadLocalData) : first_(NULL)
{
	if (shouldSimulateFailureAtCreationToAllocateThreadLocalData)
		simulateGTestFailureToPreAllocateAllTheThreadLocalData();
	reporter_ = new GTestResultReporter();
}

GTestConvertor::~GTestConvertor()
{
	delete reporter_;

	while (first_) {
		GTest* next = first_->nextGTest();
		delete first_;
		first_ = next;
	}
}

void GTestConvertor::addNewTestCaseForTestInfo(::testing::TestInfo* testinfo)
{
	first_ = new GTest(testinfo, first_);
	TestRegistry::getCurrentRegistry()->addTest(first_);
}

void GTestConvertor::addAllTestsFromTestCaseToTestRegistry(::testing::TestCase* testcase)
{
	int currentTestCount = 0;
	::testing::TestInfo* currentTest = (::testing::TestInfo*) testcase->GetTestInfo(currentTestCount);
	while (currentTest) {
		addNewTestCaseForTestInfo(currentTest);
		currentTestCount++;
		currentTest = (::testing::TestInfo*) testcase->GetTestInfo(currentTestCount);
	}
}

void GTestConvertor::createDummyInSequenceToAndFailureReporterAvoidMemoryLeakInGMock()
{
#ifdef CPPUTEST_USE_REAL_GMOCK
	::testing::InSequence seq;
	::testing::internal::GetFailureReporter();
#endif
}

void GTestConvertor::addAllGTestToTestRegistry()
{
	createDummyInSequenceToAndFailureReporterAvoidMemoryLeakInGMock();
	storeValuesOfGTestFLags();

#ifdef CPPUTEST_USE_REAL_GMOCK
	int argc = 2;
	const char * argv[] = {"NameOfTheProgram", "--gmock_catch_leaked_mocks=0"};
	::testing::InitGoogleMock(&argc, (char**) argv);
#endif

	::testing::UnitTest* unitTests = ::testing::UnitTest::GetInstance();

	int currentUnitTestCount = 0;
	::testing::TestCase* currentTestCase = (::testing::TestCase*) unitTests->GetTestCase(currentUnitTestCount);
	while (currentTestCase) {
		addAllTestsFromTestCaseToTestRegistry(currentTestCase);
		currentUnitTestCount++;
		currentTestCase = (::testing::TestCase*) unitTests->GetTestCase(currentUnitTestCount);
	}
}
#else

int totallyDummySymbolToNotCauseAnyLinkerWarningsAboutEmptyCompilationUnits = 0;

#endif
