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
#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestOutput.h"
#include "CppUTest/JUnitTestOutput.h"
#include "CppUTest/TestRegistry.h"

CommandLineTestRunner::CommandLineTestRunner(int ac, const char** av, TestOutput* output) :
	output_(output), jUnitOutput_(new JUnitTestOutput)
{
	arguments_ = new CommandLineArguments(ac, av);
}

CommandLineTestRunner::~CommandLineTestRunner()
{
	delete arguments_;
	delete jUnitOutput_;
}

int CommandLineTestRunner::RunAllTests(int ac, char** av)
{
	return RunAllTests(ac, const_cast<const char**> (av));
}

int CommandLineTestRunner::RunAllTests(int ac, const char** av)
{
	int result = 0;
	ConsoleTestOutput output;

	MemoryLeakWarningPlugin memLeakWarn(DEF_PLUGIN_MEM_LEAK);
	memLeakWarn.destroyGlobalDetectorAndTurnOffMemoryLeakDetectionInDestructor(true);
	TestRegistry::getCurrentRegistry()->installPlugin(&memLeakWarn);

	{
		CommandLineTestRunner runner(ac, av, &output);
		result = runner.runAllTestsMain();
	}

	if (result == 0) {
		output << memLeakWarn.FinalReport(0);
	}
	return result;
}

int CommandLineTestRunner::runAllTestsMain()
{
	int testResult = 0;

	SetPointerPlugin pPlugin(DEF_PLUGIN_SET_POINTER);
	TestRegistry::getCurrentRegistry()->installPlugin(&pPlugin);

	if (!parseArguments(TestRegistry::getCurrentRegistry()->getFirstPlugin())) return 1;

	testResult = runAllTests();

	return testResult;
}

void CommandLineTestRunner::initializeTestRun()
{
	TestRegistry::getCurrentRegistry()->groupFilter(arguments_->getGroupFilter());
	TestRegistry::getCurrentRegistry()->nameFilter(arguments_->getNameFilter());
	if (arguments_->isVerbose()) output_->verbose();
	if (arguments_->runTestsInSeperateProcess()) TestRegistry::getCurrentRegistry()->setRunTestsInSeperateProcess();
}

int CommandLineTestRunner::runAllTests()
{
	initializeTestRun();
	int loopCount = 0;
	int failureCount = 0;
	int repeat_ = arguments_->getRepeatCount();

	while (loopCount++ < repeat_) {
		output_->printTestRun(loopCount, repeat_);
		TestResult tr(*output_);
		TestRegistry::getCurrentRegistry()->runAllTests(tr);
		failureCount += tr.getFailureCount();
	}

	return failureCount;
}

bool CommandLineTestRunner::parseArguments(TestPlugin* plugin)
{
	if (arguments_->parse(plugin)) {
		if (arguments_->isJUnitOutput()) {
			output_ = jUnitOutput_;
		}
		return true;
	}
	else {
		output_->print(arguments_->usage());
		return false;
	}
}

bool CommandLineTestRunner::isVerbose()
{
	return arguments_->isVerbose();
}

int CommandLineTestRunner::getRepeatCount()
{
	return arguments_->getRepeatCount();
}

TestFilter CommandLineTestRunner::getGroupFilter()
{
	return arguments_->getGroupFilter();
}

TestFilter CommandLineTestRunner::getNameFilter()
{
	return arguments_->getNameFilter();
}
