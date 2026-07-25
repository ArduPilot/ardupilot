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

#ifndef D_CommandLineTestRunner_H
#define D_CommandLineTestRunner_H

#include "TestHarness.h"
#include "TestOutput.h"
#include "CommandLineArguments.h"
#include "TestFilter.h"

class JUnitTestOutput;

#define DEF_PLUGIN_MEM_LEAK "MemoryLeakPlugin"
#define DEF_PLUGIN_SET_POINTER "SetPointerPlugin"

class CommandLineTestRunner
{
public:
	enum OutputType
	{
		OUTPUT_NORMAL, OUTPUT_JUNIT
	};

	static int RunAllTests(int ac, const char** av);
	static int RunAllTests(int ac, char** av);
	CommandLineTestRunner(int ac, const char** av, TestOutput*);

	virtual ~CommandLineTestRunner();
	int runAllTestsMain();

private:
	TestOutput* output_;
	JUnitTestOutput* jUnitOutput_;
	CommandLineArguments* arguments_;

	bool parseArguments(TestPlugin*);
	int runAllTests();
	void initializeTestRun();
	bool isVerbose();
	int getRepeatCount();
	TestFilter getGroupFilter();
	TestFilter getNameFilter();
};

#endif
