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
#include "CppUTestExt/MemoryReporterPlugin.h"
#include "CppUTestExt/MemoryReportFormatter.h"
#include "CppUTestExt/CodeMemoryReportFormatter.h"

MemoryReporterPlugin::MemoryReporterPlugin()
	: TestPlugin("MemoryReporterPlugin"), formatter_(NULL)
{
}

MemoryReporterPlugin::~MemoryReporterPlugin()
{
	removeGlobalMemoryReportAllocators();
	destroyMemoryFormatter(formatter_);
}

bool MemoryReporterPlugin::parseArguments(int /* ac */, const char** av, int index)
{
	SimpleString argument (av[index]);
	if (argument.contains("-pmemoryreport=")) {
		argument.replace("-pmemoryreport=", "");

		destroyMemoryFormatter(formatter_);
		formatter_ = createMemoryFormatter(argument);
		return true;
	}
	return false;
}

MemoryReportFormatter* MemoryReporterPlugin::createMemoryFormatter(const SimpleString& type)
{
	if (type == "normal") {
		return  new NormalMemoryReportFormatter;
	}
	else if (type == "code") {
		return new CodeMemoryReportFormatter(defaultMallocAllocator());
	}
	return NULL;
}

void MemoryReporterPlugin::destroyMemoryFormatter(MemoryReportFormatter* formatter)
{
	delete formatter;
}


void MemoryReporterPlugin::setGlobalMemoryReportAllocators()
{
    mallocAllocator.setRealAllocator(getCurrentMallocAllocator());
	setCurrentMallocAllocator(&mallocAllocator);

	newAllocator.setRealAllocator(getCurrentNewAllocator());
	setCurrentNewAllocator(&newAllocator);

	newArrayAllocator.setRealAllocator(getCurrentNewArrayAllocator());
	setCurrentNewArrayAllocator(&newArrayAllocator);
}

void MemoryReporterPlugin::removeGlobalMemoryReportAllocators()
{
	if (getCurrentNewAllocator() == &newAllocator)
		setCurrentNewAllocator(newAllocator.getRealAllocator());

	if (getCurrentNewArrayAllocator() == &newArrayAllocator)
		setCurrentNewArrayAllocator(newArrayAllocator.getRealAllocator());

	if (getCurrentMallocAllocator() == &mallocAllocator)
		setCurrentMallocAllocator(mallocAllocator.getRealAllocator());
}


void MemoryReporterPlugin::initializeAllocator(MemoryReportAllocator* allocator, TestResult & result)
{
	allocator->setFormatter(formatter_);
	allocator->setTestResult((&result));
}

void MemoryReporterPlugin::preTestAction(UtestShell& test, TestResult& result)
{
	if (formatter_ == NULL) return;

	initializeAllocator(&mallocAllocator, result);
	initializeAllocator(&newAllocator, result);
	initializeAllocator(&newArrayAllocator, result);

	setGlobalMemoryReportAllocators();

	if (test.getGroup() != currentTestGroup_) {
		formatter_->report_testgroup_start(&result, test);
		currentTestGroup_ = test.getGroup();
	}

	formatter_->report_test_start(&result, test);
}

void MemoryReporterPlugin::postTestAction(UtestShell& test, TestResult& result)
{
	if (formatter_ == NULL) return;

	removeGlobalMemoryReportAllocators();
	formatter_->report_test_end(&result, test);

	if (test.getNext()->getGroup() != currentTestGroup_)
		formatter_->report_testgroup_end(&result, test);
}
