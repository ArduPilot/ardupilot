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
#include "CppUTest/PlatformSpecificFunctions.h"
#include "CppUTest/TestOutput.h"

bool doubles_equal(double d1, double d2, double threshold)
{
	if (PlatformSpecificIsNan(d1) || PlatformSpecificIsNan(d2) || PlatformSpecificIsNan(threshold))
		return false;
	return PlatformSpecificFabs(d1 - d2) <= threshold;
}

/* Sometimes stubs use the CppUTest assertions.
 * Its not correct to do so, but this small helper class will prevent a segmentation fault and instead
 * will give an error message and also the file/line of the check that was executed outside the tests.
 */
class OutsideTestRunnerUTest: public UtestShell
{
public:
	static OutsideTestRunnerUTest& instance()
	{
		static OutsideTestRunnerUTest instance_;
		return instance_;
	}
	virtual TestResult& getTestResult()
	{
		return defaultTestResult;
	}
	virtual void exitCurrentTest()
	{
	}
	virtual ~OutsideTestRunnerUTest()
	{
	}
private:
	OutsideTestRunnerUTest() :
		UtestShell("\n\t NOTE: Assertion happened without being in a test run (perhaps in main?)", "\n\t       Something is very wrong. Check this assertion and fix", "unknown file", 0),
				defaultTestResult(defaultOutput)
	{
	}
	ConsoleTestOutput defaultOutput;
	TestResult defaultTestResult;
};


UtestShell::UtestShell() :
	group_("UndefinedTestGroup"), name_("UndefinedTest"), file_("UndefinedFile"), lineNumber_(0), next_(&NullTestShell::instance()), isRunAsSeperateProcess_(false)
{
}

UtestShell::UtestShell(const char* groupName, const char* testName, const char* fileName, int lineNumber) :
		group_(groupName), name_(testName), file_(fileName), lineNumber_(lineNumber), next_(&NullTestShell::instance()), isRunAsSeperateProcess_(false)
{
}

UtestShell::UtestShell(const char* groupName, const char* testName, const char* fileName, int lineNumber, UtestShell* nextTest) :
	group_(groupName), name_(testName), file_(fileName), lineNumber_(lineNumber), next_(nextTest), isRunAsSeperateProcess_(false)
{
}

UtestShell::~UtestShell()
{
}

static void defaultCrashMethod()
{
	UtestShell* ptr = (UtestShell*) 0x0; ptr->countTests();
}

static void (*pleaseCrashMeRightNow) () = defaultCrashMethod;

void UtestShell::setCrashMethod(void (*crashme)())
{
	pleaseCrashMeRightNow = crashme;
}

void UtestShell::resetCrashMethod()
{
	pleaseCrashMeRightNow = defaultCrashMethod;
}

void UtestShell::crash()
{
	pleaseCrashMeRightNow();
}

void UtestShell::runOneTestWithPlugins(TestPlugin* plugin, TestResult& result)
{
	executePlatformSpecificRunOneTest(this, plugin, result);
}

Utest* UtestShell::createTest()
{
	return new Utest();
}

void UtestShell::destroyTest(Utest* test)
{
	delete test;
}

void UtestShell::runOneTest(TestPlugin* plugin, TestResult& result)
{
	plugin->runAllPreTestAction(*this, result);

	//save test context, so that test class can be tested
	UtestShell* savedTest = UtestShell::getCurrent();
	TestResult* savedResult = UtestShell::getTestResult();

	result.countRun();
	UtestShell::setTestResult(&result);
	UtestShell::setCurrentTest(this);

	Utest* testToRun = createTest();
	testToRun->run();
	destroyTest(testToRun);

	UtestShell::setCurrentTest(savedTest);
	UtestShell::setTestResult(savedResult);

	plugin->runAllPostTestAction(*this, result);
}

void UtestShell::exitCurrentTest()
{
	executePlatformSpecificExitCurrentTest();
}

UtestShell *UtestShell::getNext() const
{
	return next_;
}

UtestShell* UtestShell::addTest(UtestShell *test)
{
	next_ = test;
	return this;
}

int UtestShell::countTests()
{
	return next_->countTests() + 1;
}

bool UtestShell::isNull() const
{
	return false;
}

SimpleString UtestShell::getMacroName() const
{
	return "TEST";
}

const SimpleString UtestShell::getName() const
{
	return SimpleString(name_);
}

const SimpleString UtestShell::getGroup() const
{
	return SimpleString(group_);
}

SimpleString UtestShell::getFormattedName() const
{
	SimpleString formattedName(getMacroName());
	formattedName += "(";
	formattedName += group_;
	formattedName += ", ";
	formattedName += name_;
	formattedName += ")";

	return formattedName;
}

const char* UtestShell::getProgressIndicator() const
{
	return ".";
}

bool UtestShell::isRunInSeperateProcess() const
{
	return isRunAsSeperateProcess_;
}

void UtestShell::setRunInSeperateProcess()
{
	isRunAsSeperateProcess_ = true;
}


void UtestShell::setFileName(const char* fileName)
{
	file_ = fileName;
}

void UtestShell::setLineNumber(int lineNumber)
{
	lineNumber_ = lineNumber;
}

void UtestShell::setGroupName(const char* groupName)
{
	group_ = groupName;
}

void UtestShell::setTestName(const char* testName)
{
	name_ = testName;
}

const SimpleString UtestShell::getFile() const
{
	return SimpleString(file_);
}

int UtestShell::getLineNumber() const
{
	return lineNumber_;
}

bool UtestShell::shouldRun(const TestFilter& groupFilter, const TestFilter& nameFilter) const
{
	if (groupFilter.match(group_) && nameFilter.match(name_)) return true;

	return false;
}

void UtestShell::failWith(const TestFailure& failure)
{
	getTestResult()->addFailure(failure);
    UtestShell::getCurrent()->exitCurrentTest();
}

void UtestShell::assertTrue(bool condition, const char * checkString, const char* conditionString, const char* fileName, int lineNumber)
{
	getTestResult()->countCheck();
	if (!condition)
		failWith(CheckFailure(this, fileName, lineNumber, checkString, conditionString));
}

void UtestShell::fail(const char *text, const char* fileName, int lineNumber)
{
	failWith(FailFailure(this, fileName, lineNumber, text));
}

void UtestShell::assertCstrEqual(const char* expected, const char* actual, const char* fileName, int lineNumber)
{
	getTestResult()->countCheck();
	if (actual == 0 && expected == 0) return;
	if (actual == 0 || expected == 0)
		failWith(StringEqualFailure(this, fileName, lineNumber, expected, actual));
	if (PlatformSpecificStrCmp(expected, actual) != 0)
		failWith(StringEqualFailure(this, fileName, lineNumber, expected, actual));
}

void UtestShell::assertCstrNoCaseEqual(const char* expected, const char* actual, const char* fileName, int lineNumber)
{
	getTestResult()->countCheck();
	if (actual == 0 && expected == 0) return;
	if (actual == 0 || expected == 0)
		failWith(StringEqualNoCaseFailure(this, fileName, lineNumber, expected, actual));
	if (!SimpleString(expected).equalsNoCase(actual))
		failWith(StringEqualNoCaseFailure(this, fileName, lineNumber, expected, actual));
}

void UtestShell::assertCstrContains(const char* expected, const char* actual, const char* fileName, int lineNumber)
{
	getTestResult()->countCheck();
	if (actual == 0 && expected == 0) return;
    if(actual == 0 || expected == 0)
    	failWith(ContainsFailure(this, fileName, lineNumber, expected, actual));
    if (!SimpleString(actual).contains(expected))
    	failWith(ContainsFailure(this, fileName, lineNumber, expected, actual));
}

void UtestShell::assertCstrNoCaseContains(const char* expected, const char* actual, const char* fileName, int lineNumber)
{
	getTestResult()->countCheck();
	if (actual == 0 && expected == 0) return;
    if(actual == 0 || expected == 0)
    	failWith(ContainsFailure(this, fileName, lineNumber, expected, actual));
    if (!SimpleString(actual).containsNoCase(expected))
    	failWith(ContainsFailure(this, fileName, lineNumber, expected, actual));
}

void UtestShell::assertLongsEqual(long expected, long actual, const char* fileName, int lineNumber)
{
	getTestResult()->countCheck();
	if (expected != actual) {
		LongsEqualFailure f(this, fileName, lineNumber, expected, actual);
		getTestResult()->addFailure(f);
	    UtestShell::getCurrent()->exitCurrentTest();
	}
}

void UtestShell::assertPointersEqual(const void* expected, const void* actual, const char* fileName, int lineNumber)
{
	getTestResult()->countCheck();
	if (expected != actual)
		failWith(EqualsFailure(this, fileName, lineNumber, StringFrom(expected), StringFrom(actual)));
}

void UtestShell::assertDoublesEqual(double expected, double actual, double threshold, const char* fileName, int lineNumber)
{
	getTestResult()->countCheck();
	if (!doubles_equal(expected, actual, threshold))
		failWith(DoublesEqualFailure(this, fileName, lineNumber, expected, actual, threshold));
}

void UtestShell::print(const char *text, const char* fileName, int lineNumber)
{
	SimpleString stringToPrint = "\n";
	stringToPrint += fileName;
	stringToPrint += ":";
	stringToPrint += StringFrom(lineNumber);
	stringToPrint += " ";
	stringToPrint += text;
	getTestResult()->print(stringToPrint.asCharString());
}

void UtestShell::print(const SimpleString& text, const char* fileName, int lineNumber)
{
	print(text.asCharString(), fileName, lineNumber);
}

TestResult* UtestShell::testResult_ = NULL;
UtestShell* UtestShell::currentTest_ = NULL;

void UtestShell::setTestResult(TestResult* result)
{
	testResult_ = result;
}

void UtestShell::setCurrentTest(UtestShell* test)
{
	currentTest_ = test;
}

TestResult* UtestShell::getTestResult()
{
	if (testResult_ == NULL)
		return &OutsideTestRunnerUTest::instance().getTestResult();
	return testResult_;
}

UtestShell* UtestShell::getCurrent()
{
	if (currentTest_ == NULL)
		return &OutsideTestRunnerUTest::instance();
	return currentTest_;
}

////////////// Utest ////////////

Utest::Utest()
{
}

Utest::~Utest()
{
}

void Utest::run()
{

#if CPPUTEST_USE_STD_CPP_LIB
	try {
#endif
		if (executePlatformSpecificSetup(this)) {
			executePlatformSpecificTestBody(this);
		}
		executePlatformSpecificTeardown(this);

#if CPPUTEST_USE_STD_CPP_LIB
	}
	catch (CppUTestFailedException&)
	{
		try {
			executePlatformSpecificTeardown(this);
		}
		catch (CppUTestFailedException&)
		{
		}
	}
#endif
}

void Utest::setup()
{
}

void Utest::testBody()
{
}

void Utest::teardown()
{
}


////////////// NullTestShell ////////////


NullTestShell::NullTestShell() :
	UtestShell("NullGroup", "NullName", "NullFile", -1, 0)
{
}

NullTestShell::NullTestShell(const char* fileName, int lineNumber) :
	UtestShell("NullGroup", "NullName", fileName, lineNumber, 0)
{
}

NullTestShell::~NullTestShell()
{
}

NullTestShell& NullTestShell::instance()
{
	static NullTestShell _instance;
	return _instance;
}

int NullTestShell::countTests()
{
	return 0;
}

UtestShell* NullTestShell::getNext() const
{
	return &instance();
}

bool NullTestShell::isNull() const
{
	return true;
}

void NullTestShell::testBody()
{
}

//////////////////// ExecFunctionTest

ExecFunctionTest::ExecFunctionTest(ExecFunctionTestShell* shell)
	: shell_(shell)
{
}

void ExecFunctionTest::testBody()
{
	if (shell_->testFunction_) shell_->testFunction_();
}

void ExecFunctionTest::setup()
{
	if (shell_->setup_) shell_->setup_();
}

void ExecFunctionTest::teardown()
{
	if (shell_->teardown_) shell_->teardown_();
}

/////////////// IgnoredUtestShell /////////////
IgnoredUtestShell::IgnoredUtestShell()
{
}

IgnoredUtestShell::~IgnoredUtestShell()
{
}

const char* IgnoredUtestShell::getProgressIndicator() const
{
	return "!";
}

SimpleString IgnoredUtestShell::getMacroName() const
{
	return "IGNORE_TEST";
}

void IgnoredUtestShell::runOneTestWithPlugins(TestPlugin* /* plugin */, TestResult& result)
{
	result.countIgnored();
}


////////////// TestInstaller ////////////

TestInstaller::TestInstaller(UtestShell& shell, const char* groupName, const char* testName, const char* fileName, int lineNumber)
{
	shell.setGroupName(groupName);
	shell.setTestName(testName);
	shell.setFileName(fileName);
	shell.setLineNumber(lineNumber);
	TestRegistry::getCurrentRegistry()->addTest(&shell);
}

TestInstaller::~TestInstaller()
{
}

void TestInstaller::unDo()
{
	TestRegistry::getCurrentRegistry()->unDoLastAddTest();
}
