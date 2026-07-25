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

// This file contains the Test class along with the macros which make effective
// in the harness.

#ifndef D_UTest_h
#define D_UTest_h

#include "SimpleString.h"

class TestResult;
class TestPlugin;
class TestFailure;
class TestFilter;

extern bool doubles_equal(double d1, double d2, double threshold);

//////////////////// Utest

class UtestShell;

class Utest
{
public:
	Utest();
	virtual ~Utest();
	virtual void run();

    virtual void setup();
    virtual void teardown();
	virtual void testBody();
};

//////////////////// UtestShell

class UtestShell
{
public:
	UtestShell(const char* groupName, const char* testName, const char* fileName,
			int lineNumber);
	virtual ~UtestShell();

	virtual void runOneTestWithPlugins(TestPlugin* plugin, TestResult& result);
	virtual SimpleString getFormattedName() const;

	virtual UtestShell* addTest(UtestShell* test);
	virtual UtestShell *getNext() const;
	virtual bool isNull() const;
	virtual int countTests();

	bool shouldRun(const TestFilter& groupFilter, const TestFilter& nameFilter) const;
	const SimpleString getName() const;
	const SimpleString getGroup() const;
	const SimpleString getFile() const;
	int getLineNumber() const;
    const virtual char *getProgressIndicator() const;

	static TestResult *getTestResult();
    static UtestShell *getCurrent();

    virtual void assertTrue(bool condition, const char *checkString, const char *conditionString, const char *fileName, int lineNumber);
    virtual void assertCstrEqual(const char *expected, const char *actual, const char *fileName, int lineNumber);
    virtual void assertCstrNoCaseEqual(const char *expected, const char *actual, const char *fileName, int lineNumber);
    virtual void assertCstrContains(const char *expected, const char *actual, const char *fileName, int lineNumber);
    virtual void assertCstrNoCaseContains(const char *expected, const char *actual, const char *fileName, int lineNumber);
    virtual void assertLongsEqual(long  expected, long  actual, const char *fileName, int lineNumber);
    virtual void assertPointersEqual(const void *expected, const void *actual, const char *fileName, int lineNumber);
    virtual void assertDoublesEqual(double expected, double actual, double threshold, const char *fileName, int lineNumber);
    virtual void fail(const char *text, const char *fileName, int lineNumber);

    virtual void print(const char *text, const char *fileName, int lineNumber);
    virtual void print(const SimpleString & text, const char *fileName, int lineNumber);

    void setFileName(const char *fileName);
    void setLineNumber(int lineNumber);
    void setGroupName(const char *groupName);
    void setTestName(const char *testName);

    virtual void exitCurrentTest();

    static void crash();
    static void setCrashMethod(void (*crashme)());
    static void resetCrashMethod();

    virtual bool isRunInSeperateProcess() const;
    virtual void setRunInSeperateProcess();

    virtual Utest* createTest();
    virtual void destroyTest(Utest* test);

    virtual void runOneTest(TestPlugin *plugin, TestResult & result);
protected:
    UtestShell();
    UtestShell(const char *groupName, const char *testName, const char *fileName, int lineNumber, UtestShell *nextTest);

    virtual SimpleString getMacroName() const;
private:
    const char *group_;
    const char *name_;
    const char *file_;
    int lineNumber_;
    UtestShell *next_;
    bool isRunAsSeperateProcess_;

    void setTestResult(TestResult* result);
	void setCurrentTest(UtestShell* test);

	static UtestShell* currentTest_;
	static TestResult* testResult_;

    void failWith(const TestFailure& failure);
};

//////////////////// NullTest

class NullTestShell: public UtestShell
{
public:
	explicit NullTestShell();
	explicit NullTestShell(const char* fileName, int lineNumber);
	virtual ~NullTestShell();

	void testBody();

	static NullTestShell& instance();

	virtual int countTests();
	virtual UtestShell*getNext() const;
	virtual bool isNull() const;
private:

	NullTestShell(const NullTestShell&);
	NullTestShell& operator=(const NullTestShell&);

};


//////////////////// ExecFunctionTest

class ExecFunctionTestShell;

class ExecFunctionTest : public Utest
{
public:
	ExecFunctionTest(ExecFunctionTestShell* shell);
	void testBody();
	virtual void setup();
	virtual void teardown();
private:
	ExecFunctionTestShell* shell_;
};

//////////////////// ExecFunctionTestShell

class ExecFunctionTestShell: public UtestShell
{
public:
	void (*setup_)();
	void (*teardown_)();
	void (*testFunction_)();

	ExecFunctionTestShell(void(*set)() = 0, void(*tear)() = 0) :
		UtestShell("Generic", "Generic", "Generic", 1), setup_(set), teardown_(
				tear), testFunction_(0)
	{
	}
	Utest* createTest() { return new ExecFunctionTest(this); };
};

//////////////////// CppUTestFailedException

class CppUTestFailedException
{
public:
	int dummy_;
};

//////////////////// IgnoredTest

class IgnoredUtestShell : public UtestShell
{
public:
	IgnoredUtestShell();
	virtual ~IgnoredUtestShell();
	explicit IgnoredUtestShell(const char* groupName, const char* testName,
			const char* fileName, int lineNumber);
	virtual const char* getProgressIndicator() const;
	protected:  virtual SimpleString getMacroName() const;
    virtual void runOneTestWithPlugins(TestPlugin* plugin, TestResult& result);

private:

	IgnoredUtestShell(const IgnoredUtestShell&);
	IgnoredUtestShell& operator=(const IgnoredUtestShell&);

};

//////////////////// TestInstaller

class TestInstaller
{
public:
	explicit TestInstaller(UtestShell& shell, const char* groupName, const char* testName,
			const char* fileName, int lineNumber);
	virtual ~TestInstaller();

	void unDo();

private:

	TestInstaller(const TestInstaller&);
	TestInstaller& operator=(const TestInstaller&);

};

#endif
