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

///////////////////////////////////////////////////////////////////////////////
//
// FAILURE.H
//
// Failure is a class which holds information for a specific
// test failure. It can be overriden for more complex failure messages
//
///////////////////////////////////////////////////////////////////////////////


#ifndef D_TestFailure_H
#define D_TestFailure_H

#include "SimpleString.h"

class UtestShell;
class TestOutput;

class TestFailure
{

public:
	TestFailure(UtestShell*, const char* fileName, int lineNumber,
			const SimpleString& theMessage);
	TestFailure(UtestShell*, const SimpleString& theMessage);
	TestFailure(UtestShell*, const char* fileName, int lineNumber);
	TestFailure(const TestFailure&);
	virtual ~TestFailure();

	virtual SimpleString getFileName() const;
	virtual SimpleString getTestName() const;
	virtual int getFailureLineNumber() const;
	virtual SimpleString getMessage() const;
	virtual SimpleString getTestFileName() const;
	virtual int getTestLineNumber() const;
	bool isOutsideTestFile() const;
	bool isInHelperFunction() const;


protected:

	SimpleString createButWasString(const SimpleString& expected, const SimpleString& actual);
	SimpleString createDifferenceAtPosString(const SimpleString& actual, int position);

	SimpleString testName_;
	SimpleString fileName_;
	int lineNumber_;
	SimpleString testFileName_;
	int testLineNumber_;
	SimpleString message_;

	TestFailure& operator=(const TestFailure&);

};

class EqualsFailure: public TestFailure
{
public:
	EqualsFailure(UtestShell*, const char* fileName, int lineNumber, const char* expected, const char* actual);
	EqualsFailure(UtestShell*, const char* fileName, int lineNumber, const SimpleString& expected, const SimpleString& actual);
};

class DoublesEqualFailure: public TestFailure
{
public:
	DoublesEqualFailure(UtestShell*, const char* fileName, int lineNumber, double expected, double actual, double threshold);
};

class CheckEqualFailure : public TestFailure
{
public:
	CheckEqualFailure(UtestShell* test, const char* fileName, int lineNumber, const SimpleString& expected, const SimpleString& actual);
};

class ContainsFailure: public TestFailure
{
public:
	ContainsFailure(UtestShell*, const char* fileName, int lineNumber, const SimpleString& expected, const SimpleString& actual);

};

class CheckFailure : public TestFailure
{
public:
	CheckFailure(UtestShell* test, const char* fileName, int lineNumber, const SimpleString& checkString, const SimpleString& conditionString);
};

class FailFailure : public TestFailure
{
public:
	FailFailure(UtestShell* test, const char* fileName, int lineNumber, const SimpleString& message);
};

class LongsEqualFailure : public TestFailure
{
public:
	LongsEqualFailure(UtestShell* test, const char* fileName, int lineNumber, long expected, long actual);
};

class StringEqualFailure : public TestFailure
{
public:
	StringEqualFailure(UtestShell* test, const char* fileName, int lineNumber, const char* expected, const char* actual);
};

class StringEqualNoCaseFailure : public TestFailure
{
public:
	StringEqualNoCaseFailure(UtestShell* test, const char* fileName, int lineNumber, const char* expected, const char* actual);
};

#endif
