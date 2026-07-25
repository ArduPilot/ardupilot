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
#include "CppUTest/CommandLineArguments.h"
#include "CppUTest/PlatformSpecificFunctions.h"

CommandLineArguments::CommandLineArguments(int ac, const char** av) :
	ac_(ac), av_(av), verbose_(false), runTestsAsSeperateProcess_(false), repeat_(1), groupFilter_(""), nameFilter_(""), outputType_(OUTPUT_ECLIPSE)
{
}

CommandLineArguments::~CommandLineArguments()
{
}

bool CommandLineArguments::parse(TestPlugin* plugin)
{
	bool correctParameters = true;
	for (int i = 1; i < ac_; i++) {
		SimpleString argument = av_[i];
		if (argument == "-v") verbose_ = true;
		else if (argument == "-p") runTestsAsSeperateProcess_ = true;
		else if (argument.startsWith("-r")) SetRepeatCount(ac_, av_, i);
		else if (argument.startsWith("-g")) SetGroupFilter(ac_, av_, i);
		else if (argument.startsWith("-sg")) SetStrictGroupFilter(ac_, av_, i);
		else if (argument.startsWith("-n")) SetNameFilter(ac_, av_, i);
		else if (argument.startsWith("-sn")) SetStrictNameFilter(ac_, av_, i);
		else if (argument.startsWith("TEST(")) SetTestToRunBasedOnVerboseOutput(ac_, av_, i, "TEST(");
		else if (argument.startsWith("IGNORE_TEST(")) SetTestToRunBasedOnVerboseOutput(ac_, av_, i, "IGNORE_TEST(");
		else if (argument.startsWith("-o")) correctParameters = SetOutputType(ac_, av_, i);
		else if (argument.startsWith("-p")) correctParameters = plugin->parseAllArguments(ac_, av_, i);
		else correctParameters = false;

		if (correctParameters == false) {
			return false;
		}
	}
	return true;
}

const char* CommandLineArguments::usage() const
{
	return "usage [-v] [-r#] [-g|sg groupName] [-n|sn testName] [-o{normal, junit}]\n";
}

bool CommandLineArguments::isVerbose() const
{
	return verbose_;
}

bool CommandLineArguments::runTestsInSeperateProcess() const
{
	return runTestsAsSeperateProcess_;
}


int CommandLineArguments::getRepeatCount() const
{
	return repeat_;
}

TestFilter CommandLineArguments::getGroupFilter() const
{
	return groupFilter_;
}

TestFilter CommandLineArguments::getNameFilter() const
{
	return nameFilter_;
}

void CommandLineArguments::SetRepeatCount(int ac, const char** av, int& i)
{
	repeat_ = 0;

	SimpleString repeatParameter(av[i]);
	if (repeatParameter.size() > 2) repeat_ = PlatformSpecificAtoI(av[i] + 2);
	else if (i + 1 < ac) {
		repeat_ = PlatformSpecificAtoI(av[i + 1]);
		if (repeat_ != 0) i++;
	}

	if (0 == repeat_) repeat_ = 2;

}

SimpleString CommandLineArguments::getParameterField(int ac, const char** av, int& i, const SimpleString& parameterName)
{
	size_t parameterLength = parameterName.size();
	SimpleString parameter(av[i]);
	if (parameter.size() >  parameterLength) return av[i] + parameterLength;
	else if (i + 1 < ac) return av[++i];
	return "";
}

void CommandLineArguments::SetGroupFilter(int ac, const char** av, int& i)
{
	groupFilter_ = TestFilter(getParameterField(ac, av, i, "-g"));
}

void CommandLineArguments::SetStrictGroupFilter(int ac, const char** av, int& i)
{
	groupFilter_ = TestFilter(getParameterField(ac, av, i, "-sg"));
	groupFilter_.strictMatching();
}

void CommandLineArguments::SetNameFilter(int ac, const char** av, int& i)
{
	nameFilter_ = getParameterField(ac, av, i, "-n");
}

void CommandLineArguments::SetStrictNameFilter(int ac, const char** av, int& index)
{
	nameFilter_ = getParameterField(ac, av, index, "-sn");
	nameFilter_.strictMatching();
}

void CommandLineArguments::SetTestToRunBasedOnVerboseOutput(int ac, const char** av, int& index, const char* parameterName)
{
	SimpleString wholename = getParameterField(ac, av, index, parameterName);
	SimpleString testname = wholename.subStringFromTill(',', ')');
	testname = testname.subString(2, testname.size());
	groupFilter_ = wholename.subStringFromTill(wholename.at(0), ',');
	nameFilter_ = testname;
	nameFilter_.strictMatching();
	groupFilter_.strictMatching();
}

bool CommandLineArguments::SetOutputType(int ac, const char** av, int& i)
{
	SimpleString outputType = getParameterField(ac, av, i, "-o");
	if (outputType.size() == 0) return false;

	if (outputType == "normal" || outputType == "eclipse") {
		outputType_ = OUTPUT_ECLIPSE;
		return true;
	}
	if (outputType == "junit") {
		outputType_ = OUTPUT_JUNIT;
		return true;
	}
	return false;
}

bool CommandLineArguments::isEclipseOutput() const
{
	return outputType_ == OUTPUT_ECLIPSE;
}

bool CommandLineArguments::isJUnitOutput() const
{
	return outputType_ == OUTPUT_JUNIT;
}

