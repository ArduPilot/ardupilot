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

#ifndef D_MemoryReportFormatter_h
#define D_MemoryReportFormatter_h

class TestOutput;
class UtestShell;

class MemoryReportFormatter
{
public:
	virtual ~MemoryReportFormatter(){}

	virtual void report_testgroup_start(TestResult* result, UtestShell& test)=0;
	virtual void report_testgroup_end(TestResult* result, UtestShell& test)=0;

	virtual void report_test_start(TestResult* result, UtestShell& test)=0;
	virtual void report_test_end(TestResult* result, UtestShell& test)=0;

	virtual void report_alloc_memory(TestResult* result, TestMemoryAllocator* allocator, size_t size, char* memory, const char* file, int line)=0;
	virtual void report_free_memory(TestResult* result, TestMemoryAllocator* allocator, char* memory, const char* file, int line)=0;
};

class NormalMemoryReportFormatter : public MemoryReportFormatter
{
public:
	NormalMemoryReportFormatter();
	virtual ~NormalMemoryReportFormatter();

	virtual void report_testgroup_start(TestResult* /*result*/, UtestShell& /*test*/);
	virtual void report_testgroup_end(TestResult* /*result*/, UtestShell& /*test*/){};

	virtual void report_test_start(TestResult* result, UtestShell& test);
	virtual void report_test_end(TestResult* result, UtestShell& test);

	virtual void report_alloc_memory(TestResult* result, TestMemoryAllocator* allocator, size_t size, char* memory, const char* file, int line);
	virtual void report_free_memory(TestResult* result, TestMemoryAllocator* allocator, char* memory, const char* file, int line);
};

#endif
