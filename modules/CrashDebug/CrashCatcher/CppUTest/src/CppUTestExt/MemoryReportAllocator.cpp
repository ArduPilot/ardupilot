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
#include "CppUTestExt/MemoryReportAllocator.h"
#include "CppUTestExt/MemoryReportFormatter.h"

MemoryReportAllocator::MemoryReportAllocator() : result_(NULL), realAllocator_(NULL), formatter_(NULL)
{
}

MemoryReportAllocator::~MemoryReportAllocator()
{
}

const char* MemoryReportAllocator::name()
{
	return realAllocator_->name();
}

const char* MemoryReportAllocator::alloc_name()
{
	return realAllocator_->alloc_name();
}

const char* MemoryReportAllocator::free_name()
{
	return realAllocator_->free_name();
}

void MemoryReportAllocator::setRealAllocator(TestMemoryAllocator* allocator)
{
	realAllocator_ = allocator;
}

TestMemoryAllocator* MemoryReportAllocator::getRealAllocator()
{
	return realAllocator_;
}

void MemoryReportAllocator::setTestResult(TestResult* result)
{
	result_ = result;
}

void MemoryReportAllocator::setFormatter(MemoryReportFormatter* formatter)
{
	formatter_ = formatter;
}

char* MemoryReportAllocator::alloc_memory(size_t size, const char* file, int line)
{
	char* memory = realAllocator_->alloc_memory(size, file, line);
	if (result_ && formatter_)
		formatter_->report_alloc_memory(result_, this, size, memory, file, line);
	return memory;
}

void MemoryReportAllocator::free_memory(char* memory, const char* file, int line)
{
	realAllocator_->free_memory(memory, file, line);
	if (result_ && formatter_)
		formatter_->report_free_memory(result_, this, memory, file, line);
}
