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

#include "CppUTest/MemoryLeakWarning.h"

#include <stdlib.h>
#include <stdio.h>

/* Static since we REALLY can have only one of these! */
static int allocatedBlocks = 0;
static int allocatedArrays = 0;
static int firstInitialBlocks = 0;
static int firstInitialArrays = 0;
static bool reporterRegistered = false;

class MemoryLeakWarningData
{
	public:
		MemoryLeakWarningData();
		
		int initialBlocksUsed;
		int initialArraysUsed;

		int blockUsageCheckPoint;
		int arrayUsageCheckPoint;
		int expectCount;
		char message[100];
};

void MemoryLeakWarning::CreateData()
{
	_impl = (MemoryLeakWarningData*) malloc(sizeof(MemoryLeakWarningData));
	_impl->initialBlocksUsed = 0;
	_impl->initialArraysUsed = 0;

	_impl->blockUsageCheckPoint = 0;
	_impl->arrayUsageCheckPoint = 0;
	_impl->expectCount = 0;
	_impl->message_[0] = '\0';
}

void MemoryLeakWarning::DestroyData()
{
	free(_impl);
}

extern "C" {
void reportMemoryBallance();
}

void reportMemoryBallance()
{
  int blockBalance = allocatedBlocks - firstInitialBlocks;
  int arrayBalance = allocatedArrays - firstInitialArrays;
  if (blockBalance == 0 && arrayBalance == 0)
    ;
  else if (blockBalance + arrayBalance == 0)
    printf("No leaks but some arrays were deleted without []\n");
  else
    {
      if (blockBalance > 0)
        printf("Memory leak! %d blocks not deleted\n", blockBalance);
      if (arrayBalance > 0)
        printf("Memory leak! %d arrays not deleted\n", arrayBalance);
      if (blockBalance < 0)
        printf("More blocks deleted than newed! %d extra deletes\n", blockBalance);
      if (arrayBalance < 0)
        printf("More arrays deleted than newed! %d extra deletes\n", arrayBalance);

      printf("NOTE - some memory leaks appear to be allocated statics that are not released\n"
             "     - by the standard library\n"
             "     - Use the -r switch on your unit tests to repeat the test sequence\n"
             "     - If no leaks are reported on the second pass, it is likely a static\n"
             "     - that is not released\n");
    }
}


MemoryLeakWarning* MemoryLeakWarning::_latest = NULL;

MemoryLeakWarning::MemoryLeakWarning()
{
	_latest = this; 
	CreateData();	
}

MemoryLeakWarning::~MemoryLeakWarning()
{
	DestroyData();
}

MemoryLeakWarning* MemoryLeakWarning::GetLatest()
{
	return _latest;
}

void MemoryLeakWarning::SetLatest(MemoryLeakWarning* latest)
{
	_latest = latest;
}

void MemoryLeakWarning::Enable()
{
  _impl->initialBlocksUsed = allocatedBlocks;
  _impl->initialArraysUsed = allocatedArrays;

	if (!reporterRegistered) {
		firstInitialBlocks = allocatedBlocks;
		firstInitialArrays = allocatedArrays;
		reporterRegistered = true;
	}

}

const char*  MemoryLeakWarning::FinalReport(int toBeDeletedLeaks)
{
  if (_impl->initialBlocksUsed != (allocatedBlocks-toBeDeletedLeaks) 
          || _impl->initialArraysUsed != allocatedArrays )
    {
      printf("initial blocks=%d, allocated blocks=%d\ninitial arrays=%d, allocated arrays=%d\n",
             _impl->initialBlocksUsed, allocatedBlocks, _impl->initialArraysUsed, allocatedArrays);

      return "Memory new/delete imbalance after running tests\n";
    }
  else
    return "";
}

void MemoryLeakWarning::CheckPointUsage()
{
  _impl->blockUsageCheckPoint = allocatedBlocks;
  _impl->arrayUsageCheckPoint = allocatedArrays;
}

bool MemoryLeakWarning::UsageIsNotBalanced()
{
  int arrayBalance = allocatedArrays - _impl->arrayUsageCheckPoint;
  int blockBalance = allocatedBlocks - _impl->blockUsageCheckPoint;

  if (_impl->expectCount != 0 && blockBalance + arrayBalance == _impl->expectCount)
    return false;
  if (blockBalance == 0 && arrayBalance == 0)
    return false;
  else if (blockBalance + arrayBalance == 0)
    sprintf(_impl->message_, "No leaks but some arrays were deleted without []\n");
  else
    {
      int nchars = 0;
      if (_impl->blockUsageCheckPoint != allocatedBlocks)
        nchars = sprintf(_impl->message_, "this test leaks %d blocks",
                         allocatedBlocks - _impl->blockUsageCheckPoint);

      if (_impl->arrayUsageCheckPoint != allocatedArrays)
        sprintf(_impl->message_ + nchars, "this test leaks %d arrays",
                allocatedArrays - _impl->arrayUsageCheckPoint);
    }
  return true;
}

const char* MemoryLeakWarning::Message()
{
  return _impl->message_;
}

void MemoryLeakWarning::ExpectLeaks(int n)
{
	_impl->expectCount = n;
}

/* Global overloaded operators */

void* operator new(size_t size)
{
	allocatedBlocks++;
  	return malloc(size);
}

void operator delete(void* mem)
{
  	allocatedBlocks--;
  	free(mem);
}

void* operator new[](size_t size)
{
	allocatedArrays++;
  	return malloc(size);
}

void operator delete[](void* mem)
{
 	allocatedArrays--;
  	free(mem);
}

void* operator new(size_t size, const char* file, int line)
{
    allocatedBlocks++;
    return malloc(size);
    
}
