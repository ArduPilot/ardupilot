/*
 * Copyright (c) 2007, Michael Feathers, James Grenning, Bas Vodde and Timo Puronen
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

#include "MemoryLeakWarning.h"

#include <e32base.h>

MemoryLeakWarning* MemoryLeakWarning::_latest = NULL;

// naming convention due to CppUTest generic class name
class MemoryLeakWarningData : public CBase {
public:
	TInt iInitialAllocCells;
	TInt iExpectedLeaks;
	TInt iInitialThreadHandleCount;
	TInt iInitialProcessHandleCount;
};

MemoryLeakWarning::MemoryLeakWarning()
{
	_latest = this; 
	CreateData();	
}

MemoryLeakWarning::~MemoryLeakWarning()
{
	DestroyData();
}

void MemoryLeakWarning::Enable()
{
}

const char* MemoryLeakWarning::FinalReport(int toBeDeletedLeaks)
{
	TInt cellDifference(User::CountAllocCells() - _impl->iInitialAllocCells);
	if( cellDifference != toBeDeletedLeaks ) {
		return "Heap imbalance after test\n";
	}
	
	TInt processHandles;
	TInt threadHandles;
	RThread().HandleCount(processHandles, threadHandles);
	
	if(_impl->iInitialProcessHandleCount != processHandles ||
		_impl->iInitialThreadHandleCount != threadHandles) {
		return "Handle count imbalance after test\n";
	}
	
	return "";
}

void MemoryLeakWarning::CheckPointUsage()
{
	_impl->iInitialAllocCells = User::CountAllocCells();
	RThread().HandleCount(_impl->iInitialProcessHandleCount, _impl->iInitialThreadHandleCount);
}

bool MemoryLeakWarning::UsageIsNotBalanced()
{
	TInt allocatedCells(User::CountAllocCells());
	if(_impl->iExpectedLeaks != 0) {
		TInt difference(Abs(_impl->iInitialAllocCells - allocatedCells));
		return difference != _impl->iExpectedLeaks;
	}
	return allocatedCells != _impl->iInitialAllocCells;
}

const char* MemoryLeakWarning::Message()
{
	return "";
}

void MemoryLeakWarning::ExpectLeaks(int n)
{
	_impl->iExpectedLeaks = n;
}

// this method leaves (no naming convention followed due to CppUTest framework
void MemoryLeakWarning::CreateData()
{
	_impl = new(ELeave) MemoryLeakWarningData();
}

void MemoryLeakWarning::DestroyData()
{
	delete _impl;
	_impl = NULL;
}

MemoryLeakWarning* MemoryLeakWarning::GetLatest()
{
	return _latest;
}

void MemoryLeakWarning::SetLatest(MemoryLeakWarning* latest)
{
	_latest = latest;
}
