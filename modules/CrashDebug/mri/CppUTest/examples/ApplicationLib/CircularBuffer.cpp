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

#include "CircularBuffer.h"
#include "Printer.h"

CircularBuffer::CircularBuffer(int _capacity) :
	index(0), outdex(0), capacity(_capacity), empty(true), full(false)
{
	buffer = new int[this->capacity];
}

CircularBuffer::~CircularBuffer()
{
	delete[] buffer;
}

bool CircularBuffer::IsEmpty()
{
	return empty;
}

bool CircularBuffer::IsFull()
{
	return full;
}

void CircularBuffer::Put(int i)
{
	empty = false;
	buffer[index] = i;
	index = Next(index);
	if (full) outdex = Next(outdex);
	else if (index == outdex) full = true;
}

int CircularBuffer::Get()
{
	int result = -1;
	full = false;

	if (!empty) {
		result = buffer[outdex];
		outdex = Next(outdex);
		if (outdex == index) empty = true;
	}
	return result;
}

int CircularBuffer::Capacity()
{
	return capacity;
}

int CircularBuffer::Next(int i)
{
	if (++i >= capacity) i = 0;
	return i;
}

void CircularBuffer::Print(Printer* p)
{
	p->Print("Circular buffer content:\n<");

	int printIndex = outdex;
	int count = index - outdex;

	if (!empty && (index <= outdex)) count = capacity - (outdex - index);

	for (int i = 0; i < count; i++) {
		p->Print(buffer[printIndex]);
		printIndex = Next(printIndex);
		if (i + 1 != count) p->Print(", ");
	}
	p->Print(">\n");
}
