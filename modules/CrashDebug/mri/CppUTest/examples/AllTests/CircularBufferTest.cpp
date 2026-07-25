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
#include "MockPrinter.h"
#include "CircularBuffer.h"

TEST_GROUP(CircularBuffer)
{ CircularBuffer* buffer;

void setup()
{
	buffer = new CircularBuffer();
}
void teardown()
{
	delete buffer;
}

void fillTheQueue(int seed, int howMany)
{
	for (int i = 0; i < howMany; i++)
	buffer->Put(seed + i);
}
void removeFromQueue(int howMany)
{
	for (int i = 0; i < howMany; i++)
	buffer->Get();
}
};

TEST(CircularBuffer, EmptyAfterCreation)
{
	CHECK(buffer->IsEmpty());
}

TEST(CircularBuffer, NotEmpty)
{
	buffer->Put(10046);
	CHECK(!buffer->IsEmpty());
}

TEST(CircularBuffer, NotEmptyThenEmpty)
{
	buffer->Put(4567);
	CHECK(!buffer->IsEmpty());
	buffer->Get();
	CHECK(buffer->IsEmpty());
}

TEST(CircularBuffer, GetPutOneValue)
{
	buffer->Put(4567);
	LONGS_EQUAL(4567, buffer->Get());
}

TEST(CircularBuffer, GetPutAFew)
{
	buffer->Put(1);
	buffer->Put(2);
	buffer->Put(3);
	LONGS_EQUAL(1, buffer->Get());
	LONGS_EQUAL(2, buffer->Get());
	LONGS_EQUAL(3, buffer->Get());
}

TEST(CircularBuffer, Capacity)
{
	CircularBuffer b(2);
	LONGS_EQUAL(2, b.Capacity());
}

TEST(CircularBuffer, IsFull)
{
	fillTheQueue(0, buffer->Capacity());
	CHECK(buffer->IsFull());
}

TEST(CircularBuffer, EmptyToFullToEmpty)
{
	fillTheQueue(100, buffer->Capacity());

	CHECK(buffer->IsFull());

	removeFromQueue(buffer->Capacity());

	CHECK(buffer->IsEmpty());
}

TEST(CircularBuffer, WrapAround)
{
	fillTheQueue(100, buffer->Capacity());

	CHECK(buffer->IsFull());
	LONGS_EQUAL(100, buffer->Get());
	CHECK(!buffer->IsFull());
	buffer->Put(1000);
	CHECK(buffer->IsFull());

	removeFromQueue(buffer->Capacity() - 1);

	LONGS_EQUAL(1000, buffer->Get());
	CHECK(buffer->IsEmpty());
}

TEST(CircularBuffer, PutToFull)
{
	int capacity = buffer->Capacity();
	fillTheQueue(900, capacity);
	buffer->Put(9999);

	for (int i = 0; i < buffer->Capacity() - 1; i++)
		LONGS_EQUAL(i+900+1, buffer->Get());

	LONGS_EQUAL(9999, buffer->Get());
	CHECK(buffer->IsEmpty());
}

//Sometime people ask what tests the tests.
//Do you know the answer


TEST(CircularBuffer, GetFromEmpty)
{
	LONGS_EQUAL(-1, buffer->Get());
	CHECK(buffer->IsEmpty());
}

/*
 * the next tests demonstrate using a mock object for
 * capturing output
 *
 */

TEST(CircularBuffer, PrintEmpty)
{
	MockPrinter mock;
	Printer* p = &mock;

	buffer->Print(p);
	CHECK_EQUAL("Circular buffer content:\n<>\n",
			mock.getOutput());
}

TEST(CircularBuffer, PrintAfterOnePut)
{
	MockPrinter mock;

	buffer->Put(1);
	buffer->Print(&mock);
	CHECK_EQUAL("Circular buffer content:\n<1>\n",
			mock.getOutput());
}

TEST(CircularBuffer, PrintNotYetWrappedOrFull)
{
	MockPrinter mock;

	buffer->Put(1);
	buffer->Put(2);
	buffer->Put(3);
	buffer->Print(&mock);
	CHECK_EQUAL("Circular buffer content:\n<1, 2, 3>\n",
			mock.getOutput());
}

TEST(CircularBuffer, PrintNotYetWrappedAndIsFull)
{
	MockPrinter mock;

	fillTheQueue(200, buffer->Capacity());

	buffer->Print(&mock);
	const char* expected = "Circular buffer content:\n"
		"<200, 201, 202, 203, 204>\n";

	CHECK_EQUAL(expected, mock.getOutput());
}

TEST(CircularBuffer, PrintWrappedAndIsFullOldestToNewest)
{
	MockPrinter mock;

	fillTheQueue(200, buffer->Capacity());
	buffer->Get();
	buffer->Put(999);

	buffer->Print(&mock);
	const char* expected = "Circular buffer content:\n"
		"<201, 202, 203, 204, 999>\n";

	CHECK_EQUAL(expected, mock.getOutput());
}

TEST(CircularBuffer, PrintWrappedAndFullOverwriteOldest)
{
	MockPrinter mock;

	fillTheQueue(200, buffer->Capacity());
	buffer->Put(9999);

	buffer->Print(&mock);
	const char* expected = "Circular buffer content:\n"
		"<201, 202, 203, 204, 9999>\n";

	CHECK_EQUAL(expected, mock.getOutput());
}

TEST(CircularBuffer, PrintBoundary)
{
	MockPrinter mock;

	fillTheQueue(200, buffer->Capacity());
	removeFromQueue(buffer->Capacity() - 2);
	buffer->Put(888);
	fillTheQueue(300, buffer->Capacity() - 1);

	buffer->Print(&mock);
	const char* expected = "Circular buffer content:\n"
		"<888, 300, 301, 302, 303>\n";

	CHECK_EQUAL(expected, mock.getOutput());
}

TEST(CircularBuffer, FillEmptyThenPrint)
{
	MockPrinter mock;

	fillTheQueue(200, buffer->Capacity());
	removeFromQueue(buffer->Capacity());
	buffer->Print(&mock);
	const char* expected = "Circular buffer content:\n"
		"<>\n";

	CHECK_EQUAL(expected, mock.getOutput());
}
