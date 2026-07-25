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
#include "CppUTest/MemoryLeakDetector.h"
#include "CppUTest/TestMemoryAllocator.h"
#include "CppUTest/PlatformSpecificFunctions.h"

#define UNKNOWN ((char*)("<unknown>"))

SimpleStringBuffer::SimpleStringBuffer() :
	positions_filled_(0), write_limit_(SIMPLE_STRING_BUFFER_LEN-1)
{
}

void SimpleStringBuffer::clear()
{
	positions_filled_ = 0;
	buffer_[0] = '\0';
}

void SimpleStringBuffer::add(const char* format, ...)
{
	int count = 0;
	int positions_left = write_limit_ - positions_filled_;
	if (positions_left <= 0) return;

	va_list arguments;
	va_start(arguments, format);
	count = PlatformSpecificVSNprintf(buffer_ + positions_filled_, positions_left+1, format, arguments);
	if (count > 0) positions_filled_ += count;
	if (positions_filled_ > write_limit_) positions_filled_ = write_limit_;
	va_end(arguments);
}

char* SimpleStringBuffer::toString()
{
	return buffer_;
}

void SimpleStringBuffer::setWriteLimit(int write_limit)
{
	write_limit_ = write_limit;
	if (write_limit_ > SIMPLE_STRING_BUFFER_LEN-1)
		write_limit_ = SIMPLE_STRING_BUFFER_LEN-1;
}
void SimpleStringBuffer::resetWriteLimit()
{
	write_limit_ = SIMPLE_STRING_BUFFER_LEN-1;
}

bool SimpleStringBuffer::reachedItsCapacity()
{
	return positions_filled_ >= write_limit_;
}
////////////////////////

void MemoryLeakDetectorNode::init(char* memory, unsigned number, size_t size, TestMemoryAllocator* allocator, MemLeakPeriod period, const char* file, int line)
{
	number_ = number;
	memory_ = memory;
	size_ = size;
	allocator_ = allocator;
	period_ = period;
	file_ = file;
	line_ = line;
}

///////////////////////

bool MemoryLeakDetectorList::isInPeriod(MemoryLeakDetectorNode* node, MemLeakPeriod period)
{
	return period == mem_leak_period_all || node->period_ == period || (node->period_ != mem_leak_period_disabled && period == mem_leak_period_enabled);
}

void MemoryLeakDetectorList::clearAllAccounting(MemLeakPeriod period)
{
	MemoryLeakDetectorNode* cur = head_;
	MemoryLeakDetectorNode* prev = 0;

	while (cur) {
		if (isInPeriod(cur, period)) {
			if (prev) {
				prev->next_ = cur->next_;
				cur = prev;
			}
			else {
				head_ = cur->next_;
				cur = head_;
				continue;
			}
		}
		prev = cur;
		cur = cur->next_;
	}
}

void MemoryLeakDetectorList::addNewNode(MemoryLeakDetectorNode* node)
{
	node->next_ = head_;
	head_ = node;
}

MemoryLeakDetectorNode* MemoryLeakDetectorList::removeNode(char* memory)
{
	MemoryLeakDetectorNode* cur = head_;
	MemoryLeakDetectorNode* prev = 0;
	while (cur) {
		if (cur->memory_ == memory) {
			if (prev) {
				prev->next_ = cur->next_;
				return cur;
			}
			else {
				head_ = cur->next_;
				return cur;
			}
		}
		prev = cur;
		cur = cur->next_;
	}
	return 0;
}

MemoryLeakDetectorNode* MemoryLeakDetectorList::retrieveNode(char* memory)
{
  MemoryLeakDetectorNode* cur = head_;
  while (cur) {
    if (cur->memory_ == memory)
      return cur;
    cur = cur->next_;
  }
  return NULL;
}

MemoryLeakDetectorNode* MemoryLeakDetectorList::getLeakFrom(MemoryLeakDetectorNode* node, MemLeakPeriod period)
{
	for (MemoryLeakDetectorNode* cur = node; cur; cur = cur->next_)
		if (isInPeriod(cur, period)) return cur;
	return 0;
}

MemoryLeakDetectorNode* MemoryLeakDetectorList::getFirstLeak(MemLeakPeriod period)
{
	return getLeakFrom(head_, period);
}

MemoryLeakDetectorNode* MemoryLeakDetectorList::getNextLeak(MemoryLeakDetectorNode* node, MemLeakPeriod period)
{
	return getLeakFrom(node->next_, period);
}

int MemoryLeakDetectorList::getTotalLeaks(MemLeakPeriod period)
{
	int total_leaks = 0;
	for (MemoryLeakDetectorNode* node = head_; node; node = node->next_) {
		if (isInPeriod(node, period)) total_leaks++;
	}
	return total_leaks;
}

bool MemoryLeakDetectorList::hasLeaks(MemLeakPeriod period)
{
	for (MemoryLeakDetectorNode* node = head_; node; node = node->next_)
		if (isInPeriod(node, period)) return true;
	return false;
}

/////////////////////////////////////////////////////////////

unsigned long MemoryLeakDetectorTable::hash(char* memory)
{
	return ((unsigned long) memory) % hash_prime;
}

void MemoryLeakDetectorTable::clearAllAccounting(MemLeakPeriod period)
{
	for (int i = 0; i < hash_prime; i++)
		table_[i].clearAllAccounting(period);
}

void MemoryLeakDetectorTable::addNewNode(MemoryLeakDetectorNode* node)
{
	table_[hash(node->memory_)].addNewNode(node);
}

MemoryLeakDetectorNode* MemoryLeakDetectorTable::removeNode(char* memory)
{
	return table_[hash(memory)].removeNode(memory);
}

MemoryLeakDetectorNode* MemoryLeakDetectorTable::retrieveNode(char* memory)
{
  return table_[hash(memory)].retrieveNode(memory);
}

bool MemoryLeakDetectorTable::hasLeaks(MemLeakPeriod period)
{
	for (int i = 0; i < hash_prime; i++)
		if (table_[i].hasLeaks(period)) return true;
	return false;
}

int MemoryLeakDetectorTable::getTotalLeaks(MemLeakPeriod period)
{
	int total_leaks = 0;
	for (int i = 0; i < hash_prime; i++)
		total_leaks += table_[i].getTotalLeaks(period);
	return total_leaks;
}

MemoryLeakDetectorNode* MemoryLeakDetectorTable::getFirstLeak(MemLeakPeriod period)
{
	for (int i = 0; i < hash_prime; i++) {
		MemoryLeakDetectorNode* node = table_[i].getFirstLeak(period);
		if (node) return node;
	}
	return 0;
}

MemoryLeakDetectorNode* MemoryLeakDetectorTable::getNextLeak(MemoryLeakDetectorNode* leak, MemLeakPeriod period)
{
	int i = hash(leak->memory_);
	MemoryLeakDetectorNode* node = table_[i].getNextLeak(leak, period);
	if (node) return node;

	for (++i; i < hash_prime; i++) {
		node = table_[i].getFirstLeak(period);
		if (node) return node;
	}
	return 0;
}

/////////////////////////////////////////////////////////////

MemoryLeakDetector::MemoryLeakDetector(MemoryLeakFailure* reporter)
{
	doAllocationTypeChecking_ = true;
	allocationSequenceNumber_ = 1;
	current_period_ = mem_leak_period_disabled;
	reporter_ = reporter;
	output_buffer_ = SimpleStringBuffer();
	memoryTable_ = MemoryLeakDetectorTable();
}

void MemoryLeakDetector::clearAllAccounting(MemLeakPeriod period)
{
	memoryTable_.clearAllAccounting(period);
}

void MemoryLeakDetector::startChecking()
{
	output_buffer_.clear();
	current_period_ = mem_leak_period_checking;
}

void MemoryLeakDetector::stopChecking()
{
	current_period_ = mem_leak_period_enabled;
}

void MemoryLeakDetector::enable()
{
	current_period_ = mem_leak_period_enabled;
}

void MemoryLeakDetector::disable()
{
	current_period_ = mem_leak_period_disabled;
}

void MemoryLeakDetector::disableAllocationTypeChecking()
{
	doAllocationTypeChecking_ = false;
}

void MemoryLeakDetector::enableAllocationTypeChecking()
{
	doAllocationTypeChecking_ = true;
}

unsigned MemoryLeakDetector::getCurrentAllocationNumber()
{
	return allocationSequenceNumber_;
}

void MemoryLeakDetector::reportFailure(const char* message, const char* allocFile, int allocLine, size_t allocSize, TestMemoryAllocator* allocAllocator, const char* freeFile, int freeLine,
		TestMemoryAllocator* freeAllocator)
{
	output_buffer_.add(message);
	output_buffer_.add(MEM_LEAK_ALLOC_LOCATION, allocFile, allocLine, allocSize, allocAllocator->alloc_name());
	output_buffer_.add(MEM_LEAK_DEALLOC_LOCATION, freeFile, freeLine, freeAllocator->free_name());
	reporter_->fail(output_buffer_.toString());
}

size_t calculateIntAlignedSize(size_t size)
{
	return (sizeof(int) - (size % sizeof(int))) + size;
}

size_t MemoryLeakDetector::sizeOfMemoryWithCorruptionInfo(size_t size)
{
	return calculateIntAlignedSize(size + memory_corruption_buffer_size);
}

MemoryLeakDetectorNode* MemoryLeakDetector::getNodeFromMemoryPointer(char* memory, size_t memory_size)
{
	return (MemoryLeakDetectorNode*) (memory + sizeOfMemoryWithCorruptionInfo(memory_size));
}

void MemoryLeakDetector::storeLeakInformation(MemoryLeakDetectorNode * node, char *new_memory, size_t size, TestMemoryAllocator *allocator, const char *file, int line)
{
	node->init(new_memory, allocationSequenceNumber_++, size, allocator, current_period_, file, line);
	addMemoryCorruptionInformation(node->memory_ + node->size_);
	memoryTable_.addNewNode(node);
}

char* MemoryLeakDetector::reallocateMemoryAndLeakInformation(TestMemoryAllocator* allocator, char* memory, size_t size, const char* file, int line)
{
	char* new_memory = (char*) (PlatformSpecificRealloc(memory, sizeOfMemoryWithCorruptionInfo(size)));
	if (new_memory == NULL) return NULL;
	MemoryLeakDetectorNode *node = (MemoryLeakDetectorNode*) (allocator->allocMemoryLeakNode(sizeof(MemoryLeakDetectorNode)));
	storeLeakInformation(node, new_memory, size, allocator, file, line);
	return node->memory_;
}

void MemoryLeakDetector::invalidateMemory(char* memory)
{
  MemoryLeakDetectorNode* node = memoryTable_.retrieveNode(memory);
  if (node)
    PlatformSpecificMemset(memory, 0xCD, node->size_);
}

void MemoryLeakDetector::addMemoryCorruptionInformation(char* memory)
{
	memory[0] = 'B';
	memory[1] = 'A';
	memory[2] = 'S';
}

bool MemoryLeakDetector::validMemoryCorruptionInformation(char* memory)
{
	return memory[0] == 'B' && memory[1] == 'A' && memory[2] == 'S';
}

bool MemoryLeakDetector::matchingAllocation(TestMemoryAllocator *alloc_allocator, TestMemoryAllocator *free_allocator)
{
	if (alloc_allocator == free_allocator) return true;
	if (!doAllocationTypeChecking_) return true;
	return free_allocator->isOfEqualType(alloc_allocator);
}

void MemoryLeakDetector::checkForCorruption(MemoryLeakDetectorNode* node, const char* file, int line, TestMemoryAllocator* allocator, bool allocateNodesSeperately)
{
	if (!matchingAllocation(node->allocator_, allocator)) reportFailure(MEM_LEAK_ALLOC_DEALLOC_MISMATCH, node->file_, node->line_, node->size_, node->allocator_, file, line, allocator);
	else if (!validMemoryCorruptionInformation(node->memory_ + node->size_)) reportFailure(MEM_LEAK_MEMORY_CORRUPTION, node->file_, node->line_, node->size_, node->allocator_, file, line, allocator);
	else if (allocateNodesSeperately) allocator->freeMemoryLeakNode((char*) node);
}

char* MemoryLeakDetector::allocMemory(TestMemoryAllocator* allocator, size_t size, bool allocatNodesSeperately)
{
	return allocMemory(allocator, size, UNKNOWN, 0, allocatNodesSeperately);
}

char* MemoryLeakDetector::allocMemory(TestMemoryAllocator* allocator, size_t size, const char* file, int line, bool allocatNodesSeperately)
{
	/* With malloc, it is harder to guarantee that the allocator free is called.
	 * This is because operator new is overloaded via linker symbols, but malloc just via #defines.
	 * If the same allocation is used and the wrong free is called, it will deallocate the memory leak information
	 * without the memory leak detector ever noticing it!
	 * So, for malloc, we'll allocate the memory separately so we can detect this and give a proper error.
	 */
	char* memory;
	MemoryLeakDetectorNode* node;
	if (allocatNodesSeperately) {
		memory = allocator->alloc_memory(sizeOfMemoryWithCorruptionInfo(size), file, line);
		if (memory == NULL) return NULL;
		node = (MemoryLeakDetectorNode*) allocator->allocMemoryLeakNode(sizeof(MemoryLeakDetectorNode));
	}
	else {
		memory = allocator->alloc_memory(sizeOfMemoryWithCorruptionInfo(size) + sizeof(MemoryLeakDetectorNode), file, line);
		if (memory == NULL) return NULL;
		node = getNodeFromMemoryPointer(memory, size);
	}

	storeLeakInformation(node, memory, size, allocator, file, line);
	return node->memory_;
}

void MemoryLeakDetector::removeMemoryLeakInformationWithoutCheckingOrDeallocating(void* memory)
{
	memoryTable_.removeNode((char*) memory);
}

void MemoryLeakDetector::deallocMemory(TestMemoryAllocator* allocator, void* memory, const char* file, int line, bool allocatNodesSeperately)
{
	if (memory == 0) return;

	MemoryLeakDetectorNode* node = memoryTable_.removeNode((char*) memory);
	if (node == NULL) {
		reportFailure(MEM_LEAK_DEALLOC_NON_ALLOCATED, "<unknown>", 0, 0, NullUnknownAllocator::defaultAllocator(), file, line, allocator);
		return;
	}
	if (!allocator->hasBeenDestroyed()) {
		checkForCorruption(node, file, line, allocator, allocatNodesSeperately);
		allocator->free_memory((char*) memory, file, line);
	}
}

void MemoryLeakDetector::deallocMemory(TestMemoryAllocator* allocator, void* memory, bool allocatNodesSeperately)
{
	deallocMemory(allocator, (char*) memory, UNKNOWN, 0, allocatNodesSeperately);
}

char* MemoryLeakDetector::reallocMemory(TestMemoryAllocator* allocator, char* memory, size_t size, const char* file, int line, bool allocatNodesSeperately)
{
	if (memory) {
		MemoryLeakDetectorNode* node = memoryTable_.removeNode(memory);
		if (node == NULL) {
			reportFailure(MEM_LEAK_DEALLOC_NON_ALLOCATED, "<unknown>", 0, 0, NullUnknownAllocator::defaultAllocator(), file, line, allocator);
			return NULL;
		}
		checkForCorruption(node, file, line, allocator, allocatNodesSeperately);
	}
	return reallocateMemoryAndLeakInformation(allocator, memory, size, file, line);
}

void MemoryLeakDetector::ConstructMemoryLeakReport(MemLeakPeriod period)
{
	MemoryLeakDetectorNode* leak = memoryTable_.getFirstLeak(period);
	int total_leaks = 0;
	bool giveWarningOnUsingMalloc = false;
	output_buffer_.add(MEM_LEAK_HEADER);
	output_buffer_.setWriteLimit(SimpleStringBuffer::SIMPLE_STRING_BUFFER_LEN - MEM_LEAK_NORMAL_MALLOC_FOOTER_SIZE);

	while (leak) {
		output_buffer_.add(MEM_LEAK_LEAK, leak->number_, leak->size_, leak->file_, leak->line_, leak->allocator_->alloc_name(), leak->memory_, leak->memory_);
		if (PlatformSpecificStrCmp(leak->allocator_->alloc_name(), "malloc") == 0)
			giveWarningOnUsingMalloc = true;
		total_leaks++;
		leak = memoryTable_.getNextLeak(leak, period);
	}
	bool buffer_reached_its_capacity = output_buffer_.reachedItsCapacity();
	output_buffer_.resetWriteLimit();
	if (buffer_reached_its_capacity)
		output_buffer_.add(MEM_LEAK_TOO_MUCH);
	output_buffer_.add("%s %d\n", MEM_LEAK_FOOTER, total_leaks);
	if (giveWarningOnUsingMalloc)
		output_buffer_.add(MEM_LEAK_ADDITION_MALLOC_WARNING);
}

const char* MemoryLeakDetector::report(MemLeakPeriod period)
{
	if (!memoryTable_.hasLeaks(period)) return MEM_LEAK_NONE;

	output_buffer_.clear();
	ConstructMemoryLeakReport(period);

	return output_buffer_.toString();
}

void MemoryLeakDetector::markCheckingPeriodLeaksAsNonCheckingPeriod()
{
	MemoryLeakDetectorNode* leak = memoryTable_.getFirstLeak(mem_leak_period_checking);
	while (leak) {
		if (leak->period_ == mem_leak_period_checking) leak->period_ = mem_leak_period_enabled;
		leak = memoryTable_.getNextLeak(leak, mem_leak_period_checking);
	}
}

int MemoryLeakDetector::totalMemoryLeaks(MemLeakPeriod period)
{
	return memoryTable_.getTotalLeaks(period);
}
