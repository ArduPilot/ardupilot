/*
 * Copyright (c) 2016 UAVCAN Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Contributors: https://github.com/UAVCAN/libcanard/contributors
 */

#include <gtest/gtest.h>
#include "canard_internals.h"


#define AVAILABLE_BLOCKS 3


TEST(MemoryAllocatorTestGroup, FreeListIsConstructedCorrectly)
{
    CanardPoolAllocator allocator;
    CanardPoolAllocatorBlock buffer[AVAILABLE_BLOCKS];
    initPoolAllocator(&allocator, buffer, AVAILABLE_BLOCKS);

    // Check that the memory list is constructed correctly.
    ASSERT_TRUE(&buffer[0] == allocator.free_list);
    ASSERT_TRUE(&buffer[1] == allocator.free_list->next);
    ASSERT_TRUE(&buffer[2] == allocator.free_list->next->next);
    ASSERT_TRUE(NULL == allocator.free_list->next->next->next);

    // Check statistics
    ASSERT_TRUE(AVAILABLE_BLOCKS == allocator.statistics.capacity_blocks);
    ASSERT_TRUE(0 ==                allocator.statistics.current_usage_blocks);
    ASSERT_TRUE(0 ==                allocator.statistics.peak_usage_blocks);
}

TEST(MemoryAllocatorTestGroup, CanAllocateBlock)
{
    CanardPoolAllocator allocator;
    CanardPoolAllocatorBlock buffer[AVAILABLE_BLOCKS];
    initPoolAllocator(&allocator, buffer, AVAILABLE_BLOCKS);

    void* block = allocateBlock(&allocator);

    // Check that the first free memory block was used and that the next block is ready.
    ASSERT_TRUE(&buffer[0] == block);
    ASSERT_TRUE(&buffer[1] == allocator.free_list);

    // Check statistics
    ASSERT_TRUE(AVAILABLE_BLOCKS == allocator.statistics.capacity_blocks);
    ASSERT_TRUE(1 ==                allocator.statistics.current_usage_blocks);
    ASSERT_TRUE(1 ==                allocator.statistics.peak_usage_blocks);
}

TEST(MemoryAllocatorTestGroup, ReturnsNullIfThereIsNoBlockLeft)
{
    CanardPoolAllocator allocator;
    CanardPoolAllocatorBlock buffer[AVAILABLE_BLOCKS];
    initPoolAllocator(&allocator, buffer, AVAILABLE_BLOCKS);

    // First exhaust all availables block
    for (int i = 0; i < AVAILABLE_BLOCKS; ++i)
    {
        allocateBlock(&allocator);
    }

    // Try to allocate one extra block
    void* block = allocateBlock(&allocator);
    ASSERT_TRUE(NULL == block);

    // Check statistics
    ASSERT_TRUE(AVAILABLE_BLOCKS == allocator.statistics.capacity_blocks);
    ASSERT_TRUE(AVAILABLE_BLOCKS == allocator.statistics.current_usage_blocks);
    ASSERT_TRUE(AVAILABLE_BLOCKS == allocator.statistics.peak_usage_blocks);
}

TEST(MemoryAllocatorTestGroup, CanFreeBlock)
{
    CanardPoolAllocator allocator;
    CanardPoolAllocatorBlock buffer[AVAILABLE_BLOCKS];
    initPoolAllocator(&allocator, buffer, AVAILABLE_BLOCKS);

    void* block = allocateBlock(&allocator);

    freeBlock(&allocator, block);

    // Check that the block was added back to the beginning
    ASSERT_TRUE(&buffer[0] == allocator.free_list);
    ASSERT_TRUE(&buffer[1] == allocator.free_list->next);

    // Check statistics
    ASSERT_TRUE(AVAILABLE_BLOCKS == allocator.statistics.capacity_blocks);
    ASSERT_TRUE(0 ==                allocator.statistics.current_usage_blocks);
    ASSERT_TRUE(1 ==                allocator.statistics.peak_usage_blocks);
}
