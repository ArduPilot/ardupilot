/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*
  based on dynamic_memory.hpp which is:
  Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include "AP_UAVCAN.h"

#ifndef UAVCAN_NODE_POOL_BLOCK_SIZE
#if HAL_CANFD_SUPPORTED
#define UAVCAN_NODE_POOL_BLOCK_SIZE 128
#else
#define UAVCAN_NODE_POOL_BLOCK_SIZE 64
#endif
#endif

class AP_PoolAllocator : public uavcan::IPoolAllocator
{
public:
    AP_PoolAllocator(uint16_t _pool_size);

    bool init(void);
    void *allocate(std::size_t size) override;
    void deallocate(const void* ptr) override;

    uint16_t getBlockCapacity() const override {
        return num_blocks;
    }

private:
    const uint16_t num_blocks;

    union Node {
        uint8_t data[UAVCAN_NODE_POOL_BLOCK_SIZE];
        Node* next;
    };

    Node *free_list;
    Node *pool_nodes;
    HAL_Semaphore sem;

    uint16_t used;
    uint16_t max_used;
};
