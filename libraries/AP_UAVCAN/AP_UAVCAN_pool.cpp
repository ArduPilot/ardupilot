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

#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_LIBUAVCAN_DRIVERS

#include "AP_UAVCAN.h"
#include "AP_UAVCAN_pool.h"
#include <AP_InternalError/AP_InternalError.h>

AP_PoolAllocator::AP_PoolAllocator(uint16_t _pool_size) :
    num_blocks(_pool_size / UAVCAN_NODE_POOL_BLOCK_SIZE)
{
}

bool AP_PoolAllocator::init(void)
{
    WITH_SEMAPHORE(sem);
    pool_nodes = (Node *)calloc(num_blocks, UAVCAN_NODE_POOL_BLOCK_SIZE);
    if (pool_nodes == nullptr) {
        return false;
    }
    for (uint16_t i=0; i<(num_blocks-1); i++) {
        pool_nodes[i].next = &pool_nodes[i+1];
    }
    free_list = &pool_nodes[0];
    return true;
}

void* AP_PoolAllocator::allocate(std::size_t size)
{
    WITH_SEMAPHORE(sem);
    if (free_list == nullptr || size > UAVCAN_NODE_POOL_BLOCK_SIZE) {
        return nullptr;
    }
    Node *ret = free_list;
    const uint32_t blk = ret - pool_nodes;
    if (blk >= num_blocks) {
        INTERNAL_ERROR(AP_InternalError::error_t::mem_guard);
        return nullptr;
    }
    free_list = free_list->next;

    used++;
    if (used > max_used) {
        max_used = used;
    }

    return ret;
}

void AP_PoolAllocator::deallocate(const void* ptr)
{
    if (ptr == nullptr) {
        return;
    }
    WITH_SEMAPHORE(sem);

    Node *p = reinterpret_cast<Node*>(const_cast<void*>(ptr));
    const uint32_t blk = p - pool_nodes;
    if (blk >= num_blocks) {
        INTERNAL_ERROR(AP_InternalError::error_t::mem_guard);
        return;
    }
    p->next = free_list;
    free_list = p;

    used--;
}

#endif // HAL_ENABLE_LIBUAVCAN_DRIVERS

