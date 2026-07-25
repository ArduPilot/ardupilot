/*
 * Copyright (c) 2022 Siddharth B Purohit, CubePilot Pty Ltd
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
 */
#pragma once

#include <stdint.h>
#include <canard.h>
#include "helpers.h"

namespace Canard {

#define MAKE_TRANSFER_DESCRIPTOR(data_type_id, transfer_type, src_node_id, dst_node_id)             \
    (((uint32_t)(data_type_id)) | (((uint32_t)(transfer_type)) << 16U) |                            \
    (((uint32_t)(src_node_id)) << 18U) | (((uint32_t)(dst_node_id)) << 25U))

/// @brief list of object to retain transfer id for transfer descriptor
class TransferObject {
public:
    TransferObject(uint32_t _transfer_desc) : next(nullptr), transfer_desc(_transfer_desc), tid(0) {}

    static uint8_t* get_tid_ptr(uint8_t index, uint16_t data_type_id, CanardTransferType transfer_type, uint8_t src_node_id, uint8_t dst_node_id) NOINLINE_FUNC {
        if (index >= CANARD_NUM_HANDLERS) {
            return nullptr;
        }
#ifdef WITH_SEMAPHORE
        WITH_SEMAPHORE(sem[index]);
#endif
        uint32_t _transfer_desc = MAKE_TRANSFER_DESCRIPTOR(data_type_id, transfer_type, src_node_id, dst_node_id);
        // check head
        if (tid_map_head[index] == nullptr) {
            tid_map_head[index] = allocate<TransferObject>(_transfer_desc);
            if (tid_map_head[index]  == nullptr) {
                return nullptr;
            }
            return &tid_map_head[index]->tid;
        } else if (tid_map_head[index]->transfer_desc == _transfer_desc) {
            return &tid_map_head[index]->tid;
        }

        // search through the list for an existing entry
        TransferObject *tid_map_ptr = tid_map_head[index];
        while(tid_map_ptr->next) {
            tid_map_ptr = tid_map_ptr->next;
            if (tid_map_ptr->transfer_desc == _transfer_desc) {
                return &tid_map_ptr->tid;
            }
        }

        // create a new entry, if not found
        tid_map_ptr->next = allocate<TransferObject>(_transfer_desc);
        if (tid_map_ptr->next == nullptr) {
            return nullptr;
        }
        return &tid_map_ptr->next->tid;
    }

    static void free_tid_ptr(uint8_t index) NOINLINE_FUNC {
        if (index >= CANARD_NUM_HANDLERS) {
            return;
        }
#ifdef WITH_SEMAPHORE
        WITH_SEMAPHORE(sem[index]);
#endif
        TransferObject *tid_map_ptr = tid_map_head[index];
        while(tid_map_ptr) {
            TransferObject *next = tid_map_ptr->next;
            deallocate(tid_map_ptr);
            tid_map_ptr = next;
        }
        tid_map_head[index] = nullptr;
    }
private:
    static TransferObject *tid_map_head[CANARD_NUM_HANDLERS];
#ifdef WITH_SEMAPHORE
    static Canard::Semaphore sem[CANARD_NUM_HANDLERS];
#endif
    TransferObject *next;
    uint32_t transfer_desc;
    uint8_t tid;
};

} // namespace Canard

#define DEFINE_TRANSFER_OBJECT_HEADS() Canard::TransferObject* Canard::TransferObject::tid_map_head[CANARD_NUM_HANDLERS] = {nullptr}

#define DEFINE_TRANSFER_OBJECT_SEMAPHORES() Canard::Semaphore Canard::TransferObject::sem[CANARD_NUM_HANDLERS];
