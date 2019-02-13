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
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include "shared_dma.h"

/*
  code to handle sharing of DMA channels between peripherals
 */

#if CH_CFG_USE_SEMAPHORES == TRUE && STM32_DMA_ADVANCED

using namespace ChibiOS;

Shared_DMA::dma_lock Shared_DMA::locks[SHARED_DMA_MAX_STREAM_ID+1];

void Shared_DMA::init(void)
{
    for (uint8_t i=0; i<SHARED_DMA_MAX_STREAM_ID; i++) {
        chBSemObjectInit(&locks[i].semaphore, false);
    }
}

// constructor
Shared_DMA::Shared_DMA(uint8_t _stream_id1,
                       uint8_t _stream_id2,
                       dma_allocate_fn_t _allocate,
                       dma_deallocate_fn_t _deallocate)
{
    stream_id1 = _stream_id1;
    stream_id2 = _stream_id2;
    allocate = _allocate;
    deallocate = _deallocate;
}

//remove any assigned deallocator or allocator
void Shared_DMA::unregister()
{
    if (stream_id1 < SHARED_DMA_MAX_STREAM_ID &&
        locks[stream_id1].obj == this) {
        locks[stream_id1].deallocate(this);
        locks[stream_id1].obj = nullptr;
    }

    if (stream_id2 < SHARED_DMA_MAX_STREAM_ID &&
        locks[stream_id2].obj == this) {
        locks[stream_id2].deallocate(this);
        locks[stream_id2].obj = nullptr;
    }
}

// lock one stream
void Shared_DMA::lock_stream(uint8_t stream_id)
{
    if (stream_id < SHARED_DMA_MAX_STREAM_ID) {
        chBSemWait(&locks[stream_id].semaphore);
    }
}

// unlock one stream
void Shared_DMA::unlock_stream(uint8_t stream_id)
{
    if (stream_id < SHARED_DMA_MAX_STREAM_ID) {
        chBSemSignal(&locks[stream_id].semaphore);
    }
}

// unlock one stream from an IRQ handler
void Shared_DMA::unlock_stream_from_IRQ(uint8_t stream_id)
{
    if (stream_id < SHARED_DMA_MAX_STREAM_ID) {
        chBSemSignalI(&locks[stream_id].semaphore);
    }
}

// lock one stream, non-blocking
bool Shared_DMA::lock_stream_nonblocking(uint8_t stream_id)
{
    if (stream_id < SHARED_DMA_MAX_STREAM_ID) {
        return chBSemWaitTimeout(&locks[stream_id].semaphore, 1) == MSG_OK;
    }
    return true;
}


// lock the DMA channels
void Shared_DMA::lock_core(void)
{
    // see if another driver has DMA allocated. If so, call their
    // deallocation function
    if (stream_id1 < SHARED_DMA_MAX_STREAM_ID &&
        locks[stream_id1].obj && locks[stream_id1].obj != this) {
        locks[stream_id1].deallocate(locks[stream_id1].obj);
        locks[stream_id1].obj = nullptr;
    }
    if (stream_id2 < SHARED_DMA_MAX_STREAM_ID &&
        locks[stream_id2].obj && locks[stream_id2].obj != this) {
        locks[stream_id2].deallocate(locks[stream_id2].obj);
        locks[stream_id2].obj = nullptr;
    }
    if ((stream_id1 < SHARED_DMA_MAX_STREAM_ID && locks[stream_id1].obj == nullptr) ||
        (stream_id2 < SHARED_DMA_MAX_STREAM_ID && locks[stream_id2].obj == nullptr)) {
        // allocate the DMA channels and put our deallocation function in place
        allocate(this);
        if (stream_id1 < SHARED_DMA_MAX_STREAM_ID) {
            locks[stream_id1].deallocate = deallocate;
            locks[stream_id1].obj = this;
        }
        if (stream_id2 < SHARED_DMA_MAX_STREAM_ID) {
            locks[stream_id2].deallocate = deallocate;
            locks[stream_id2].obj = this;
        }
    }
#ifdef STM32_DMA_STREAM_ID_ANY
    else if (stream_id1 == STM32_DMA_STREAM_ID_ANY ||
             stream_id2 == STM32_DMA_STREAM_ID_ANY) {
        // call allocator without needing locking
        allocate(this);
    }
#endif
    have_lock = true;
}

// lock the DMA channels, blocking method
void Shared_DMA::lock(void)
{
    lock_stream(stream_id1);
    lock_stream(stream_id2);
    lock_core();
}

// lock the DMA channels, non-blocking
bool Shared_DMA::lock_nonblock(void)
{
    if (!lock_stream_nonblocking(stream_id1)) {
        chSysDisable();
        if (locks[stream_id1].obj != nullptr && locks[stream_id1].obj != this) {
            locks[stream_id1].obj->contention = true;
        }
        chSysEnable();
        contention = true;
        return false;
    }
    if (!lock_stream_nonblocking(stream_id2)) {
        unlock_stream(stream_id1);
        chSysDisable();
        if (locks[stream_id2].obj != nullptr && locks[stream_id2].obj != this) {
            locks[stream_id2].obj->contention = true;
        }
        chSysEnable();
        contention = true;
        return false;
    }
    lock_core();
    return true;
}

// unlock the DMA channels
void Shared_DMA::unlock(void)
{
    osalDbgAssert(have_lock, "must have lock");
    unlock_stream(stream_id2);
    unlock_stream(stream_id1);
    have_lock = false;
}

// unlock the DMA channels from a lock zone
void Shared_DMA::unlock_from_lockzone(void)
{
    osalDbgAssert(have_lock, "must have lock");
    if (stream_id2 < SHARED_DMA_MAX_STREAM_ID) {
        unlock_stream_from_IRQ(stream_id2);
        chSchRescheduleS();
    }
    if (stream_id1 < SHARED_DMA_MAX_STREAM_ID) {
        unlock_stream_from_IRQ(stream_id1);
        chSchRescheduleS();
    }
    have_lock = false;
}

// unlock the DMA channels from an IRQ
void Shared_DMA::unlock_from_IRQ(void)
{
    osalDbgAssert(have_lock, "must have lock");
    unlock_stream_from_IRQ(stream_id2);
    unlock_stream_from_IRQ(stream_id1);
    have_lock = false;
}

/*
  lock all channels - used on reboot to ensure no sensor DMA is in
  progress
 */
void Shared_DMA::lock_all(void)
{
    for (uint8_t i=0; i<SHARED_DMA_MAX_STREAM_ID; i++) {
        lock_stream(i);
    }
}

#endif // CH_CFG_USE_SEMAPHORES && STM32_DMA_ADVANCED
