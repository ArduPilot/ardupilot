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

#if CH_CFG_USE_SEMAPHORES == TRUE

using namespace ChibiOS;

Shared_DMA::dma_lock Shared_DMA::locks[SHARED_DMA_MAX_STREAM_ID];

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
    if (locks[stream_id1].obj == this) {
        locks[stream_id1].deallocate(this);
        locks[stream_id1].obj = nullptr;
    }

    if (locks[stream_id2].obj == this) {
        locks[stream_id2].deallocate(this);
        locks[stream_id2].obj = nullptr;
    }
}

// lock the DMA channels
void Shared_DMA::lock_core(void)
{
    // see if another driver has DMA allocated. If so, call their
    // deallocation function
    if (stream_id1 != SHARED_DMA_NONE &&
        locks[stream_id1].obj && locks[stream_id1].obj != this) {
        locks[stream_id1].deallocate(locks[stream_id1].obj);
        locks[stream_id1].obj = nullptr;
    }
    if (stream_id2 != SHARED_DMA_NONE &&
        locks[stream_id2].obj && locks[stream_id2].obj != this) {
        locks[stream_id2].deallocate(locks[stream_id2].obj);
        locks[stream_id2].obj = nullptr;
    }
    if ((stream_id1 != SHARED_DMA_NONE && locks[stream_id1].obj == nullptr) ||
        (stream_id2 != SHARED_DMA_NONE && locks[stream_id2].obj == nullptr)) {
        // allocate the DMA channels and put our deallocation function in place
        allocate(this);
        if (stream_id1 != SHARED_DMA_NONE) {
            locks[stream_id1].deallocate = deallocate;
            locks[stream_id1].obj = this;
        }
        if (stream_id2 != SHARED_DMA_NONE) {
            locks[stream_id2].deallocate = deallocate;
            locks[stream_id2].obj = this;
        }
    }
    have_lock = true;
}

// lock the DMA channels, blocking method
void Shared_DMA::lock(void)
{
    if (stream_id1 != SHARED_DMA_NONE) {
        chBSemWait(&locks[stream_id1].semaphore);
    }
    if (stream_id2 != SHARED_DMA_NONE) {
        chBSemWait(&locks[stream_id2].semaphore);
    }
    lock_core();
}

// lock the DMA channels, non-blocking
bool Shared_DMA::lock_nonblock(void)
{
    if (stream_id1 != SHARED_DMA_NONE) {
        if (chBSemWaitTimeout(&locks[stream_id1].semaphore, 1) != MSG_OK) {
            chSysDisable();
            if (locks[stream_id1].obj != nullptr && locks[stream_id1].obj != this) {
                locks[stream_id1].obj->contention = true;
            }
            chSysEnable();
            contention = true;
            return false;
        }
    }
    if (stream_id2 != SHARED_DMA_NONE) {
        if (chBSemWaitTimeout(&locks[stream_id2].semaphore, 1) != MSG_OK) {
            if (stream_id1 != SHARED_DMA_NONE) {
                chBSemSignal(&locks[stream_id1].semaphore);
            }
            chSysDisable();
            if (locks[stream_id2].obj != nullptr && locks[stream_id2].obj != this) {
                locks[stream_id2].obj->contention = true;
            }
            chSysEnable();
            contention = true;
            return false;
        }
    }
    lock_core();
    return true;
}

// unlock the DMA channels
void Shared_DMA::unlock(void)
{
    osalDbgAssert(have_lock, "must have lock");
    if (stream_id2 != SHARED_DMA_NONE) {
        chBSemSignal(&locks[stream_id2].semaphore);        
    }
    if (stream_id1 != SHARED_DMA_NONE) {
        chBSemSignal(&locks[stream_id1].semaphore);
    }
    have_lock = false;
}

// unlock the DMA channels from a lock zone
void Shared_DMA::unlock_from_lockzone(void)
{
    osalDbgAssert(have_lock, "must have lock");
    if (stream_id2 != SHARED_DMA_NONE) {
        chBSemSignalI(&locks[stream_id2].semaphore);
        chSchRescheduleS();
    }
    if (stream_id1 != SHARED_DMA_NONE) {
        chBSemSignalI(&locks[stream_id1].semaphore);
        chSchRescheduleS();
    }
    have_lock = false;
}

// unlock the DMA channels from an IRQ
void Shared_DMA::unlock_from_IRQ(void)
{
    osalDbgAssert(have_lock, "must have lock");
    if (stream_id2 != SHARED_DMA_NONE) {
        chBSemSignalI(&locks[stream_id2].semaphore);        
    }
    if (stream_id1 != SHARED_DMA_NONE) {
        chBSemSignalI(&locks[stream_id1].semaphore);
    }
    have_lock = false;
}

/*
  lock all channels - used on reboot to ensure no sensor DMA is in
  progress
 */
void Shared_DMA::lock_all(void)
{
    for (uint8_t i=0; i<SHARED_DMA_MAX_STREAM_ID; i++) {
        chBSemWait(&locks[i].semaphore);
    }
}

#endif // CH_CFG_USE_SEMAPHORES
