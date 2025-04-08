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

#include <hal.h>
#include "shared_dma.h"

/*
  code to handle sharing of DMA channels between peripherals
 */

#if CH_CFG_USE_MUTEXES == TRUE && AP_HAL_SHARED_DMA_ENABLED

#include <AP_Common/ExpandingString.h>

using namespace ChibiOS;
extern const AP_HAL::HAL& hal;

Shared_DMA::dma_lock Shared_DMA::locks[SHARED_DMA_MAX_STREAM_ID+1];
volatile Shared_DMA::dma_stats* Shared_DMA::_contention_stats;

void Shared_DMA::init(void)
{
    for (uint8_t i=0; i<SHARED_DMA_MAX_STREAM_ID; i++) {
        chMtxObjectInit(&locks[i].mutex);
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
    if (stream_id2 < stream_id1) {
        stream_id1 = _stream_id2;
        stream_id2 = _stream_id1;
    }
    allocate = _allocate;
    deallocate = _deallocate;
}

/*
  return true if a stream ID is shared between two peripherals
*/
bool Shared_DMA::is_shared(uint8_t stream_id)
{
    return (stream_id < SHARED_DMA_MAX_STREAM_ID) && ((1U<<stream_id) & SHARED_DMA_MASK) != 0;
}

bool Shared_DMA::is_shared()
{
    return is_shared(stream_id1) || is_shared(stream_id2);
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
bool Shared_DMA::lock_stream(uint8_t stream_id)
{
    bool cont = false;
    if (stream_id < SHARED_DMA_MAX_STREAM_ID) {
        const thread_t* curr_owner = locks[stream_id].mutex.owner;
        chMtxLock(&locks[stream_id].mutex);
        cont = curr_owner != nullptr && curr_owner != locks[stream_id].mutex.owner;
    }
    return cont;
}

// unlock one stream
void Shared_DMA::unlock_stream(uint8_t stream_id, bool success)
{
    if (stream_id < SHARED_DMA_MAX_STREAM_ID) {
        chMtxUnlock(&locks[stream_id].mutex);
        if (success && _contention_stats != nullptr) {
            _contention_stats[stream_id1].transactions++;
        }
    }
}

// lock one stream, non-blocking
bool Shared_DMA::lock_stream_nonblocking(uint8_t stream_id)
{
    if (stream_id < SHARED_DMA_MAX_STREAM_ID) {
        return chMtxTryLock(&locks[stream_id].mutex);
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
    // update contention stats
    if (_contention_stats != nullptr) {
        if (stream_id1 < SHARED_DMA_MAX_STREAM_ID) {
            if (contention) {
                _contention_stats[stream_id1].contended_locks++;
            } else {
                _contention_stats[stream_id1].uncontended_locks++;
            }
        }
        if (stream_id2 < SHARED_DMA_MAX_STREAM_ID) {
            if (contention) {
                _contention_stats[stream_id2].contended_locks++;
            } else {
                _contention_stats[stream_id2].uncontended_locks++;
            }
        }
    }
    have_lock = true;
}

// lock the DMA channels, blocking method
void Shared_DMA::lock(void)
{
    bool c1 = lock_stream(stream_id1);
    bool c2 = lock_stream(stream_id2);
    contention = c1 || c2;
    lock_core();
}

// lock the DMA channels, non-blocking
bool Shared_DMA::lock_nonblock(void)
{
    if (!lock_stream_nonblocking(stream_id1)) {
        chSysDisable();
        if (locks[stream_id1].obj != nullptr && locks[stream_id1].obj != this) {
            locks[stream_id1].obj->contention = true;
            if (_contention_stats != nullptr) {
                _contention_stats[stream_id1].contended_locks++;
            }
        }
        chSysEnable();
        contention = true;
        return false;
    }

    if (_contention_stats != nullptr && stream_id1 < SHARED_DMA_MAX_STREAM_ID) {
        _contention_stats[stream_id1].uncontended_locks++;
    }

    if (!lock_stream_nonblocking(stream_id2)) {
        unlock_stream(stream_id1, false);
        chSysDisable();
        if (locks[stream_id2].obj != nullptr && locks[stream_id2].obj != this) {
            locks[stream_id2].obj->contention = true;
            if (_contention_stats != nullptr) {
                _contention_stats[stream_id2].contended_locks++;
            }
        }
        chSysEnable();
        contention = true;
        return false;
    }
    lock_core();
    if (_contention_stats != nullptr && stream_id2 < SHARED_DMA_MAX_STREAM_ID) {
        _contention_stats[stream_id2].uncontended_locks++;
    }
    return true;
}

// unlock the DMA channels
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wframe-larger-than=128"
void Shared_DMA::unlock(bool success)
{
    osalDbgAssert(have_lock, "must have lock");
    have_lock = false;
    unlock_stream(stream_id2, success);
    unlock_stream(stream_id1, success);
}
#pragma GCC diagnostic pop

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

// display dma contention statistics as text buffer for @SYS/dma.txt
void Shared_DMA::dma_info(ExpandingString &str)
{
    // no buffer allocated, start counting
    if (_contention_stats == nullptr) {
        _contention_stats = NEW_NOTHROW dma_stats[SHARED_DMA_MAX_STREAM_ID+1];
        // return zeros on first fetch
    }

    // a header to allow for machine parsers to determine format
    str.printf("DMAV1\n");

    for (uint8_t i = 0; i < SHARED_DMA_MAX_STREAM_ID; i++) {
        // ignore locks not in use
        if (_contention_stats[i].contended_locks == 0
            && _contention_stats[i].uncontended_locks == 0
            && _contention_stats[i].transactions == 0) {
            continue;
        }
#if STM32_DMA_ADVANCED
#define STREAM_MUX 8
#define STREAM_OFFSET 0
#else
#define STREAM_MUX 7
#define STREAM_OFFSET 1
#endif
        const char* fmt = "DMA=%1u:%1u TX=%8u ULCK=%8u CLCK=%8u CONT=%4.1f%%\n";
        float cond_per = 100.0f * float(_contention_stats[i].contended_locks)
            / (1 + _contention_stats[i].contended_locks + _contention_stats[i].uncontended_locks);
        str.printf(fmt, i / STREAM_MUX + 1, i % STREAM_MUX + STREAM_OFFSET,  _contention_stats[i].transactions,
            _contention_stats[i].uncontended_locks, _contention_stats[i].contended_locks, cond_per);

        _contention_stats[i].contended_locks = 0;
        _contention_stats[i].uncontended_locks = 0;
    }
}

#endif // CH_CFG_USE_SEMAPHORES

