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
#pragma once

#include "AP_HAL_ChibiOS.h"

#if STM32_DMA_ADVANCED

#define SHARED_DMA_MAX_STREAM_ID (8*2)

// DMA stream ID for stream_id2 when only one is needed
#define SHARED_DMA_NONE 255

class ChibiOS::Shared_DMA
{
public:
    FUNCTOR_TYPEDEF(dma_allocate_fn_t, void, Shared_DMA *);
    FUNCTOR_TYPEDEF(dma_deallocate_fn_t, void, Shared_DMA *);

    // the use of two stream IDs is for support of peripherals that
    // need both a RX and TX DMA channel
    Shared_DMA(uint8_t stream_id1, uint8_t stream_id2,
               dma_allocate_fn_t allocate,
               dma_allocate_fn_t deallocate);

    // initialise the stream locks
    static void init(void);
    
    // blocking lock call
    void lock(void);

    // non-blocking lock call
    bool lock_nonblock(void);
    
    // unlock call. The DMA channel will not be immediately
    // deallocated. Instead it will be deallocated if another driver
    // needs it
    void unlock(void);

    // unlock call from an IRQ
    void unlock_from_IRQ(void);

    // unlock call from a chSysLock zone
    void unlock_from_lockzone(void);
    
    //should be called inside the destructor of Shared DMA participants
    void unregister(void);

    // return true if this DMA channel is being actively contended for
    // by multiple drivers
    bool has_contention(void) const { return contention; }
    
    // lock all shared DMA channels. Used on reboot
    static void lock_all(void);
    
private:
    dma_allocate_fn_t allocate;
    dma_allocate_fn_t deallocate;
    uint8_t stream_id1;
    uint8_t stream_id2;
    bool have_lock;

    // we set the contention flag if two drivers are fighting over a DMA channel.
    // the UART driver uses this to change its max transmit size to reduce latency
    bool contention;

    // core of lock call, after semaphores gained
    void lock_core(void);

    // lock one stream
    static void lock_stream(uint8_t stream_id);

    // unlock one stream
    void unlock_stream(uint8_t stream_id);

    // unlock one stream from an IRQ handler
    void unlock_stream_from_IRQ(uint8_t stream_id);

    // lock one stream, non-blocking
    bool lock_stream_nonblocking(uint8_t stream_id);
    
    static struct dma_lock {
        // semaphore to ensure only one peripheral uses a DMA channel at a time
#if CH_CFG_USE_SEMAPHORES == TRUE
        binary_semaphore_t semaphore;
#endif // CH_CFG_USE_SEMAPHORES

        // a de-allocation function that is called to release an existing user
        dma_deallocate_fn_t deallocate;

        // point to object that holds the allocation, if allocated
        Shared_DMA *obj;
    } locks[SHARED_DMA_MAX_STREAM_ID+1];
};
#endif //#if STM32_DMA_ADVANCED


