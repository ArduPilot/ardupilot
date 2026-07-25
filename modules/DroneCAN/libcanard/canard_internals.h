/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016-2017 UAVCAN Team
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

/*
 * This file holds function declarations that expose the library's internal definitions for unit testing.
 * It is NOT part of the library's API and should not even be looked at by the user.
 */

#ifndef CANARD_INTERNALS_H
#define CANARD_INTERNALS_H

#include "canard.h"

#ifdef __cplusplus
extern "C" {
#endif

/// This macro is needed only for testing and development. Do not redefine this in production.
#ifndef CANARD_INTERNAL
# define CANARD_INTERNAL static
#endif

/*
 * Some MCUs like TMS320 have 16-bits addressing, so
 * 1.   (uint8_t) same (uint16_t)
 * 2.   sizeof(float) is 2
 * 3.   union not same like STM32, because type uint8_t does not exist in hardware
 *
 *      union
 *      {
 *          uint64_t u8;
 *          uint64_t u16;
 *          uint64_t u32;
 *          uint64_t u64;
 *          uint8_t bytes[8];
 *      } storage;
 *
 *      address:|   bytes:      |   u64:            |   u32:            |   u16:    |   u8:
 *      0x00    |   bytes[0]    |   (u64    )&0xFF  |   (u32    )&0xFF  |   u16     |   u8
 *      0x01    |   bytes[1]    |   (u64>>16)&0xFF  |   (u32>>16)&0xFF  |
 *      0x02    |   bytes[2]    |   (u64>>32)&0xFF  |
 *      0x03    |   bytes[3]    |   (u64>>48)&0xFF  |
 *      0x04    |   bytes[4]    |
 *      0x05    |   bytes[5]    |
 *      0x06    |   bytes[6]    |
 *      0x07    |   bytes[7]    |
 *
 */
#ifndef WORD_ADDRESSING_IS_16BITS
#if defined(__TI_COMPILER_VERSION__) || defined(__TMS320C2000__)
#define WORD_ADDRESSING_IS_16BITS 1
#else
#define WORD_ADDRESSING_IS_16BITS 0
#endif
#endif

#if WORD_ADDRESSING_IS_16BITS
# define uint8_t               uint16_t
# define int8_t                int16_t
# define CANARD_SIZEOF_FLOAT   2
#else
# define CANARD_SIZEOF_FLOAT   4
#endif

CANARD_INTERNAL CanardRxState* traverseRxStates(CanardInstance* ins,
                                                uint32_t transfer_descriptor);

CANARD_INTERNAL CanardRxState* createRxState(CanardPoolAllocator* allocator,
                                             uint32_t transfer_descriptor);

CANARD_INTERNAL CanardRxState* prependRxState(CanardInstance* ins,
                                              uint32_t transfer_descriptor);

CANARD_INTERNAL CanardRxState* findRxState(CanardInstance *ins,
                                           uint32_t transfer_descriptor);

CANARD_INTERNAL int16_t bufferBlockPushBytes(CanardPoolAllocator* allocator,
                                             CanardRxState* state,
                                             const uint8_t* data,
                                             uint8_t data_len);

CANARD_INTERNAL CanardBufferBlock* createBufferBlock(CanardPoolAllocator* allocator);

CANARD_INTERNAL void pushTxQueue(CanardInstance* ins,
                                 CanardTxQueueItem* item);

CANARD_INTERNAL bool isPriorityHigher(uint32_t id,
                                      uint32_t rhs);

CANARD_INTERNAL CanardTxQueueItem* createTxItem(CanardPoolAllocator* allocator);

CANARD_INTERNAL void prepareForNextTransfer(CanardRxState* state);

CANARD_INTERNAL int16_t computeTransferIDForwardDistance(uint8_t a,
                                                         uint8_t b);

CANARD_INTERNAL void incrementTransferID(uint8_t* transfer_id);

CANARD_INTERNAL uint64_t releaseStatePayload(CanardInstance* ins,
                                             CanardRxState* rxstate);

CANARD_INTERNAL uint16_t dlcToDataLength(uint16_t dlc);
CANARD_INTERNAL uint16_t dataLengthToDlc(uint16_t data_length);

/// Returns the number of frames enqueued
CANARD_INTERNAL int16_t enqueueTxFrames(CanardInstance* ins,
                                        uint32_t can_id,
                                        uint16_t crc,
                                        CanardTxTransfer* transfer);

CANARD_INTERNAL void copyBitArray(const uint8_t* src,
                                  uint32_t src_offset,
                                  uint32_t src_len,
                                  uint8_t* dst,
                                  uint32_t dst_offset);

/**
 * Moves specified bits from the scattered transfer storage to a specified contiguous buffer.
 * Returns the number of bits copied, or negated error code.
 */
CANARD_INTERNAL int16_t descatterTransferPayload(const CanardRxTransfer* transfer,
                                                 uint32_t bit_offset,
                                                 uint8_t bit_length,
                                                 void* output);

CANARD_INTERNAL bool isBigEndian(void);

CANARD_INTERNAL void swapByteOrder(void* data, unsigned size);

/*
 * Transfer CRC
 */
CANARD_INTERNAL uint16_t crcAddByte(uint16_t crc_val,
                                    uint8_t byte);

CANARD_INTERNAL uint16_t crcAddSignature(uint16_t crc_val,
                                         uint64_t data_type_signature);

CANARD_INTERNAL uint16_t crcAdd(uint16_t crc_val,
                                const uint8_t* bytes,
                                size_t len);

/**
 * Inits a memory allocator.
 *
 * @param [in] allocator The memory allocator to initialize.
 * @param [in] buf The buffer used by the memory allocator.
 * @param [in] buf_len The number of blocks in buf.
 */
CANARD_INTERNAL void initPoolAllocator(CanardPoolAllocator* allocator,
                                       void *buf,
                                       uint16_t buf_len);

/**
 * Allocates a block from the given pool allocator.
 */
CANARD_INTERNAL void* allocateBlock(CanardPoolAllocator* allocator);

/**
 * Frees a memory block previously returned by canardAllocateBlock.
 */
CANARD_INTERNAL void freeBlock(CanardPoolAllocator* allocator,
                               void* p);

CANARD_INTERNAL uint16_t calculateCRC(const CanardTxTransfer* transfer_object);

CANARD_INTERNAL CanardBufferBlock *canardBufferFromIdx(CanardPoolAllocator* allocator, canard_buffer_idx_t idx);

CANARD_INTERNAL canard_buffer_idx_t canardBufferToIdx(CanardPoolAllocator* allocator, const CanardBufferBlock *buf);

CANARD_INTERNAL CanardRxState *canardRxFromIdx(CanardPoolAllocator* allocator, canard_buffer_idx_t idx);

CANARD_INTERNAL canard_buffer_idx_t canardRxToIdx(CanardPoolAllocator* allocator, const CanardRxState *rx);

#ifdef __cplusplus
}
#endif
#endif
