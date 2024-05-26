/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/transport/transfer_buffer.hpp>
#include <cassert>
#include <cstdlib>

namespace uavcan
{
/*
 * StaticTransferBufferImpl
 */
int StaticTransferBufferImpl::read(unsigned offset, uint8_t* data, unsigned len) const
{
    if (!data)
    {
        UAVCAN_ASSERT(0);
        return -ErrInvalidParam;
    }
    if (offset >= max_write_pos_)
    {
        return 0;
    }
    if ((offset + len) > max_write_pos_)
    {
        len = max_write_pos_ - offset;
    }
    UAVCAN_ASSERT((offset + len) <= max_write_pos_);
    (void)copy(data_ + offset, data_ + offset + len, data);
    return int(len);
}

int StaticTransferBufferImpl::write(unsigned offset, const uint8_t* data, unsigned len)
{
    if (!data)
    {
        UAVCAN_ASSERT(0);
        return -ErrInvalidParam;
    }
    if (offset >= size_)
    {
        return 0;
    }
    if ((offset + len) > size_)
    {
        len = size_ - offset;
    }
    UAVCAN_ASSERT((offset + len) <= size_);
    (void)copy(data, data + len, data_ + offset);
    max_write_pos_ = max(uint16_t(offset + len), uint16_t(max_write_pos_));
    return int(len);
}

void StaticTransferBufferImpl::reset()
{
    max_write_pos_ = 0;
#if UAVCAN_DEBUG
    fill(data_, data_ + size_, uint8_t(0));
#endif
}

/*
 * TransferBufferManagerKey
 */
#if UAVCAN_TOSTRING
std::string TransferBufferManagerKey::toString() const
{
    char buf[24];
    (void)snprintf(buf, sizeof(buf), "nid=%i tt=%i", int(node_id_.get()), int(transfer_type_));
    return std::string(buf);
}
#endif

/*
 * DynamicTransferBuffer::Block
 */
TransferBufferManagerEntry::Block*
TransferBufferManagerEntry::Block::instantiate(IPoolAllocator& allocator)
{
    void* const praw = allocator.allocate(sizeof(Block));
    if (praw == UAVCAN_NULLPTR)
    {
        return UAVCAN_NULLPTR;
    }
    return new (praw) Block;
}

void TransferBufferManagerEntry::Block::destroy(Block*& obj, IPoolAllocator& allocator)
{
    if (obj != UAVCAN_NULLPTR)
    {
        obj->~Block();
        allocator.deallocate(obj);
        obj = UAVCAN_NULLPTR;
    }
}

void TransferBufferManagerEntry::Block::read(uint8_t*& outptr, unsigned target_offset,
                                             unsigned& total_offset, unsigned& left_to_read)
{
    UAVCAN_ASSERT(outptr);
    for (unsigned i = 0; (i < Block::Size) && (left_to_read > 0); i++, total_offset++)
    {
        if (total_offset >= target_offset)
        {
            *outptr++ = data[i];
            left_to_read--;
        }
    }
}

void TransferBufferManagerEntry::Block::write(const uint8_t*& inptr, unsigned target_offset,
                                                     unsigned& total_offset, unsigned& left_to_write)
{
    UAVCAN_ASSERT(inptr);
    for (unsigned i = 0; (i < Block::Size) && (left_to_write > 0); i++, total_offset++)
    {
        if (total_offset >= target_offset)
        {
            data[i] = *inptr++;
            left_to_write--;
        }
    }
}

/*
 * DynamicTransferBuffer
 */
TransferBufferManagerEntry* TransferBufferManagerEntry::instantiate(IPoolAllocator& allocator,
                                                                                  uint16_t max_size)
{
    void* const praw = allocator.allocate(sizeof(TransferBufferManagerEntry));
    if (praw == UAVCAN_NULLPTR)
    {
        return UAVCAN_NULLPTR;
    }
    return new (praw) TransferBufferManagerEntry(allocator, max_size);
}

void TransferBufferManagerEntry::destroy(TransferBufferManagerEntry*& obj, IPoolAllocator& allocator)
{
    if (obj != UAVCAN_NULLPTR)
    {
        obj->~TransferBufferManagerEntry();
        allocator.deallocate(obj);
        obj = UAVCAN_NULLPTR;
    }
}

int TransferBufferManagerEntry::read(unsigned offset, uint8_t* data, unsigned len) const
{
    if (!data)
    {
        UAVCAN_ASSERT(0);
        return -ErrInvalidParam;
    }
    if (offset >= max_write_pos_)
    {
        return 0;
    }
    if ((offset + len) > max_write_pos_)
    {
        len = max_write_pos_ - offset;
    }
    UAVCAN_ASSERT((offset + len) <= max_write_pos_);

    // This shall be optimized.
    unsigned total_offset = 0;
    unsigned left_to_read = len;
    uint8_t* outptr = data;
    Block* p = blocks_.get();
    while (p)
    {
        p->read(outptr, offset, total_offset, left_to_read);
        if (left_to_read == 0)
        {
            break;
        }
        p = p->getNextListNode();
    }

    UAVCAN_ASSERT(left_to_read == 0);
    return int(len);
}

int TransferBufferManagerEntry::write(unsigned offset, const uint8_t* data, unsigned len)
{
    if (!data)
    {
        UAVCAN_ASSERT(0);
        return -ErrInvalidParam;
    }

    if (offset >= max_size_)
    {
        return 0;
    }
    if ((offset + len) > max_size_)
    {
        len = max_size_ - offset;
    }
    UAVCAN_ASSERT((offset + len) <= max_size_);

    unsigned total_offset = 0;
    unsigned left_to_write = len;
    const uint8_t* inptr = data;
    Block* p = blocks_.get();
    Block* last_written_block = UAVCAN_NULLPTR;

    // First we need to write the part that is already allocated
    while (p)
    {
        last_written_block = p;
        p->write(inptr, offset, total_offset, left_to_write);
        if (left_to_write == 0)
        {
            break;
        }
        p = p->getNextListNode();
    }

    // Then we need to append new chunks until all data is written
    while (left_to_write > 0)
    {
        // cppcheck-suppress nullPointer
        UAVCAN_ASSERT(p == UAVCAN_NULLPTR);

        // Allocating the chunk
        Block* new_block = Block::instantiate(allocator_);
        if (new_block == UAVCAN_NULLPTR)
        {
            break;                        // We're in deep shit.
        }
        // Appending the chain with the new block
        if (last_written_block != UAVCAN_NULLPTR)
        {
            UAVCAN_ASSERT(last_written_block->getNextListNode() == UAVCAN_NULLPTR);  // Because it is last in the chain
            last_written_block->setNextListNode(new_block);
            new_block->setNextListNode(UAVCAN_NULLPTR);
        }
        else
        {
            blocks_.insert(new_block);
        }
        last_written_block = new_block;

        // Writing the data
        new_block->write(inptr, offset, total_offset, left_to_write);
    }

    UAVCAN_ASSERT(len >= left_to_write);
    const unsigned actually_written = len - left_to_write;
    max_write_pos_ = max(uint16_t(offset + actually_written), uint16_t(max_write_pos_));
    return int(actually_written);
}

void TransferBufferManagerEntry::reset(const TransferBufferManagerKey& key)
{
    key_ = key;
    max_write_pos_ = 0;
    Block* p = blocks_.get();
    while (p)
    {
        Block* const next = p->getNextListNode();
        blocks_.remove(p);
        Block::destroy(p, allocator_);
        p = next;
    }
}

/*
 * TransferBufferManager
 */
TransferBufferManagerEntry* TransferBufferManager::findFirst(const TransferBufferManagerKey& key)
{
    TransferBufferManagerEntry* dyn = buffers_.get();
    while (dyn)
    {
        UAVCAN_ASSERT(!dyn->isEmpty());
        if (dyn->getKey() == key)
        {
            return dyn;
        }
        dyn = dyn->getNextListNode();
    }
    return UAVCAN_NULLPTR;
}

TransferBufferManager::~TransferBufferManager()
{
    TransferBufferManagerEntry* dyn = buffers_.get();
    while (dyn)
    {
        TransferBufferManagerEntry* const next = dyn->getNextListNode();
        buffers_.remove(dyn);
        TransferBufferManagerEntry::destroy(dyn, allocator_);
        dyn = next;
    }
}

ITransferBuffer* TransferBufferManager::access(const TransferBufferManagerKey& key)
{
    if (key.isEmpty())
    {
        UAVCAN_ASSERT(0);
        return UAVCAN_NULLPTR;
    }
    return findFirst(key);
}

ITransferBuffer* TransferBufferManager::create(const TransferBufferManagerKey& key)
{
    if (key.isEmpty())
    {
        UAVCAN_ASSERT(0);
        return UAVCAN_NULLPTR;
    }
    remove(key);

    TransferBufferManagerEntry* tbme = TransferBufferManagerEntry::instantiate(allocator_, max_buf_size_);
    if (tbme == UAVCAN_NULLPTR)
    {
        return UAVCAN_NULLPTR;     // Epic fail.
    }

    buffers_.insert(tbme);

    UAVCAN_TRACE("TransferBufferManager", "Buffer created [num=%u], %s", getNumBuffers(), key.toString().c_str());

    if (tbme != UAVCAN_NULLPTR)
    {
        UAVCAN_ASSERT(tbme->isEmpty());
        tbme->reset(key);
    }
    return tbme;
}

void TransferBufferManager::remove(const TransferBufferManagerKey& key)
{
    UAVCAN_ASSERT(!key.isEmpty());

    TransferBufferManagerEntry* dyn = findFirst(key);
    if (dyn != UAVCAN_NULLPTR)
    {
        UAVCAN_TRACE("TransferBufferManager", "Buffer deleted, %s", key.toString().c_str());
        buffers_.remove(dyn);
        TransferBufferManagerEntry::destroy(dyn, allocator_);
    }
}

bool TransferBufferManager::isEmpty() const
{
    return getNumBuffers() == 0;
}

unsigned TransferBufferManager::getNumBuffers() const
{
    return buffers_.getLength();
}

}
