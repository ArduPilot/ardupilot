/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_TRANSPORT_TRANSFER_BUFFER_HPP_INCLUDED
#define UAVCAN_TRANSPORT_TRANSFER_BUFFER_HPP_INCLUDED

#include <uavcan/std.hpp>
#include <uavcan/error.hpp>
#include <uavcan/transport/frame.hpp>
#include <uavcan/transport/abstract_transfer_buffer.hpp>
#include <uavcan/util/linked_list.hpp>
#include <uavcan/dynamic_memory.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>

namespace uavcan
{
/**
 * Standalone static buffer
 */
class StaticTransferBufferImpl : public ITransferBuffer
{
    uint8_t* const data_;
    const uint16_t size_;
    uint16_t max_write_pos_;

public:
    StaticTransferBufferImpl(uint8_t* buf, uint16_t buf_size) :
        data_(buf),
        size_(buf_size),
        max_write_pos_(0)
    { }

    virtual int read(unsigned offset, uint8_t* data, unsigned len) const override;
    virtual int write(unsigned offset, const uint8_t* data, unsigned len) override;

    void reset();

    uint16_t getSize() const { return size_; }

    uint8_t* getRawPtr() { return data_; }
    const uint8_t* getRawPtr() const { return data_; }

    uint16_t getMaxWritePos() const { return max_write_pos_; }
    void setMaxWritePos(uint16_t value) { max_write_pos_ = value; }
};

template <uint16_t Size>
class UAVCAN_EXPORT StaticTransferBuffer : public StaticTransferBufferImpl
{
    uint8_t buffer_[Size];
public:
    StaticTransferBuffer() : StaticTransferBufferImpl(buffer_, Size)
    {
        StaticAssert<(Size > 0)>::check();
    }
};

/**
 * Internal for TransferBufferManager
 */
class UAVCAN_EXPORT TransferBufferManagerKey
{
    NodeID node_id_;
    uint8_t transfer_type_;

public:
    TransferBufferManagerKey()
        : transfer_type_(TransferType(0))
    {
        UAVCAN_ASSERT(isEmpty());
    }

    TransferBufferManagerKey(NodeID node_id, TransferType ttype)
        : node_id_(node_id)
        , transfer_type_(ttype)
    {
        UAVCAN_ASSERT(!isEmpty());
    }

    bool operator==(const TransferBufferManagerKey& rhs) const
    {
        return node_id_ == rhs.node_id_ && transfer_type_ == rhs.transfer_type_;
    }

    bool isEmpty() const { return !node_id_.isValid(); }

    NodeID getNodeID() const { return node_id_; }
    TransferType getTransferType() const { return TransferType(transfer_type_); }

#if UAVCAN_TOSTRING
    std::string toString() const;
#endif
};

/**
 * Resizable gather/scatter storage.
 * reset() call releases all memory blocks.
 * Supports unordered write operations - from higher to lower offsets
 */
class UAVCAN_EXPORT TransferBufferManagerEntry : public ITransferBuffer
                                               , public LinkedListNode<TransferBufferManagerEntry>
{
    struct Block : LinkedListNode<Block>
    {
        enum { Size = MemPoolBlockSize - sizeof(LinkedListNode<Block>) };
        uint8_t data[static_cast<unsigned>(Size)];

        static Block* instantiate(IPoolAllocator& allocator);
        static void destroy(Block*& obj, IPoolAllocator& allocator);

        void read(uint8_t*& outptr, unsigned target_offset,
                  unsigned& total_offset, unsigned& left_to_read);
        void write(const uint8_t*& inptr, unsigned target_offset,
                   unsigned& total_offset, unsigned& left_to_write);
    };

    IPoolAllocator& allocator_;
    LinkedListRoot<Block> blocks_;    // Blocks are ordered from lower to higher buffer offset
    uint16_t max_write_pos_;
    const uint16_t max_size_;
    TransferBufferManagerKey key_;

public:
    TransferBufferManagerEntry(IPoolAllocator& allocator, uint16_t max_size) :
        allocator_(allocator),
        max_write_pos_(0),
        max_size_(max_size)
    {
        StaticAssert<(Block::Size > 8)>::check();
        IsDynamicallyAllocatable<Block>::check();
        IsDynamicallyAllocatable<TransferBufferManagerEntry>::check();
    }

    virtual ~TransferBufferManagerEntry() { reset(); }

    static TransferBufferManagerEntry* instantiate(IPoolAllocator& allocator, uint16_t max_size);
    static void destroy(TransferBufferManagerEntry*& obj, IPoolAllocator& allocator);

    virtual int read(unsigned offset, uint8_t* data, unsigned len) const override;
    virtual int write(unsigned offset, const uint8_t* data, unsigned len) override;

    void reset(const TransferBufferManagerKey& key = TransferBufferManagerKey());

    const TransferBufferManagerKey& getKey() const { return key_; }
    bool isEmpty() const { return key_.isEmpty(); }
};

/**
 * Buffer manager implementation.
 */
class TransferBufferManager : public Noncopyable
{
    LinkedListRoot<TransferBufferManagerEntry> buffers_;
    IPoolAllocator& allocator_;
    const uint16_t max_buf_size_;

    TransferBufferManagerEntry* findFirst(const TransferBufferManagerKey& key);

public:
    TransferBufferManager(uint16_t max_buf_size, IPoolAllocator& allocator) :
        allocator_(allocator),
        max_buf_size_(max_buf_size)
    { }

    ~TransferBufferManager();

    ITransferBuffer* access(const TransferBufferManagerKey& key);
    ITransferBuffer* create(const TransferBufferManagerKey& key);
    void remove(const TransferBufferManagerKey& key);
    bool isEmpty() const;

    unsigned getNumBuffers() const;
};

/**
 * Convinience class.
 */
class UAVCAN_EXPORT TransferBufferAccessor
{
    TransferBufferManager& bufmgr_;
    const TransferBufferManagerKey key_;

public:
    TransferBufferAccessor(TransferBufferManager& bufmgr, TransferBufferManagerKey key) :
        bufmgr_(bufmgr),
        key_(key)
    {
        UAVCAN_ASSERT(!key.isEmpty());
    }
    ITransferBuffer* access() { return bufmgr_.access(key_); }
    ITransferBuffer* create() { return bufmgr_.create(key_); }
    void remove() { bufmgr_.remove(key_); }
};

}

#endif // UAVCAN_TRANSPORT_TRANSFER_BUFFER_HPP_INCLUDED
