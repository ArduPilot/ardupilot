/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_TRANSPORT_TRANSFER_LISTENER_HPP_INCLUDED
#define UAVCAN_TRANSPORT_TRANSFER_LISTENER_HPP_INCLUDED

#include <cassert>
#include <uavcan/error.hpp>
#include <uavcan/std.hpp>
#include <uavcan/transport/transfer_receiver.hpp>
#include <uavcan/transport/perf_counter.hpp>
#include <uavcan/util/linked_list.hpp>
#include <uavcan/util/map.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/transport/crc.hpp>
#include <uavcan/data_type.hpp>

namespace uavcan
{
/**
 * Container for received transfer.
 */
class UAVCAN_EXPORT IncomingTransfer : public ITransferBuffer
{
    MonotonicTime ts_mono_;
    UtcTime ts_utc_;
    TransferPriority transfer_priority_;
    TransferType transfer_type_;
    TransferID transfer_id_;
    NodeID src_node_id_;
    uint8_t iface_index_;
    bool canfd_transfer_;
    bool tao_disabled_;

    /// That's a no-op, asserts in debug builds
    virtual int write(unsigned offset, const uint8_t* data, unsigned len) override;

protected:
    IncomingTransfer(MonotonicTime ts_mono, UtcTime ts_utc, TransferPriority transfer_priority,
                     TransferType transfer_type, TransferID transfer_id, NodeID source_node_id,
                     uint8_t iface_index, bool canfd_transfer, bool tao_disabled)
        : ts_mono_(ts_mono)
        , ts_utc_(ts_utc)
        , transfer_priority_(transfer_priority)
        , transfer_type_(transfer_type)
        , transfer_id_(transfer_id)
        , src_node_id_(source_node_id)
        , iface_index_(iface_index)
        , canfd_transfer_(canfd_transfer)
        , tao_disabled_(tao_disabled)
    { }

public:
    /**
     * Dispose the payload buffer. Further calls to read() will not be possible.
     */
    virtual void release() { }

    /**
     * Whether this is a anonymous transfer
     */
    virtual bool isAnonymousTransfer() const { return false; }

    MonotonicTime getMonotonicTimestamp() const { return ts_mono_; }
    UtcTime getUtcTimestamp()             const { return ts_utc_; }
    TransferPriority getPriority()        const { return transfer_priority_; }
    TransferType getTransferType()        const { return transfer_type_; }
    TransferID getTransferID()            const { return transfer_id_; }
    NodeID getSrcNodeID()                 const { return src_node_id_; }
    uint8_t getIfaceIndex()               const { return iface_index_; }
    bool isCanFDTransfer()                const { return canfd_transfer_; }
    bool isTaoDisabled()                  const { return tao_disabled_; }
};

/**
 * Internal.
 */
class UAVCAN_EXPORT SingleFrameIncomingTransfer : public IncomingTransfer
{
    const uint8_t* const payload_;
    const uint8_t payload_len_;
public:
    explicit SingleFrameIncomingTransfer(const RxFrame& frm, bool tao_disabled);
    virtual int read(unsigned offset, uint8_t* data, unsigned len) const override;
    virtual bool isAnonymousTransfer() const override;
};

/**
 * Internal.
 */
class UAVCAN_EXPORT MultiFrameIncomingTransfer : public IncomingTransfer, Noncopyable
{
    TransferBufferAccessor& buf_acc_;
public:
    MultiFrameIncomingTransfer(MonotonicTime ts_mono, UtcTime ts_utc, const RxFrame& last_frame,
                               TransferBufferAccessor& tba, bool tao_disabled);
    virtual int read(unsigned offset, uint8_t* data, unsigned len) const override;
    virtual void release() override { buf_acc_.remove(); }
};

/**
 * Internal, refer to the transport dispatcher class.
 */
class UAVCAN_EXPORT TransferListener : public LinkedListNode<TransferListener>
{
    const DataTypeDescriptor& data_type_;
    TransferBufferManager bufmgr_;
    Map<TransferBufferManagerKey, TransferReceiver> receivers_;
    TransferPerfCounter& perf_;
    const TransferCRC crc_base_;                      ///< Pre-initialized with data type hash, thus constant
    bool allow_anonymous_transfers_;

    class TimedOutReceiverPredicate
    {
        const MonotonicTime ts_;
        TransferBufferManager& parent_bufmgr_;

    public:
        TimedOutReceiverPredicate(MonotonicTime arg_ts, TransferBufferManager& arg_bufmgr)
            : ts_(arg_ts)
            , parent_bufmgr_(arg_bufmgr)
        { }

        bool operator()(const TransferBufferManagerKey& key, const TransferReceiver& value) const;
    };

    bool checkPayloadCrc(const uint16_t compare_with, const ITransferBuffer& tbb) const;

protected:
    void handleReception(TransferReceiver& receiver, const RxFrame& frame, TransferBufferAccessor& tba, bool tao_disabled);
    void handleAnonymousTransferReception(const RxFrame& frame, bool tao_disabled);

    uint16_t actual_max_buffer_size(uint16_t max_buffer_size);

    virtual void handleIncomingTransfer(IncomingTransfer& transfer) = 0;

public:
    TransferListener(TransferPerfCounter& perf, const DataTypeDescriptor& data_type,
                     uint16_t max_buffer_size, IPoolAllocator& allocator)
        : data_type_(data_type)
        , bufmgr_(actual_max_buffer_size(max_buffer_size), allocator)
        , receivers_(allocator)
        , perf_(perf)
        , crc_base_(data_type.getSignature().toTransferCRC())
        , allow_anonymous_transfers_(false)
    { }

    virtual ~TransferListener();

    const DataTypeDescriptor& getDataTypeDescriptor() const { return data_type_; }

    /**
     * By default, anonymous transfers will be ignored.
     * This option allows to enable reception of anonymous transfers.
     */
    void allowAnonymousTransfers() { allow_anonymous_transfers_ = true; }

    void cleanup(MonotonicTime ts);

    virtual void handleFrame(const RxFrame& frame, bool tao_disabled);
};

/**
 * This class is used by transfer listener to decide if the frame should be accepted or ignored.
 */
class ITransferAcceptanceFilter
{
public:
    /**
     * If it returns false, the frame will be ignored, otherwise accepted.
     */
    virtual bool shouldAcceptFrame(const RxFrame& frame) const = 0;

    virtual ~ITransferAcceptanceFilter() { }
};

/**
 * This class should be derived by callers.
 */
class UAVCAN_EXPORT TransferListenerWithFilter : public TransferListener
{
    const ITransferAcceptanceFilter* filter_;

    virtual void handleFrame(const RxFrame& frame, bool tao_disabled) override;

public:
    TransferListenerWithFilter(TransferPerfCounter& perf, const DataTypeDescriptor& data_type,
                               uint16_t max_buffer_size, IPoolAllocator& allocator)
        : TransferListener(perf, data_type, max_buffer_size, allocator)
        , filter_(UAVCAN_NULLPTR)
    { }

    void installAcceptanceFilter(const ITransferAcceptanceFilter* acceptance_filter)
    {
        filter_ = acceptance_filter;
    }
};

}

#endif // UAVCAN_TRANSPORT_TRANSFER_LISTENER_HPP_INCLUDED
