/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_NODE_GENERIC_PUBLISHER_HPP_INCLUDED
#define UAVCAN_NODE_GENERIC_PUBLISHER_HPP_INCLUDED

#include <uavcan/error.hpp>
#include <uavcan/node/abstract_node.hpp>
#include <uavcan/data_type.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/transport/transfer_buffer.hpp>
#include <uavcan/transport/transfer_sender.hpp>
#include <uavcan/marshal/scalar_codec.hpp>
#include <uavcan/marshal/types.hpp>

namespace uavcan
{

class GenericPublisherBase : Noncopyable
{
    TransferSender sender_;
    MonotonicDuration tx_timeout_;
    INode& node_;
    bool force_std_can_;
protected:
    GenericPublisherBase(INode& node, bool force_std_can, MonotonicDuration tx_timeout,
                         MonotonicDuration max_transfer_interval)
        : sender_(node.getDispatcher(), max_transfer_interval)
        , tx_timeout_(tx_timeout)
        , node_(node)
        , force_std_can_(force_std_can)
    {
        setTxTimeout(tx_timeout);
#if UAVCAN_DEBUG
        UAVCAN_ASSERT(getTxTimeout() == tx_timeout);  // Making sure default values are OK
#endif
    }

    ~GenericPublisherBase() { }

    bool isInited() const;

    bool isStdOnly() const { return force_std_can_; }

    int doInit(DataTypeKind dtkind, const char* dtname, CanTxQueue::Qos qos);

    MonotonicTime getTxDeadline() const;

    int genericPublish(const StaticTransferBufferImpl& buffer, TransferType transfer_type,
                       NodeID dst_node_id, TransferID* tid, MonotonicTime blocking_deadline);

    TransferSender& getTransferSender() { return sender_; }
    const TransferSender& getTransferSender() const { return sender_; }

public:
    static MonotonicDuration getMinTxTimeout() { return MonotonicDuration::fromUSec(200); }
    static MonotonicDuration getMaxTxTimeout() { return MonotonicDuration::fromMSec(60000); }

    MonotonicDuration getTxTimeout() const { return tx_timeout_; }
    void setTxTimeout(MonotonicDuration tx_timeout);

    /**
     * By default, attempt to send a transfer from passive mode will result in an error @ref ErrPassive.
     * This option allows to enable sending anonymous transfers from passive mode.
     */
    void allowAnonymousTransfers()
    {
        sender_.allowAnonymousTransfers();
    }

    /**
     * Priority of outgoing transfers.
     */
    TransferPriority getPriority() const { return sender_.getPriority(); }
    void setPriority(const TransferPriority prio) { sender_.setPriority(prio); }

    INode& getNode() const { return node_; }
};

/**
 * Generic publisher, suitable for messages and services.
 * DataSpec - data type specification class
 * DataStruct - instantiable class
 */
template <typename DataSpec, typename DataStruct>
class UAVCAN_EXPORT GenericPublisher : public GenericPublisherBase
{
    struct ZeroTransferBuffer : public StaticTransferBufferImpl
    {
        ZeroTransferBuffer() : StaticTransferBufferImpl(UAVCAN_NULLPTR, 0) { }
    };

    typedef typename Select<DataStruct::MaxBitLen == 0,
                            ZeroTransferBuffer,
                            StaticTransferBuffer<BitLenToByteLen<DataStruct::MaxBitLen>::Result> >::Result Buffer;

    enum
    {
        Qos = (DataTypeKind(DataSpec::DataTypeKind) == DataTypeKindMessage) ?
              CanTxQueue::Volatile : CanTxQueue::Persistent
    };

    int checkInit();

    int doEncode(const DataStruct& message, ITransferBuffer& buffer) const;

    int genericPublish(const DataStruct& message, TransferType transfer_type, NodeID dst_node_id,
                       TransferID* tid, MonotonicTime blocking_deadline);

public:
    /**
     * @param max_transfer_interval     Maximum expected time interval between subsequent publications. Leave default.
     */
    GenericPublisher(INode& node, bool force_std_can, MonotonicDuration tx_timeout,
                     MonotonicDuration max_transfer_interval = TransferSender::getDefaultMaxTransferInterval())
        : GenericPublisherBase(node, force_std_can, tx_timeout, max_transfer_interval)
    { }

    ~GenericPublisher() { }

    /**
     * Init method can be called prior first publication, but it's not necessary
     * because the publisher can be automatically initialized ad-hoc.
     */
    int init()
    {
        return checkInit();
    }

    /**
     * This overload allows to set the priority; otherwise it's the same.
     */
    int init(TransferPriority priority)
    {
        setPriority(priority);
        return checkInit();
    }

    int publish(const DataStruct& message, TransferType transfer_type, NodeID dst_node_id,
                MonotonicTime blocking_deadline = MonotonicTime())
    {
        return genericPublish(message, transfer_type, dst_node_id, UAVCAN_NULLPTR, blocking_deadline);
    }

    int publish(const DataStruct& message, TransferType transfer_type, NodeID dst_node_id, TransferID tid,
                MonotonicTime blocking_deadline = MonotonicTime())
    {
        return genericPublish(message, transfer_type, dst_node_id, &tid, blocking_deadline);
    }
};

// ----------------------------------------------------------------------------

template <typename DataSpec, typename DataStruct>
int GenericPublisher<DataSpec, DataStruct>::checkInit()
{
    if (isInited())
    {
        return 0;
    }
    return doInit(DataTypeKind(DataSpec::DataTypeKind), DataSpec::getDataTypeFullName(), CanTxQueue::Qos(Qos));
}

template <typename DataSpec, typename DataStruct>
int GenericPublisher<DataSpec, DataStruct>::doEncode(const DataStruct& message, ITransferBuffer& buffer) const
{
    BitStream bitstream(buffer);
    ScalarCodec codec(bitstream);
    // if doing canfd transfer tail array optimisation is disabled
    TailArrayOptimizationMode tao_mode = (!isStdOnly() && (getNode().isTaoDisabled() || getNode().isCanFdEnabled())) ? TailArrayOptDisabled:TailArrayOptEnabled;
    const int encode_res = DataStruct::encode(message, codec, tao_mode);
    if (encode_res <= 0)
    {
        UAVCAN_ASSERT(0);   // Impossible, internal error
        return -ErrInvalidMarshalData;
    }
    return encode_res;
}

template <typename DataSpec, typename DataStruct>
int GenericPublisher<DataSpec, DataStruct>::genericPublish(const DataStruct& message, TransferType transfer_type,
                                                           NodeID dst_node_id, TransferID* tid,
                                                           MonotonicTime blocking_deadline)
{
    const int res = checkInit();
    if (res < 0)
    {
        return res;
    }

    Buffer buffer;

    const int encode_res = doEncode(message, buffer);
    if (encode_res < 0)
    {
        return encode_res;
    }

    return GenericPublisherBase::genericPublish(buffer, transfer_type, dst_node_id, tid, blocking_deadline);
}

}

#endif // UAVCAN_NODE_GENERIC_PUBLISHER_HPP_INCLUDED
