/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_NODE_GENERIC_SUBSCRIBER_HPP_INCLUDED
#define UAVCAN_NODE_GENERIC_SUBSCRIBER_HPP_INCLUDED

#include <uavcan/error.hpp>
#include <uavcan/node/abstract_node.hpp>
#include <uavcan/data_type.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/util/templates.hpp>
#include <uavcan/util/lazy_constructor.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/transport/transfer_listener.hpp>
#include <uavcan/marshal/scalar_codec.hpp>
#include <uavcan/marshal/types.hpp>

namespace uavcan
{
/**
 * This class extends the data structure with extra information obtained from the transport layer,
 * such as Source Node ID, timestamps, Transfer ID, index of the interface this transfer was picked up from, etc.
 *
 * PLEASE NOTE that since this class inherits the data structure type, subscription callbacks can accept either
 * object of this class or the data structure type directly if the extra information is not needed.
 *
 * For example, both of these callbacks can be used with the same data structure 'Foo':
 *  void first(const ReceivedDataStructure<Foo>& msg);
 *  void second(const Foo& msg);
 * In the latter case, an implicit cast will happen before the callback is invoked.
 *
 * This class is not copyable because it holds a reference to a stack-allocated transfer descriptor object.
 * You can slice cast it to the underlying data type though, which would be copyable:
 *  DataType dt = rds;  // where rds is of type ReceivedDataStructure<DataType>
 *  // dt is now copyable
 */
template <typename DataType_>
class UAVCAN_EXPORT ReceivedDataStructure : public DataType_, Noncopyable
{
    const IncomingTransfer* const _transfer_;   ///< Such weird name is necessary to avoid clashing with DataType fields

    template <typename Ret, Ret(IncomingTransfer::*Fun) () const>
    Ret safeget() const
    {
        if (_transfer_ == UAVCAN_NULLPTR)
        {
            return Ret();
        }
        return (_transfer_->*Fun)();
    }

protected:
    ReceivedDataStructure()
        : _transfer_(UAVCAN_NULLPTR)
    { }

    ReceivedDataStructure(const IncomingTransfer* arg_transfer)
        : _transfer_(arg_transfer)
    {
        UAVCAN_ASSERT(arg_transfer != UAVCAN_NULLPTR);
    }

public:
    typedef DataType_ DataType;

    MonotonicTime getMonotonicTimestamp() const
    {
        return safeget<MonotonicTime, &IncomingTransfer::getMonotonicTimestamp>();
    }
    UtcTime getUtcTimestamp()        const { return safeget<UtcTime, &IncomingTransfer::getUtcTimestamp>(); }
    TransferPriority getPriority()   const { return safeget<TransferPriority, &IncomingTransfer::getPriority>(); }
    TransferType getTransferType()   const { return safeget<TransferType, &IncomingTransfer::getTransferType>(); }
    TransferID getTransferID()       const { return safeget<TransferID, &IncomingTransfer::getTransferID>(); }
    NodeID getSrcNodeID()            const { return safeget<NodeID, &IncomingTransfer::getSrcNodeID>(); }
    uint8_t getIfaceIndex()          const { return safeget<uint8_t, &IncomingTransfer::getIfaceIndex>(); }
    bool isAnonymousTransfer()       const { return safeget<bool, &IncomingTransfer::isAnonymousTransfer>(); }
    bool isCanFDTransfer()           const { return safeget<bool, &IncomingTransfer::isCanFDTransfer>(); }
    bool isTaoDisabled()             const { return safeget<bool, &IncomingTransfer::isTaoDisabled>(); }
};

/**
 * This operator neatly prints the data structure prepended with extra data from the transport layer.
 * The extra data will be represented as YAML comment.
 */
template <typename Stream, typename DataType>
static Stream& operator<<(Stream& s, const ReceivedDataStructure<DataType>& rds)
{
    s << "# Received struct ts_m=" << rds.getMonotonicTimestamp()
      << " ts_utc=" << rds.getUtcTimestamp()
      << " snid=" << int(rds.getSrcNodeID().get()) << "\n";
    s << static_cast<const DataType&>(rds);
    return s;
}


class GenericSubscriberBase : Noncopyable
{
protected:
    INode& node_;
    uint32_t failure_count_;

    explicit GenericSubscriberBase(INode& node)
        : node_(node)
        , failure_count_(0)
    { }

    ~GenericSubscriberBase() { }

    int genericStart(TransferListener* listener, bool (Dispatcher::*registration_method)(TransferListener*));

    void stop(TransferListener* listener);

public:
    /**
     * Returns the number of failed attempts to decode received message. Generally, a failed attempt means either:
     * - Transient failure in the transport layer.
     * - Incompatible data types.
     */
    uint32_t getFailureCount() const { return failure_count_; }

    INode& getNode() const { return node_; }
};

/**
 * Please note that the reference passed to the RX callback points to a stack-allocated object, which means
 * that it gets invalidated shortly after the callback returns.
 */
template <typename DataSpec, typename DataStruct, typename TransferListenerType>
class UAVCAN_EXPORT GenericSubscriber : public GenericSubscriberBase
{
    typedef GenericSubscriber<DataSpec, DataStruct, TransferListenerType> SelfType;

    // We need to break the inheritance chain here to implement lazy initialization
    class TransferForwarder : public TransferListenerType
    {
        SelfType& obj_;

        void handleIncomingTransfer(IncomingTransfer& transfer) override
        {
            obj_.handleIncomingTransfer(transfer);
        }

    public:
        TransferForwarder(SelfType& obj,
                          const DataTypeDescriptor& data_type,
                          uint16_t max_buffer_size,
                          IPoolAllocator& allocator) :
            TransferListenerType(obj.node_.getDispatcher().getTransferPerfCounter(),
                                 data_type,
                                 max_buffer_size,
                                 allocator),
            obj_(obj)
        { }
    };

    LazyConstructor<TransferForwarder> forwarder_;

    int checkInit();

    void handleIncomingTransfer(IncomingTransfer& transfer);

    int genericStart(bool (Dispatcher::*registration_method)(TransferListener*));

protected:
    struct ReceivedDataStructureSpec : public ReceivedDataStructure<DataStruct>
    {
        ReceivedDataStructureSpec() { }

        ReceivedDataStructureSpec(const IncomingTransfer* arg_transfer) :
            ReceivedDataStructure<DataStruct>(arg_transfer)
        { }
    };

    explicit GenericSubscriber(INode& node) : GenericSubscriberBase(node)
    { }

    virtual ~GenericSubscriber() { stop(); }

    virtual void handleReceivedDataStruct(ReceivedDataStructure<DataStruct>&) = 0;

    int startAsMessageListener()
    {
        UAVCAN_TRACE("GenericSubscriber", "Start as message listener; dtname=%s", DataSpec::getDataTypeFullName());
        return genericStart(&Dispatcher::registerMessageListener);
    }

    int startAsServiceRequestListener()
    {
        UAVCAN_TRACE("GenericSubscriber", "Start as service request listener; dtname=%s",
                     DataSpec::getDataTypeFullName());
        return genericStart(&Dispatcher::registerServiceRequestListener);
    }

    int startAsServiceResponseListener()
    {
        UAVCAN_TRACE("GenericSubscriber", "Start as service response listener; dtname=%s",
                     DataSpec::getDataTypeFullName());
        return genericStart(&Dispatcher::registerServiceResponseListener);
    }

    /**
     * By default, anonymous transfers will be ignored.
     * This option allows to enable reception of anonymous transfers.
     */
    void allowAnonymousTransfers()
    {
        forwarder_->allowAnonymousTransfers();
    }

    /**
     * Terminate the subscription.
     * Dispatcher core will remove this instance from the subscribers list.
     */
    void stop()
    {
        UAVCAN_TRACE("GenericSubscriber", "Stop; dtname=%s", DataSpec::getDataTypeFullName());
        GenericSubscriberBase::stop(forwarder_);
    }

    TransferListenerType* getTransferListener() { return forwarder_; }
};

// ----------------------------------------------------------------------------

/*
 * GenericSubscriber
 */
template <typename DataSpec, typename DataStruct, typename TransferListenerType>
int GenericSubscriber<DataSpec, DataStruct, TransferListenerType>::checkInit()
{
    if (forwarder_)
    {
        return 0;
    }

    GlobalDataTypeRegistry::instance().freeze();
    const DataTypeDescriptor* const descr =
        GlobalDataTypeRegistry::instance().find(DataTypeKind(DataSpec::DataTypeKind), DataSpec::getDataTypeFullName());
    if (descr == UAVCAN_NULLPTR)
    {
        UAVCAN_TRACE("GenericSubscriber", "Type [%s] is not registered", DataSpec::getDataTypeFullName());
        return -ErrUnknownDataType;
    }

    static const uint16_t MaxBufferSize = BitLenToByteLen<DataStruct::MaxBitLen>::Result;

    forwarder_.template construct<SelfType&, const DataTypeDescriptor&, uint16_t, IPoolAllocator&>
        (*this, *descr, MaxBufferSize, node_.getAllocator());

    return 0;
}

template <typename DataSpec, typename DataStruct, typename TransferListenerType>
void GenericSubscriber<DataSpec, DataStruct, TransferListenerType>::handleIncomingTransfer(IncomingTransfer& transfer)
{
    ReceivedDataStructureSpec rx_struct(&transfer);

    /*
     * Decoding into the temporary storage
     */
    BitStream bitstream(transfer);
    ScalarCodec codec(bitstream);

    // disable tail array optimisation if CANFD transfer
    TailArrayOptimizationMode tao_mode = (transfer.isCanFDTransfer() || transfer.isTaoDisabled()) ? TailArrayOptDisabled:TailArrayOptEnabled;
    const int decode_res = DataStruct::decode(rx_struct, codec, tao_mode);

    // We don't need the data anymore, the memory can be reused from the callback:
    transfer.release();

    if (decode_res <= 0)
    {
        UAVCAN_TRACE("GenericSubscriber", "Unable to decode the message [%i] [%s]",
                     decode_res, DataSpec::getDataTypeFullName());
        failure_count_++;
        node_.getDispatcher().getTransferPerfCounter().addError();
        return;
    }

    /*
     * Invoking the callback
     */
    handleReceivedDataStruct(rx_struct);
}

template <typename DataSpec, typename DataStruct, typename TransferListenerType>
int GenericSubscriber<DataSpec, DataStruct, TransferListenerType>::
genericStart(bool (Dispatcher::*registration_method)(TransferListener*))
{
    const int res = checkInit();
    if (res < 0)
    {
        UAVCAN_TRACE("GenericSubscriber", "Initialization failure [%s]", DataSpec::getDataTypeFullName());
        return res;
    }
    return GenericSubscriberBase::genericStart(forwarder_, registration_method);
}


}

#endif // UAVCAN_NODE_GENERIC_SUBSCRIBER_HPP_INCLUDED
