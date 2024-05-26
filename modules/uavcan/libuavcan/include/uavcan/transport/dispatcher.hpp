/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_TRANSPORT_DISPATCHER_HPP_INCLUDED
#define UAVCAN_TRANSPORT_DISPATCHER_HPP_INCLUDED

#include <cassert>
#include <uavcan/error.hpp>
#include <uavcan/std.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/transport/perf_counter.hpp>
#include <uavcan/transport/transfer_listener.hpp>
#include <uavcan/transport/outgoing_transfer_registry.hpp>
#include <uavcan/transport/can_io.hpp>
#include <uavcan/util/linked_list.hpp>

namespace uavcan
{

class UAVCAN_EXPORT Dispatcher;

#if !UAVCAN_TINY
/**
 * Inherit this class to receive notifications about all TX CAN frames that were transmitted with the loopback flag.
 */
class UAVCAN_EXPORT LoopbackFrameListenerBase : public LinkedListNode<LoopbackFrameListenerBase>
{
    Dispatcher& dispatcher_;

protected:
    explicit LoopbackFrameListenerBase(Dispatcher& dispatcher)
        : dispatcher_(dispatcher)
    { }

    virtual ~LoopbackFrameListenerBase() { stopListening(); }

    void startListening();
    void stopListening();
    bool isListening() const;

    Dispatcher& getDispatcher() { return dispatcher_; }

public:
    virtual void handleLoopbackFrame(const RxFrame& frame) = 0;
};


class UAVCAN_EXPORT LoopbackFrameListenerRegistry : Noncopyable
{
    LinkedListRoot<LoopbackFrameListenerBase> listeners_;

public:
    void add(LoopbackFrameListenerBase* listener);
    void remove(LoopbackFrameListenerBase* listener);
    bool doesExist(const LoopbackFrameListenerBase* listener) const;
    unsigned getNumListeners() const { return listeners_.getLength(); }

    void invokeListeners(RxFrame& frame);
};

/**
 * Implement this interface to receive notifications about all incoming CAN frames, including loopback.
 */
class UAVCAN_EXPORT IRxFrameListener
{
public:
    virtual ~IRxFrameListener() { }

    /**
     * Make sure to filter out loopback frames if they are not wanted.
     */
    virtual void handleRxFrame(const CanRxFrame& frame, CanIOFlags flags) = 0;
};
#endif

/**
 * This class performs low-level CAN frame routing.
 */
class UAVCAN_EXPORT Dispatcher : Noncopyable
{
    CanIOManager canio_;
    ISystemClock& sysclock_;
    OutgoingTransferRegistry outgoing_transfer_reg_;
    TransferPerfCounter perf_;
    bool tao_disabled_ = false;
    bool canfd_ = false;

    class ListenerRegistry
    {
        LinkedListRoot<TransferListener> list_;

        class DataTypeIDInsertionComparator
        {
            const DataTypeID id_;
        public:
            explicit DataTypeIDInsertionComparator(DataTypeID id) : id_(id) { }
            bool operator()(const TransferListener* listener) const
            {
                UAVCAN_ASSERT(listener);
                return id_ > listener->getDataTypeDescriptor().getID();
            }
        };

    public:
        enum Mode { UniqueListener, ManyListeners };

        bool add(TransferListener* listener, Mode mode);
        void remove(TransferListener* listener);
        bool exists(DataTypeID dtid) const;
        void cleanup(MonotonicTime ts);
        void handleFrame(const RxFrame& frame, bool tao_disabled);

        unsigned getNumEntries() const { return list_.getLength(); }

        const LinkedListRoot<TransferListener>& getList() const { return list_; }
    };

    ListenerRegistry lmsg_;
    ListenerRegistry lsrv_req_;
    ListenerRegistry lsrv_resp_;

#if !UAVCAN_TINY
    LoopbackFrameListenerRegistry loopback_listeners_;
    IRxFrameListener* rx_listener_;
#endif

    NodeID self_node_id_;
    bool self_node_id_is_set_;

    void handleFrame(const CanRxFrame& can_frame);

    void handleLoopbackFrame(const CanRxFrame& can_frame);

    void notifyRxFrameListener(const CanRxFrame& can_frame, CanIOFlags flags);

public:
    Dispatcher(ICanDriver& driver, IPoolAllocator& allocator, ISystemClock& sysclock)
        : canio_(driver, allocator, sysclock)
        , sysclock_(sysclock)
        , outgoing_transfer_reg_(allocator)
#if !UAVCAN_TINY
        , rx_listener_(UAVCAN_NULLPTR)
#endif
        , self_node_id_(NodeID::Broadcast)  // Default
        , self_node_id_is_set_(false)
    { }

    /**
     * This version returns strictly when the deadline is reached.
     */
    int spin(MonotonicTime deadline);

    /**
     * This version does not return until all available frames are processed.
     */
    int spinOnce();

    /**
     * Refer to CanIOManager::send() for the parameter description
     */
    int send(const Frame& frame, MonotonicTime tx_deadline, MonotonicTime blocking_deadline, CanTxQueue::Qos qos,
             CanIOFlags flags, uint8_t iface_mask);

    void cleanup(MonotonicTime ts);

    bool registerMessageListener(TransferListener* listener);
    bool registerServiceRequestListener(TransferListener* listener);
    bool registerServiceResponseListener(TransferListener* listener);

    void unregisterMessageListener(TransferListener* listener);
    void unregisterServiceRequestListener(TransferListener* listener);
    void unregisterServiceResponseListener(TransferListener* listener);

    bool hasSubscriber(DataTypeID dtid) const;
    bool hasPublisher(DataTypeID dtid) const;
    bool hasServer(DataTypeID dtid) const;

    bool isTaoDisabled() const { return tao_disabled_; }
    bool isCanFdEnabled() const { return canfd_; }

    unsigned getNumMessageListeners()         const { return lmsg_.getNumEntries(); }
    unsigned getNumServiceRequestListeners()  const { return lsrv_req_.getNumEntries(); }
    unsigned getNumServiceResponseListeners() const { return lsrv_resp_.getNumEntries(); }

    /**
     * These methods can be used to retreive lists of messages, service requests and service responses the
     * dispatcher is currently listening to.
     * Note that the list of service response listeners is very volatile, because a response listener will be
     * removed from this list as soon as the corresponding service call is complete.
     * @{
     */
    const LinkedListRoot<TransferListener>& getListOfMessageListeners() const
    {
        return lmsg_.getList();
    }
    const LinkedListRoot<TransferListener>& getListOfServiceRequestListeners() const
    {
        return lsrv_req_.getList();
    }
    const LinkedListRoot<TransferListener>& getListOfServiceResponseListeners() const
    {
        return lsrv_resp_.getList();
    }
    /**
     * @}
     */

    OutgoingTransferRegistry& getOutgoingTransferRegistry() { return outgoing_transfer_reg_; }

#if !UAVCAN_TINY
    LoopbackFrameListenerRegistry& getLoopbackFrameListenerRegistry() { return loopback_listeners_; }

    IRxFrameListener* getRxFrameListener() const { return rx_listener_; }
    void removeRxFrameListener() { rx_listener_ = UAVCAN_NULLPTR; }
    void installRxFrameListener(IRxFrameListener* listener)
    {
        UAVCAN_ASSERT(listener != UAVCAN_NULLPTR);
        rx_listener_ = listener;
    }
#endif

    /**
     * Node ID can be set only once.
     * Non-unicast Node ID puts the node into passive mode.
     */
    NodeID getNodeID() const { return self_node_id_; }
    bool setNodeID(NodeID nid);

    /**
     * Refer to the specs to learn more about passive mode.
     */
    bool isPassiveMode() const { return !getNodeID().isUnicast(); }

    const ISystemClock& getSystemClock() const { return sysclock_; }
    ISystemClock& getSystemClock() { return sysclock_; }

    const CanIOManager& getCanIOManager() const { return canio_; }
    CanIOManager& getCanIOManager() { return canio_; }

    const TransferPerfCounter& getTransferPerfCounter() const { return perf_; }
    TransferPerfCounter& getTransferPerfCounter() { return perf_; }

    void set_options(bool tao_disabled, bool canfd) { tao_disabled_ = tao_disabled; canfd_ = canfd; }
};

}

#endif // UAVCAN_TRANSPORT_DISPATCHER_HPP_INCLUDED
