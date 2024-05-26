/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_NODE_ABSTRACT_NODE_HPP_INCLUDED
#define UAVCAN_NODE_ABSTRACT_NODE_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/dynamic_memory.hpp>
#include <uavcan/node/scheduler.hpp>

namespace uavcan
{
/**
 * Abstract node class. If you're going to implement your own node class for your application,
 * please inherit this class so it can be used with default publisher, subscriber, server, etc. classes.
 * Normally you don't need to use it directly though - please refer to the class Node<> instead.
 */
class UAVCAN_EXPORT INode
{
    bool canfd_ = false;
    bool tao_disabled_ = false;
public:
    virtual ~INode() { }
    virtual IPoolAllocator& getAllocator() = 0;
    virtual Scheduler& getScheduler() = 0;
    virtual const Scheduler& getScheduler() const = 0;
    virtual void registerInternalFailure(const char* msg) = 0;

    Dispatcher& getDispatcher()             { return getScheduler().getDispatcher(); }
    const Dispatcher& getDispatcher() const { return getScheduler().getDispatcher(); }

    ISystemClock& getSystemClock()         { return getScheduler().getSystemClock(); }
    MonotonicTime getMonotonicTime() const { return getScheduler().getMonotonicTime(); }
    UtcTime getUtcTime()             const { return getScheduler().getUtcTime(); }

    /**
     * Returns the Node ID of this node.
     * If Node ID was not set yet, an invalid value will be returned.
     */
    NodeID getNodeID() const { return getScheduler().getDispatcher().getNodeID(); }

    /**
     * Sets the Node ID of this node.
     * Node ID can be assigned only once. This method returns true if the Node ID was successfully assigned, otherwise
     * it returns false.
     * As long as a valid Node ID is not set, the node will remain in passive mode.
     * Using a non-unicast Node ID puts the node into passive mode (as default).
     */
    bool setNodeID(NodeID nid)
    {
        return getScheduler().getDispatcher().setNodeID(nid);
    }

    /**
     * Whether the node is in passive mode, i.e. can't transmit anything to the bus.
     * Please read the specs to learn more.
     */
    bool isPassiveMode() const { return getScheduler().getDispatcher().isPassiveMode(); }

    /**
     * Same as @ref spin(MonotonicDuration), but the deadline is specified as an absolute time value
     * rather than duration.
     */
    int spin(MonotonicTime deadline)
    {
        getScheduler().getDispatcher().set_options(tao_disabled_, canfd_);
        return getScheduler().spin(deadline);
    }

    /**
     * Runs the node.
     * Normally your application should not block anywhere else.
     * Block inside this method forever or call it periodically.
     * This method returns 0 if no errors occurred, or a negative error code if something failed (see error.hpp).
     */
    int spin(MonotonicDuration duration)
    {
        getScheduler().getDispatcher().set_options(tao_disabled_, canfd_);
        return getScheduler().spin(getMonotonicTime() + duration);
    }

    /**
     * This method is designed for non-blocking applications.
     * Instead of blocking, it returns immediately once all available CAN frames and timer events are processed.
     * Note that this is unlike plain @ref spin(), which will strictly return when the deadline is reached,
     * even if there still are unprocessed events.
     * This method returns 0 if no errors occurred, or a negative error code if something failed (see error.hpp).
     */
    int spinOnce()
    {
        getScheduler().getDispatcher().set_options(tao_disabled_, canfd_);
        return getScheduler().spinOnce();
    }

    /**
     * This method allows to directly transmit a raw CAN frame circumventing the whole UAVCAN stack.
     * Mandatory parameters:
     *
     * @param frame             CAN frame to be transmitted.
     *
     * @param tx_deadline       The frame will be discarded if it could not be transmitted by this time.
     *
     * @param iface_mask        This bitmask allows to select what CAN interfaces this frame should go into.
     *                          Example:
     *                           - 1 - the frame will be sent only to iface 0.
     *                           - 4 - the frame will be sent only to iface 2.
     *                           - 3 - the frame will be sent to ifaces 0 and 1.
     *
     * Optional parameters:
     *
     * @param qos               Quality of service. Please refer to the CAN IO manager for details.
     *
     * @param flags             CAN IO flags. Please refer to the CAN driver API for details.
     */
    int injectTxFrame(const CanFrame& frame, MonotonicTime tx_deadline, uint8_t iface_mask,
                      CanTxQueue::Qos qos = CanTxQueue::Volatile,
                      CanIOFlags flags = 0)
    {
        return getDispatcher().getCanIOManager().send(frame, tx_deadline, MonotonicTime(), iface_mask, qos, flags);
    }

    /**
     * @brief Disable Tail Array Optimisation
     * 
     */
    void disableTao()            { tao_disabled_ = true; }

    /**
     * @brief Enable Tail Array Optimisation
     * 
     */
    void enableTao()             { tao_disabled_ = false; }

    /**
     * @brief Check if Tail Array Optimisation is enabled
     * 
     * @return true 
     * @return false 
     */
    bool isTaoDisabled()          { return tao_disabled_; }

    /**
     * @brief Check if CAN FD is enabled
     * 
     * @return true 
     * @return false 
     */
    bool isCanFdEnabled()        { return canfd_; }

    /**
     * @brief Enable CAN FD
     * 
     */
    void enableCanFd()           { canfd_ = true; }

    /**
     * @brief Disable CAN FD
     * 
     */
    void disableCanFd()          { canfd_ = false; }

#if !UAVCAN_TINY
    /**
     * The @ref IRxFrameListener interface allows one to monitor all incoming CAN frames.
     * This feature can be used to implement multithreaded nodes, or to add secondary protocol support.
     */
    void removeRxFrameListener()                       { getDispatcher().removeRxFrameListener(); }
    void installRxFrameListener(IRxFrameListener* lst) { getDispatcher().installRxFrameListener(lst); }
#endif
};

}

#endif // UAVCAN_NODE_ABSTRACT_NODE_HPP_INCLUDED
