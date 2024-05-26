/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_NODE_STATUS_PROVIDER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_NODE_STATUS_PROVIDER_HPP_INCLUDED

#include <uavcan/node/publisher.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/service_server.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>

namespace uavcan
{
/**
 * This optional interface can be implemented by the user in order to update the node status as necessary,
 * immediately before the next NodeStatus message is emitted by @ref NodeStatusProvider.
 */
class IAdHocNodeStatusUpdater
{
public:
    /**
     * This method is invoked by the library from @ref NodeStatusProvider from the library's thread immediately
     * before the next NodeStatus message is transmitted. The application can implement this method to perform
     * node status updates only as necessary.
     * The application is expected to invoke the methods of @ref NodeStatusProvider to update the status
     * from this method.
     * Note that this method is only invoked when publication is happening by the timer event.
     * It will NOT be invoked if the following methods are used to trigger node status publication:
     *  - @ref NodeStatusProvider::startAndPublish()
     *  - @ref NodeStatusProvider::forcePublish()
     */
    virtual void updateNodeStatus() = 0;

    virtual ~IAdHocNodeStatusUpdater() { }
};

/**
 * Provides the status and basic information about this node to other network participants.
 *
 * Usually the application does not need to deal with this class directly - it's instantiated by the node class.
 *
 * Default values:
 *  - health - OK
 *  - mode   - INITIALIZATION
 */
class UAVCAN_EXPORT NodeStatusProvider : private TimerBase
{
    typedef MethodBinder<NodeStatusProvider*,
                         void (NodeStatusProvider::*)(const protocol::GetNodeInfo::Request&,
                                                      protocol::GetNodeInfo::Response&)> GetNodeInfoCallback;

    const MonotonicTime creation_timestamp_;

    Publisher<protocol::NodeStatus> node_status_pub_;
    ServiceServer<protocol::GetNodeInfo, GetNodeInfoCallback> gni_srv_;

    protocol::GetNodeInfo::Response node_info_;

    IAdHocNodeStatusUpdater* ad_hoc_status_updater_;

    INode& getNode() { return node_status_pub_.getNode(); }

    bool isNodeInfoInitialized() const;

    int publish();

    virtual void handleTimerEvent(const TimerEvent&) override;
    void handleGetNodeInfoRequest(const protocol::GetNodeInfo::Request&, protocol::GetNodeInfo::Response& rsp);

public:
    typedef typename StorageType<typename protocol::NodeStatus::FieldTypes::vendor_specific_status_code>::Type
        VendorSpecificStatusCode;

    typedef typename StorageType<typename protocol::GetNodeInfo::Response::FieldTypes::name>::Type NodeName;

    explicit NodeStatusProvider(INode& node)
        : TimerBase(node)
        , creation_timestamp_(node.getMonotonicTime())
        , node_status_pub_(node)
        , gni_srv_(node)
        , ad_hoc_status_updater_(UAVCAN_NULLPTR)
    {
        UAVCAN_ASSERT(!creation_timestamp_.isZero());

        node_info_.status.mode = protocol::NodeStatus::MODE_INITIALIZATION;

        node_info_.status.health = protocol::NodeStatus::HEALTH_OK;
    }

    /**
     * Starts the provider and immediately broadcasts uavcan.protocol.NodeStatus.
     * Returns negative error code.
     */
    int startAndPublish(const TransferPriority priority = TransferPriority::Default);

    /**
     * Publish the message uavcan.protocol.NodeStatus right now, out of schedule.
     * Returns negative error code.
     */
    int forcePublish() { return publish(); }

    /**
     * Allows to override default publishing rate for uavcan.protocol.NodeStatus.
     * Refer to the DSDL definition of uavcan.protocol.NodeStatus to see what is the default rate.
     * Doesn't fail; if the value is outside of acceptable range, a closest valid value will be used instead.
     */
    void setStatusPublicationPeriod(uavcan::MonotonicDuration period);
    uavcan::MonotonicDuration getStatusPublicationPeriod() const;

    /**
     * Configure the optional handler that is invoked before every node status message is emitted.
     * By default no handler is installed.
     * It is allowed to pass a null pointer, that will disable the ad-hoc update feature.
     * @ref IAdHocNodeStatusUpdater
     */
    void setAdHocNodeStatusUpdater(IAdHocNodeStatusUpdater* updater);
    IAdHocNodeStatusUpdater* getAdHocNodeStatusUpdater() const { return ad_hoc_status_updater_; }

    /**
     * Local node health code control.
     */
    uint8_t getHealth() const { return node_info_.status.health; }
    void setHealth(uint8_t code);
    void setHealthOk()           { setHealth(protocol::NodeStatus::HEALTH_OK); }
    void setHealthWarning()      { setHealth(protocol::NodeStatus::HEALTH_WARNING); }
    void setHealthError()        { setHealth(protocol::NodeStatus::HEALTH_ERROR); }
    void setHealthCritical()     { setHealth(protocol::NodeStatus::HEALTH_CRITICAL); }

    /**
     * Local node mode code control.
     */
    uint8_t getMode() const { return node_info_.status.mode; }
    void setMode(uint8_t code);
    void setModeOperational()    { setMode(protocol::NodeStatus::MODE_OPERATIONAL); }
    void setModeInitialization() { setMode(protocol::NodeStatus::MODE_INITIALIZATION); }
    void setModeMaintenance()    { setMode(protocol::NodeStatus::MODE_MAINTENANCE); }
    void setModeSoftwareUpdate() { setMode(protocol::NodeStatus::MODE_SOFTWARE_UPDATE); }
    void setModeOffline()        { setMode(protocol::NodeStatus::MODE_OFFLINE); }

    /**
     * Local node vendor-specific status code control.
     */
    void setVendorSpecificStatusCode(VendorSpecificStatusCode code);
    VendorSpecificStatusCode getVendorSpecificStatusCode() const
    {
        return node_info_.status.vendor_specific_status_code;
    }

    /**
     * Local node name control.
     * Can be set only once before the provider is started.
     * The provider will refuse to start if the node name is not set.
     */
    const NodeName& getName() const { return node_info_.name; }
    void setName(const NodeName& name);

    /**
     * Node version information.
     * Can be set only once before the provider is started.
     */
    const protocol::SoftwareVersion& getSoftwareVersion() const { return node_info_.software_version; }
    const protocol::HardwareVersion& getHardwareVersion() const { return node_info_.hardware_version; }
    void setSoftwareVersion(const protocol::SoftwareVersion& version);
    void setHardwareVersion(const protocol::HardwareVersion& version);
};

}

#endif // UAVCAN_PROTOCOL_NODE_STATUS_PROVIDER_HPP_INCLUDED
