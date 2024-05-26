/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/protocol/node_status_provider.hpp>
#include <uavcan/debug.hpp>
#include <cassert>

namespace uavcan
{

bool NodeStatusProvider::isNodeInfoInitialized() const
{
    // Hardware/Software versions are not required
    return !node_info_.name.empty();
}

int NodeStatusProvider::publish()
{
    const MonotonicDuration uptime = getNode().getMonotonicTime() - creation_timestamp_;
    UAVCAN_ASSERT(uptime.isPositive());
    node_info_.status.uptime_sec = uint32_t(uptime.toMSec() / 1000);

    UAVCAN_ASSERT(node_info_.status.health <= protocol::NodeStatus::FieldTypes::health::max());

    return node_status_pub_.broadcast(node_info_.status);
}

void NodeStatusProvider::handleTimerEvent(const TimerEvent&)
{
    if (getNode().isPassiveMode())
    {
        UAVCAN_TRACE("NodeStatusProvider", "NodeStatus pub skipped - passive mode");
    }
    else
    {
        if (ad_hoc_status_updater_ != UAVCAN_NULLPTR)
        {
            ad_hoc_status_updater_->updateNodeStatus();
        }

        const int res = publish();
        if (res < 0)
        {
            getNode().registerInternalFailure("NodeStatus pub failed");
        }
    }
}

void NodeStatusProvider::handleGetNodeInfoRequest(const protocol::GetNodeInfo::Request&,
                                                  protocol::GetNodeInfo::Response& rsp)
{
    UAVCAN_TRACE("NodeStatusProvider", "Got GetNodeInfo request");
    UAVCAN_ASSERT(isNodeInfoInitialized());
    rsp = node_info_;
}

int NodeStatusProvider::startAndPublish(const TransferPriority priority)
{
    if (!isNodeInfoInitialized())
    {
        UAVCAN_TRACE("NodeStatusProvider", "Node info was not initialized");
        return -ErrNotInited;
    }

    int res = -1;

    node_status_pub_.setPriority(priority);

    if (!getNode().isPassiveMode())
    {
        res = publish();
        if (res < 0)  // Initial broadcast
        {
            goto fail;
        }
    }

    res = gni_srv_.start(GetNodeInfoCallback(this, &NodeStatusProvider::handleGetNodeInfoRequest));
    if (res < 0)
    {
        goto fail;
    }

    setStatusPublicationPeriod(MonotonicDuration::fromMSec(protocol::NodeStatus::MAX_BROADCASTING_PERIOD_MS));

    return res;

fail:
    UAVCAN_ASSERT(res < 0);
    gni_srv_.stop();
    TimerBase::stop();
    return res;
}

void NodeStatusProvider::setStatusPublicationPeriod(uavcan::MonotonicDuration period)
{
    const MonotonicDuration maximum = MonotonicDuration::fromMSec(protocol::NodeStatus::MAX_BROADCASTING_PERIOD_MS);
    const MonotonicDuration minimum = MonotonicDuration::fromMSec(protocol::NodeStatus::MIN_BROADCASTING_PERIOD_MS);

    period = min(period, maximum);
    period = max(period, minimum);
    TimerBase::startPeriodic(period);

    const MonotonicDuration tx_timeout = period - MonotonicDuration::fromUSec(period.toUSec() / 20);
    node_status_pub_.setTxTimeout(tx_timeout);

    UAVCAN_TRACE("NodeStatusProvider", "Status pub period: %s, TX timeout: %s",
                 period.toString().c_str(), node_status_pub_.getTxTimeout().toString().c_str());
}

uavcan::MonotonicDuration NodeStatusProvider::getStatusPublicationPeriod() const
{
    return TimerBase::getPeriod();
}

void NodeStatusProvider::setAdHocNodeStatusUpdater(IAdHocNodeStatusUpdater* updater)
{
    ad_hoc_status_updater_ = updater;   // Can be nullptr, that's okay
}

void NodeStatusProvider::setHealth(uint8_t code)
{
    node_info_.status.health = code;
}

void NodeStatusProvider::setMode(uint8_t code)
{
    node_info_.status.mode = code;
}

void NodeStatusProvider::setVendorSpecificStatusCode(VendorSpecificStatusCode code)
{
    node_info_.status.vendor_specific_status_code = code;
}

void NodeStatusProvider::setName(const NodeName& name)
{
    if (node_info_.name.empty())
    {
        node_info_.name = name;
    }
}

void NodeStatusProvider::setSoftwareVersion(const protocol::SoftwareVersion& version)
{
    if (node_info_.software_version == protocol::SoftwareVersion())
    {
        node_info_.software_version = version;
    }
}

void NodeStatusProvider::setHardwareVersion(const protocol::HardwareVersion& version)
{
    if (node_info_.hardware_version == protocol::HardwareVersion())
    {
        node_info_.hardware_version = version;
    }
}

}
