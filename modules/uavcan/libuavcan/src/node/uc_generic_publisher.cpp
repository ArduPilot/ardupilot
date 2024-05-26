/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/node/generic_publisher.hpp>

namespace uavcan
{

bool GenericPublisherBase::isInited() const
{
    return sender_.isInitialized();
}

int GenericPublisherBase::doInit(DataTypeKind dtkind, const char* dtname, CanTxQueue::Qos qos)
{
    if (isInited())
    {
        return 0;
    }

    GlobalDataTypeRegistry::instance().freeze();

    const DataTypeDescriptor* const descr = GlobalDataTypeRegistry::instance().find(dtkind, dtname);
    if (descr == UAVCAN_NULLPTR)
    {
        UAVCAN_TRACE("GenericPublisher", "Type [%s] is not registered", dtname);
        return -ErrUnknownDataType;
    }

    sender_.init(*descr, qos);

    return 0;
}

MonotonicTime GenericPublisherBase::getTxDeadline() const
{
    return node_.getMonotonicTime() + tx_timeout_;
}

int GenericPublisherBase::genericPublish(const StaticTransferBufferImpl& buffer, TransferType transfer_type,
                                         NodeID dst_node_id, TransferID* tid, MonotonicTime blocking_deadline)
{
    if (tid)
    {
        return sender_.send(buffer.getRawPtr(), buffer.getMaxWritePos(), getTxDeadline(),
                            blocking_deadline, transfer_type, dst_node_id, *tid, force_std_can_);
    }
    else
    {
        return sender_.send(buffer.getRawPtr(), buffer.getMaxWritePos(), getTxDeadline(),
                            blocking_deadline, transfer_type, dst_node_id, force_std_can_);
    }
}

void GenericPublisherBase::setTxTimeout(MonotonicDuration tx_timeout)
{
    tx_timeout = max(tx_timeout, getMinTxTimeout());
    tx_timeout = min(tx_timeout, getMaxTxTimeout());
    tx_timeout_ = tx_timeout;
}

}
