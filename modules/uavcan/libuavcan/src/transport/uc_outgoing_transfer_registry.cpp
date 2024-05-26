/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/transport/outgoing_transfer_registry.hpp>

namespace uavcan
{
/*
 * OutgoingTransferRegistryKey
 */
#if UAVCAN_TOSTRING
std::string OutgoingTransferRegistryKey::toString() const
{
    char buf[40];
    (void)snprintf(buf, sizeof(buf), "dtid=%u tt=%u dnid=%u",
                   int(data_type_id_.get()), int(transfer_type_), int(destination_node_id_.get()));
    return std::string(buf);
}
#endif

/*
 * OutgoingTransferRegistry
 */
const MonotonicDuration OutgoingTransferRegistry::MinEntryLifetime = MonotonicDuration::fromMSec(2000);

TransferID* OutgoingTransferRegistry::accessOrCreate(const OutgoingTransferRegistryKey& key,
                                                     MonotonicTime new_deadline)
{
    UAVCAN_ASSERT(!new_deadline.isZero());
    Value* p = map_.access(key);
    if (p == UAVCAN_NULLPTR)
    {
        p = map_.insert(key, Value());
        if (p == UAVCAN_NULLPTR)
        {
            return UAVCAN_NULLPTR;
        }
        UAVCAN_TRACE("OutgoingTransferRegistry", "Created %s", key.toString().c_str());
    }
    p->deadline = new_deadline;
    return &p->tid;
}

bool OutgoingTransferRegistry::exists(DataTypeID dtid, TransferType tt) const
{
    return UAVCAN_NULLPTR != map_.find(ExistenceCheckingPredicate(dtid, tt));
}

void OutgoingTransferRegistry::cleanup(MonotonicTime ts)
{
    map_.removeAllWhere(DeadlineExpiredPredicate(ts));
}

}
