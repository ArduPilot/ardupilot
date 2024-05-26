/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_TRANSPORT_STATS_PROVIDER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_TRANSPORT_STATS_PROVIDER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/service_server.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/protocol/GetTransportStats.hpp>

namespace uavcan
{
/**
 * This class provides statistics about the transport layer performance on the local node.
 * The user's application does not deal with this class directly because it's instantiated by the node class.
 */
class UAVCAN_EXPORT TransportStatsProvider : Noncopyable
{
    typedef MethodBinder<const TransportStatsProvider*,
                         void (TransportStatsProvider::*)(const protocol::GetTransportStats::Request&,
                                                          protocol::GetTransportStats::Response&) const>
            GetTransportStatsCallback;

    ServiceServer<protocol::GetTransportStats, GetTransportStatsCallback> srv_;

    void handleGetTransportStats(const protocol::GetTransportStats::Request&,
                                 protocol::GetTransportStats::Response& resp) const
    {
        const TransferPerfCounter& perf = srv_.getNode().getDispatcher().getTransferPerfCounter();
        resp.transfer_errors = perf.getErrorCount();
        resp.transfers_tx = perf.getTxTransferCount();
        resp.transfers_rx = perf.getRxTransferCount();

        const CanIOManager& canio = srv_.getNode().getDispatcher().getCanIOManager();
        for (uint8_t i = 0; i < canio.getNumIfaces(); i++)
        {
            const CanIfacePerfCounters can_perf = canio.getIfacePerfCounters(i);
            protocol::CANIfaceStats stats;
            stats.errors = can_perf.errors;
            stats.frames_tx = can_perf.frames_tx;
            stats.frames_rx = can_perf.frames_rx;
            resp.can_iface_stats.push_back(stats);
        }
    }

public:
    explicit TransportStatsProvider(INode& node)
        : srv_(node)
    { }

    /**
     * Once started, this class requires no further attention.
     * Returns negative error code.
     */
    int start()
    {
        return srv_.start(GetTransportStatsCallback(this, &TransportStatsProvider::handleGetTransportStats));
    }
};

}

#endif // UAVCAN_PROTOCOL_TRANSPORT_STATS_PROVIDER_HPP_INCLUDED
