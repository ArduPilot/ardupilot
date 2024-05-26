/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_FIRMWARE_UPDATE_TRIGGER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_FIRMWARE_UPDATE_TRIGGER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/node/service_client.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/util/map.hpp>
#include <uavcan/protocol/node_info_retriever.hpp>
// UAVCAN types
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>

namespace uavcan
{
/**
 * Application-specific firmware version checking logic.
 * Refer to @ref FirmwareUpdateTrigger for details.
 */
class IFirmwareVersionChecker
{
public:
    /**
     * This value is limited by the pool block size minus some extra data. Please refer to the Map<> documentation
     * for details. If this size is set too high, the compilation will fail in the Map<> template.
     */
    enum { MaxFirmwareFilePathLength = 40 };

    /**
     * This type is used to store path to firmware file that the target node will retrieve using the
     * service uavcan.protocol.file.Read. Note that the maximum length is limited due to some specifics of
     * libuavcan (@ref MaxFirmwareFilePathLength), this is NOT a protocol-level limitation.
     */
    typedef MakeString<MaxFirmwareFilePathLength>::Type FirmwareFilePath;

    /**
     * This method will be invoked when the class obtains a response to GetNodeInfo request.
     *
     * @param node_id                   Node ID that this GetNodeInfo response was received from.
     *
     * @param node_info                 Actual node info structure; refer to uavcan.protocol.GetNodeInfo for details.
     *
     * @param out_firmware_file_path    The implementation should return the firmware image path via this argument.
     *                                  Note that this path must be reachable via uavcan.protocol.file.Read service.
     *                                  Refer to @ref FileServer and @ref BasicFileServer for details.
     *
     * @return                          True - the class will begin sending update requests.
     *                                  False - the node will be ignored, no request will be sent.
     */
    virtual bool shouldRequestFirmwareUpdate(NodeID node_id, const protocol::GetNodeInfo::Response& node_info,
                                             FirmwareFilePath& out_firmware_file_path) = 0;

    /**
     * This method will be invoked when a node responds to the update request with an error. If the request simply
     * times out, this method will not be invoked.
     * Note that if by the time of arrival of the response the node is already removed, this method will not be called.
     *
     * SPECIAL CASE: If the node responds with ERROR_IN_PROGRESS, the class will assume that further requesting
     *               is not needed anymore. This method will not be invoked.
     *
     * @param node_id                   Node ID that returned this error.
     *
     * @param error_response            Contents of the error response. It contains error code and text.
     *
     * @param out_firmware_file_path    New firmware path if a retry is needed. Note that this argument will be
     *                                  initialized with old path, so if the same path needs to be reused, this
     *                                  argument should be left unchanged.
     *
     * @return                          True - the class will continue sending update requests with new firmware path.
     *                                  False - the node will be forgotten, new requests will not be sent.
     */
    virtual bool shouldRetryFirmwareUpdate(NodeID node_id,
                                           const protocol::file::BeginFirmwareUpdate::Response& error_response,
                                           FirmwareFilePath& out_firmware_file_path) = 0;

    /**
     * This node is invoked when the node responds to the update request with confirmation.
     * Note that if by the time of arrival of the response the node is already removed, this method will not be called.
     *
     * Implementation is optional; default one does nothing.
     *
     * @param node_id   Node ID that confirmed the request.
     *
     * @param response  Actual response.
     */
    virtual void handleFirmwareUpdateConfirmation(NodeID node_id,
                                                  const protocol::file::BeginFirmwareUpdate::Response& response)
    {
        (void)node_id;
        (void)response;
    }

    virtual ~IFirmwareVersionChecker() { }
};

/**
 * This class subscribes to updates from @ref NodeInfoRetriever in order to detect nodes that need firmware
 * updates. The decision process of whether a firmware update is needed is relayed to the application via
 * @ref IFirmwareVersionChecker. If the application confirms that the update is needed, this class will begin
 * sending uavcan.protocol.file.BeginFirmwareUpdate periodically (period is configurable) to every node that
 * needs an update in a round-robin fashion. There are the following termination conditions for the periodical
 * sending process:
 *
 * - The node responds with confirmation. In this case the class will forget about the node on the assumption
 *   that its job is done here. Confirmation will be reported to the application via the interface.
 *
 * - The node responds with an error, and the error code is not ERROR_IN_PROGRESS. In this case the class will
 *   request the application via the interface mentioned above about its further actions - either give up or
 *   retry, possibly with a different firmware.
 *
 * - The node responds with error ERROR_IN_PROGRESS. In this case the class behaves exactly in the same way as if
 *   response was successful (because the firmware is alredy being updated, so the goal is fulfilled).
 *   Confirmation will be reported to the application via the interface.
 *
 * - The node goes offline or restarts. In this case the node will be immediately forgotten, and the process
 *   will repeat again later because the node info retriever re-queries GetNodeInfo every time when a node restarts.
 *
 * Since the target node (i.e. node that is being updated) will try to retrieve the specified firmware file using
 * the file services (uavcan.protocol.file.*), the provided firmware path must be reachable for the file server
 * (@ref FileServer, @ref BasicFileServer). Normally, an application that serves as UAVCAN firmware update server
 * will include at least the following components:
 * - this firmware update trigger;
 * - dynamic node ID allocation server;
 * - file server.
 *
 * Implementation details: the class uses memory pool to keep the list of nodes that have not responded yet, which
 * limits the maximum length of the path to the firmware file, which is covered in @ref IFirmwareVersionChecker.
 * To somewhat relieve the maximum path length limitation, the class can be supplied with a common prefix that
 * will be prepended to firmware pathes before sending requests.
 * Interval at which requests are being sent is configurable, but the default value should cover the needs of
 * virtually all use cases (as always).
 */
class FirmwareUpdateTrigger : public INodeInfoListener,
                              private TimerBase
{
    typedef MethodBinder<FirmwareUpdateTrigger*,
        void (FirmwareUpdateTrigger::*)(const ServiceCallResult<protocol::file::BeginFirmwareUpdate>&)>
            BeginFirmwareUpdateResponseCallback;

    typedef IFirmwareVersionChecker::FirmwareFilePath FirmwareFilePath;

    enum { DefaultRequestIntervalMs = 1000 };   ///< Shall not be less than default service response timeout.

    struct NextNodeIDSearchPredicate : ::uavcan::Noncopyable
    {
        enum { DefaultOutput = 0xFFU };

        const FirmwareUpdateTrigger& owner;
        uint8_t output;

        NextNodeIDSearchPredicate(const FirmwareUpdateTrigger& arg_owner)
            : owner(arg_owner)
            , output(DefaultOutput)
        { }

        bool operator()(const NodeID node_id, const FirmwareFilePath&)
        {
            if (node_id.get() > owner.last_queried_node_id_ &&
                !owner.begin_fw_update_client_.hasPendingCallToServer(node_id))
            {
                output = min(output, node_id.get());
            }
            return false;
        }
    };

    /*
     * State
     */
    ServiceClient<protocol::file::BeginFirmwareUpdate, BeginFirmwareUpdateResponseCallback> begin_fw_update_client_;

    IFirmwareVersionChecker& checker_;

    NodeInfoRetriever* node_info_retriever_;

    Map<NodeID, FirmwareFilePath> pending_nodes_;

    MonotonicDuration request_interval_;

    FirmwareFilePath common_path_prefix_;

    mutable uint8_t last_queried_node_id_;

    /*
     * Methods of INodeInfoListener
     */
    virtual void handleNodeInfoUnavailable(NodeID node_id)
    {
        UAVCAN_TRACE("FirmwareUpdateTrigger", "Node ID %d could not provide GetNodeInfo response", int(node_id.get()));
        pending_nodes_.remove(node_id); // For extra paranoia
    }

    virtual void handleNodeInfoRetrieved(const NodeID node_id, const protocol::GetNodeInfo::Response& node_info)
    {
        FirmwareFilePath firmware_file_path;
        const bool update_needed = checker_.shouldRequestFirmwareUpdate(node_id, node_info, firmware_file_path);
        if (update_needed)
        {
            UAVCAN_TRACE("FirmwareUpdateTrigger", "Node ID %d requires update; file path: %s",
                         int(node_id.get()), firmware_file_path.c_str());
            trySetPendingNode(node_id, firmware_file_path);
        }
        else
        {
            UAVCAN_TRACE("FirmwareUpdateTrigger", "Node ID %d does not need update", int(node_id.get()));
            pending_nodes_.remove(node_id);
        }
    }

    virtual void handleNodeStatusChange(const NodeStatusMonitor::NodeStatusChangeEvent& event)
    {
        if (event.status.mode == protocol::NodeStatus::MODE_OFFLINE)
        {
            pending_nodes_.remove(event.node_id);
            UAVCAN_TRACE("FirmwareUpdateTrigger", "Node ID %d is offline hence forgotten", int(event.node_id.get()));
        }
    }

    /*
     * Own methods
     */
    INode& getNode() { return begin_fw_update_client_.getNode(); }

    void trySetPendingNode(const NodeID node_id, const FirmwareFilePath& path)
    {
        if (UAVCAN_NULLPTR != pending_nodes_.insert(node_id, path))
        {
            if (!TimerBase::isRunning())
            {
                TimerBase::startPeriodic(request_interval_);
                UAVCAN_TRACE("FirmwareUpdateTrigger", "Timer started");
            }
        }
        else
        {
            getNode().registerInternalFailure("FirmwareUpdateTrigger OOM");
        }
    }

    NodeID pickNextNodeID() const
    {
        // We can't do index search because indices are unstable in Map<>
        // First try - from the current node up
        NextNodeIDSearchPredicate s1(*this);
        (void)pending_nodes_.find<NextNodeIDSearchPredicate&>(s1);

        if (s1.output != NextNodeIDSearchPredicate::DefaultOutput)
        {
            last_queried_node_id_ = s1.output;
        }
        else if (last_queried_node_id_ != 0)
        {
            // Nothing was found, resetting the selector and trying again
            UAVCAN_TRACE("FirmwareUpdateTrigger", "Node selector reset, last value: %d", int(last_queried_node_id_));
            last_queried_node_id_ = 0;

            NextNodeIDSearchPredicate s2(*this);
            (void)pending_nodes_.find<NextNodeIDSearchPredicate&>(s2);

            if (s2.output != NextNodeIDSearchPredicate::DefaultOutput)
            {
                last_queried_node_id_ = s2.output;
            }
        }
        else
        {
            ; // Hopeless
        }
        UAVCAN_TRACE("FirmwareUpdateTrigger", "Next node ID to query: %d, pending nodes: %u, pending calls: %u",
                     int(last_queried_node_id_), pending_nodes_.getSize(),
                     begin_fw_update_client_.getNumPendingCalls());
        return last_queried_node_id_;
    }

    void handleBeginFirmwareUpdateResponse(const ServiceCallResult<protocol::file::BeginFirmwareUpdate>& result)
    {
        if (!result.isSuccessful())
        {
            UAVCAN_TRACE("FirmwareUpdateTrigger", "Request to %d has timed out, will retry",
                         int(result.getCallID().server_node_id.get()));
            return;
        }

        FirmwareFilePath* const old_path = pending_nodes_.access(result.getCallID().server_node_id);
        if (old_path == UAVCAN_NULLPTR)
        {
            // The entry has been removed, assuming that it's not needed anymore
            return;
        }

        const bool confirmed =
            result.getResponse().error == protocol::file::BeginFirmwareUpdate::Response::ERROR_OK ||
            result.getResponse().error == protocol::file::BeginFirmwareUpdate::Response::ERROR_IN_PROGRESS;

        if (confirmed)
        {
            UAVCAN_TRACE("FirmwareUpdateTrigger", "Node %d confirmed the update request",
                         int(result.getCallID().server_node_id.get()));
            pending_nodes_.remove(result.getCallID().server_node_id);
            checker_.handleFirmwareUpdateConfirmation(result.getCallID().server_node_id, result.getResponse());
        }
        else
        {
            UAVCAN_ASSERT(old_path != UAVCAN_NULLPTR);
            UAVCAN_ASSERT(TimerBase::isRunning());
            // We won't have to call trySetPendingNode(), because we'll directly update the old path via the pointer

            const bool update_needed =
                checker_.shouldRetryFirmwareUpdate(result.getCallID().server_node_id, result.getResponse(), *old_path);

            if (!update_needed)
            {
                UAVCAN_TRACE("FirmwareUpdateTrigger", "Node %d does not need retry",
                             int(result.getCallID().server_node_id.get()));
                pending_nodes_.remove(result.getCallID().server_node_id);
            }
        }
    }

    virtual void handleTimerEvent(const TimerEvent&)
    {
        if (pending_nodes_.isEmpty())
        {
            TimerBase::stop();
            UAVCAN_TRACE("FirmwareUpdateTrigger", "Timer stopped");
            return;
        }

        const NodeID node_id = pickNextNodeID();
        if (!node_id.isUnicast())
        {
            return;
        }

        FirmwareFilePath* const path = pending_nodes_.access(node_id);
        if (path == UAVCAN_NULLPTR)
        {
            UAVCAN_ASSERT(0);   // pickNextNodeID() returned a node ID that is not present in the map
            return;
        }

        protocol::file::BeginFirmwareUpdate::Request req;

        req.source_node_id = getNode().getNodeID().get();
        req.image_file_remote_path.path = path->c_str();

        UAVCAN_TRACE("FirmwareUpdateTrigger", "Request to %d with path: %s",
                     int(node_id.get()), req.image_file_remote_path.path.c_str());

        const int call_res = begin_fw_update_client_.call(node_id, req);
        if (call_res < 0)
        {
            getNode().registerInternalFailure("FirmwareUpdateTrigger call");
        }
    }

public:
    FirmwareUpdateTrigger(INode& node, IFirmwareVersionChecker& checker)
        : TimerBase(node)
        , begin_fw_update_client_(node)
        , checker_(checker)
        , node_info_retriever_(UAVCAN_NULLPTR)
        , pending_nodes_(node.getAllocator())
        , request_interval_(MonotonicDuration::fromMSec(DefaultRequestIntervalMs))
        , last_queried_node_id_(0)
    { }

    ~FirmwareUpdateTrigger()
    {
        if (node_info_retriever_ != UAVCAN_NULLPTR)
        {
            node_info_retriever_->removeListener(this);
        }
    }

    /**
     * Starts the object. Once started, it can't be stopped unless destroyed.
     *
     * @param node_info_retriever       The object will register itself against this retriever.
     *                                  When the destructor is called, the object will unregister itself.
     *
     * @param common_path_prefix        If set, this path will be prefixed to all firmware pathes provided by the
     *                                  application interface. The prefix does not need to end with path separator;
     *                                  the last trailing one will be removed (so use '//' if you need '/').
     *                                  By default the prefix is empty.
     *
     * @return                          Negative error code.
     */
    int start(NodeInfoRetriever& node_info_retriever,
              const FirmwareFilePath& arg_common_path_prefix = FirmwareFilePath(),
              const FirmwareFilePath& arg_alt_path_prefix = FirmwareFilePath(),
              const TransferPriority priority = TransferPriority::OneHigherThanLowest)
    {
        /*
         * Configuring the node info retriever
         */
        node_info_retriever_ = &node_info_retriever;

        int res = node_info_retriever_->addListener(this);
        if (res < 0)
        {
            return res;
        }

        /*
         * Initializing the prefix, removing only the last trailing path separator.
         */
        common_path_prefix_ = arg_common_path_prefix;

        if (!common_path_prefix_.empty() &&
            *(common_path_prefix_.end() - 1) == protocol::file::Path::SEPARATOR)
        {
            common_path_prefix_.resize(uint8_t(common_path_prefix_.size() - 1U));
        }

        /*
         * Initializing the client
         */
        res = begin_fw_update_client_.init(priority);
        if (res < 0)
        {
            return res;
        }
        begin_fw_update_client_.setCallback(
            BeginFirmwareUpdateResponseCallback(this, &FirmwareUpdateTrigger::handleBeginFirmwareUpdateResponse));

        // The timer will be started ad-hoc
        return 0;
    }

    /**
     * Interval at which uavcan.protocol.file.BeginFirmwareUpdate requests are being sent.
     * Note that default value should be OK for any use case.
     */
    MonotonicDuration getRequestInterval() const { return request_interval_; }
    void setRequestInterval(const MonotonicDuration interval)
    {
        if (interval.isPositive())
        {
            request_interval_ = interval;
            if (TimerBase::isRunning())     // Restarting with new interval
            {
                TimerBase::startPeriodic(request_interval_);
            }
        }
        else
        {
            UAVCAN_ASSERT(0);
        }
    }

    /**
     * This method is mostly needed for testing.
     * When triggering is not in progress, the class consumes zero CPU time.
     */
    bool isTimerRunning() const { return TimerBase::isRunning(); }

    unsigned getNumPendingNodes() const
    {
        const unsigned ret = pending_nodes_.getSize();
        UAVCAN_ASSERT((ret > 0) ? isTimerRunning() : true);
        return ret;
    }

};

}

#endif // Include guard
