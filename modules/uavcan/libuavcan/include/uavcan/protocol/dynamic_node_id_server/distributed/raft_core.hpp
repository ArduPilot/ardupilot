/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISTRIBUTED_RAFT_CORE_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISTRIBUTED_RAFT_CORE_HPP_INCLUDED

#include <cstdlib>
#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/node/service_server.hpp>
#include <uavcan/node/service_client.hpp>
#include <uavcan/protocol/dynamic_node_id_server/distributed/types.hpp>
#include <uavcan/protocol/dynamic_node_id_server/distributed/persistent_state.hpp>
#include <uavcan/protocol/dynamic_node_id_server/distributed/cluster_manager.hpp>
#include <uavcan/protocol/dynamic_node_id_server/event.hpp>
// UAVCAN types
#include <uavcan/protocol/dynamic_node_id/server/AppendEntries.hpp>
#include <uavcan/protocol/dynamic_node_id/server/RequestVote.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{
namespace distributed
{
/**
 * Allocator has to implement this interface so the RaftCore can inform it when a new entry gets committed to the log.
 */
class IRaftLeaderMonitor
{
public:
    /**
     * This method will be invoked when a new log entry is committed (only if the local server is the current Leader).
     */
    virtual void handleLogCommitOnLeader(const Entry& committed_entry) = 0;

    /**
     * Invoked by the Raft core when the local node becomes a leader or ceases to be one.
     * By default the local node is not leader.
     * It is possible to commit to the log right from this method.
     */
    virtual void handleLocalLeadershipChange(bool local_node_is_leader) = 0;

    virtual ~IRaftLeaderMonitor() { }
};

/**
 * This class implements log replication and voting.
 * It does not implement client-server interaction at all; instead it just exposes a public method for adding
 * allocation entries.
 *
 * Note that this class uses std::rand(), so the RNG must be properly seeded by the application.
 *
 * Activity registration:
 *   - persistent state update error
 *   - switch to candidate (this defines timeout between reelections)
 *   - newer term in response (also switch to follower)
 *   - append entries request with term >= currentTerm
 *   - vote granted
 */
class RaftCore : private TimerBase
{
public:
    enum ServerState
    {
        ServerStateFollower,
        ServerStateCandidate,
        ServerStateLeader
    };

private:
    typedef MethodBinder<RaftCore*, void (RaftCore::*)(const ReceivedDataStructure<AppendEntries::Request>&,
                                                       ServiceResponseDataStructure<AppendEntries::Response>&)>
        AppendEntriesCallback;

    typedef MethodBinder<RaftCore*, void (RaftCore::*)(const ServiceCallResult<AppendEntries>&)>
        AppendEntriesResponseCallback;

    typedef MethodBinder<RaftCore*, void (RaftCore::*)(const ReceivedDataStructure<RequestVote::Request>&,
                                                       ServiceResponseDataStructure<RequestVote::Response>&)>
        RequestVoteCallback;

    typedef MethodBinder<RaftCore*, void (RaftCore::*)(const ServiceCallResult<RequestVote>&)>
        RequestVoteResponseCallback;

    struct PendingAppendEntriesFields
    {
        Log::Index prev_log_index;
        Log::Index num_entries;

        PendingAppendEntriesFields()
            : prev_log_index(0)
            , num_entries(0)
        { }
    };

    /*
     * Constants
     */
    enum { MaxNumFollowers = ClusterManager::MaxClusterSize - 1 };

    IEventTracer& tracer_;
    IRaftLeaderMonitor& leader_monitor_;

    /*
     * States
     */
    PersistentState persistent_state_;
    ClusterManager cluster_;
    Log::Index commit_index_;

    MonotonicTime last_activity_timestamp_;
    MonotonicDuration randomized_activity_timeout_;
    ServerState server_state_;

    uint8_t next_server_index_;         ///< Next server to query AE from
    uint8_t num_votes_received_in_this_campaign_;

    PendingAppendEntriesFields pending_append_entries_fields_;

    /*
     * Transport
     */
    ServiceServer<AppendEntries, AppendEntriesCallback>         append_entries_srv_;
    ServiceClient<AppendEntries, AppendEntriesResponseCallback> append_entries_client_;
    ServiceServer<RequestVote, RequestVoteCallback>         request_vote_srv_;
    ServiceClient<RequestVote, RequestVoteResponseCallback> request_vote_client_;

    /*
     * Methods
     */
    void trace(TraceCode event, int64_t argument) { tracer_.onEvent(event, argument); }

    INode&       getNode()       { return append_entries_srv_.getNode(); }
    const INode& getNode() const { return append_entries_srv_.getNode(); }

    void checkInvariants() const
    {
        // Commit index
        UAVCAN_ASSERT(commit_index_ <= persistent_state_.getLog().getLastIndex());

        // Term
        UAVCAN_ASSERT(persistent_state_.getLog().getEntryAtIndex(persistent_state_.getLog().getLastIndex()) !=
                      UAVCAN_NULLPTR);
        UAVCAN_ASSERT(persistent_state_.getLog().getEntryAtIndex(persistent_state_.getLog().getLastIndex())->term <=
                      persistent_state_.getCurrentTerm());

        // Elections
        UAVCAN_ASSERT(server_state_ != ServerStateCandidate || !request_vote_client_.hasPendingCalls() ||
                      persistent_state_.getVotedFor() == getNode().getNodeID());
        UAVCAN_ASSERT(num_votes_received_in_this_campaign_ <= cluster_.getClusterSize());

        // Transport
        UAVCAN_ASSERT(append_entries_client_.getNumPendingCalls() <= 1);
        UAVCAN_ASSERT(request_vote_client_.getNumPendingCalls() <= cluster_.getNumKnownServers());
        UAVCAN_ASSERT(server_state_ != ServerStateCandidate || !append_entries_client_.hasPendingCalls());
        UAVCAN_ASSERT(server_state_ != ServerStateLeader    || !request_vote_client_.hasPendingCalls());
        UAVCAN_ASSERT(server_state_ != ServerStateFollower  ||
                      (!append_entries_client_.hasPendingCalls() && !request_vote_client_.hasPendingCalls()));
    }

    void registerActivity()
    {
        last_activity_timestamp_ = getNode().getMonotonicTime();

        static const int32_t randomization_range_msec = AppendEntries::Request::DEFAULT_MAX_ELECTION_TIMEOUT_MS -
                                                        AppendEntries::Request::DEFAULT_MIN_ELECTION_TIMEOUT_MS;
        // coverity[dont_call]
        const int32_t random_msec = (std::rand() % randomization_range_msec) + 1;

        randomized_activity_timeout_ =
            MonotonicDuration::fromMSec(AppendEntries::Request::DEFAULT_MIN_ELECTION_TIMEOUT_MS + random_msec);

        UAVCAN_ASSERT(randomized_activity_timeout_.toMSec() > AppendEntries::Request::DEFAULT_MIN_ELECTION_TIMEOUT_MS);
        UAVCAN_ASSERT(randomized_activity_timeout_.toMSec() <= AppendEntries::Request::DEFAULT_MAX_ELECTION_TIMEOUT_MS);
    }

    bool isActivityTimedOut() const
    {
        return getNode().getMonotonicTime() > (last_activity_timestamp_ + randomized_activity_timeout_);
    }

    void handlePersistentStateUpdateError(int error)
    {
        UAVCAN_ASSERT(error < 0);
        trace(TraceRaftPersistStateUpdateError, error);
        switchState(ServerStateFollower);
        registerActivity();                 // Deferring reelections
    }

    void updateFollower()
    {
        if (isActivityTimedOut())
        {
            switchState(ServerStateCandidate);
            registerActivity();
        }
    }

    void updateCandidate()
    {
        if (num_votes_received_in_this_campaign_ > 0)
        {
            trace(TraceRaftElectionComplete, num_votes_received_in_this_campaign_);
            const bool won = num_votes_received_in_this_campaign_ >= cluster_.getQuorumSize();

            UAVCAN_TRACE("dynamic_node_id_server::distributed::RaftCore", "Election complete, won: %d", int(won));

            switchState(won ? ServerStateLeader : ServerStateFollower);       // Start over or become leader
        }
        else
        {
            // Set votedFor, abort on failure
            int res = persistent_state_.setVotedFor(getNode().getNodeID());
            if (res < 0)
            {
                handlePersistentStateUpdateError(res);
                return;
            }

            // Increment current term, abort on failure
            res = persistent_state_.setCurrentTerm(persistent_state_.getCurrentTerm() + 1U);
            if (res < 0)
            {
                handlePersistentStateUpdateError(res);
                return;
            }

            num_votes_received_in_this_campaign_ = 1;               // Voting for self

            RequestVote::Request req;
            req.last_log_index = persistent_state_.getLog().getLastIndex();
            req.last_log_term = persistent_state_.getLog().getEntryAtIndex(req.last_log_index)->term;
            req.term = persistent_state_.getCurrentTerm();

            for (uint8_t i = 0; i < MaxNumFollowers; i++)
            {
                const NodeID node_id = cluster_.getRemoteServerNodeIDAtIndex(i);
                if (!node_id.isUnicast())
                {
                    break;
                }

                UAVCAN_TRACE("dynamic_node_id_server::distributed::RaftCore",
                             "Requesting vote from %d", int(node_id.get()));
                trace(TraceRaftVoteRequestInitiation, node_id.get());

                res = request_vote_client_.call(node_id, req);
                if (res < 0)
                {
                    trace(TraceError, res);
                }
            }
        }
    }

    void updateLeader()
    {
        if (append_entries_client_.hasPendingCalls())
        {
            append_entries_client_.cancelAllCalls();    // Refer to the response callback to learn why
        }

        if (cluster_.getClusterSize() > 1)
        {
            const NodeID node_id = cluster_.getRemoteServerNodeIDAtIndex(next_server_index_);
            UAVCAN_ASSERT(node_id.isUnicast());

            next_server_index_++;
            if (next_server_index_ >= cluster_.getNumKnownServers())
            {
                next_server_index_ = 0;
            }

            AppendEntries::Request req;
            req.term = persistent_state_.getCurrentTerm();
            req.leader_commit = commit_index_;

            req.prev_log_index = Log::Index(cluster_.getServerNextIndex(node_id) - 1U);

            const Entry* const entry = persistent_state_.getLog().getEntryAtIndex(req.prev_log_index);
            if (entry == UAVCAN_NULLPTR)
            {
                UAVCAN_ASSERT(0);
                handlePersistentStateUpdateError(-ErrLogic);
                return;
            }

            req.prev_log_term = entry->term;

            for (Log::Index index = cluster_.getServerNextIndex(node_id);
                 index <= persistent_state_.getLog().getLastIndex();
                 index++)
            {
                req.entries.push_back(*persistent_state_.getLog().getEntryAtIndex(index));
                if (req.entries.size() == req.entries.capacity())
                {
                    break;
                }
            }

            pending_append_entries_fields_.num_entries = req.entries.size();
            pending_append_entries_fields_.prev_log_index = req.prev_log_index;

            const int res = append_entries_client_.call(node_id, req);
            if (res < 0)
            {
                trace(TraceRaftAppendEntriesCallFailure, res);
            }
        }

        propagateCommitIndex();
    }

    void switchState(ServerState new_state)
    {
        if (server_state_ == new_state)
        {
            return;
        }

        /*
         * Logging
         */
        UAVCAN_TRACE("dynamic_node_id_server::distributed::RaftCore", "State switch: %d --> %d",
                     int(server_state_), int(new_state));
        trace(TraceRaftStateSwitch, new_state);

        /*
         * Updating the current state
         */
        const ServerState old_state = server_state_;
        server_state_ = new_state;

        /*
         * Resetting specific states
         */
        cluster_.resetAllServerIndices();

        next_server_index_ = 0;
        num_votes_received_in_this_campaign_ = 0;

        request_vote_client_.cancelAllCalls();
        append_entries_client_.cancelAllCalls();

        /*
         * Calling the switch handler
         * Note that the handler may commit to the log directly
         */
        if ((old_state == ServerStateLeader) ||
            (new_state == ServerStateLeader))
        {
            leader_monitor_.handleLocalLeadershipChange(new_state == ServerStateLeader);
        }
    }

    void tryIncrementCurrentTermFromResponse(Term new_term)
    {
        trace(TraceRaftNewerTermInResponse, new_term);
        const int res = persistent_state_.setCurrentTerm(new_term);
        if (res < 0)
        {
            trace(TraceRaftPersistStateUpdateError, res);
        }
        registerActivity();                             // Deferring future elections
        switchState(ServerStateFollower);
    }

    void propagateCommitIndex()
    {
        // Objective is to estimate whether we can safely increment commit index value
        UAVCAN_ASSERT(server_state_ == ServerStateLeader);
        UAVCAN_ASSERT(commit_index_ <= persistent_state_.getLog().getLastIndex());

        if (commit_index_ < persistent_state_.getLog().getLastIndex())
        {
            /*
             * Not all local entries are committed.
             * Deciding if it is safe to increment commit index.
             */
            uint8_t num_nodes_with_next_log_entry_available = 1; // Local node
            for (uint8_t i = 0; i < cluster_.getNumKnownServers(); i++)
            {
                const Log::Index match_index = cluster_.getServerMatchIndex(cluster_.getRemoteServerNodeIDAtIndex(i));
                if (match_index > commit_index_)
                {
                    num_nodes_with_next_log_entry_available++;
                }
            }

            if (num_nodes_with_next_log_entry_available >= cluster_.getQuorumSize())
            {
                commit_index_++;
                UAVCAN_ASSERT(commit_index_ > 0);   // Index 0 is always committed
                trace(TraceRaftNewEntryCommitted, commit_index_);

                // AT THIS POINT ALLOCATION IS COMPLETE
                leader_monitor_.handleLogCommitOnLeader(*persistent_state_.getLog().getEntryAtIndex(commit_index_));
            }
        }
    }

    void handleAppendEntriesRequest(const ReceivedDataStructure<AppendEntries::Request>& request,
                                    ServiceResponseDataStructure<AppendEntries::Response>& response)
    {
        checkInvariants();

        if (!cluster_.isKnownServer(request.getSrcNodeID()))
        {
            if (cluster_.isClusterDiscovered())
            {
                trace(TraceRaftRequestIgnored, request.getSrcNodeID().get());
                response.setResponseEnabled(false);
                return;
            }
            else
            {
                cluster_.addServer(request.getSrcNodeID());
            }
        }

        UAVCAN_ASSERT(response.isResponseEnabled());  // This is default

        /*
         * Checking if our current state is up to date.
         * The request will be ignored if persistent state cannot be updated.
         */
        if (request.term > persistent_state_.getCurrentTerm())
        {
            int res = persistent_state_.setCurrentTerm(request.term);
            if (res < 0)
            {
                handlePersistentStateUpdateError(res);
                response.setResponseEnabled(false);
                return;
            }

            res = persistent_state_.resetVotedFor();
            if (res < 0)
            {
                handlePersistentStateUpdateError(res);
                response.setResponseEnabled(false);
                return;
            }
        }

        /*
         * Preparing the response
         */
        response.term = persistent_state_.getCurrentTerm();
        response.success = false;

        /*
         * Step 1 (see Raft paper)
         * Reject the request if the leader has stale term number.
         */
        if (request.term < persistent_state_.getCurrentTerm())
        {
            response.setResponseEnabled(true);
            return;
        }

        registerActivity();
        switchState(ServerStateFollower);

        /*
         * Step 2
         * Reject the request if the assumed log index does not exist on the local node.
         */
        const Entry* const prev_entry = persistent_state_.getLog().getEntryAtIndex(request.prev_log_index);
        if (prev_entry == UAVCAN_NULLPTR)
        {
            response.setResponseEnabled(true);
            return;
        }

        /*
         * Step 3
         * Drop log entries if term number does not match.
         * Ignore the request if the persistent state cannot be updated.
         */
        if (prev_entry->term != request.prev_log_term)
        {
            const int res = persistent_state_.getLog().removeEntriesWhereIndexGreaterOrEqual(request.prev_log_index);
            response.setResponseEnabled(res >= 0);
            if (res < 0)
            {
                trace(TraceRaftPersistStateUpdateError, res);
            }
            return;
        }

        /*
         * Step 4
         * Update the log with new entries - this will possibly require to rewrite existing entries.
         * Ignore the request if the persistent state cannot be updated.
         */
        if (request.prev_log_index != persistent_state_.getLog().getLastIndex())
        {
            const int res = persistent_state_.getLog().removeEntriesWhereIndexGreater(request.prev_log_index);
            if (res < 0)
            {
                trace(TraceRaftPersistStateUpdateError, res);
                response.setResponseEnabled(false);
                return;
            }
        }

        for (uint8_t i = 0; i < request.entries.size(); i++)
        {
            const int res = persistent_state_.getLog().append(request.entries[i]);
            if (res < 0)
            {
                trace(TraceRaftPersistStateUpdateError, res);
                response.setResponseEnabled(false);
                return;                     // Response will not be sent, the server will assume that we're dead
            }
        }

        /*
         * Step 5
         * Update the commit index.
         */
        if (request.leader_commit > commit_index_)
        {
            commit_index_ = min(request.leader_commit, persistent_state_.getLog().getLastIndex());
            trace(TraceRaftCommitIndexUpdate, commit_index_);
        }

        response.setResponseEnabled(true);
        response.success = true;
    }

    void handleAppendEntriesResponse(const ServiceCallResult<AppendEntries>& result)
    {
        UAVCAN_ASSERT(server_state_ == ServerStateLeader);  // When state switches, all requests must be cancelled
        checkInvariants();

        if (!result.isSuccessful())
        {
            return;
        }

        if (result.getResponse().term > persistent_state_.getCurrentTerm())
        {
            tryIncrementCurrentTermFromResponse(result.getResponse().term);
        }
        else
        {
            if (result.getResponse().success)
            {
                cluster_.incrementServerNextIndexBy(result.getCallID().server_node_id,
                                                    pending_append_entries_fields_.num_entries);
                cluster_.setServerMatchIndex(result.getCallID().server_node_id,
                                             Log::Index(pending_append_entries_fields_.prev_log_index +
                                                        pending_append_entries_fields_.num_entries));
            }
            else
            {
                cluster_.decrementServerNextIndex(result.getCallID().server_node_id);
                trace(TraceRaftAppendEntriesRespUnsucfl, result.getCallID().server_node_id.get());
            }
        }

        pending_append_entries_fields_ = PendingAppendEntriesFields();
        // Rest of the logic is implemented in periodic update handlers.
    }

    void handleRequestVoteRequest(const ReceivedDataStructure<RequestVote::Request>& request,
                                  ServiceResponseDataStructure<RequestVote::Response>& response)
    {
        checkInvariants();
        trace(TraceRaftVoteRequestReceived, request.getSrcNodeID().get());

        if (!cluster_.isKnownServer(request.getSrcNodeID()))
        {
            trace(TraceRaftRequestIgnored, request.getSrcNodeID().get());
            response.setResponseEnabled(false);
            return;
        }

        UAVCAN_ASSERT(response.isResponseEnabled());  // This is default

        /*
         * Checking if our current state is up to date.
         * The request will be ignored if persistent state cannot be updated.
         */
        if (request.term > persistent_state_.getCurrentTerm())
        {
            switchState(ServerStateFollower);   // Our term is stale, so we can't serve as leader

            int res = persistent_state_.setCurrentTerm(request.term);
            if (res < 0)
            {
                handlePersistentStateUpdateError(res);
                response.setResponseEnabled(false);
                return;
            }

            res = persistent_state_.resetVotedFor();
            if (res < 0)
            {
                handlePersistentStateUpdateError(res);
                response.setResponseEnabled(false);
                return;
            }
        }

        /*
         * Preparing the response
         */
        response.term = persistent_state_.getCurrentTerm();

        if (request.term < response.term)
        {
            response.vote_granted = false;
        }
        else
        {
            const bool can_vote = !persistent_state_.isVotedForSet() ||
                                  (persistent_state_.getVotedFor() == request.getSrcNodeID());
            const bool log_is_up_to_date =
                persistent_state_.getLog().isOtherLogUpToDate(request.last_log_index, request.last_log_term);

            response.vote_granted = can_vote && log_is_up_to_date;

            if (response.vote_granted)
            {
                switchState(ServerStateFollower);   // Avoiding race condition when Candidate
                registerActivity();                 // This is necessary to avoid excessive elections

                const int res = persistent_state_.setVotedFor(request.getSrcNodeID());
                if (res < 0)
                {
                    trace(TraceRaftPersistStateUpdateError, res);
                    response.setResponseEnabled(false);
                    return;
                }
            }
        }
    }

    void handleRequestVoteResponse(const ServiceCallResult<RequestVote>& result)
    {
        UAVCAN_ASSERT(server_state_ == ServerStateCandidate); // When state switches, all requests must be cancelled
        checkInvariants();

        if (!result.isSuccessful())
        {
            return;
        }

        trace(TraceRaftVoteRequestSucceeded, result.getCallID().server_node_id.get());

        if (result.getResponse().term > persistent_state_.getCurrentTerm())
        {
            tryIncrementCurrentTermFromResponse(result.getResponse().term);
        }
        else
        {
            if (result.getResponse().vote_granted)
            {
                num_votes_received_in_this_campaign_++;
            }
        }
        // Rest of the logic is implemented in periodic update handlers.
        // I'm no fan of asynchronous programming. At all.
    }

    virtual void handleTimerEvent(const TimerEvent&)
    {
        checkInvariants();

        switch (server_state_)
        {
        case ServerStateFollower:
        {
            updateFollower();
            break;
        }
        case ServerStateCandidate:
        {
            updateCandidate();
            break;
        }
        case ServerStateLeader:
        {
            updateLeader();
            break;
        }
        default:
        {
            UAVCAN_ASSERT(0);
            break;
        }
        }
    }

public:
    RaftCore(INode& node,
             IStorageBackend& storage,
             IEventTracer& tracer,
             IRaftLeaderMonitor& leader_monitor)
        : TimerBase(node)
        , tracer_(tracer)
        , leader_monitor_(leader_monitor)
        , persistent_state_(storage, tracer)
        , cluster_(node, storage, persistent_state_.getLog(), tracer)
        , commit_index_(0)                                  // Per Raft paper, commitIndex must be initialized to zero
        , last_activity_timestamp_(node.getMonotonicTime())
        , randomized_activity_timeout_(
            MonotonicDuration::fromMSec(AppendEntries::Request::DEFAULT_MAX_ELECTION_TIMEOUT_MS))
        , server_state_(ServerStateFollower)
        , next_server_index_(0)
        , num_votes_received_in_this_campaign_(0)
        , append_entries_srv_(node)
        , append_entries_client_(node)
        , request_vote_srv_(node)
        , request_vote_client_(node)
    { }

    /**
     * Once started, the logic runs in the background until destructor is called.
     * @param cluster_size      If set, this value will be used and stored in the persistent storage. If not set,
     *                          value from the persistent storage will be used. If not set and there's no such key
     *                          in the persistent storage, initialization will fail.
     */
    int init(const uint8_t cluster_size, const TransferPriority priority)
    {
        /*
         * Initializing state variables
         */
        server_state_ = ServerStateFollower;
        next_server_index_ = 0;
        num_votes_received_in_this_campaign_ = 0;
        commit_index_ = 0;

        registerActivity();

        /*
         * Initializing internals
         */
        int res = persistent_state_.init();
        if (res < 0)
        {
            return res;
        }

        res = cluster_.init(cluster_size, priority);
        if (res < 0)
        {
            return res;
        }

        res = append_entries_srv_.start(AppendEntriesCallback(this, &RaftCore::handleAppendEntriesRequest));
        if (res < 0)
        {
            return res;
        }

        res = request_vote_srv_.start(RequestVoteCallback(this, &RaftCore::handleRequestVoteRequest));
        if (res < 0)
        {
            return res;
        }

        res = append_entries_client_.init(priority);
        if (res < 0)
        {
            return res;
        }
        append_entries_client_.setCallback(AppendEntriesResponseCallback(this,
                                                                         &RaftCore::handleAppendEntriesResponse));

        res = request_vote_client_.init(priority);
        if (res < 0)
        {
            return res;
        }
        request_vote_client_.setCallback(RequestVoteResponseCallback(this, &RaftCore::handleRequestVoteResponse));

        /*
         * Initializing timing constants
         * Refer to the specification for the formula
         */
        const uint8_t num_followers = static_cast<uint8_t>(cluster_.getClusterSize() - 1);

        const MonotonicDuration update_interval =
            MonotonicDuration::fromMSec(AppendEntries::Request::DEFAULT_MIN_ELECTION_TIMEOUT_MS /
                                        2 / max(static_cast<uint8_t>(2), num_followers));

        UAVCAN_TRACE("dynamic_node_id_server::distributed::RaftCore",
                     "Update interval: %ld msec", static_cast<long>(update_interval.toMSec()));

        append_entries_client_.setRequestTimeout(min(append_entries_client_.getDefaultRequestTimeout(),
                                                     update_interval));

        request_vote_client_.setRequestTimeout(min(request_vote_client_.getDefaultRequestTimeout(),
                                                   update_interval));

        startPeriodic(update_interval);

        trace(TraceRaftCoreInited, update_interval.toUSec());

        UAVCAN_ASSERT(res >= 0);
        return 0;
    }

    /**
     * This function is mostly needed for testing.
     */
    Log::Index getCommitIndex() const { return commit_index_; }

    /**
     * This essentially indicates whether the server could replicate log since last allocation.
     */
    bool areAllLogEntriesCommitted() const { return commit_index_ == persistent_state_.getLog().getLastIndex(); }

    /**
     * Only the leader can call @ref appendLog().
     */
    bool isLeader() const { return server_state_ == ServerStateLeader; }

    /**
     * Inserts one entry into log.
     * This method will trigger an assertion failure and return error if the current node is not the leader.
     * If operation fails, the node may give up its Leader status.
     */
    void appendLog(const Entry::FieldTypes::unique_id& unique_id, NodeID node_id)
    {
        if (isLeader())
        {
            Entry entry;
            entry.node_id = node_id.get();
            entry.unique_id = unique_id;
            entry.term = persistent_state_.getCurrentTerm();

            trace(TraceRaftNewLogEntry, entry.node_id);
            const int res = persistent_state_.getLog().append(entry);
            if (res < 0)
            {
                handlePersistentStateUpdateError(res);
            }
        }
        else
        {
            UAVCAN_ASSERT(0);
        }
    }

    /**
     * This class is used to perform log searches.
     */
    struct LogEntryInfo
    {
        Entry entry;
        bool committed;

        LogEntryInfo(const Entry& arg_entry, bool arg_committed)
            : entry(arg_entry)
            , committed(arg_committed)
        { }
    };

    /**
     * This method is used by the allocator to query existence of certain entries in the Raft log.
     * Predicate is a callable of the following prototype:
     *  bool (const LogEntryInfo& entry)
     * Once the predicate returns true, the loop will be terminated and the method will return an initialized lazy
     * contructor with the last visited entry; otherwise the constructor will not be initialized.
     * In this case, lazy constructor is used as boost::optional.
     * The log is always traversed from HIGH to LOW index values, i.e. entry 0 will be traversed last.
     */
    template <typename Predicate>
    inline LazyConstructor<LogEntryInfo> traverseLogFromEndUntil(const Predicate& predicate) const
    {
        UAVCAN_ASSERT(coerceOrFallback<bool>(predicate, true));
        for (int index = static_cast<int>(persistent_state_.getLog().getLastIndex()); index >= 0; index--)
        {
            const Entry* const entry = persistent_state_.getLog().getEntryAtIndex(Log::Index(index));
            UAVCAN_ASSERT(entry != UAVCAN_NULLPTR);
            const LogEntryInfo info(*entry, Log::Index(index) <= commit_index_);
            if (predicate(info))
            {
                LazyConstructor<LogEntryInfo> ret;
                ret.template construct<const LogEntryInfo&>(info);
                return ret;
            }
        }
        return LazyConstructor<LogEntryInfo>();
    }

    Log::Index getNumAllocations() const
    {
        // Remember that index zero contains a special-purpose entry that doesn't count as allocation
        return persistent_state_.getLog().getLastIndex();
    }

    /**
     * These accessors are needed for debugging, visualization and testing.
     */
    const PersistentState& getPersistentState() const { return persistent_state_; }
    const ClusterManager& getClusterManager()   const { return cluster_; }
    MonotonicTime getLastActivityTimestamp()    const { return last_activity_timestamp_; }
    ServerState getServerState()                const { return server_state_; }
    MonotonicDuration getUpdateInterval()       const { return getPeriod(); }
    MonotonicDuration getRandomizedTimeout()    const { return randomized_activity_timeout_; }
};

}
}
}

#endif // Include guard
