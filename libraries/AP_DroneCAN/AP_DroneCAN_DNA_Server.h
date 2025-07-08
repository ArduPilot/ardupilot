#pragma once
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>

#if HAL_ENABLE_DRONECAN_DRIVERS
#include <AP_Common/Bitmask.h>
#include <StorageManager/StorageManager.h>
#include <AP_CANManager/AP_CANManager.h>
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include "AP_Canard_iface.h"
#include <dronecan_msgs.h>

class AP_DroneCAN;
//Forward declaring classes
class AP_DroneCAN_DNA_Server
{
    StorageAccess storage;

    struct NodeRecord {
        uint8_t uid_hash[6];
        uint8_t crc;
    };

    /*
     * For each node ID (1 through MAX_NODE_ID), the database can have one
     * registration for it. Each registration consists of a NodeRecord which
     * contains the (hash of the) unique ID reported by that node ID. Other
     * info could be added to the registration in the future.
     *
     * Physically, the database is stored as a header and format version,
     * followed by an array of NodeRecords indexed by node ID. If a particular
     * NodeRecord has an all-zero unique ID hash or an invalid CRC, then that
     * node ID isn't considerd to have a registration.
     *
     * The database has public methods which handle the server behavior for the
     * relevant message. The methods can be used by multiple servers in
     * different threads, so each holds a lock for its duration.
     */
    class Database {
    public:
        Database() {};

        // initialize database (storage accessor is always replaced with the one supplied)
        void init(StorageAccess *storage_);

        // remove all registrations from the database
        void reset();

        // return true if the given node ID is registered
        bool is_registered(uint8_t node_id) {
            return node_registered.get(node_id);
        }

        // handle initializing the server with its own node ID and unique ID
        void init_server(uint8_t own_node_id, const uint8_t own_unique_id[], uint8_t own_unique_id_len);

        // handle processing the node info message. returns true if from a duplicate node
        bool handle_node_info(uint8_t source_node_id, const uint8_t unique_id[]);

        // handle the allocation message. returns the allocated node ID, or 0 if allocation failed
        uint8_t handle_allocation(const uint8_t unique_id[]);

    private:
        // retrieve node ID that matches the given unique ID. returns 0 if not found
        uint8_t find_node_id(const uint8_t unique_id[], uint8_t size);

        // fill the given record with the hash of the given unique ID
        void compute_uid_hash(NodeRecord &record, const uint8_t unique_id[], uint8_t size) const;

        // register a given unique ID to a given node ID, deleting any existing registration for the unique ID
        void register_uid(uint8_t node_id, const uint8_t unique_id[], uint8_t size);

        // create the registration for the given node ID and set its record's unique ID
        void create_registration(uint8_t node_id, const uint8_t unique_id[], uint8_t size);

        // delete the given node ID's registration
        void delete_registration(uint8_t node_id);

        // return true if the given node ID has a registration
        bool check_registration(uint8_t node_id);

        // read the given node ID's registration's record
        void read_record(NodeRecord &record, uint8_t node_id);

        // write the given node ID's registration's record
        void write_record(const NodeRecord &record, uint8_t node_id);

        // bitmasks containing a status for each possible node ID (except 0 and > MAX_NODE_ID)
        Bitmask<128> node_registered; // have a registration for this node ID

        StorageAccess *storage;
        HAL_Semaphore sem;
    };

    static Database db;

    enum ServerState {
        NODE_STATUS_UNHEALTHY = -5,
        DUPLICATE_NODES = -2,
        HEALTHY = 0
    };

    uint32_t last_verification_request;
    uint8_t curr_verifying_node;
    uint8_t self_node_id;
    bool nodeInfo_resp_rcvd;

    // bitmasks containing a status for each possible node ID (except 0 and > MAX_NODE_ID)
    Bitmask<128> node_verified; // node seen and unique ID matches stored
    Bitmask<128> node_seen; // received NodeStatus
    Bitmask<128> node_logged; // written to log fle
    Bitmask<128> node_healthy; // reports healthy

    uint8_t last_logging_count;

    //Error State
    enum ServerState server_state;
    uint8_t fault_node_id;
    char fault_node_name[15];


    // dynamic node ID allocation state variables
    uint8_t rcvd_unique_id[16];
    uint8_t rcvd_unique_id_offset;
    uint32_t last_alloc_msg_ms;

    AP_DroneCAN &_ap_dronecan;
    CanardInterface &_canard_iface;

    Canard::Publisher<uavcan_protocol_dynamic_node_id_Allocation> allocation_pub{_canard_iface};

    Canard::ObjCallback<AP_DroneCAN_DNA_Server, uavcan_protocol_dynamic_node_id_Allocation> allocation_cb{this, &AP_DroneCAN_DNA_Server::handle_allocation};
    Canard::Subscriber<uavcan_protocol_dynamic_node_id_Allocation> allocation_sub;

    Canard::ObjCallback<AP_DroneCAN_DNA_Server, uavcan_protocol_NodeStatus> node_status_cb{this, &AP_DroneCAN_DNA_Server::handleNodeStatus};
    Canard::Subscriber<uavcan_protocol_NodeStatus> node_status_sub;

    Canard::ObjCallback<AP_DroneCAN_DNA_Server, uavcan_protocol_GetNodeInfoResponse> node_info_cb{this, &AP_DroneCAN_DNA_Server::handleNodeInfo};
    Canard::Client<uavcan_protocol_GetNodeInfoResponse> node_info_client;

public:
    AP_DroneCAN_DNA_Server(AP_DroneCAN &ap_dronecan, CanardInterface &canard_iface, uint8_t driver_index);


    // Do not allow copies
    CLASS_NO_COPY(AP_DroneCAN_DNA_Server);

    //Initialises publisher and Server Record for specified uavcan driver
    bool init(uint8_t own_unique_id[], uint8_t own_unique_id_len, uint8_t node_id);

    //report the server state, along with failure message if any
    bool prearm_check(char* fail_msg, uint8_t fail_msg_len) const;

    // canard message handler callbacks
    void handle_allocation(const CanardRxTransfer& transfer, const uavcan_protocol_dynamic_node_id_Allocation& msg);
    void handleNodeStatus(const CanardRxTransfer& transfer, const uavcan_protocol_NodeStatus& msg);
    void handleNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoResponse& rsp);

    //Run through the list of seen node ids for verification
    void verify_nodes();
};

#endif
