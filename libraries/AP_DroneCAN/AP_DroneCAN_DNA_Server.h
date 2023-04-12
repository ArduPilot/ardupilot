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

    struct NodeData {
        uint8_t hwid_hash[6];
        uint8_t crc;
    };

    enum ServerState {
        NODE_STATUS_UNHEALTHY = -5,
        STORAGE_FAILURE = -3,
        DUPLICATE_NODES = -2,
        FAILED_TO_ADD_NODE = -1,
        HEALTHY = 0
    };

    uint32_t last_verification_request;
    uint8_t curr_verifying_node;
    uint8_t self_node_id;
    bool nodeInfo_resp_rcvd;

    Bitmask<128> occupation_mask;
    Bitmask<128> verified_mask;
    Bitmask<128> node_seen_mask;
    Bitmask<128> logged;
    Bitmask<128> node_healthy_mask;

    uint8_t last_logging_count;

    //Error State
    enum ServerState server_state;
    uint8_t fault_node_id;
    char fault_node_name[15];


    //Allocation params
    uint8_t rcvd_unique_id[16];
    uint8_t rcvd_unique_id_offset;
    uint32_t last_alloc_msg_ms;

    //Methods to handle and report Node IDs seen on the bus
    void addToSeenNodeMask(uint8_t node_id);
    bool isNodeSeen(uint8_t node_id);

    //Generates 6Byte long hash from the specified unique_id
    void getHash(NodeData &node_data, const uint8_t unique_id[], uint8_t size) const;

    //Reads the Server Record from storage for specified node id
    bool readNodeData(NodeData &data, uint8_t node_id);

    //Writes the Server Record from storage for specified node id
    bool writeNodeData(const NodeData &data, uint8_t node_id);

    //Methods to set, clear and report NodeIDs allocated/registered so far
    bool setOccupationMask(uint8_t node_id);
    bool isNodeIDOccupied(uint8_t node_id) const;
    bool freeNodeID(uint8_t node_id);

    //Set the mask to report that the unique id matches the record
    void setVerificationMask(uint8_t node_id);

    //Go through List to find node id for specified unique id
    uint8_t getNodeIDForUniqueID(const uint8_t unique_id[], uint8_t size);

    //Add Node ID info to the record and setup necessary mask fields
    bool addNodeIDForUniqueID(uint8_t node_id, const uint8_t unique_id[], uint8_t size);

    //Finds next available free Node, starting from preferred NodeID
    uint8_t findFreeNodeID(uint8_t preferred);

    //Look in the storage and check if there's a valid Server Record there
    bool isValidNodeDataAvailable(uint8_t node_id);

    HAL_Semaphore storage_sem;
    AP_DroneCAN &_ap_dronecan;
    CanardInterface &_canard_iface;

    Canard::Publisher<uavcan_protocol_dynamic_node_id_Allocation> allocation_pub{_canard_iface};

    Canard::ObjCallback<AP_DroneCAN_DNA_Server, uavcan_protocol_dynamic_node_id_Allocation> allocation_cb{this, &AP_DroneCAN_DNA_Server::handleAllocation};
    Canard::Subscriber<uavcan_protocol_dynamic_node_id_Allocation> allocation_sub;

    Canard::ObjCallback<AP_DroneCAN_DNA_Server, uavcan_protocol_NodeStatus> node_status_cb{this, &AP_DroneCAN_DNA_Server::handleNodeStatus};
    Canard::Subscriber<uavcan_protocol_NodeStatus> node_status_sub;

    Canard::ObjCallback<AP_DroneCAN_DNA_Server, uavcan_protocol_GetNodeInfoResponse> node_info_cb{this, &AP_DroneCAN_DNA_Server::handleNodeInfo};
    Canard::Client<uavcan_protocol_GetNodeInfoResponse> node_info_client;

public:
    AP_DroneCAN_DNA_Server(AP_DroneCAN &ap_dronecan);


    // Do not allow copies
    CLASS_NO_COPY(AP_DroneCAN_DNA_Server);

    //Initialises publisher and Server Record for specified uavcan driver
    bool init(uint8_t own_unique_id[], uint8_t own_unique_id_len, uint8_t node_id);

    //Reset the Server Record
    void reset();

    /* Checks if the node id has been verified against the record
    Specific CAN drivers are expected to check use this method to 
    verify if the node is healthy and has static node_id against 
    hwid in the records */
    bool isNodeIDVerified(uint8_t node_id) const;

    /* Subscribe to the messages to be handled for maintaining and allocating
    Node ID list */
    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);

    //report the server state, along with failure message if any
    bool prearm_check(char* fail_msg, uint8_t fail_msg_len) const;

    //Callbacks
    void handleAllocation(const CanardRxTransfer& transfer, const uavcan_protocol_dynamic_node_id_Allocation& msg);
    void handleNodeStatus(const CanardRxTransfer& transfer, const uavcan_protocol_NodeStatus& msg);
    void handleNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoResponse& rsp);

    //Run through the list of seen node ids for verification
    void verify_nodes();
};

#endif
