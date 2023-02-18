#pragma once
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>

#if HAL_ENABLE_LIBUAVCAN_DRIVERS
#include <uavcan/uavcan.hpp>
#include <AP_Common/Bitmask.h>
#include <StorageManager/StorageManager.h>
#include <AP_CANManager/AP_CANManager.h>

//Forward declaring classes
class AllocationCb;
class NodeStatusCb;
class NodeInfoCb;
class GetNodeInfoCb;
class AP_UAVCAN;

class AP_UAVCAN_DNA_Server
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
    uint8_t self_node_id[HAL_MAX_CAN_PROTOCOL_DRIVERS];
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

    static void trampoline_handleNodeInfo(AP_UAVCAN* ap_uavcan, uint8_t node_id, const GetNodeInfoCb& resp);
    static void trampoline_handleAllocation(AP_UAVCAN* ap_uavcan, uint8_t node_id, const AllocationCb &cb);
    static void trampoline_handleNodeStatus(AP_UAVCAN* ap_uavcan, uint8_t node_id, const NodeStatusCb &cb);


    HAL_Semaphore storage_sem;
    AP_UAVCAN *_ap_uavcan;
    uint8_t driver_index;

public:
    AP_UAVCAN_DNA_Server(AP_UAVCAN *ap_uavcan, StorageAccess _storage);


    // Do not allow copies
    CLASS_NO_COPY(AP_UAVCAN_DNA_Server);

    //Initialises publisher and Server Record for specified uavcan driver
    bool init();

    //Reset the Server Record
    void reset();

    /* Checks if the node id has been verified against the record
    Specific CAN drivers are expected to check use this method to 
    verify if the node is healthy and has static node_id against 
    hwid in the records */
    bool isNodeIDVerified(uint8_t node_id) const;

    /* Subscribe to the messages to be handled for maintaining and allocating
    Node ID list */
    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);

    //report the server state, along with failure message if any
    bool prearm_check(char* fail_msg, uint8_t fail_msg_len) const;

    //Callbacks
    void handleAllocation(uint8_t node_id, const AllocationCb &cb);
    void handleNodeStatus(uint8_t node_id, const NodeStatusCb &cb);
    void handleNodeInfo(uint8_t node_id, uint8_t unique_id[], char name[], uint8_t major, uint8_t minor, uint32_t vcs_commit);

    //Run through the list of seen node ids for verification
    void verify_nodes();
};

#endif
