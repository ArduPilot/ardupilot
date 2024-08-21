
/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Siddharth Bharat Purohit
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_DRONECAN_DRIVERS

#include "AP_DroneCAN_DNA_Server.h"
#include "AP_DroneCAN.h"
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <stdio.h>
extern const AP_HAL::HAL& hal;

#define NODEDATA_MAGIC 0xAC01
#define NODEDATA_MAGIC_LEN 2
#define MAX_NODE_ID    125

#define debug_dronecan(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "DroneCAN", fmt, ##args); } while (0)

AP_DroneCAN_DNA_Server::AP_DroneCAN_DNA_Server(AP_DroneCAN &ap_dronecan, CanardInterface &canard_iface, uint8_t driver_index) :
    _ap_dronecan(ap_dronecan),
    _canard_iface(canard_iface),
    storage(StorageManager::StorageCANDNA),
    allocation_sub(allocation_cb, driver_index),
    node_status_sub(node_status_cb, driver_index),
    node_info_client(_canard_iface, node_info_cb)
{}

/* Method to generate 6byte hash from the Unique ID.
We return it packed inside the referenced NodeData structure */
void AP_DroneCAN_DNA_Server::getHash(NodeData &node_data, const uint8_t unique_id[], uint8_t size) const
{
    uint64_t hash = FNV_1_OFFSET_BASIS_64;
    hash_fnv_1a(size, unique_id, &hash);

    // xor-folding per http://www.isthe.com/chongo/tech/comp/fnv/
    hash = (hash>>56) ^ (hash&(((uint64_t)1<<56)-1));

    // write it to ret
    for (uint8_t i=0; i<6; i++) {
        node_data.hwid_hash[i] = (hash >> (8*i)) & 0xff;
    }
}

//Read Node Data from Storage Region
bool AP_DroneCAN_DNA_Server::readNodeData(NodeData &data, uint8_t node_id)
{
    if (node_id > MAX_NODE_ID) {
        return false;
    }
    WITH_SEMAPHORE(storage_sem);
    if (!storage.read_block(&data, (node_id * sizeof(struct NodeData)) + NODEDATA_MAGIC_LEN, sizeof(struct NodeData))) {
        //This will fall through to Prearm Check
        server_state = STORAGE_FAILURE;
        return false;
    }
    return true;
}

//Write Node Data to Storage Region
bool AP_DroneCAN_DNA_Server::writeNodeData(const NodeData &data, uint8_t node_id)
{
    if (node_id > MAX_NODE_ID) {
        return false;
    }
    WITH_SEMAPHORE(storage_sem);
    if (!storage.write_block((node_id * sizeof(struct NodeData)) + NODEDATA_MAGIC_LEN,
                             &data, sizeof(struct NodeData))) {
        server_state = STORAGE_FAILURE;
        return false;
    }
    return true;
}

/* Set Occupation Mask, handy for keeping track of all node ids that
are allocated and all that are available. */
bool AP_DroneCAN_DNA_Server::setOccupationMask(uint8_t node_id)
{
    if (node_id > MAX_NODE_ID) {
        return false;
    }
    occupation_mask.set(node_id);
    return true;
}

/* Remove Node Data from Server Record in Storage,
and also clear Occupation Mask */
bool AP_DroneCAN_DNA_Server::freeNodeID(uint8_t node_id)
{
    if (node_id > MAX_NODE_ID) {
        return false;
    }

    struct NodeData node_data;

    //Eliminate from Server Record
    memset(&node_data, 0, sizeof(node_data));
    writeNodeData(node_data, node_id);

    //Clear Occupation Mask
    occupation_mask.clear(node_id);

    return true;
}

/* Sets the verification mask. This is to be called, once
The Seen Node has been both registered and verified against the
Server Records. */
void AP_DroneCAN_DNA_Server::setVerificationMask(uint8_t node_id)
{
    if (node_id > MAX_NODE_ID) {
        return;
    }
    verified_mask.set(node_id);
}

/* Checks if the NodeID is occupied, i.e. its recorded
in the Server Records against a unique ID */
bool AP_DroneCAN_DNA_Server::isNodeIDOccupied(uint8_t node_id) const
{
    if (node_id > MAX_NODE_ID) {
        return false;
    }
    return occupation_mask.get(node_id);
}

/* Checks if NodeID is verified, i.e. the unique id in
Storage Records matches the one provided by Device with this node id. */
bool AP_DroneCAN_DNA_Server::isNodeIDVerified(uint8_t node_id) const
{
    if (node_id > MAX_NODE_ID) {
        return false;
    }
    return verified_mask.get(node_id);
}

/* Go through Server Records, and fetch node id that matches the provided
Unique IDs hash.
Returns 255 if no Node ID was detected */
uint8_t AP_DroneCAN_DNA_Server::getNodeIDForUniqueID(const uint8_t unique_id[], uint8_t size)
{
    uint8_t node_id = 255;
    NodeData node_data, cmp_node_data;
    getHash(cmp_node_data, unique_id, size);

    for (int i = MAX_NODE_ID; i >= 0; i--) {
        if (!isNodeIDOccupied(i)) { // No point in checking NodeID that's not taken
            continue;
        }
        if (!readNodeData(node_data, i)) {
            break;  //Storage module has failed, report that as no NodeID detected
        }
        if (memcmp(node_data.hwid_hash, cmp_node_data.hwid_hash, sizeof(NodeData::hwid_hash)) == 0) {
            node_id = i;
            break;
        }
    }
    return node_id;
}

/* Hash the Unique ID and add it to the Server Record
for specified Node ID. */
bool AP_DroneCAN_DNA_Server::addNodeIDForUniqueID(uint8_t node_id, const uint8_t unique_id[], uint8_t size)
{
    NodeData node_data;
    getHash(node_data, unique_id, size);
    //Generate CRC for validating the data when read back
    node_data.crc = crc_crc8(node_data.hwid_hash, sizeof(node_data.hwid_hash));

    //Write Data to the records
    if (!writeNodeData(node_data, node_id)) {
        server_state = FAILED_TO_ADD_NODE;
        fault_node_id = node_id;
        return false;
    }

    setOccupationMask(node_id);
    return true;
}

//Checks if a valid Server Record is present for specified Node ID
bool AP_DroneCAN_DNA_Server::isValidNodeDataAvailable(uint8_t node_id)
{
    NodeData node_data;
    readNodeData(node_data, node_id);

    uint8_t empty_hwid[sizeof(NodeData::hwid_hash)] {};
    uint8_t crc = crc_crc8(node_data.hwid_hash, sizeof(node_data.hwid_hash));
    if (crc == node_data.crc && memcmp(&node_data.hwid_hash[0], &empty_hwid[0], sizeof(empty_hwid)) != 0) {
        return true;
    }
    return false;
}

/* Initialises Publishers for respective UAVCAN Instance
Also resets the Server Record in case there is a mismatch
between specified node id and unique id against the existing
Server Record. */
bool AP_DroneCAN_DNA_Server::init(uint8_t own_unique_id[], uint8_t own_unique_id_len, uint8_t node_id)
{
    //Read the details from AP_DroneCAN
    server_state = HEALTHY;
    /* Go through our records and look for valid NodeData, to initialise
    occupation mask */
    for (uint8_t i = 0; i <= MAX_NODE_ID; i++) {
        if (isValidNodeDataAvailable(i)) {
            occupation_mask.set(i);
        }
    }

    // Check if the magic is present
    uint16_t magic;
    {
        WITH_SEMAPHORE(storage_sem);
        storage.read_block(&magic, 0, NODEDATA_MAGIC_LEN);
    }
    if (magic != NODEDATA_MAGIC) {
        //Its not there a reset should write it in the Storage
        reset();
    }
    if (_ap_dronecan.check_and_reset_option(AP_DroneCAN::Options::DNA_CLEAR_DATABASE)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UC DNA database reset");
        reset();
    }
    // Making sure that the server is started with the same node ID
    const uint8_t stored_own_node_id = getNodeIDForUniqueID(own_unique_id, own_unique_id_len);
    static bool reset_done;
    if (stored_own_node_id != 255) {
        if (stored_own_node_id != node_id) {
            /* We have a different node id recorded against our own unique id
            This calls for a reset */
            if (!reset_done) {
                /* ensure we only reset once per power cycle
                else we will wipe own record on next init(s) */
                reset();
                reset_done = true;
            }
            //Add ourselves to the Server Record
            if (!addNodeIDForUniqueID(node_id, own_unique_id, own_unique_id_len)) {
                return false;
            }
        }
    } else {
        //We have no record of our own Unique ID do a reset
        if (!reset_done) {
            /* ensure we only reset once per power cycle
            else we will wipe own record on next init(s) */
            reset();
            reset_done = true;
        }
        //Add ourselves to the Server Record
        if (!addNodeIDForUniqueID(node_id, own_unique_id, own_unique_id_len)) {
            return false;
        }
    }
    /* Also add to seen node id this is to verify
    if any duplicates are on the bus carrying our Node ID */
    addToSeenNodeMask(node_id);
    setVerificationMask(node_id);
    node_healthy_mask.set(node_id);
    self_node_id = node_id;
    return true;
}


//Reset the Server Records
void AP_DroneCAN_DNA_Server::reset()
{
    NodeData node_data;
    memset(&node_data, 0, sizeof(node_data));
    occupation_mask.clearall();

    //Just write empty Node Data to the Records
    for (uint8_t i = 0; i <= MAX_NODE_ID; i++) {
        writeNodeData(node_data, i);
    }
    WITH_SEMAPHORE(storage_sem);
    //Ensure we mark magic at the end
    uint16_t magic = NODEDATA_MAGIC;
    storage.write_block(0, &magic, NODEDATA_MAGIC_LEN);
}

/* Go through the Occupation mask for available Node ID
based on pseudo code provided in
uavcan/protocol/dynamic_node_id/1.Allocation.uavcan */
uint8_t AP_DroneCAN_DNA_Server::findFreeNodeID(uint8_t preferred)
{
    // Search up
    uint8_t candidate = (preferred > 0) ? preferred : 125;
    while (candidate <= 125) {
        if (!isNodeIDOccupied(candidate)) {
            return candidate;
        }
        candidate++;
    }
    //Search down
    candidate = (preferred > 0) ? preferred : 125;
    while (candidate > 0) {
        if (!isNodeIDOccupied(candidate)) {
            return candidate;
        }
        candidate--;
    }
    // Not found
    return 255;
}

//Check if we have received Node Status from this node_id
bool AP_DroneCAN_DNA_Server::isNodeSeen(uint8_t node_id)
{
    if (node_id > MAX_NODE_ID) {
        return false;
    }
    return node_seen_mask.get(node_id);
}

/* Set the Seen Node Mask, to be called when received
Node Status from the node id */
void AP_DroneCAN_DNA_Server::addToSeenNodeMask(uint8_t node_id)
{
    if (node_id > MAX_NODE_ID) {
        return;
    }
    node_seen_mask.set(node_id);
}

/* Run through the list of seen node ids for verification no more
than once per 5 second. We continually verify the nodes in our 
seen list, So that we can raise issue if there are duplicates
on the bus. */
void AP_DroneCAN_DNA_Server::verify_nodes()
{
    uint32_t now = AP_HAL::millis();
    if ((now - last_verification_request) < 5000) {
        return;
    }

#if HAL_LOGGING_ENABLED
    uint8_t log_count = AP::logger().get_log_start_count();
    if (log_count != last_logging_count) {
        last_logging_count = log_count;
        logged.clearall();
    }
#endif

    //Check if we got acknowledgement from previous request
    //except for requests using our own node_id
    if (curr_verifying_node == self_node_id) {
        nodeInfo_resp_rcvd = true;
    }

    if (!nodeInfo_resp_rcvd) {
        /* Also notify GCS about this
        Reason for this could be either the node was disconnected
        Or a node with conflicting ID appeared and is sending response
        at the same time. */
        /* Only report if the node was verified, otherwise ignore
        as this could be just Bootloader to Application transition. */
        if (isNodeIDVerified(curr_verifying_node)) {
            // remove verification flag for this node
            verified_mask.clear(curr_verifying_node);
        }
    }

    last_verification_request = now;
    //Find the next registered Node ID to be verified.
    for (uint8_t i = 0; i <= MAX_NODE_ID; i++) {
        curr_verifying_node = (curr_verifying_node + 1) % (MAX_NODE_ID + 1);
        if ((curr_verifying_node == self_node_id) || (curr_verifying_node == 0)) {
            continue;
        }
        if (isNodeSeen(curr_verifying_node)) {
            break;
        }
    }
    if (isNodeIDOccupied(curr_verifying_node)) {
        uavcan_protocol_GetNodeInfoRequest request;
        node_info_client.request(curr_verifying_node, request);
        nodeInfo_resp_rcvd = false;
    }
}

/* Handles Node Status Message, adds to the Seen Node list
Also starts the Service call for Node Info to complete the
Verification process. */
void AP_DroneCAN_DNA_Server::handleNodeStatus(const CanardRxTransfer& transfer, const uavcan_protocol_NodeStatus& msg)
{
    if (transfer.source_node_id > MAX_NODE_ID || transfer.source_node_id == 0) {
        return;
    }
    if ((msg.health != UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK ||
        msg.mode != UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL) &&
        !_ap_dronecan.option_is_set(AP_DroneCAN::Options::DNA_IGNORE_UNHEALTHY_NODE)) {
        //if node is not healthy or operational, clear resp health mask, and set fault_node_id
        fault_node_id = transfer.source_node_id;
        server_state = NODE_STATUS_UNHEALTHY;
        node_healthy_mask.clear(transfer.source_node_id);
    } else {
        node_healthy_mask.set(transfer.source_node_id);
        if (node_healthy_mask == verified_mask) {
            server_state = HEALTHY;
        }
    }
    if (!isNodeIDVerified(transfer.source_node_id)) {
        //immediately begin verification of the node_id
        uavcan_protocol_GetNodeInfoRequest request;
        node_info_client.request(transfer.source_node_id, request);
    }
    //Add node to seen list if not seen before
    addToSeenNodeMask(transfer.source_node_id);
}

/* Node Info message handler
Handle responses from GetNodeInfo Request. We verify the node info
against our records. Marks Verification mask if already recorded,
Or register if the node id is available and not recorded for the
received Unique ID */
void AP_DroneCAN_DNA_Server::handleNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoResponse& rsp)
{
    if (transfer.source_node_id > MAX_NODE_ID) {
        return;
    }
    /*
      if we haven't logged this node then log it now
     */
#if HAL_LOGGING_ENABLED
    if (!logged.get(transfer.source_node_id) && AP::logger().logging_started()) {
        logged.set(transfer.source_node_id);
        uint64_t uid[2];
        memcpy(uid, rsp.hardware_version.unique_id, sizeof(rsp.hardware_version.unique_id));
        // @LoggerMessage: CAND
        // @Description: Info from GetNodeInfo request
        // @Field: TimeUS: Time since system startup
        // @Field: NodeId: Node ID
        // @Field: UID1: Hardware ID, part 1
        // @Field: UID2: Hardware ID, part 2
        // @Field: Name: Name string
        // @Field: Major: major revision id
        // @Field: Minor: minor revision id
        // @Field: Version: AP_Periph git hash
        AP::logger().Write("CAND", "TimeUS,NodeId,UID1,UID2,Name,Major,Minor,Version",
                           "s#------", "F-------", "QBQQZBBI",
                           AP_HAL::micros64(),
                           transfer.source_node_id,
                           uid[0], uid[1],
                           rsp.name.data,
                           rsp.software_version.major,
                           rsp.software_version.minor,
                           rsp.software_version.vcs_commit);
    }
#endif

    if (isNodeIDOccupied(transfer.source_node_id)) {
        //if node_id already registered, just verify if Unique ID matches as well
        if (transfer.source_node_id == getNodeIDForUniqueID(rsp.hardware_version.unique_id, 16)) {
            if (transfer.source_node_id == curr_verifying_node) {
                nodeInfo_resp_rcvd = true;
            }
            setVerificationMask(transfer.source_node_id);
        } else if (!_ap_dronecan.option_is_set(AP_DroneCAN::Options::DNA_IGNORE_DUPLICATE_NODE)) {
            /* This is a device with node_id already registered
            for another device */
            server_state = DUPLICATE_NODES;
            fault_node_id = transfer.source_node_id;
            memcpy(fault_node_name, rsp.name.data, sizeof(fault_node_name));
        }
    } else {
        /* Node Id was not allocated by us, or during this boot, let's register this in our records
        Check if we allocated this Node before */
        uint8_t prev_node_id = getNodeIDForUniqueID(rsp.hardware_version.unique_id, 16);
        if (prev_node_id != 255) {
            //yes we did, remove this registration
            freeNodeID(prev_node_id);
        }
        //add a new server record
        addNodeIDForUniqueID(transfer.source_node_id, rsp.hardware_version.unique_id, 16);
        //Verify as well
        setVerificationMask(transfer.source_node_id);
        if (transfer.source_node_id == curr_verifying_node) {
            nodeInfo_resp_rcvd = true;
        }
    }
}

/* Handle the allocation message from the devices supporting
dynamic node allocation. */
void AP_DroneCAN_DNA_Server::handleAllocation(const CanardRxTransfer& transfer, const uavcan_protocol_dynamic_node_id_Allocation& msg)
{
    if (transfer.source_node_id != 0) {
        //Ignore Allocation messages that are not DNA requests
        return;
    }
    uint32_t now = AP_HAL::millis();

    if (rcvd_unique_id_offset == 0 ||
        (now - last_alloc_msg_ms) > 500) {
        if (msg.first_part_of_unique_id) {
            rcvd_unique_id_offset = 0;
            memset(rcvd_unique_id, 0, sizeof(rcvd_unique_id));
        } else {
            //we are only accepting first part
            return;
        }
    } else if (msg.first_part_of_unique_id) {
        // we are only accepting follow up messages
        return;
    }

    if (rcvd_unique_id_offset) {
        debug_dronecan(AP_CANManager::LOG_DEBUG, "TIME: %ld  -- Accepting Followup part! %u\n",
                     (long int)AP_HAL::millis(),
                     unsigned((now - last_alloc_msg_ms)));
    } else {
        debug_dronecan(AP_CANManager::LOG_DEBUG, "TIME: %ld  -- Accepting First part! %u\n",
                     (long int)AP_HAL::millis(),
                     unsigned((now - last_alloc_msg_ms)));
    }

    last_alloc_msg_ms = now;
    if ((rcvd_unique_id_offset + msg.unique_id.len) > 16) {
        //This request is malformed, Reset!
        rcvd_unique_id_offset = 0;
        memset(rcvd_unique_id, 0, sizeof(rcvd_unique_id));
        return;
    }

    //copy over the unique_id
    for (uint8_t i=rcvd_unique_id_offset; i<(rcvd_unique_id_offset + msg.unique_id.len); i++) {
        rcvd_unique_id[i] = msg.unique_id.data[i - rcvd_unique_id_offset];
    }
    rcvd_unique_id_offset += msg.unique_id.len;

    //send follow up message
    uavcan_protocol_dynamic_node_id_Allocation rsp {};

    /* Respond with the message containing the received unique ID so far
    or with node id if we successfully allocated one. */
    memcpy(rsp.unique_id.data, rcvd_unique_id, rcvd_unique_id_offset);
    rsp.unique_id.len = rcvd_unique_id_offset;

    if (rcvd_unique_id_offset == 16) {
        //We have received the full Unique ID, time to do allocation
        uint8_t resp_node_id = getNodeIDForUniqueID((const uint8_t*)rcvd_unique_id, 16);
        if (resp_node_id == 255) {
            resp_node_id = findFreeNodeID(msg.node_id);
            if (resp_node_id != 255) {
                if (addNodeIDForUniqueID(resp_node_id, (const uint8_t*)rcvd_unique_id, 16)) {
                    rsp.node_id = resp_node_id;
                }
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "UC Node Alloc Failed!");
            }
        } else {
            rsp.node_id = resp_node_id;
        }
        //reset states as well        
        rcvd_unique_id_offset = 0;
        memset(rcvd_unique_id, 0, sizeof(rcvd_unique_id));
    }

    allocation_pub.broadcast(rsp, false); // never publish allocation message with CAN FD
}

//report the server state, along with failure message if any
bool AP_DroneCAN_DNA_Server::prearm_check(char* fail_msg, uint8_t fail_msg_len) const
{
    switch (server_state) {
    case HEALTHY:
        return true;
    case STORAGE_FAILURE: {
        snprintf(fail_msg, fail_msg_len, "Failed to access storage!");
        return false;
    }
    case DUPLICATE_NODES: {
        if (_ap_dronecan.option_is_set(AP_DroneCAN::Options::DNA_IGNORE_DUPLICATE_NODE)) {
            // ignore error
            return true;
        }
        snprintf(fail_msg, fail_msg_len, "Duplicate Node %s../%d!", fault_node_name, fault_node_id);
        return false;
    }
    case FAILED_TO_ADD_NODE: {
        snprintf(fail_msg, fail_msg_len, "Failed to add Node %d!", fault_node_id);
        return false;
    }
    case NODE_STATUS_UNHEALTHY: {
        if (_ap_dronecan.option_is_set(AP_DroneCAN::Options::DNA_IGNORE_UNHEALTHY_NODE)) {
            // ignore error
            return true;
        }
        snprintf(fail_msg, fail_msg_len, "Node %d unhealthy!", fault_node_id);
        return false;
    }
    }
    // should never get; compiler should enforce all server_states are covered
    return false;
}

#endif //HAL_NUM_CAN_IFACES
