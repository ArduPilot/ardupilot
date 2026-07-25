/*
  A simple example DroneCAN node implementing a 4-in-1 ESC using the C++ API

  This example implements 5 features:

   - announces on the bus using NodeStatus at 1Hz
   - answers GetNodeInfo requests
   - implements dynamic node allocation
   - listens for ESC RawCommand commands and extracts throttle levels
   - sends ESC Status messages (with synthetic data based on throttles)
   - a parameter server for reading and writing node parameters

  This example uses socketcan on Linux for CAN transport

  Example usage: ./esc_node vcan0
*/
/*
 This example application is distributed under the terms of CC0 (public domain dedication).
 More info: https://creativecommons.org/publicdomain/zero/1.0/
*/

// system includes
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <stdbool.h>

// include the canard C++ APIs
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include <canard/service_server.h>
#include <canard/handler_list.h>
#include <canard/transfer_object.h>

// include the base canard API
#include <canard.h>

// we are using the socketcan driver
#include <socketcan.h>

// include the headers for the generated DroneCAN messages from the
// dronecan_dsdlc compiler
#include <dronecan_msgs.h>

/*
  in this example we will use dynamic node allocation if MY_NODE_ID is zero
 */
#define MY_NODE_ID 0

/*
  our preferred node ID if nobody else has it
 */
#define PREFERRED_NODE_ID 73

// implement a 4-in-1 ESC
#define NUM_ESCS 4

/*
  create a CanardInterface class for interfacing with the hardware
 */
class CanardInterface : public Canard::Interface {
    friend class ESCNode;

    CanardInterface(uint8_t iface_index) :
        Interface(iface_index) {}
    
public:
    void init(const char *interface_name);

    // implement required interface functions
    bool broadcast(const Canard::Transfer &bcast_transfer) override;
    bool request(uint8_t destination_node_id, const Canard::Transfer &req_transfer) override;
    bool respond(uint8_t destination_node_id, const Canard::Transfer &res_transfer) override;

    void process(uint32_t duration_ms);

    static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);
    static bool shouldAcceptTransfer(const CanardInstance* ins,
                                     uint64_t* out_data_type_signature,
                                     uint16_t data_type_id,
                                     CanardTransferType transfer_type,
                                     uint8_t source_node_id);

    uint8_t get_node_id() const override { return canard.node_id; }
    void set_node_id(uint8_t node_id) {
        canardSetLocalNodeID(&canard, node_id);
    }

private:
    uint8_t memory_pool[2048];
    CanardInstance canard;
    CanardTxTransfer tx_transfer;

    // we will use socketcan driver for this example
    SocketCANInstance socketcan;
};

/*
  declare heads of handler and transfer lists
 */
DEFINE_HANDLER_LIST_HEADS();
DEFINE_TRANSFER_OBJECT_HEADS();

class ESCNode {
public:
    void start_node(const char *interface_name);

private:
    CanardInterface canard_iface{0};

    // declare publishers for outgoing messages
    Canard::Publisher<uavcan_protocol_NodeStatus> node_status{canard_iface};
    Canard::Publisher<uavcan_equipment_esc_Status> esc_status{canard_iface};

    // incoming messages
    void handle_RawCommand(const CanardRxTransfer& transfer, const uavcan_equipment_esc_RawCommand& cmd);
    Canard::ObjCallback<ESCNode, uavcan_equipment_esc_RawCommand> raw_command_cb{this, &ESCNode::handle_RawCommand};
    Canard::Subscriber<uavcan_equipment_esc_RawCommand> raw_command_listener{raw_command_cb, 0};

    // Node Info Server
    void handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req);
    Canard::ObjCallback<ESCNode, uavcan_protocol_GetNodeInfoRequest> node_info_req_cb{this, &ESCNode::handle_GetNodeInfo};
    Canard::Server<uavcan_protocol_GetNodeInfoRequest> node_info_server{canard_iface, node_info_req_cb};

    // parameter server
    void handle_param_GetSet(const CanardRxTransfer& transfer, const uavcan_protocol_param_GetSetRequest& req);
    Canard::ObjCallback<ESCNode, uavcan_protocol_param_GetSetRequest> param_get_set_req_cb{this, &ESCNode::handle_param_GetSet};
    Canard::Server<uavcan_protocol_param_GetSetRequest> param_server{canard_iface, param_get_set_req_cb};
    void handle_param_ExecuteOpcode(const CanardRxTransfer& transfer, const uavcan_protocol_param_ExecuteOpcodeRequest& req);
    Canard::ObjCallback<ESCNode, uavcan_protocol_param_ExecuteOpcodeRequest> param_executeopcode_req_cb{this, &ESCNode::handle_param_ExecuteOpcode};
    Canard::Server<uavcan_protocol_param_ExecuteOpcodeRequest> param_opcode_server{canard_iface, param_executeopcode_req_cb};
    
    // handlers for dynamic node allocation (DNA)
    Canard::Publisher<uavcan_protocol_dynamic_node_id_Allocation> allocation_pub{canard_iface};
    void handle_DNA_Allocation(const CanardRxTransfer& transfer, const uavcan_protocol_dynamic_node_id_Allocation& msg);
    Canard::ObjCallback<ESCNode, uavcan_protocol_dynamic_node_id_Allocation> allocation_cb{this, &ESCNode::handle_DNA_Allocation};
    Canard::Subscriber<uavcan_protocol_dynamic_node_id_Allocation> allocation_listener{allocation_cb, 0};

    // DNA request call
    void request_DNA();

    void send_NodeStatus(void);
    void process1HzTasks(uint64_t timestamp_usec);
    void send_ESCStatus(void);

    /*
      keep the state of 4 ESCs, simulating a 4 in 1 ESC node
    */
    struct esc_state {
        float throttle;
        uint64_t last_update_us;
    } escs[NUM_ESCS];

    // keep node_status around for updating status
    uavcan_protocol_NodeStatus node_status_msg;

    /*
      data for dynamic node allocation process
    */
    struct {
        uint32_t send_next_node_id_allocation_request_at_ms;
        uint32_t node_id_allocation_unique_id_offset;
    } DNA;

    static struct parameter {
        const char *name;
        enum uavcan_protocol_param_Value_type_t type;
        float value;
        float min_value;
        float max_value;
    } parameters[];
};

/*
  a set of parameters to present to the user. In this example we don't
  actually save parameters, this is just to show how to handle the
  parameter protocool
 */
ESCNode::parameter ESCNode::parameters[] = {
    { "CAN_NODE", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, MY_NODE_ID, 0, 127 },
    { "MyPID_P", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, 1.2, 0.1, 5.0 },
    { "MyPID_I", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, 1.35, 0.1, 5.0 },
    { "MyPID_D", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, 0.025, 0.001, 1.0 },
};

/*
  get a 64 bit monotonic timestamp in microseconds since start. This
  is platform specific
 */
static uint64_t micros64(void)
{
    static uint64_t first_us;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t tus = (uint64_t)(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL);
    if (first_us == 0) {
        first_us = tus;
    }
    return tus - first_us;
}

/*
  get monotonic time in milliseconds since startup
 */
static uint32_t millis32(void)
{
    return micros64() / 1000ULL;
}

/*
  get a 16 byte unique ID for this node, this should be based on the CPU unique ID or other unique ID
 */
static void getUniqueID(uint8_t id[16])
{
    memset(id, 0, 16);
    FILE *f = fopen("/etc/machine-id", "r");
    if (f) {
        fread(id, 1, 16, f);
        fclose(f);
    }
}

bool CanardInterface::broadcast(const Canard::Transfer &bcast_transfer) {
    tx_transfer = {
        .transfer_type = bcast_transfer.transfer_type,
        .data_type_signature = bcast_transfer.data_type_signature,
        .data_type_id = bcast_transfer.data_type_id,
        .inout_transfer_id = bcast_transfer.inout_transfer_id,
        .priority = bcast_transfer.priority,
        .payload = (const uint8_t*)bcast_transfer.payload,
        .payload_len = uint16_t(bcast_transfer.payload_len),
#if CANARD_ENABLE_CANFD
        .canfd = bcast_transfer.canfd,
#endif
#if CANARD_ENABLE_DEADLINE
        .deadline_usec = micros64() + (bcast_transfer.timeout_ms * 1000),
#endif
#if CANARD_MULTI_IFACE
        .iface_mask = uint8_t((1<<num_ifaces) - 1),
#endif
    };
    // do canard broadcast
    bool success = canardBroadcastObj(&canard, &tx_transfer) > 0;
    return success;
}

bool CanardInterface::request(uint8_t destination_node_id, const Canard::Transfer &req_transfer) {
    tx_transfer = {
        .transfer_type = req_transfer.transfer_type,
        .data_type_signature = req_transfer.data_type_signature,
        .data_type_id = req_transfer.data_type_id,
        .inout_transfer_id = req_transfer.inout_transfer_id,
        .priority = req_transfer.priority,
        .payload = (const uint8_t*)req_transfer.payload,
        .payload_len = uint16_t(req_transfer.payload_len),
#if CANARD_ENABLE_CANFD
        .canfd = req_transfer.canfd,
#endif
#if CANARD_ENABLE_DEADLINE
        .deadline_usec = micros64() + (req_transfer.timeout_ms * 1000),
#endif
#if CANARD_MULTI_IFACE
        .iface_mask = uint8_t((1<<num_ifaces) - 1),
#endif
    };
    // do canard request
    return canardRequestOrRespondObj(&canard, destination_node_id, &tx_transfer) > 0;
}

bool CanardInterface::respond(uint8_t destination_node_id, const Canard::Transfer &res_transfer) {
    tx_transfer = {
        .transfer_type = res_transfer.transfer_type,
        .data_type_signature = res_transfer.data_type_signature,
        .data_type_id = res_transfer.data_type_id,
        .inout_transfer_id = res_transfer.inout_transfer_id,
        .priority = res_transfer.priority,
        .payload = (const uint8_t*)res_transfer.payload,
        .payload_len = uint16_t(res_transfer.payload_len),
#if CANARD_ENABLE_CANFD
        .canfd = res_transfer.canfd,
#endif
#if CANARD_ENABLE_DEADLINE
        .deadline_usec = micros64() + (res_transfer.timeout_ms * 1000),
#endif
#if CANARD_MULTI_IFACE
        .iface_mask = uint8_t((1<<num_ifaces) - 1),
#endif
    };
    // do canard respond
    return canardRequestOrRespondObj(&canard, destination_node_id, &tx_transfer) > 0;
}

// convenience macros
#define MIN(a,b) ((a)<(b)?(a):(b))
#define C_TO_KELVIN(temp) (temp + 273.15f)
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

/*
  handle a GetNodeInfo request
 */
void ESCNode::handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req)
{
    uavcan_protocol_GetNodeInfoResponse node_info_rsp {};

    // fill in node name
    node_info_rsp.name.len = snprintf((char*)node_info_rsp.name.data, sizeof(node_info_rsp.name.data), "ESCNode");

    // fill in software and hardware versions
    node_info_rsp.software_version.major = 1;
    node_info_rsp.software_version.minor = 2;
    node_info_rsp.hardware_version.major = 3;
    node_info_rsp.hardware_version.minor = 7;
    getUniqueID(node_info_rsp.hardware_version.unique_id);
    node_info_rsp.status = node_status_msg;
    node_info_rsp.status.uptime_sec = millis32() / 1000UL;

    node_info_server.respond(transfer, node_info_rsp);
}

/*
  handle a ESC RawCommand request
*/
void ESCNode::handle_RawCommand(const CanardRxTransfer& transfer, const uavcan_equipment_esc_RawCommand& cmd)
{
    // remember the demand for the ESC status output
    const uint8_t num_throttles = MIN(cmd.cmd.len, NUM_ESCS);
    const uint64_t tnow = micros64();
    for (uint8_t i=0; i<num_throttles; i++) {
        // convert throttle to -1.0 to 1.0 range
        escs[i].throttle = cmd.cmd.data[i]/8192.0;
        escs[i].last_update_us = tnow;
    }
}

/*
  handle parameter GetSet request
 */
void ESCNode::handle_param_GetSet(const CanardRxTransfer& transfer, const uavcan_protocol_param_GetSetRequest& req)
{
    struct parameter *p = nullptr;
    if (req.name.len != 0) {
        for (uint16_t i=0; i<ARRAY_SIZE(parameters); i++) {
            if (req.name.len == strlen(parameters[i].name) &&
                strncmp((const char *)req.name.data, parameters[i].name, req.name.len) == 0) {
                p = &parameters[i];
                break;
            }
        }
    } else if (req.index < ARRAY_SIZE(parameters)) {
        p = &parameters[req.index];
    }
    if (p != nullptr && req.name.len != 0 && req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY) {
        /*
          this is a parameter set command. The implementation can
          either choose to store the value in a persistent manner
          immediately or can instead store it in memory and save to permanent storage on a
         */
        switch (p->type) {
        case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
            p->value = req.value.integer_value;
            break;
        case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE:
            p->value = req.value.real_value;
            break;
        default:
            return;
        }
    }

    /*
      for both set and get we reply with the current value
     */
    uavcan_protocol_param_GetSetResponse pkt {};

    if (p != NULL) {
        pkt.value.union_tag = p->type;
        switch (p->type) {
        case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
            pkt.value.integer_value = p->value;
            break;
        case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE:
            pkt.value.real_value = p->value;
            break;
        default:
            return;
        }
        pkt.name.len = strlen(p->name);
        strcpy((char *)pkt.name.data, p->name);
    }

    param_server.respond(transfer, pkt);
}

/*
  handle parameter executeopcode request
 */
void ESCNode::handle_param_ExecuteOpcode(const CanardRxTransfer& transfer, const uavcan_protocol_param_ExecuteOpcodeRequest& req)
{
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        // here is where you would reset all parameters to defaults
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
        // here is where you would save all the changed parameters to permanent storage
    }

    uavcan_protocol_param_ExecuteOpcodeResponse pkt {};
    pkt.ok = true;
    param_opcode_server.respond(transfer, pkt);
}

/*
  handle DNA allocation responses
 */
void ESCNode::handle_DNA_Allocation(const CanardRxTransfer& transfer, const uavcan_protocol_dynamic_node_id_Allocation& msg)
{
    if (canard_iface.get_node_id() != CANARD_BROADCAST_NODE_ID) {
        // already allocated
        return;
    }

    // Rule C - updating the randomized time interval
    DNA.send_next_node_id_allocation_request_at_ms =
        millis32() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (random() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer.source_node_id == CANARD_BROADCAST_NODE_ID) {
        printf("Allocation request from another allocatee\n");
        DNA.node_id_allocation_unique_id_offset = 0;
        return;
    }

    // Obtaining the local unique ID
    uint8_t my_unique_id[sizeof(msg.unique_id.data)];
    getUniqueID(my_unique_id);

    // Matching the received UID against the local one
    if (memcmp(msg.unique_id.data, my_unique_id, msg.unique_id.len) != 0) {
        printf("Mismatching allocation response\n");
        DNA.node_id_allocation_unique_id_offset = 0;
        // No match, return
        return;
    }

    if (msg.unique_id.len < sizeof(msg.unique_id.data)) {
        // The allocator has confirmed part of unique ID, switching to
        // the next stage and updating the timeout.
        DNA.node_id_allocation_unique_id_offset = msg.unique_id.len;
        DNA.send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

        printf("Matching allocation response: %d\n", msg.unique_id.len);
    } else {
        // Allocation complete - copying the allocated node ID from the message
        canard_iface.set_node_id(msg.node_id);
        printf("Node ID allocated: %d\n", msg.node_id);
    }
}

/*
  ask for a dynamic node allocation
 */
void ESCNode::request_DNA()
{
    const uint32_t now = millis32();

    DNA.send_next_node_id_allocation_request_at_ms =
        now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (random() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    uint8_t my_unique_id[16];
    getUniqueID(my_unique_id);
    
    // send allocation message
    uavcan_protocol_dynamic_node_id_Allocation req {};

    req.node_id = PREFERRED_NODE_ID;
    req.first_part_of_unique_id = (DNA.node_id_allocation_unique_id_offset == 0);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(16 - DNA.node_id_allocation_unique_id_offset);
    
    if (uid_size > MaxLenOfUniqueIDInRequest) {
        uid_size = MaxLenOfUniqueIDInRequest;
    }

    req.unique_id.len = uid_size;
    memcpy(req.unique_id.data, &my_unique_id[DNA.node_id_allocation_unique_id_offset], uid_size);

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    DNA.node_id_allocation_unique_id_offset = 0;

    allocation_pub.broadcast(req);
}

/*
  send the 1Hz NodeStatus message. This is what allows a node to show
  up in the DroneCAN GUI tool and in the flight controller logs
 */
void ESCNode::send_NodeStatus(void)
{
    node_status_msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status_msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status_msg.sub_mode = 0;
    node_status_msg.uptime_sec = millis32() / 1000UL;

    node_status.broadcast(node_status_msg);
}

/*
  This function is called at 1 Hz rate from the main loop.
*/
void ESCNode::process1HzTasks(uint64_t timestamp_usec)
{
    /*
      Transmit the node status message
    */
    send_NodeStatus();
}

/*
  send ESC status at 50Hz
*/
void ESCNode::send_ESCStatus(void)
{
    // send a separate status packet for each ESC
    for (uint8_t i=0; i<NUM_ESCS; i++) {
        uavcan_equipment_esc_Status pkt {};

        // make up some synthetic status data
        pkt.error_count = 0;
        pkt.voltage = 16.8 - 2.0 * escs[i].throttle;
        pkt.current = 20 * escs[i].throttle;
        pkt.temperature = C_TO_KELVIN(25.0);
        pkt.rpm = 10000 * escs[i].throttle;
        pkt.power_rating_pct = 100.0 * escs[i].throttle;

        esc_status.broadcast(pkt);
    }
}


/*
  Transmits all frames from the TX queue, receives up to one frame.
*/
void CanardInterface::process(uint32_t timeout_msec)
{
    // Transmitting
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        const int16_t tx_res = socketcanTransmit(&socketcan, txf, 0);
        if (tx_res != 0) {
            canardPopTxQueue(&canard);
        }
    }

    // Receiving
    const uint32_t start_ms = millis32();
    while (millis32() - start_ms < timeout_msec) {
        CanardCANFrame rx_frame;
        const int16_t rx_res = socketcanReceive(&socketcan, &rx_frame, timeout_msec);
        if (rx_res > 0) {
            canardHandleRxFrame(&canard, &rx_frame, micros64());
        }
    }
}

/*
  handle an incoming message
 */
void CanardInterface::onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
    CanardInterface* iface = (CanardInterface*) ins->user_reference;
    iface->handle_message(*transfer);
}

/*
  check if we want the message. This is based on what we have subscribed to
 */
bool CanardInterface::shouldAcceptTransfer(const CanardInstance* ins,
                                   uint64_t* out_data_type_signature,
                                   uint16_t data_type_id,
                                   CanardTransferType transfer_type,
                                   uint8_t source_node_id)
{
    CanardInterface* iface = (CanardInterface*)ins->user_reference;
    return iface->accept_message(data_type_id, transfer_type, *out_data_type_signature);
}

/*
  Initializing the Libcanard instance.
*/
void CanardInterface::init(const char *interface_name)
{
    int16_t res = socketcanInit(&socketcan, interface_name);
    if (res < 0) {
        (void)fprintf(stderr, "Failed to open CAN iface '%s'\n", interface_name);
        exit(1);
    }

    // init canard object
    canardInit(&canard,
               memory_pool,
               sizeof(memory_pool),
               onTransferReceived,
               shouldAcceptTransfer,
               this);

    // set node ID if not doing DNA
    if (MY_NODE_ID > 0) {
        canardSetLocalNodeID(&canard, MY_NODE_ID);
    } else {
        printf("Waiting for DNA node ID allocation\n");
    }
}

/*
 Initializing the CAN backend driver; in this example we're using SocketCAN
 */
void ESCNode::start_node(const char *interface_name)
{
    // init the interface
    canard_iface.init(interface_name);

    /*
      Run the main loop.
     */
    uint64_t next_1hz_service_at = micros64();
    uint64_t next_50hz_service_at = micros64();

    while (true) {
        canard_iface.process(1);

        const uint64_t ts = micros64();

        // see if we are still doing DNA
        if (canard_iface.get_node_id() == CANARD_BROADCAST_NODE_ID) {
            // we're still waiting for a DNA allocation of our node ID
            if (millis32() > DNA.send_next_node_id_allocation_request_at_ms) {
                request_DNA();
            }
            continue;
        }

        if (ts >= next_1hz_service_at) {
            next_1hz_service_at += 1000000ULL;
            process1HzTasks(ts);
        }
        if (ts >= next_50hz_service_at) {
            next_50hz_service_at += 1000000ULL/50U;
            send_ESCStatus();
        }
    }
}

// declare our ESC node
static ESCNode node;

/*
  main program
 */
int main(int argc, char** argv)
{
    if (argc < 2) {
        (void)fprintf(stderr,
                      "Usage:\n"
                      "\t%s <can iface name>\n",
                      argv[0]);
        return 1;
    }

    node.start_node(argv[1]);
    return 0;
}
