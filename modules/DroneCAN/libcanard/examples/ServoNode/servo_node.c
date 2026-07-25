/*
  A simple example DroneCAN node implementing a servo actuator

  This example implements 6 features:

   - announces on the bus using NodeStatus at 1Hz
   - answers GetNodeInfo requests
   - implements dynamic node allocation
   - listens for servo actuator commands and extracts demanded position
   - sends servo status messages (with synthetic data based on position)
   - a parameter server for reading and writing node parameters

  This example uses socketcan on Linux for CAN transport

  Example usage: ./servo_node vcan0
*/
/*
 This example application is distributed under the terms of CC0 (public domain dedication).
 More info: https://creativecommons.org/publicdomain/zero/1.0/
*/

#ifndef _GNU_SOURCE
# define _GNU_SOURCE
#endif

#include <canard.h>
#include <linux.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

// include the headers for the generated DroneCAN messages from the
// dronecan_dsdlc compiler
#include <dronecan_msgs.h>

/*
  libcanard library instance and a memory pool for it to use
 */
static CanardInstance canard;
static uint8_t memory_pool[1024];

/*
  in this example we will use dynamic node allocation if MY_NODE_ID is zero
 */
#define MY_NODE_ID 0

/*
  our preferred node ID if nobody else has it
 */
#define PREFERRED_NODE_ID 73


/*
  keep the state of 4 servos, simulating a 4 servo node
 */
#define NUM_SERVOS 4
static struct servo_state {
    float position; // -1 to 1
    uint64_t last_update_us;
} servos[NUM_SERVOS];

/*
  keep the state for firmware update
 */
static struct {
    char path[256];
    uint8_t node_id;
    uint8_t transfer_id;
    uint32_t last_read_ms;
    int fd;
    uint32_t offset;
} fwupdate;

/*
  a set of parameters to present to the user. In this example we don't
  actually save parameters, this is just to show how to handle the
  parameter protocool
 */
static struct parameter {
    char *name;
    enum uavcan_protocol_param_Value_type_t type;
    float value;
    float min_value;
    float max_value;
} parameters[] = {
    { "CAN_NODE", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, MY_NODE_ID, 0, 127 },
    { "MyPID_P", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, 1.2, 0.1, 5.0 },
    { "MyPID_I", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, 1.35, 0.1, 5.0 },
    { "MyPID_D", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, 0.025, 0.001, 1.0 },
};

// some convenience macros
#define MIN(a,b) ((a)<(b)?(a):(b))
#define C_TO_KELVIN(temp) (temp + 273.15f)
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

/*
  hold our node status as a static variable. It will be updated on any errors
 */
static struct uavcan_protocol_NodeStatus node_status;

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
void getUniqueID(uint8_t id[16])
{
    memset(id, 0, 16);
    FILE *f = fopen("/etc/machine-id", "r");
    if (f) {
        fread(id, 1, 16, f);
        fclose(f);
    }
}

/*
  handle a GetNodeInfo request
*/
static void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer)
{
    printf("GetNodeInfo request from %d\n", transfer->source_node_id);

    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_GetNodeInfoResponse pkt;

    memset(&pkt, 0, sizeof(pkt));

    node_status.uptime_sec = micros64() / 1000000ULL;
    pkt.status = node_status;

    // fill in your major and minor firmware version
    pkt.software_version.major = 1;
    pkt.software_version.minor = 2;
    pkt.software_version.optional_field_flags = 0;
    pkt.software_version.vcs_commit = 0; // should put git hash in here

    // should fill in hardware version
    pkt.hardware_version.major = 2;
    pkt.hardware_version.minor = 3;

    getUniqueID(pkt.hardware_version.unique_id);

    strncpy((char*)pkt.name.data, "ServoNode", sizeof(pkt.name.data));
    pkt.name.len = strnlen((char*)pkt.name.data, sizeof(pkt.name.data));

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
  handle a servo ArrayCommand request
*/
static void handle_ArrayCommand(CanardInstance *ins, CanardRxTransfer *transfer)
{
    struct uavcan_equipment_actuator_ArrayCommand cmd;
    if (uavcan_equipment_actuator_ArrayCommand_decode(transfer, &cmd)) {
        return;
    }
    uint64_t tnow = micros64();
    for (uint8_t i=0; i < cmd.commands.len; i++) {
        if (cmd.commands.data[i].actuator_id >= NUM_SERVOS) {
            // not for us
            continue;
        }
        switch (cmd.commands.data[i].command_type) {
        case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS:
            servos[cmd.commands.data[i].actuator_id].position = cmd.commands.data[i].command_value;
            break;
        case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM:
            // map PWM to -1 to 1, assuming 1500 trim. If the servo has natural PWM
            // support then we should use it directly instead
            servos[cmd.commands.data[i].actuator_id].position = (cmd.commands.data[i].command_value-1500)/500.0;
            break;
        }
        servos[cmd.commands.data[i].actuator_id].last_update_us = tnow;
    }
}


/*
  handle parameter GetSet request
 */
static void handle_param_GetSet(CanardInstance* ins, CanardRxTransfer* transfer)
{
    struct uavcan_protocol_param_GetSetRequest req;
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, &req)) {
        return;
    }

    struct parameter *p = NULL;
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
    if (p != NULL && req.name.len != 0 && req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY) {
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
    struct uavcan_protocol_param_GetSetResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

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

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_GetSetResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
  handle parameter executeopcode request
 */
static void handle_param_ExecuteOpcode(CanardInstance* ins, CanardRxTransfer* transfer)
{
    struct uavcan_protocol_param_ExecuteOpcodeRequest req;
    if (uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &req)) {
        return;
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        // here is where you would reset all parameters to defaults
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
        // here is where you would save all the changed parameters to permanent storage
    }

    struct uavcan_protocol_param_ExecuteOpcodeResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.ok = true;

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
  data for dynamic node allocation process
 */
static struct {
    uint32_t send_next_node_id_allocation_request_at_ms;
    uint32_t node_id_allocation_unique_id_offset;
} DNA;

/*
  handle a DNA allocation packet
 */
static void handle_DNA_Allocation(CanardInstance *ins, CanardRxTransfer *transfer)
{
    if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID) {
        // already allocated
        return;
    }

    // Rule C - updating the randomized time interval
    DNA.send_next_node_id_allocation_request_at_ms =
        millis32() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (random() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID) {
        printf("Allocation request from another allocatee\n");
        DNA.node_id_allocation_unique_id_offset = 0;
        return;
    }

    // Copying the unique ID from the message
    struct uavcan_protocol_dynamic_node_id_Allocation msg;

    uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg);

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
        canardSetLocalNodeID(ins, msg.node_id);
        printf("Node ID allocated: %d\n", msg.node_id);
    }
}

/*
  ask for a dynamic node allocation
 */
static void request_DNA()
{
    const uint32_t now = millis32();
    static uint8_t node_id_allocation_transfer_id = 0;

    DNA.send_next_node_id_allocation_request_at_ms =
        now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (random() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    allocation_request[0] = (uint8_t)(PREFERRED_NODE_ID << 1U);

    if (DNA.node_id_allocation_unique_id_offset == 0) {
        allocation_request[0] |= 1;     // First part of unique ID
    }

    uint8_t my_unique_id[16];
    getUniqueID(my_unique_id);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(16 - DNA.node_id_allocation_unique_id_offset);
    
    if (uid_size > MaxLenOfUniqueIDInRequest) {
        uid_size = MaxLenOfUniqueIDInRequest;
    }

    memmove(&allocation_request[1], &my_unique_id[DNA.node_id_allocation_unique_id_offset], uid_size);

    // Broadcasting the request
    const int16_t bcast_res = canardBroadcast(&canard,
                                              UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                                              UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                                              &node_id_allocation_transfer_id,
                                              CANARD_TRANSFER_PRIORITY_LOW,
                                              &allocation_request[0],
                                              (uint16_t) (uid_size + 1));
    if (bcast_res < 0) {
        printf("Could not broadcast ID allocation req; error %d\n", bcast_res);
    }

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    DNA.node_id_allocation_unique_id_offset = 0;
}

/*
  handle a BeginFirmwareUpdate request from a management tool like DroneCAN GUI tool or MissionPlanner

  There are multiple ways to handle firmware update over DroneCAN:

    1) on BeginFirmwareUpdate reboot to the bootloader, and implement
       the firmware upudate process in the bootloader. This is good on
       boards with smaller amounts of flash

    2) if you have enough flash for 2 copies of your firmware then you
       can use an A/B scheme, where the new firmware is saved to the
       inactive flash region and a tag is used to indicate which
       firmware to boot next time

    3) you could write the firmware to secondary storage (such as a
       microSD) and the bootloader would flash it on next boot

    In this example firmware we will write it to a file
    newfirmware.bin, which is option 3

    Note that you cannot rely on the form of the filename. The client
    may hash the filename before sending
 */
static void handle_begin_firmware_update(CanardInstance* ins, CanardRxTransfer* transfer)
{
    /*
      on real hardware this is where you would save the current node
      ID to some piece of memory that is not cleared by reboot so the
      bootloader knows what node number to use. Most MCUs have some
      registers (eg. RTC/backup registers) that can be used for that
      purpose.
     */

    /*
      decode the request
     */
    struct uavcan_protocol_file_BeginFirmwareUpdateRequest req;
    if (uavcan_protocol_file_BeginFirmwareUpdateRequest_decode(transfer, &req)) {
        return;
    }

    /*
      check for a repeated BeginFirmwareUpdateRequest
     */
    if (fwupdate.node_id == transfer->source_node_id &&
	fwupdate.fd != -1 &&
	memcmp(fwupdate.path, req.image_file_remote_path.path.data, req.image_file_remote_path.path.len) == 0) {
	/* ignore duplicate request */
	return;
    }


    /*
      open the file to hold the new firmware
     */
    if (fwupdate.fd != -1) {
	close(fwupdate.fd);
    }
    fwupdate.fd = open("newfirmware.bin", O_WRONLY|O_CREAT|O_TRUNC, 0644);
    if (fwupdate.fd == -1) {
	printf("Open of newfirmware.bin failed\n");
	return;
    }

    fwupdate.offset = 0;
    fwupdate.node_id = transfer->source_node_id;
    strncpy(fwupdate.path, (char*)req.image_file_remote_path.path.data, req.image_file_remote_path.path.len);

    uint8_t buffer[UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_file_BeginFirmwareUpdateResponse reply;
    memset(&reply, 0, sizeof(reply));
    reply.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;

    uint32_t total_size = uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(&reply, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE,
                           UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);

    printf("Started firmware update\n");

    /*
      this is where you would reboot to the bootloader if implementing option (1) above
    */
}

/*
  send a read for a firmware update. This asks the client (firmware
  server) for a piece of the new firmware
 */
static void send_firmware_read(void)
{
    uint32_t now = millis32();
    if (now - fwupdate.last_read_ms < 750) {
        // the server may still be responding
        return;
    }
    fwupdate.last_read_ms = now;

    uint8_t buffer[UAVCAN_PROTOCOL_FILE_READ_REQUEST_MAX_SIZE];

    struct uavcan_protocol_file_ReadRequest pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.path.path.len = strlen((const char *)fwupdate.path);
    pkt.offset = fwupdate.offset;
    memcpy(pkt.path.path.data, fwupdate.path, pkt.path.path.len);

    uint16_t total_size = uavcan_protocol_file_ReadRequest_encode(&pkt, buffer);

    canardRequestOrRespond(&canard,
			   fwupdate.node_id,
                           UAVCAN_PROTOCOL_FILE_READ_SIGNATURE,
                           UAVCAN_PROTOCOL_FILE_READ_ID,
			   &fwupdate.transfer_id,
                           CANARD_TRANSFER_PRIORITY_HIGH,
                           CanardRequest,
                           &buffer[0],
                           total_size);
}

/*
  handle response to send_firmware_read()
 */
static void handle_file_read_response(CanardInstance* ins, CanardRxTransfer* transfer)
{
    if ((transfer->transfer_id+1)%32 != fwupdate.transfer_id ||
	transfer->source_node_id != fwupdate.node_id) {
	/* not for us */
	printf("Firmware update: not for us id=%u/%u\n", (unsigned)transfer->transfer_id, (unsigned)fwupdate.transfer_id);
	return;
    }
    struct uavcan_protocol_file_ReadResponse pkt;
    if (uavcan_protocol_file_ReadResponse_decode(transfer, &pkt)) {
	/* bad packet */
	printf("Firmware update: bad packet\n");
	return;
    }
    if (pkt.error.value != UAVCAN_PROTOCOL_FILE_ERROR_OK) {
	/* read failed */
	fwupdate.node_id = 0;
	printf("Firmware update read failure\n");
	return;
    }
    write(fwupdate.fd, pkt.data.data, pkt.data.len);
    if (pkt.data.len < 256) {
	/* firmware updare done */
	close(fwupdate.fd);
	printf("Firmwate update complete\n");
	fwupdate.node_id = 0;
	return;
    }
    fwupdate.offset += pkt.data.len;

    /* trigger a new read */
    fwupdate.last_read_ms = 0;
}

/*
 This callback is invoked by the library when a new message or request or response is received.
*/
static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    // switch on data type ID to pass to the right handler function
    if (transfer->transfer_type == CanardTransferTypeRequest) {
        // check if we want to handle a specific service request
        switch (transfer->data_type_id) {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
            handle_GetNodeInfo(ins, transfer);
            break;
        }
        case UAVCAN_PROTOCOL_PARAM_GETSET_ID: {
            handle_param_GetSet(ins, transfer);
            break;
        }
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
            handle_param_ExecuteOpcode(ins, transfer);
            break;
        }
	case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID: {
	    handle_begin_firmware_update(ins, transfer);
	    break;
	}
	}
    }
    if (transfer->transfer_type == CanardTransferTypeResponse) {
	switch (transfer->data_type_id) {
	case UAVCAN_PROTOCOL_FILE_READ_ID:
	    handle_file_read_response(ins, transfer);
	    break;
        }
    }
    if (transfer->transfer_type == CanardTransferTypeBroadcast) {
        // check if we want to handle a specific broadcast message
        switch (transfer->data_type_id) {
        case UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID: {
            handle_ArrayCommand(ins, transfer);
            break;
        }
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID: {
            handle_DNA_Allocation(ins, transfer);
            break;
        }
        }
    }
}


/*
 This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
 by the local node.
 If the callback returns true, the library will receive the transfer.
 If the callback returns false, the library will ignore the transfer.
 All transfers that are addressed to other nodes are always ignored.

 This function must fill in the out_data_type_signature to be the signature of the message.
 */
static bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    if (transfer_type == CanardTransferTypeRequest) {
        // check if we want to handle a specific service request
        switch (data_type_id) {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_PARAM_GETSET_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
            return true;
        }
	case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID: {
	    *out_data_type_signature = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE;
	    return true;
	}
        }
    }
    if (transfer_type == CanardTransferTypeResponse) {
        // check if we want to handle a specific service request
        switch (data_type_id) {
	case UAVCAN_PROTOCOL_FILE_READ_ID:
	    *out_data_type_signature = UAVCAN_PROTOCOL_FILE_READ_SIGNATURE;
	    return true;
	}
    }
    if (transfer_type == CanardTransferTypeBroadcast) {
        // see if we want to handle a specific broadcast packet
        switch (data_type_id) {
        case UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID: {
            *out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
            return true;
        }
        }
    }
    // we don't want any other messages
    return false;
}

/*
  send the 1Hz NodeStatus message. This is what allows a node to show
  up in the DroneCAN GUI tool and in the flight controller logs
 */
static void send_NodeStatus(void)
{
    uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];

    node_status.uptime_sec = micros64() / 1000000ULL;
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status.sub_mode = 0;
    // put whatever you like in here for display in GUI
    node_status.vendor_specific_status_code = 1234;

    /*
      when doing a firmware update put the size in kbytes in VSSC so
      the user can see how far it has reached
     */
    if (fwupdate.node_id != 0) {
	node_status.vendor_specific_status_code = fwupdate.offset / 1024;
	node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE;
    }

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer);

    // we need a static variable for the transfer ID. This is
    // incremeneted on each transfer, allowing for detection of packet
    // loss
    static uint8_t transfer_id;

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

/*
  This function is called at 1 Hz rate from the main loop.
*/
static void process1HzTasks(uint64_t timestamp_usec)
{
    /*
      Purge transfers that are no longer transmitted. This can free up some memory
    */
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    /*
      Transmit the node status message
    */
    send_NodeStatus();
}

/*
  send servo status at 25Hz
*/
static void send_ServoStatus(void)
{
    // send a separate status packet for each servo
    for (uint8_t i=0; i<NUM_SERVOS; i++) {
        struct uavcan_equipment_actuator_Status pkt;
        memset(&pkt, 0, sizeof(pkt));
        uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE];

        // make up some synthetic status data
        pkt.actuator_id = i;
        pkt.position = servos[i].position;
        pkt.force = 3.5 * servos[i].position;
        pkt.speed = 0.12; // m/s or rad/s
        pkt.power_rating_pct = 17;

        uint32_t len = uavcan_equipment_actuator_Status_encode(&pkt, buffer);

        // we need a static variable for the transfer ID. This is
        // incremeneted on each transfer, allowing for detection of packet
        // loss
        static uint8_t transfer_id;

        canardBroadcast(&canard,
                        UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
                        UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        buffer,
                        len);
    }
}


/*
  Transmits all frames from the TX queue, receives up to one frame.
*/
static void processTxRxOnce(LinuxCANInstance *can, int32_t timeout_msec)
{
    // Transmitting
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        const int16_t tx_res = LinuxCANTransmit(can, txf, 0);
        if (tx_res < 0) {         // Failure - drop the frame
            canardPopTxQueue(&canard);
        }
        else if (tx_res > 0)    // Success - just drop the frame
        {
            canardPopTxQueue(&canard);
        }
        else                    // Timeout - just exit and try again later
        {
            break;
        }
    }

    // Receiving
    CanardCANFrame rx_frame;

    const uint64_t timestamp = micros64();
    const int16_t rx_res = LinuxCANReceive(can, &rx_frame, timeout_msec);
    if (rx_res < 0) {
        (void)fprintf(stderr, "Receive error %d, errno '%s'\n", rx_res, strerror(errno));
    }
    else if (rx_res > 0)        // Success - process the frame
    {
        canardHandleRxFrame(&canard, &rx_frame, timestamp);
    }
}

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

    fwupdate.fd = -1;

    /*
     * Initializing the CAN backend driver
     */
    LinuxCANInstance can;
    const char* const can_iface_name = argv[1];
    int16_t res = LinuxCANInit(&can, can_iface_name);
    if (res < 0) {
        (void)fprintf(stderr, "Failed to open CAN iface '%s'\n", can_iface_name);
        return 1;
    }

    /*
     Initializing the Libcanard instance.
     */
    canardInit(&canard,
               memory_pool,
               sizeof(memory_pool),
               onTransferReceived,
               shouldAcceptTransfer,
               NULL);

    if (MY_NODE_ID > 0) {
        canardSetLocalNodeID(&canard, MY_NODE_ID);
    } else {
        printf("Waiting for DNA node allocation\n");
    }

    /*
      Run the main loop.
     */
    uint64_t next_1hz_service_at = micros64();
    uint64_t next_25hz_service_at = micros64();

    while (true) {
	processTxRxOnce(&can, 10);

        const uint64_t ts = micros64();

        if (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID) {
            // waiting for DNA
        }

        // see if we are still doing DNA
        if (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID) {
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
        if (ts >= next_25hz_service_at) {
            next_25hz_service_at += 1000000ULL/25U;
            send_ServoStatus();
        }

	if (fwupdate.node_id != 0) {
	    send_firmware_read();
	}
    }

    return 0;
}
