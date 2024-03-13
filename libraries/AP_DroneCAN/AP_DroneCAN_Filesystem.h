#pragma once

#include <uavcan.protocol.file.GetInfo.h>
#include <uavcan.protocol.file.GetDirectoryEntryInfo.h>

class AP_DroneCAN;

class AP_DroneCAN_Filesystem
{
public:
    AP_DroneCAN_Filesystem(CanardInterface &canard_iface);
    CLASS_NO_COPY(AP_DroneCAN_Filesystem);

    /*
        Client
    */

    // Get info request and response
    void request_info(const char* path, const uint8_t node_id);
    bool request_info_response(uavcan_protocol_file_GetInfoResponse &msg);

    // Get Directory Entry Info request and response
    void request_directory_entry_info(const char* path, const uint8_t node_id, const uint32_t entry_index);
    bool request_get_directory_entry_info_response(uavcan_protocol_file_GetDirectoryEntryInfoResponse &msg);

private:

    /*
        Client
    */

    // Get info request and response
    Canard::Client<uavcan_protocol_file_GetInfoResponse> get_info_client;
    Canard::ObjCallback<AP_DroneCAN_Filesystem, uavcan_protocol_file_GetInfoResponse> get_info_cb{this, &AP_DroneCAN_Filesystem::handle_get_info_response};
    void handle_get_info_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_GetInfoResponse &msg);
    struct {
        uavcan_protocol_file_GetInfoResponse msg;
        bool valid;
    } get_info_response;

    // Get Directory Entry Info request and response
    Canard::Client<uavcan_protocol_file_GetDirectoryEntryInfoResponse> get_directory_entry_info_client;
    Canard::ObjCallback<AP_DroneCAN_Filesystem, uavcan_protocol_file_GetDirectoryEntryInfoResponse> get_directory_entry_info_cb{this, &AP_DroneCAN_Filesystem::handle_get_directory_entry_info_response};
    void handle_get_directory_entry_info_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_GetDirectoryEntryInfoResponse &msg);
    struct {
        uavcan_protocol_file_GetDirectoryEntryInfoResponse msg;
        bool valid;
    } get_directory_entry_info_response;

    /*
        Server
    */

    // Get info
    Canard::Server<uavcan_protocol_file_GetInfoRequest> get_info_server;
    Canard::ObjCallback<AP_DroneCAN_Filesystem, uavcan_protocol_file_GetInfoRequest> get_info_req_cb{this, &AP_DroneCAN_Filesystem::handle_get_info_request};
    void handle_get_info_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_GetInfoRequest &msg);

    // Get Directory Entry Info 
    Canard::Server<uavcan_protocol_file_GetDirectoryEntryInfoRequest> get_directory_entry_info_server;
    Canard::ObjCallback<AP_DroneCAN_Filesystem, uavcan_protocol_file_GetDirectoryEntryInfoRequest> get_directory_entry_info_req_cb{this, &AP_DroneCAN_Filesystem::handle_get_directory_entry_info_request};
    void handle_get_directory_entry_info_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_GetDirectoryEntryInfoRequest &msg);

};
