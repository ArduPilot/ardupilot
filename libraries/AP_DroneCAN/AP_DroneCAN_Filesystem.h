#pragma once

#include <uavcan.protocol.file.GetInfo.h>
#include <uavcan.protocol.file.GetDirectoryEntryInfo.h>
#include <uavcan.protocol.file.Delete.h>
#include <uavcan.protocol.file.Read.h>
#include <uavcan.protocol.file.Write.h>

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

    // Delete request and response
    void request_delete(const char* path, const uint8_t node_id);
    bool request_delete_response(uavcan_protocol_file_DeleteResponse &msg);

    // Read request and response
    void request_read(const char* path, const uint8_t node_id, const uint32_t offset);
    bool request_read_response(uavcan_protocol_file_ReadResponse &msg);

    // Write request and response
    uint32_t request_write(const char* path, const uint8_t node_id, const uint32_t offset, const void *buf, uint32_t count);
    bool request_write_response(uavcan_protocol_file_WriteResponse &msg);

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

    // Delete request and response
    Canard::Client<uavcan_protocol_file_DeleteResponse> delete_client;
    Canard::ObjCallback<AP_DroneCAN_Filesystem, uavcan_protocol_file_DeleteResponse> delete_cb{this, &AP_DroneCAN_Filesystem::handle_delete_response};
    void handle_delete_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_DeleteResponse &msg);
    struct {
        uavcan_protocol_file_DeleteResponse msg;
        bool valid;
    } delete_response;

    // Read request and response
    Canard::Client<uavcan_protocol_file_ReadResponse> read_client;
    Canard::ObjCallback<AP_DroneCAN_Filesystem, uavcan_protocol_file_ReadResponse> read_cb{this, &AP_DroneCAN_Filesystem::handle_read_response};
    void handle_read_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_ReadResponse &msg);
    struct {
        uavcan_protocol_file_ReadResponse msg;
        bool valid;
    } read_response;

    // Write request and response
    Canard::Client<uavcan_protocol_file_WriteResponse> write_client;
    Canard::ObjCallback<AP_DroneCAN_Filesystem, uavcan_protocol_file_WriteResponse> write_cb{this, &AP_DroneCAN_Filesystem::handle_write_response};
    void handle_write_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_WriteResponse &msg);
    struct {
        uavcan_protocol_file_WriteResponse msg;
        bool valid;
    } write_response;

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

    // Delete
    Canard::Server<uavcan_protocol_file_DeleteRequest> delete_server;
    Canard::ObjCallback<AP_DroneCAN_Filesystem, uavcan_protocol_file_DeleteRequest> delete_req_cb{this, &AP_DroneCAN_Filesystem::handle_delete_request};
    void handle_delete_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_DeleteRequest &msg);

    // Read
    Canard::Server<uavcan_protocol_file_ReadRequest> read_server;
    Canard::ObjCallback<AP_DroneCAN_Filesystem, uavcan_protocol_file_ReadRequest> read_req_cb{this, &AP_DroneCAN_Filesystem::handle_read_request};
    void handle_read_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_ReadRequest &msg);

    // Write
    Canard::Server<uavcan_protocol_file_WriteRequest> write_server;
    Canard::ObjCallback<AP_DroneCAN_Filesystem, uavcan_protocol_file_WriteRequest> write_req_cb{this, &AP_DroneCAN_Filesystem::handle_write_request};
    void handle_write_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_WriteRequest &msg);
};
