#include "AP_DroneCAN.h"

#if HAL_ENABLE_DRONECAN_DRIVERS

#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Filesystem/AP_Filesystem.h>

AP_DroneCAN_Filesystem::AP_DroneCAN_Filesystem(CanardInterface &canard_iface) :
    get_info_client(canard_iface, get_info_cb),
    get_directory_entry_info_client(canard_iface, get_directory_entry_info_cb),
    get_info_server(canard_iface, get_info_req_cb),
    get_directory_entry_info_server(canard_iface, get_directory_entry_info_req_cb)
{}

/*
    Client
*/

// Send request for info
void AP_DroneCAN_Filesystem::request_info(const char* path, const uint8_t node_id)
{
    uavcan_protocol_file_GetInfoRequest request {};
    request.path.path.len = strncpy_noterm((char*)request.path.path.data, path, sizeof(request.path.path.data)-1);

    get_info_client.request(node_id, request);

    get_info_response.valid = false;
}

// Receive response to request
void AP_DroneCAN_Filesystem::handle_get_info_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_GetInfoResponse &msg)
{
    get_info_response.valid = true;
    get_info_response.msg = msg;
}

// Return response
bool AP_DroneCAN_Filesystem::request_info_response(uavcan_protocol_file_GetInfoResponse &msg)
{
    if (!get_info_response.valid) {
        return false;
    }
    msg = get_info_response.msg;
    return true;
}

// Send request for directory entry info
void AP_DroneCAN_Filesystem::request_directory_entry_info(const char* path, const uint8_t node_id, const uint32_t entry_index)
{
    uavcan_protocol_file_GetDirectoryEntryInfoRequest request {};
    request.directory_path.path.len = strncpy_noterm((char*)request.directory_path.path.data, path, sizeof(request.directory_path.path.data)-1);
    request.entry_index = entry_index;

    get_directory_entry_info_client.request(node_id, request);

    get_directory_entry_info_response.valid = false;
}

// Receive response to request
void AP_DroneCAN_Filesystem::handle_get_directory_entry_info_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_GetDirectoryEntryInfoResponse &msg)
{
    get_directory_entry_info_response.valid = true;
    get_directory_entry_info_response.msg = msg;
}

// Return response
bool AP_DroneCAN_Filesystem::request_get_directory_entry_info_response(uavcan_protocol_file_GetDirectoryEntryInfoResponse &msg)
{
    if (!get_directory_entry_info_response.valid) {
        return false;
    }
    msg = get_directory_entry_info_response.msg;
    return true;
}

/*
    Server
*/

// Get info
void AP_DroneCAN_Filesystem::handle_get_info_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_GetInfoRequest &msg)
{
    uavcan_protocol_file_GetInfoResponse response {};

    struct stat st;
    if (AP::FS().stat((char*)msg.path.path.data, &st) != 0) {
        // Failed
        response.error.value = errno;

    } else {
        response.size = st.st_size;

        // Translate flags
        if (st.st_mode & S_IFREG) {
            response.entry_type.flags |= UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_FILE;
        }
        if (st.st_mode & S_IFDIR) {
            response.entry_type.flags |= UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_DIRECTORY;
        }
        if (st.st_mode & S_IFLNK) {
            response.entry_type.flags |= UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_SYMLINK;
        }
        if (st.st_mode & S_IREAD) {
            response.entry_type.flags |= UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_READABLE;
        }
        if (st.st_mode & S_IWRITE) {
            response.entry_type.flags |= UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_WRITEABLE;
        }
    }

    // Send response
    get_info_server.respond(transfer, response);
}

// Get Directory Entry Info
void AP_DroneCAN_Filesystem::handle_get_directory_entry_info_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_GetDirectoryEntryInfoRequest &msg)
{
    uavcan_protocol_file_GetDirectoryEntryInfoResponse response {};

    // Open
    auto dir = AP::FS().opendir((char*)msg.directory_path.path.data);
    if (dir == nullptr) {
        response.error.value = errno;
        get_directory_entry_info_server.respond(transfer, response);
        return;
    }

    // Increment to desired offset
    struct dirent *entry;
    for (uint32_t i = 0; i <= msg.entry_index; i++ ) {
        entry = AP::FS().readdir(dir);
        if (entry == nullptr) {
            break;
        }
    }

    // Close directory
    AP::FS().closedir(dir);

    // See if we got anything
    if (entry == nullptr) {
        response.error.value = errno;
        get_directory_entry_info_server.respond(transfer, response);
        return;
    }

    // Found entry, fill in details
    response.entry_full_path.path.len = strncpy_noterm((char*)response.entry_full_path.path.data, entry->d_name, sizeof(response.entry_full_path.path.data)-1);

    if (entry->d_type == DT_REG) {
        response.entry_type.flags |= UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_FILE;
    } else if (entry->d_type == DT_DIR) {
        response.entry_type.flags |= UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_DIRECTORY;
    } else if (entry->d_type == DT_LNK) {
        response.entry_type.flags |= UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_SYMLINK;
    }

    get_directory_entry_info_server.respond(transfer, response);

}

#endif // HAL_ENABLE_DRONECAN_DRIVERS
