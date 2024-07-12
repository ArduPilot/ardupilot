#include "AP_DroneCAN.h"

#if HAL_ENABLE_DRONECAN_DRIVERS

#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Filesystem/AP_Filesystem.h>

#if AP_FILESYSTEM_DRONECAN_ENABLED
AP_DroneCAN_Filesystem_Client::AP_DroneCAN_Filesystem_Client(CanardInterface &canard_iface) :
    get_info_client(canard_iface, get_info_cb),
    get_directory_entry_info_client(canard_iface, get_directory_entry_info_cb),
    delete_client(canard_iface, delete_cb),
    read_client(canard_iface, read_cb),
    write_client(canard_iface, write_cb)
{}

// Send request for info
void AP_DroneCAN_Filesystem_Client::request_info(const char* path, const uint8_t node_id)
{
    WITH_SEMAPHORE(get_info_response.sem);

    uavcan_protocol_file_GetInfoRequest request {};
    request.path.path.len = strncpy_noterm((char*)request.path.path.data, path, sizeof(request.path.path.data)-1);

    get_info_client.request(node_id, request);
    get_info_response.valid = false;
}

// Receive response to request
void AP_DroneCAN_Filesystem_Client::handle_get_info_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_GetInfoResponse &msg)
{
    WITH_SEMAPHORE(get_info_response.sem);
    get_info_response.valid = true;
    get_info_response.msg = msg;
}

// Return response
bool AP_DroneCAN_Filesystem_Client::request_info_response(uavcan_protocol_file_GetInfoResponse &msg)
{
    WITH_SEMAPHORE(get_info_response.sem);
    if (!get_info_response.valid) {
        return false;
    }
    msg = get_info_response.msg;
    return true;
}

// Send request for directory entry info
void AP_DroneCAN_Filesystem_Client::request_directory_entry_info(const char* path, const uint8_t node_id, const uint32_t entry_index)
{
    WITH_SEMAPHORE(get_directory_entry_info_response.sem);

    uavcan_protocol_file_GetDirectoryEntryInfoRequest request {};
    request.directory_path.path.len = strncpy_noterm((char*)request.directory_path.path.data, path, sizeof(request.directory_path.path.data)-1);
    request.entry_index = entry_index;

    get_directory_entry_info_client.request(node_id, request);
    get_directory_entry_info_response.valid = false;
}

// Receive response to request
void AP_DroneCAN_Filesystem_Client::handle_get_directory_entry_info_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_GetDirectoryEntryInfoResponse &msg)
{
    WITH_SEMAPHORE(get_directory_entry_info_response.sem);
    get_directory_entry_info_response.valid = true;
    get_directory_entry_info_response.msg = msg;
}

// Return response
bool AP_DroneCAN_Filesystem_Client::request_get_directory_entry_info_response(uavcan_protocol_file_GetDirectoryEntryInfoResponse &msg)
{
    WITH_SEMAPHORE(get_directory_entry_info_response.sem);
    if (!get_directory_entry_info_response.valid) {
        return false;
    }
    msg = get_directory_entry_info_response.msg;
    return true;
}

// Send request to delete
void AP_DroneCAN_Filesystem_Client::request_delete(const char* path, const uint8_t node_id)
{
    WITH_SEMAPHORE(delete_response.sem);

    uavcan_protocol_file_DeleteRequest request {};
    request.path.path.len = strncpy_noterm((char*)request.path.path.data, path, sizeof(request.path.path.data)-1);

    delete_client.request(node_id, request);
    delete_response.valid = false;
}

// Receive response to request
void AP_DroneCAN_Filesystem_Client::handle_delete_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_DeleteResponse &msg)
{
    WITH_SEMAPHORE(delete_response.sem);
    delete_response.valid = true;
    delete_response.msg = msg;
}

// Return response
bool AP_DroneCAN_Filesystem_Client::request_delete_response(uavcan_protocol_file_DeleteResponse &msg)
{
    WITH_SEMAPHORE(delete_response.sem);
    if (!delete_response.valid) {
        return false;
    }
    msg = delete_response.msg;
    return true;
}

// Send request to Read
void AP_DroneCAN_Filesystem_Client::request_read(const char* path, const uint8_t node_id, const uint32_t offset)
{
    WITH_SEMAPHORE(read_response.sem);

    uavcan_protocol_file_ReadRequest request {};
    request.path.path.len = strncpy_noterm((char*)request.path.path.data, path, sizeof(request.path.path.data)-1);
    request.offset = offset;

    read_client.request(node_id, request);
    read_response.valid = false;
}

// Receive response to request
void AP_DroneCAN_Filesystem_Client::handle_read_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_ReadResponse &msg)
{
    WITH_SEMAPHORE(read_response.sem);
    read_response.valid = true;
    read_response.msg = msg;
}

// Return response
bool AP_DroneCAN_Filesystem_Client::request_read_response(uavcan_protocol_file_ReadResponse &msg)
{
    WITH_SEMAPHORE(read_response.sem);
    if (!read_response.valid) {
        return false;
    }
    msg = read_response.msg;
    return true;
}

// Send request to Write
uint32_t AP_DroneCAN_Filesystem_Client::request_write(const char* path, const uint8_t node_id, const uint32_t offset, const void *buf, uint32_t count)
{
    WITH_SEMAPHORE(write_response.sem);

    uavcan_protocol_file_WriteRequest request {};
    request.path.path.len = strncpy_noterm((char*)request.path.path.data, path, sizeof(request.path.path.data)-1);
    request.offset = offset;

    request.data.len = MIN(count, sizeof(request.data.data));
    memcpy(request.data.data, buf, request.data.len);

    write_client.request(node_id, request);
    write_response.valid = false;

    return request.data.len;
}

// Receive response to request
void AP_DroneCAN_Filesystem_Client::handle_write_response(const CanardRxTransfer& transfer, const uavcan_protocol_file_WriteResponse &msg)
{
    WITH_SEMAPHORE(write_response.sem);
    write_response.valid = true;
    write_response.msg = msg;
}

// Return response
bool AP_DroneCAN_Filesystem_Client::request_write_response(uavcan_protocol_file_WriteResponse &msg)
{
    WITH_SEMAPHORE(write_response.sem);
    if (!write_response.valid) {
        return false;
    }
    msg = write_response.msg;
    return true;
}
#endif // AP_FILESYSTEM_DRONECAN_ENABLED

AP_DroneCAN_Filesystem_Server::AP_DroneCAN_Filesystem_Server(CanardInterface &canard_iface) :
    get_info_server(canard_iface, get_info_req_cb),
    get_directory_entry_info_server(canard_iface, get_directory_entry_info_req_cb),
    delete_server(canard_iface, delete_req_cb),
    read_server(canard_iface, read_req_cb),
    write_server(canard_iface, write_req_cb)
{}

// Get info
void AP_DroneCAN_Filesystem_Server::handle_get_info_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_GetInfoRequest &msg)
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
void AP_DroneCAN_Filesystem_Server::handle_get_directory_entry_info_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_GetDirectoryEntryInfoRequest &msg)
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
    struct dirent *entry = nullptr;
    for (uint32_t i = 0; i <= msg.entry_index; i++ ) {
        entry = AP::FS().readdir(dir);
        if (entry == nullptr) {
            break;
        }
    }

    if (entry == nullptr) {
        // No entry, fill in error
        response.error.value = errno;

    } else {
        // Found entry, fill in details
        response.entry_full_path.path.len = strncpy_noterm((char*)response.entry_full_path.path.data, entry->d_name, sizeof(response.entry_full_path.path.data)-1);

        if (entry->d_type == DT_REG) {
            response.entry_type.flags |= UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_FILE;
        } else if (entry->d_type == DT_DIR) {
            response.entry_type.flags |= UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_DIRECTORY;
        } else if (entry->d_type == DT_LNK) {
            response.entry_type.flags |= UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_SYMLINK;
        }
    }

    // Close directory
    AP::FS().closedir(dir);

    get_directory_entry_info_server.respond(transfer, response);
}

// Delete file
void AP_DroneCAN_Filesystem_Server::handle_delete_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_DeleteRequest &msg)
{
    uavcan_protocol_file_DeleteResponse response {};

    if (AP::FS().unlink((char*)msg.path.path.data) != 0) {
        response.error.value = errno;
        if (response.error.value == 0) {
            // Make sure error is none zero if unlink failed
            response.error.value = ENOENT;
        }
    }

    delete_server.respond(transfer, response);
}

// Read file
void AP_DroneCAN_Filesystem_Server::handle_read_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_ReadRequest &msg)
{
    uavcan_protocol_file_ReadResponse response {};

    const int file = AP::FS().open((char*)msg.path.path.data, O_RDONLY);
    if ((file == -1) || (AP::FS().lseek(file, msg.offset, SEEK_SET) == -1)) {
        response.error.value = errno;
        if (response.error.value == 0) {
            // Make sure error is none zero if open or seek fail
            response.error.value = ENOENT;
        }
        read_server.respond(transfer, response);
        AP::FS().close(file);
        return;
    }

    // Try reading
    const ssize_t read_bytes = AP::FS().read(file, response.data.data, sizeof(response.data.data));

    // Close file
    AP::FS().close(file);

    // If read failed set error, else set read length
    if (read_bytes < 0) {
        response.error.value = errno;
        if (response.error.value == 0) {
            // Make sure error is none zero if read fails
            response.error.value = ENOENT;
        }
    } else {
        response.data.len = read_bytes;
    }

    read_server.respond(transfer, response);
}

// Write file
void AP_DroneCAN_Filesystem_Server::handle_write_request(const CanardRxTransfer& transfer, const uavcan_protocol_file_WriteRequest &msg)
{
    uavcan_protocol_file_WriteResponse response {};

    int flags = O_WRONLY;
    if (msg.offset == 0) {
        // If file does not exist and offset is 0 then create it
        struct stat st;
        if (AP::FS().stat((char*)msg.path.path.data, &st) != 0) {
            flags |= O_CREAT;
        }
    }

    const int file = AP::FS().open((char*)msg.path.path.data, flags);
    if ((file == -1) || (AP::FS().lseek(file, msg.offset, SEEK_SET) == -1)) {
        response.error.value = errno;
        if (response.error.value == 0) {
            // Make sure error is none zero if open or seek fail
            response.error.value = ENOENT;
        }
        write_server.respond(transfer, response);
        AP::FS().close(file);
        return;
    }

    // Try writing
    const ssize_t write_bytes = AP::FS().write(file, msg.data.data, msg.data.len);

    // Close file
    AP::FS().close(file);

    // If did not write full length then set error
    if (write_bytes != msg.data.len) {
        response.error.value = errno;
        if (response.error.value == 0) {
            // Make sure error is none zero if write fails
            response.error.value = EIO;
        }
    }

    write_server.respond(transfer, response);
}

#endif // HAL_ENABLE_DRONECAN_DRIVERS
