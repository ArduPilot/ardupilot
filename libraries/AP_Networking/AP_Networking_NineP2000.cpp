#include "AP_Networking_Config.h"

#if AP_NETWORKING_FILESYSTEM_ENABLED

#include "AP_Networking.h"
#include <AP_HAL/utility/Socket.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Math/AP_Math.h>
#include <errno.h>
#include <AP_Filesystem/AP_Filesystem.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Networking::NineP2000::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable 9P2000 client
    // @Description: 9P2000 client allows file access to enteral server over a TCP connection.
    // @Values: 0:Disabled, 1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_Networking::NineP2000, enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Group: IP
    // @Path: AP_Networking_address.cpp
    // @RebootRequired: True
    AP_SUBGROUPINFO(ip, "IP", 2, AP_Networking::NineP2000, AP_Networking_IPV4),

    // @Param: PORT
    // @DisplayName: Port number
    // @Description: Port number
    // @Range: 0 65535
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("PORT", 3, AP_Networking::NineP2000, port, 0),

    AP_GROUPEND
};

/*
  initialise mapped network ports
 */
void AP_Networking::NineP2000::init()
{
    sock = NEW_NOTHROW SocketAPM(false);
    if (sock != nullptr) {
        sock->set_blocking(true);

        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&NineP2000::loop, void), "9P2000", 1024, AP_HAL::Scheduler::PRIORITY_STORAGE, 0)) {
            AP_BoardConfig::allocation_error("9P2000 thread");
        }
    }

}

void AP_Networking::NineP2000::loop()
{
    AP::network().startup_wait();

    bool active = false;
    while (true) {
        if (!active) {
            hal.scheduler->delay_microseconds(100);
        }
        if (sock == nullptr) {
            sock = NEW_NOTHROW SocketAPM(false);
            if (sock == nullptr) {
                continue;
            }
            sock->set_blocking(true);
            connected = false;
        }
        if (!connected) {
            const char *dest = ip.get_str();
            connected = sock->connect(dest, port.get());
            if (!connected) {
                delete sock;
                sock = nullptr;
                // don't try and connect too fast
                hal.scheduler->delay(100);
                continue;
            }
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "9P2000: connected to %s:%u", dest, unsigned(port.get()));
            sock->set_blocking(false);

            // Clear message and file tracking
            memset(&receive, 0, sizeof(receive));
            memset(&fileIds, 0, sizeof(fileIds));

            // Restart connection process
            request_version();

        }
        active = update();
    }
}

// Return true if connected and mounted
bool AP_Networking::NineP2000::mounted()
{
    return connected && (state == State::Mounted);
}

// Deal with incoming data
bool AP_Networking::NineP2000::update()
{
    const auto len = sock->recv(receive.buffer, sizeof(receive.buffer), 0);
    if (len == 0) {
        // Zero means disconnected
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "9P2000: closed connection");
        delete sock;
        sock = nullptr;
        return false;
    }
    if (len < 0) {
        // Could return -1 if there is no pending data
        return false;
    }

    parse(len);
    return true;
}

void AP_Networking::NineP2000::parse(const uint32_t len)
{
    // Need at least the length the of the massage
    if (len < receive.content.header.length) {
        return;
    }

    // Use semaphore for thread safety
    WITH_SEMAPHORE(request_sem);

    // Deal with each message type
    const Type msg_type = (Type)receive.content.header.type;
    switch (msg_type) {
        case Type::Rversion: {
            if (state != State::Version) {
                // Should only get a version response if we asked for one
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            }
            handle_version();
            break;
        }

        case Type::Rattach: {
            if (state != State::Attach) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            }
            handle_attach();
            break;
        }

        // Auth and flush are not supported
        case Type::Rauth:
        case Type::Rflush:
            break;

        // Clear tag from clunk
        case Type::Rclunk: {
            // Note that there is no timeout, so we could leak a tag and a file id
            // Clear the tag and file ID so they can be used again
            const uint16_t tag = receive.content.header.tag;

            // Check tag is valid
            if ((tag >= ARRAY_SIZE(request)) || !request[tag].pending) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return;
            }

            // Check that the type matches what is expected
            if (request[tag].expectedType != Type::Rclunk) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return;
            }

            clear_file_id(request[tag].fileId);
            clear_tag(tag);
            break;
        }

        // Stash result for callback
        case Type::Rerror:
        case Type::Rwalk:
        case Type::Ropen:
        case Type::Rcreate:
        case Type::Rread:
        case Type::Rwrite:
        case Type::Rremove:
        case Type::Rstat:
        case Type::Rwstat: {
            // Should be mounted before responses start turning up
            if (state != State::Mounted) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return;
            }

            // Check tag is valid
            const uint16_t tag = receive.content.header.tag;
            if ((tag >= ARRAY_SIZE(request)) || !request[tag].pending) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return;
            }

            // Check that the type matches what is expected (unexpected errors are allowed)
            if ((msg_type != Type::Rerror) && (request[tag].expectedType != msg_type)) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return;
            }

            // Valid tag, fill in message
            memcpy(&request[tag].result.buffer, &receive.buffer, sizeof(request[tag].result.buffer));

            // No longer pending
            request[tag].pending = false;
            break;
        }

        // Not expecting to receive any requests
        case Type::Tversion:
        case Type::Tauth:
        case Type::Tattach:
        case Type::Tflush:
        case Type::Twalk:
        case Type::Topen:
        case Type::Tcreate:
        case Type::Tread:
        case Type::Twrite:
        case Type::Tclunk:
        case Type::Tremove:
        case Type::Tstat:
        case Type::Twstat:
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
    }

    // Parsed full length of message
    if (receive.content.header.length >= len) {
        return;
    }

    // Try parsing the remainder
    const uint16_t remaining = len - receive.content.header.length;
    memmove(&receive.buffer[0], &receive.buffer[receive.content.header.length], remaining);

    parse(remaining);
}

// Add a string to the end of a message
void AP_Networking::NineP2000::add_string(Message &msg, const char *str) const
{
    const size_t offset = msg.content.header.length;

    const uint16_t len = strlen(str);

    if ((offset + sizeof(len) + len) > MIN(bufferLen, sizeof(Message))) {
        // This would be a huge file name!
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }

    // Add string length and string content;
    memcpy(&msg.buffer[offset], &len, sizeof(len));
    strncpy_noterm((char*)&msg.buffer[offset + sizeof(len)], str, len);

    msg.content.header.length += sizeof(len) + len;
}

// Request version and message size
void AP_Networking::NineP2000::request_version()
{
    state = State::Version;
    bufferLen = 32; // Assume a minimum message length

    send.content.header.type = (uint8_t)Type::Tversion;
    send.content.header.tag = NOTAG;
    send.content.header.length = sizeof(send.content.header);

    const uint32_t msize = sizeof(Message);
    memcpy(&send.content.payload, &msize, sizeof(msize));
    send.content.header.length += sizeof(msize);

    add_string(send, "9P2000");

    sock->send(send.buffer, send.content.header.length);
}

// handle version response
void AP_Networking::NineP2000::handle_version()
{
    // Should have header, length and a string
    if (receive.content.header.length < sizeof(receive.content.header) + sizeof(uint32_t) + sizeof(uint16_t)) {
        return;
    }

    if (receive.content.header.tag != NOTAG) {
        // tag should always be NOTAG, throw a error?
        return;
    }

    // Get message length
    uint32_t msize;
    memcpy(&msize, &send.content.payload, sizeof(msize));

    // Message length should be equal to or less than the value requested
    if (msize > sizeof(Message)) {
        return;
    }

    // Get and check string length
    const size_t expected_string_len = receive.content.header.length - sizeof(receive.content.header) - sizeof(uint32_t) - sizeof(uint16_t);
    uint16_t len;
    memcpy(&len, &send.content.payload[sizeof(msize)], sizeof(len));
    if (len != expected_string_len) {
        return;
    }

    // String should match what was requested
    if (strncmp("9P2000", (char*)&send.content.payload[sizeof(msize)+sizeof(len)], len) != 0) {
        return;
    }

    // Limit to the agreed message length
    bufferLen = msize;

    // Try and attach
    request_attach();
}

// Request attach
void AP_Networking::NineP2000::request_attach()
{
    state = State::Attach;

    send.content.header.type = (uint8_t)Type::Tattach;
    send.content.header.tag = ARRAY_SIZE(request); // Use a tag that will not be used in normal operation
    send.content.header.length = sizeof(send.content.header);

    // Use zero file id
    const uint32_t fid = 0;
    memcpy(&send.buffer[send.content.header.length], &fid, sizeof(fid));
    send.content.header.length += sizeof(fid);

    // No auth setup
    const uint32_t afid = 0;
    memcpy(&send.buffer[send.content.header.length], &afid, sizeof(afid));
    send.content.header.length += sizeof(afid);

    // User name ArduPilot, no aname.
    add_string(send, "ArduPilot");
    add_string(send, "");

    sock->send(send.buffer, send.content.header.length);
}

// Handle attach response
void AP_Networking::NineP2000::handle_attach()
{
    // Fixed length message, header and qid
    if (receive.content.header.length != sizeof(receive.content.header) + sizeof(qid_t)) {
        return;
    }

    // tag should match the request
    if (receive.content.header.tag != ARRAY_SIZE(request)) {
        return;
    }

    // Read in qid
    const qid_t &id = (qid_t&)receive.content.payload;

    // Expecting a directory
    if (id.type != qidType::QTDIR) {
        return;
    }

    state = State::Mounted;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "9P2000: mounted file system");
}

// Return the next available tag, NOTAG is none free
uint16_t AP_Networking::NineP2000::get_free_tag()
{
    WITH_SEMAPHORE(request_sem);

    // Must be mounted for operations to be valid
    if (state != State::Mounted) {
        return NOTAG;
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(request); i++) {
        if (!request[i].active) {
            request[i].active = true;
            return i;
        }
    }

    return NOTAG;
}

// Return true if there is a response for the given tag
bool AP_Networking::NineP2000::tag_response(const uint16_t tag)
{
    if (tag >= ARRAY_SIZE(request)) {
        return false;
    }

    WITH_SEMAPHORE(request_sem);
    return request[tag].active && !request[tag].pending;
}

// Called when a command is timed out
void AP_Networking::NineP2000::clear_tag(const uint16_t tag)
{
    if (tag >= ARRAY_SIZE(request)) {
        return;
    }

    WITH_SEMAPHORE(request_sem);
    memset(&request[tag], 0, sizeof(request[tag]));
}

// Generate a new unique file id
uint32_t AP_Networking::NineP2000::generate_unique_file_id()
{
    // Generate a random id and check it does not clash with existing IDs.
    // If it does we try again, this could recurse forever! (but its very unlikely)
    WITH_SEMAPHORE(request_sem);

    for (uint8_t i = 0; i < ARRAY_SIZE(fileIds); i++) {
        if (!fileIds[i].active) {
            fileIds[i].active = true;
            fileIds[i].clunked = false;
            return i + 1;
        }
    }
    return 0;
}

// Clear a file id now the file has been closed
void AP_Networking::NineP2000::clear_file_id(const uint32_t fileId)
{
    WITH_SEMAPHORE(request_sem);

    const uint32_t index = fileId - 1;

    if ((index >= ARRAY_SIZE(fileIds)) || !fileIds[index].active) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }

    fileIds[index].active = false;
}

// Check if a given ID active
bool AP_Networking::NineP2000::valid_file_id(const uint32_t fileId)
{
    const uint32_t index = fileId - 1;
    return (index < ARRAY_SIZE(fileIds)) && fileIds[index].active && !fileIds[index].clunked;
}

// Walk to a new file or directory, return tag, NOTAG if failed
uint16_t AP_Networking::NineP2000::request_walk(const char* path)
{
    WITH_SEMAPHORE(request_sem);

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        return NOTAG;
    }

    // Get a new file ID
    const uint32_t id = generate_unique_file_id();

    // Check if id is valid
    if (id == 0) {
        clear_tag(tag);
        return NOTAG;
    }

    // Fill in message
    send.content.header.type = (uint8_t)Type::Twalk;
    send.content.header.tag = tag;
    send.content.header.length = sizeof(send.content.header);

    // Start at root
    const uint32_t root = 0;
    memcpy(&send.buffer[send.content.header.length], &root, sizeof(root));
    send.content.header.length += sizeof(root);

    // Clone to new ID
    memcpy(&send.buffer[send.content.header.length], &id, sizeof(id));
    send.content.header.length += sizeof(id);

    // Take reference to name count
    uint16_t &num_names = (uint16_t&) send.buffer[send.content.header.length];
    num_names = 0;
    send.content.header.length += sizeof(num_names);

    // Step through path and add strings
    const uint16_t len = strlen(path);
    uint16_t name_start = 0; 
    for (uint16_t i = 0; i < len; i++) {
        const bool split = path[i] == '/';
        const bool last = i == (len - 1);
        if (split || last) {
            // Should check total length of message is still valid
            const uint16_t name_len = i - name_start + (split ? 0 : 1);
            const uint32_t new_len = send.content.header.length + sizeof(name_len) + name_len;

            // Check total message length is still valid
            if (new_len > bufferLen) {
                clear_tag(tag);
                return NOTAG;
            }

            // Add string length
            memcpy(&send.buffer[send.content.header.length], &name_len, sizeof(name_len));
            send.content.header.length += sizeof(name_len);

            // Add string itself
            memcpy(&send.buffer[send.content.header.length], &path[name_start], name_len);
            send.content.header.length += name_len;

            // Increment number of names
            num_names += 1;

            // Next section starts after this one
            name_start = i + 1;
        }
    }

    // Make tag as active
    request[tag].pending = true;
    request[tag].fileId = id;
    request[tag].expectedType = Type::Rwalk;

    // Send command
    sock->send(send.buffer, send.content.header.length);

    return tag;
}

void AP_Networking::NineP2000::print_if_error(Message &msg)
{
    // Nothing to do if not error
    if (msg.content.header.type != (uint8_t)Type::Rerror) {
        return;
    }

    // null terminate string and print directly out of buffer
    const uint16_t &len = (uint16_t&)msg.content.payload;
    msg.content.payload[MIN(len, 100) + sizeof(len)] = 0;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "9P2000: error: %s", (char*)&msg.content.payload[sizeof(len)]);
}

uint32_t AP_Networking::NineP2000::walk_result(const uint16_t tag, const bool dir)
{
    WITH_SEMAPHORE(request_sem);

    // Make sure the tag is valid and there is a waiting response
    if (!tag_response(tag)) {
        clear_tag(tag);
        return 0;
    }

    // Should be a walk response
    Message &msg = request[tag].result;
    if (msg.content.header.type != (uint8_t)Type::Rwalk) {
        print_if_error(msg);
        clear_tag(tag);
        return 0;
    }

    const uint16_t &num_ids = (uint16_t&)msg.content.payload;

    // Zero length walk means we must be root
    if (num_ids == 0) {
        const uint32_t fileId = request[tag].fileId;
        clear_tag(tag);
        // Root must be a dir
        if (!dir) {
            free_file_id(fileId);
            return 0;
        }
        return fileId;
    }

    // Calculate the offset of the last id
    const uint16_t id_offset = sizeof(num_ids) + (num_ids - 1) * sizeof(qid_t);

    // Make sure the message is long enough
    if (msg.content.header.length != (sizeof(msg.content.header) + id_offset + sizeof(qid_t))) {
        clear_tag(tag);
        return 0;
    }

    // Read in last id
    const qid_t &qid = (qid_t&)msg.content.payload[id_offset];

    // Expecting the correct type
    const uint8_t expected_type = dir ? qidType::QTDIR : qidType::QTFILE;
    const uint32_t fileId = request[tag].fileId;
    if (qid.type != expected_type) {
        clear_tag(tag);
        free_file_id(fileId);
        return 0;
    }

    // success, return id
    clear_tag(tag);
    return fileId;
}

// Return the file id to the server for re-use
void AP_Networking::NineP2000::free_file_id(const uint32_t id)
{
    WITH_SEMAPHORE(request_sem);

    // Check id is valid
    if (!valid_file_id(id)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        // This is bad, it means we leak a file id.
        return;
    }

    // Mark as clunked so we only free once
    fileIds[id - 1].clunked = true;

    // Mark tag as active
    request[tag].pending = true;
    request[tag].fileId = id;
    request[tag].expectedType = Type::Rclunk;

    // Fill in message
    send.content.header.type = (uint8_t)Type::Tclunk;
    send.content.header.tag = tag;
    send.content.header.length = sizeof(send.content.header) + sizeof(id);
    memcpy(&send.content.payload, &id, sizeof(id));

    sock->send(send.buffer, send.content.header.length);
}

// Request read of given file or directory with given flags
uint16_t AP_Networking::NineP2000::request_open(const uint32_t id, const int flags)
{
    WITH_SEMAPHORE(request_sem);

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        return NOTAG;
    }

    // ID invalid
    if (!valid_file_id(id)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return NOTAG;
    }

    // Mark tag as active
    request[tag].pending = true;
    request[tag].fileId = id;
    request[tag].expectedType = Type::Ropen;

    // Translate flags to mode
    uint8_t mode = 0;
    if ((flags & O_RDWR) != 0) {
        mode = openMode::ORDWR;

    } else if ((flags & O_WRONLY) != 0) {
        mode = openMode::OWRITE;

    } else {
        mode = openMode::OREAD;
    }

    // Fill in message
    send.content.header.type = (uint8_t)Type::Topen;
    send.content.header.tag = tag;
    send.content.header.length = sizeof(send.content.header) + sizeof(id) + sizeof(mode);
    memcpy(&send.content.payload, &id, sizeof(id));
    memcpy(&send.content.payload[sizeof(id)], &mode, sizeof(mode));

    sock->send(send.buffer, send.content.header.length);

    return tag;
}


// Fill in a directory item based on the read result, returns none zero if success
bool AP_Networking::NineP2000::open_result(const uint16_t tag)
{
    WITH_SEMAPHORE(request_sem);

    // Make sure the tag is valid and there is a waiting response
    if (!tag_response(tag)) {
        clear_tag(tag);
        return false;
    }

    // Should be a open response
    Message &msg = request[tag].result;
    if (msg.content.header.type != (uint8_t)Type::Ropen) {
        print_if_error(msg);
        clear_tag(tag);
        return false;
    }

    // Should be the correct length
    const uint16_t len = sizeof(msg.content.header) + sizeof(qid_t) + 4;
    if (msg.content.header.length != len) {
        clear_tag(tag);
        return false;
    }

    clear_tag(tag);
    return true;
}

// Read a directory or file, return tag, NOTAG if failed
uint16_t AP_Networking::NineP2000::request_read(const uint32_t id, const uint64_t offset, uint32_t count)
{
    WITH_SEMAPHORE(request_sem);

    // ID invalid
    if (!valid_file_id(id)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return NOTAG;
    }

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        return NOTAG;
    }

    // Mark tag as active
    request[tag].pending = true;
    request[tag].fileId = id;
    request[tag].expectedType = Type::Rread;

    // Limit count so as not to request a message over the max length
    // Response has a header and 32 bit count.
    const uint16_t maxLen = bufferLen - sizeof(send.content.header) - 4;
    count = MIN(count, maxLen);

    // Fill in message
    send.content.header.type = (uint8_t)Type::Tread;
    send.content.header.tag = tag;
    send.content.header.length = sizeof(send.content.header) + sizeof(id) + sizeof(offset) + sizeof(count);
    memcpy(&send.content.payload, &id, sizeof(id));
    memcpy(&send.content.payload[sizeof(id)], &offset, sizeof(offset));
    memcpy(&send.content.payload[sizeof(id)+sizeof(offset)], &count, sizeof(count));

    sock->send(send.buffer, send.content.header.length);

    return tag;
}

// Fill in a directory item based on the read result, returns none zero if success
uint32_t AP_Networking::NineP2000::dir_read_result(const uint16_t tag, struct dirent &de)
{
    WITH_SEMAPHORE(request_sem);

    // Make sure the tag is valid and there is a waiting response
    if (!tag_response(tag)) {
        clear_tag(tag);
        return 0;
    }

    // Should be a read response
    Message &msg = request[tag].result;
    if (msg.content.header.type != (uint8_t)Type::Rread) {
        print_if_error(msg);
        clear_tag(tag);
        return 0;
    }

    // Should at least contain header count and stat
    const uint16_t stat_end = sizeof(msg.content.header) + 4 + sizeof(stat_t);
    if (msg.content.header.length < stat_end) {
        clear_tag(tag);
        return 0;
    }

    const stat_t &info = (stat_t&)msg.content.payload[4];

    // Length feild does not include itself
    const uint16_t stat_len = info.msg_size + sizeof(info.msg_size);

    // Make sure there is room for the whole stat now we know the full size
    if (msg.content.header.length < (sizeof(msg.content.header) + 4 + stat_len)) {
        clear_tag(tag);
        return 0;
    }

    // Copy name, comes after the stat
    const uint16_t name_len = (uint16_t&)msg.buffer[stat_end];

    memset(de.d_name, 0, sizeof(de.d_name));
    strncpy(de.d_name, (char*)&msg.buffer[stat_end + 2], MIN(sizeof(de.d_name), name_len));

    // Fill in file flags
    const uint8_t &mode = info.qid.type;
    if (mode == qidType::QTFILE) {
        de.d_type = DT_REG;
    } else if (mode == qidType::QTDIR) {
        de.d_type = DT_DIR;
    } else {
        // Invalid type
        clear_tag(tag);
        return 0;
    }

    // Return the size of the stat object so the directory offset can be updated
    clear_tag(tag);
    return stat_len;
}

// Return the number of bytes read, -1 for error
int32_t AP_Networking::NineP2000::file_read_result(const uint16_t tag, void *buf)
{
    WITH_SEMAPHORE(request_sem);

    // Make sure the tag is valid and there is a waiting response
    if (!tag_response(tag)) {
        clear_tag(tag);
        return -1;
    }

    // Should be a read response
    Message &msg = request[tag].result;
    if (msg.content.header.type != (uint8_t)Type::Rread) {
        print_if_error(msg);
        clear_tag(tag);
        return -1;
    }

    // Copy result
    uint32_t count;
    memcpy(&count, &msg.content.payload, sizeof(count));
    memcpy(buf, &msg.content.payload[sizeof(count)], count);

    // Finished with tag
    clear_tag(tag);

    return count;
}

// Request stat for a given file id
uint16_t AP_Networking::NineP2000::request_stat(const uint32_t id)
{
    WITH_SEMAPHORE(request_sem);

    // ID invalid
    if (!valid_file_id(id)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return NOTAG;
    }

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        return NOTAG;
    }

    // Mark tag as active
    request[tag].pending = true;
    request[tag].fileId = id;
    request[tag].expectedType = Type::Rstat;

    // Fill in message
    send.content.header.type = (uint8_t)Type::Tstat;
    send.content.header.tag = tag;
    send.content.header.length = sizeof(send.content.header) + sizeof(id);
    memcpy(&send.content.payload, &id, sizeof(id));

    sock->send(send.buffer, send.content.header.length);

    return tag;
}

bool AP_Networking::NineP2000::stat_result(const uint16_t tag, struct stat *stbuf)
{
    WITH_SEMAPHORE(request_sem);

    // Make sure the tag is valid and there is a waiting response
    if (!tag_response(tag)) {
        clear_tag(tag);
        return false;
    }

    // Should be a stat response
    Message &msg = request[tag].result;
    if (msg.content.header.type != (uint8_t)Type::Rstat) {
        print_if_error(msg);
        clear_tag(tag);
        return false;
    }

    // Get stat object
    const stat_t &info = (stat_t&)msg.content.payload[2];

    // Clear stats
    memset(stbuf, 0, sizeof(*stbuf));

    // length in bytes
    stbuf->st_size = info.length;

    // Access and modification timestamps
    stbuf->st_atime = info.atime;
    stbuf->st_mtime = info.mtime;

    // Fill in file flags
    const uint8_t &mode = info.qid.type;
    if (mode == qidType::QTFILE) {
        stbuf->st_mode |= S_IFREG;
    } else if (mode == qidType::QTDIR) {
        stbuf->st_mode |= S_IFDIR;
    }

    // Finished with tag
    clear_tag(tag);

    return true;
}

// Request write for given file id, return tag
uint16_t AP_Networking::NineP2000::request_write(const uint32_t id, const uint64_t offset, uint32_t count, const void *buf)
{
    WITH_SEMAPHORE(request_sem);

    // ID invalid
    if (!valid_file_id(id)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return NOTAG;
    }

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        return NOTAG;
    }

    // Limit write to max packet size
    const uint16_t data_offset = sizeof(send.content.header) + sizeof(id) + sizeof(offset) + sizeof(count);
    count = MIN(count, uint32_t(bufferLen - data_offset));

    // Mark tag as active
    request[tag].pending = true;
    request[tag].fileId = id;
    request[tag].expectedType = Type::Rwrite;

    // Fill in message
    send.content.header.type = (uint8_t)Type::Twrite;
    send.content.header.tag = tag;
    send.content.header.length = data_offset + count;
    memcpy(&send.content.payload, &id, sizeof(id));
    memcpy(&send.content.payload[sizeof(id)], &offset, sizeof(offset));
    memcpy(&send.content.payload[sizeof(id)+sizeof(offset)], &count, sizeof(count));
    memcpy(&send.buffer[data_offset], buf, count);

    sock->send(send.buffer, send.content.header.length);

    return tag;
}

int32_t AP_Networking::NineP2000::write_result(const uint16_t tag)
{
    WITH_SEMAPHORE(request_sem);

    // Make sure the tag is valid and there is a waiting response
    if (!tag_response(tag)) {
        clear_tag(tag);
        return -1;
    }

    // Should be a write response
    Message &msg = request[tag].result;
    if (msg.content.header.type != (uint8_t)Type::Rwrite) {
        print_if_error(msg);
        clear_tag(tag);
        return -1;
    }

    uint32_t count;
    memcpy(&count, &msg.content.payload, sizeof(count));

    // Finished with tag
    clear_tag(tag);

    return count;
}

#endif // AP_NETWORKING_FILESYSTEM_ENABLED
