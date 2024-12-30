#include "AP_Networking_Config.h"

#if AP_NETWORKING_FILESYSTEM_ENABLED

#include "AP_Networking.h"
#include <AP_HAL/utility/Socket.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Math/AP_Math.h>
#include <errno.h>

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

    while (true) {
        hal.scheduler->delay_microseconds(100);
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

            // Restart connection process
            request_version();

        }
        update();
    }
}

// Return true if connected and mounted
bool AP_Networking::NineP2000::mounted()
{
    return connected && (state == State::Mounted);
}

// Deal with incoming data
void AP_Networking::NineP2000::update()
{
    const auto len = sock->recv(receive.buffer, sizeof(receive.buffer), 0);
    if (len == 0) {
        // Zero means disconnected
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "9P2000: closed connection");
        delete sock;
        sock = nullptr;
        return;
    }
    if (len < 0) {
        // Could return -1 if there is no pending data
        return;
    }

    // Need at least a header
    if ((uint32_t)len < sizeof(receive.content.header)) {
        return;
    }

    // Should receive the correct message length
    if (len != receive.content.header.length) {
        return;
    }

    // Use semaphore for thread safety
    WITH_SEMAPHORE(request_sem);

    // Deal with each message type
    switch ((Type)receive.content.header.type) {
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
            if (tag < ARRAY_SIZE(request)) {
                clear_file_id(request[tag].fileId);
            }
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
    qid_t id;
    memcpy(&id, &receive.content.payload, sizeof(id));

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
    // Must be mounted for operations to be valid
    if (state != State::Mounted) {
        return NOTAG;
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(request); i++) {
        if (!request[i].active) {
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
    request[tag].active = false;
}

// Request stat for given file id
uint16_t AP_Networking::NineP2000::request_stat(const uint32_t fid)
{
    WITH_SEMAPHORE(request_sem);

    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        return NOTAG;
    }

    // Fill in message
    send.content.header.type = (uint8_t)Type::Tstat;
    send.content.header.tag = tag;
    send.content.header.length = sizeof(send.content.header) + sizeof(fid);

    memcpy(&send.content.payload, &fid, sizeof(fid));

    // Send and mark active
    sock->send(send.buffer, send.content.header.length);
    request[tag].active = true;
    request[tag].fileId = fid;

    // Return tag which the caller can use to check for the result
    return tag;
}

// Generate a new unique file id
uint32_t AP_Networking::NineP2000::generate_unique_file_id() const
{
    // Generate a random id and check it does not clash with existing IDs.
    // If it does we try again, this could recurse forever! (but its very unlikely)

    uint32_t id;
    hal.util->get_random_vals((uint8_t*)&id, sizeof(id));

    // 0 is reserved for root
    if (id == 0) {
        return generate_unique_file_id();
    }

    // Check active IDs
    for (uint8_t i = 0; i < ARRAY_SIZE(fileIds); i++) {
        if (id == fileIds[i]) {
            return generate_unique_file_id();
        }
    }

    // No conflicts!
    return id;
}

// Add a file ID to the list of those being used, return false if not space available
bool AP_Networking::NineP2000::add_file_id(const uint32_t fileId)
{
    for (uint8_t i = 0; i < ARRAY_SIZE(fileIds); i++) {
        if (fileIds[i] == 0) {
            fileIds[i] = fileId;
            return true;
        }
    }

    return false;
}

// Clear a file id now the file has been closed
void AP_Networking::NineP2000::clear_file_id(const uint32_t fileId)
{
    for (uint8_t i = 0; i < ARRAY_SIZE(fileIds); i++) {
        if (fileIds[i] == fileId) {
            fileIds[i] = 0;
        }
    }
}

// Clone file ID prior to walking tree
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

    // Make sure we can reserve it
    bool reserved = false;
    for (uint8_t i = 0; i < ARRAY_SIZE(fileIds); i++) {
        if (fileIds[i] == 0) {
            fileIds[i] = id;
            reserved = true;
            break;
        }
    }

    // No free tags, give up
    if (!reserved) {
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
    request[tag].active = true;
    request[tag].pending = true;
    request[tag].fileId = id;

    // Send command
    sock->send(send.buffer, send.content.header.length);

    return tag;
}

uint32_t AP_Networking::NineP2000::dir_walk_result(const uint16_t tag)
{
    WITH_SEMAPHORE(request_sem);

    // Make sure the tag is valid and there is a waiting response
    if (!tag_response(tag)) {
        clear_tag(tag);
        return 0;
    }

    // Should be a walk response
    if (request[tag].result.content.header.type != (uint8_t)Type::Rwalk) {
        if (request[tag].result.content.header.type == (uint8_t)Type::Rerror) {
            uint16_t len;
            memcpy(&len, &receive.content.payload, sizeof(len));

            // null terminate string and print directly out of buffer
            receive.content.payload[MIN(len, 100) + sizeof(len)] = 0;

            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "9P2000: error: %s", (char*)&receive.content.payload[sizeof(len)]);
        }
        clear_tag(tag);
        return 0;
    }

    uint16_t num_ids;
    memcpy(&num_ids, &receive.content.payload, sizeof(num_ids));

    // Read in ids
    const uint32_t fileId = request[tag].fileId;
    qid_t qid;
    memcpy(&qid, &receive.content.payload[sizeof(num_ids)], sizeof(qid));

    // Copied everything from message, can clear
    clear_tag(tag);

    // Expecting a directory
    if (qid.type != qidType::QTDIR) {
        // clunk file id
        return 0;
    }

    // success, return id
    return fileId;
}

// Return the file id to the server for re-use
void AP_Networking::NineP2000::free_file_id(const uint32_t id)
{
    WITH_SEMAPHORE(request_sem);

    // See if there are any tags free
    const uint16_t tag = get_free_tag();
    if (tag == NOTAG) {
        // This is bad, it means we leak a file id.
        return;
    }

    // Mark tag as active
    request[tag].active = true;
    request[tag].pending = true;
    request[tag].fileId = id;

    // Fill in message
    send.content.header.type = (uint8_t)Type::Tclunk;
    send.content.header.tag = tag;
    send.content.header.length = sizeof(send.content.header) + sizeof(id);
    memcpy(&send.content.payload, &id, sizeof(id));

    sock->send(send.buffer, send.content.header.length);
}

#endif // AP_NETWORKING_FILESYSTEM_ENABLED
