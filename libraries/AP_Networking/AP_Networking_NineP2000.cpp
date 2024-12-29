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

        default:
            break;

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
    send.content.header.tag = 0;
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

    if (receive.content.header.tag != 0) {
        // tag should always be 0, throw a error?
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
    send.content.header.tag = 0;
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
    if (receive.content.header.length != sizeof(receive.content.header) + sizeof(qid)) {
        return;
    }

    if (receive.content.header.tag != 0) {
        // tag should always be 0
        return;
    }

    // Read in qid
    qid id;
    memcpy(&id, &receive.content.payload, sizeof(qid));

    // Expecting a directory
    if (id.type != qidType::QTDIR) {
        return;
    }

    // Keep track of root directory and mark as mounted
    root = id.path;
    state = State::Mounted;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "9P2000: mounted file system");

}

#endif // AP_NETWORKING_FILESYSTEM_ENABLED
