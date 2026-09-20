/*
   Read-only MAVFTP camera definition for the simulated MT11.
*/

#include "SIM_config.h"

#if AP_SIM_MT11_ENABLED

#include "SIM_MT11.h"
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>
#include <string.h>

namespace SITL
{

// No extended parameters are simulated; the standard camera commands
// provide mode, capture, zoom and focus controls.
static const char camera_xml[] =
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<mavlinkcamera>\n"
    "  <definition version=\"1\">\n"
    "    <model>MT11</model>\n"
    "    <vendor>ArduPilot</vendor>\n"
    "  </definition>\n"
    "  <!-- Controls use the standard MAVLink camera commands. -->\n"
    "  <parameters/>\n"
    "</mavlinkcamera>\n";

void MT11::get_camera_definition(mavlink_camera_information_t &info) const
{
    info.cam_definition_version = 1;
    // Explicitly identify the file server; this is not a parameter endpoint.
    snprintf(info.cam_definition_uri, sizeof(info.cam_definition_uri),
             "mftp://[;comp=%u]/camera.xml", camera_compid());
}

void MT11::handle_ftp(const mavlink_message_t &msg)
{
    if (msg.msgid != MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL || vehicle_sysid() == 0) {
        return;
    }
    mavlink_file_transfer_protocol_t request;
    mavlink_msg_file_transfer_protocol_decode(&msg, &request);
    if (request.target_network != 0 || request.target_system != vehicle_sysid() ||
        request.target_component != camera_compid()) {
        return;
    }

    const uint8_t *p = request.payload;
    const uint16_t seq = le16toh_ptr(p) + 1;
    const uint8_t session = p[2];
    const uint8_t opcode = p[3];
    const uint8_t size = p[4];
    const uint32_t offset = le32toh_ptr(&p[8]);
    constexpr uint8_t max_data = sizeof(request.payload) - 12;
    constexpr uint32_t file_size = sizeof(camera_xml) - 1;

    // Re-send the last response on retries, including after termination.
    if (!_ftp.reply_valid || _ftp.reply.target_system != msg.sysid ||
        _ftp.reply.target_component != msg.compid ||
        le16toh_ptr(_ftp.reply.payload) != seq || _ftp.reply.payload[5] != opcode) {
        mavlink_file_transfer_protocol_t &reply = _ftp.reply;
        reply = {};
        reply.target_system = msg.sysid;
        reply.target_component = msg.compid;
        uint8_t *r = reply.payload;
        put_le16_ptr(r, seq);
        r[2] = session;
        r[3] = MAV_FTP_OPCODE_ACK;
        r[5] = opcode;
        put_le32_ptr(&r[8], offset);

        uint8_t error = MAV_FTP_ERR_NONE;
        const bool valid_session = _ftp.open && session == _ftp.session &&
                                   msg.sysid == _ftp.sysid && msg.compid == _ftp.compid;
        char path[max_data + 1] {};
        if (size > max_data) {
            error = MAV_FTP_ERR_INVALIDDATASIZE;
        } else {
            memcpy(path, &p[12], size);
            // GCS URI parsers may retain an extra leading slash.
            const char *name = path;
            while (*name == '/') {
                name++;
            }
            switch (opcode) {
            case MAV_FTP_OPCODE_NONE:
                break;
            case MAV_FTP_OPCODE_RESETSESSION:
                _ftp.open = false;
                break;
            case MAV_FTP_OPCODE_TERMINATESESSION:
                if (!valid_session) {
                    error = MAV_FTP_ERR_INVALIDSESSION;
                    break;
                }
                _ftp.open = false;
                break;
            case MAV_FTP_OPCODE_LISTDIRECTORY:
                if (strcmp(name, "") != 0 && strcmp(name, ".") != 0) {
                    error = MAV_FTP_ERR_FILENOTFOUND;
                    break;
                }
                if (offset != 0) {
                    error = MAV_FTP_ERR_EOF;
                    break;
                }
                r[4] = snprintf((char *)&r[12], max_data, "Fcamera.xml\t%u", unsigned(file_size)) + 1;
                break;
            case MAV_FTP_OPCODE_OPENFILERO:
                if (strcmp(name, "camera.xml") != 0) {
                    error = MAV_FTP_ERR_FILENOTFOUND;
                    break;
                }
                if (_ftp.open && !valid_session) {
                    error = MAV_FTP_ERR_NOSESSIONSAVAILABLE;
                    break;
                }
                _ftp.open = true;
                _ftp.session = session;
                _ftp.sysid = msg.sysid;
                _ftp.compid = msg.compid;
                r[4] = sizeof(uint32_t);
                put_le32_ptr(&r[12], file_size);
                break;
            case MAV_FTP_OPCODE_READFILE:
            case MAV_FTP_OPCODE_BURSTREADFILE:
                if (!valid_session) {
                    error = MAV_FTP_ERR_INVALIDSESSION;
                    break;
                }
                if (offset >= file_size) {
                    error = MAV_FTP_ERR_EOF;
                    break;
                }
                r[4] = MIN(size == 0 ? max_data : size, file_size - offset);
                memcpy(&r[12], &camera_xml[offset], r[4]);
                // One packet per burst bounds serial buffer use and allows
                // the client to request the next offset without a worker.
                r[6] = opcode == MAV_FTP_OPCODE_BURSTREADFILE;
                break;
            default:
                error = MAV_FTP_ERR_UNKNOWNCOMMAND;
                break;
            }
        }
        if (error != MAV_FTP_ERR_NONE) {
            r[3] = MAV_FTP_OPCODE_NAK;
            r[4] = 1;
            r[12] = error;
        }
        _ftp.reply_valid = true;
    }

    mavlink_message_t reply_msg;
    mavlink_msg_file_transfer_protocol_encode_status(
        vehicle_sysid(), camera_compid(), &camera_mav_status(), &reply_msg, &_ftp.reply);
    camera_send_mavlink_message(reply_msg);
}

} // namespace SITL

#endif // AP_SIM_MT11_ENABLED
