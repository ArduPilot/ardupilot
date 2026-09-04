/*
   Simulator for an MT11 MAVLink camera and gimbal
*/

#include "SIM_config.h"

#if AP_SIM_MT11_ENABLED

#include "SIM_MT11.h"
#include "SIM_Aircraft.h"

#include <AP_HAL/utility/Socket_native.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_ROMFS/AP_ROMFS.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

extern const AP_HAL::HAL &hal;

namespace SITL {

class MT11_RTSPServer {
public:
    MT11_RTSPServer(uint8_t sitl_instance, uint8_t camera_instance) :
        _instance(camera_instance),
        _port(8554U + sitl_instance * 10U + camera_instance)
    {
        start();
    }

    bool running() const { return _running; }
    uint16_t port() const { return _port; }

    void update(uint64_t wall_time_us)
    {
        if (!_running) {
            return;
        }

        while (true) {
            SocketAPM_native *sock = _listener->accept(0);
            if (sock == nullptr) {
                break;
            }
            if (!sock->set_blocking(false)) {
                delete sock;
                continue;
            }
            bool accepted = false;
            for (uint8_t i = 0; i < ARRAY_SIZE(_clients); i++) {
                if (_clients[i].control != nullptr) {
                    continue;
                }
                _clients[i] = Client{};
                _clients[i].output = NEW_NOTHROW ByteBuffer(OUTPUT_BUFFER_SIZE);
                if (_clients[i].output == nullptr ||
                    _clients[i].output->get_size() != OUTPUT_BUFFER_SIZE) {
                    delete _clients[i].output;
                    _clients[i].output = nullptr;
                    break;
                }
                _clients[i].control = sock;
                _clients[i].session_id = 1000U + 10U * _instance + i;
                _clients[i].sequence = 1;
                _clients[i].timestamp = 90000U * (i + 1U);
                accepted = true;
                break;
            }
            if (!accepted) {
                delete sock;
            }
        }

        for (uint8_t i = 0; i < ARRAY_SIZE(_clients); i++) {
            Client &client = _clients[i];
            if (client.control == nullptr) {
                continue;
            }
            if (!flush_output(client) ||
                (!client.closing && !read_requests(client, i, wall_time_us))) {
                disconnect(client);
                continue;
            }
            // Keep at most one video frame queued for a slow TCP client.
            if (!client.closing && client.playing && client.output->is_empty() &&
                wall_time_us >= client.next_frame_us) {
                if (!send_frame(client)) {
                    disconnect(client);
                    continue;
                }
                client.next_frame_us = wall_time_us + FRAME_INTERVAL_US;
            }
            if (!flush_output(client) || (client.closing && client.output->is_empty())) {
                disconnect(client);
            }
        }
    }

private:
    static constexpr uint8_t MAX_CLIENTS = 4;
    static constexpr uint32_t FRAME_INTERVAL_US = 1000000U / 30U;
    static constexpr uint16_t RTP_MAX_PAYLOAD = 1388;
    static constexpr uint32_t OUTPUT_BUFFER_SIZE = 64U * 1024U;
    static constexpr uint16_t MAX_RESPONSE_SIZE = 2048;

    enum class Transport : uint8_t {
        NONE,
        UDP,
        TCP,
    };

    struct Media {
        const uint8_t *data;
        uint32_t size;
        char sps[192];
        char pps[96];
        char profile_level_id[7];
    };

    struct Client {
        SocketAPM_native *control;
        SocketAPM_native *rtp_socket;
        ByteBuffer *output;
        char request[4096];
        uint16_t request_len;
        uint32_t session_id;
        uint32_t frame_offset;
        uint64_t next_frame_us;
        uint32_t timestamp;
        uint32_t ssrc;
        uint16_t sequence;
        uint16_t client_rtp_port;
        uint16_t server_rtp_port;
        uint8_t stream_index;
        uint8_t interleaved_channel;
        Transport transport;
        bool playing;
        bool closing;
    };

    static const uint8_t *find_start_code(const uint8_t *ptr,
                                          const uint8_t *end,
                                          uint8_t &prefix_len)
    {
        while (end - ptr >= 3) {
            if (ptr[0] == 0 && ptr[1] == 0 && ptr[2] == 1) {
                prefix_len = 3;
                return ptr;
            }
            if (end - ptr >= 4 && ptr[0] == 0 && ptr[1] == 0 &&
                ptr[2] == 0 && ptr[3] == 1) {
                prefix_len = 4;
                return ptr;
            }
            ptr++;
        }
        return nullptr;
    }

    static bool base64_encode(const uint8_t *src, uint16_t src_len,
                              char *dst, uint16_t dst_len)
    {
        static const char alphabet[] =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        const uint32_t required = 4U * ((uint32_t(src_len) + 2U) / 3U) + 1U;
        if (required > dst_len) {
            return false;
        }
        uint16_t in = 0;
        uint16_t out = 0;
        while (in < src_len) {
            const uint32_t a = src[in++];
            const bool have_b = in < src_len;
            const uint32_t b = have_b ? src[in++] : 0;
            const bool have_c = in < src_len;
            const uint32_t c = have_c ? src[in++] : 0;
            const uint32_t value = (a << 16) | (b << 8) | c;
            dst[out++] = alphabet[(value >> 18) & 0x3f];
            dst[out++] = alphabet[(value >> 12) & 0x3f];
            dst[out++] = have_b ? alphabet[(value >> 6) & 0x3f] : '=';
            dst[out++] = have_c ? alphabet[value & 0x3f] : '=';
        }
        dst[out] = 0;
        return true;
    }

    bool load_media(Media &media, const char *filename)
    {
        media.data = AP_ROMFS::find_decompress(filename, media.size);
        if (media.data == nullptr || media.size == 0) {
            return false;
        }

        const uint8_t *sps = nullptr;
        const uint8_t *pps = nullptr;
        uint16_t sps_len = 0;
        uint16_t pps_len = 0;
        const uint8_t *const end = media.data + media.size;
        const uint8_t *scan = media.data;
        while (scan < end && (sps == nullptr || pps == nullptr)) {
            uint8_t prefix_len = 0;
            const uint8_t *start = find_start_code(scan, end, prefix_len);
            if (start == nullptr || start + prefix_len >= end) {
                break;
            }
            const uint8_t *nal = start + prefix_len;
            uint8_t next_prefix_len = 0;
            const uint8_t *next = find_start_code(nal + 1, end,
                                                   next_prefix_len);
            const uint32_t nal_len = (next == nullptr) ? end - nal : next - nal;
            if (nal_len > UINT16_MAX) {
                return false;
            }
            switch (nal[0] & 0x1f) {
            case 7:
                sps = nal;
                sps_len = nal_len;
                break;
            case 8:
                pps = nal;
                pps_len = nal_len;
                break;
            default:
                break;
            }
            scan = (next == nullptr) ? end : next;
        }
        if (sps == nullptr || pps == nullptr || sps_len < 4 ||
            !base64_encode(sps, sps_len, media.sps, sizeof(media.sps)) ||
            !base64_encode(pps, pps_len, media.pps, sizeof(media.pps))) {
            return false;
        }
        hal.util->snprintf(media.profile_level_id,
                           sizeof(media.profile_level_id),
                           "%02X%02X%02X", sps[1], sps[2], sps[3]);
        return true;
    }

    void start()
    {
        if (!load_media(_media[0], "models/mt11_visible.h264") ||
            !load_media(_media[1], "models/mt11_thermal.h264")) {
            ::printf("SIM_MT11: failed to load video streams from ROMFS\n");
            return;
        }

        _listener = NEW_NOTHROW SocketAPM_native(false);
        if (_listener == nullptr) {
            return;
        }
        _listener->reuseaddress();
        if (!_listener->bind("127.0.0.1", _port) ||
            !_listener->listen(MAX_CLIENTS) ||
            !_listener->set_blocking(false)) {
            ::printf("SIM_MT11: failed to listen for RTSP on port %u\n",
                     unsigned(_port));
            delete _listener;
            _listener = nullptr;
            return;
        }
        _running = true;
        ::printf("SIM_MT11: RTSP streams on rtsp://127.0.0.1:%u/video{1,2}\n",
                 unsigned(_port));
    }

    void disconnect(Client &client)
    {
        delete client.control;
        delete client.rtp_socket;
        delete client.output;
        client = Client{};
    }

    bool flush_output(Client &client)
    {
        uint32_t len;
        const uint8_t *data = client.output->readptr(len);
        if (len == 0) {
            return true;
        }
        const ssize_t sent = client.control->send(data, len);
        if (sent < 0) {
            return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
        }
        if (sent == 0) {
            return false;
        }
        client.output->advance(sent);
        return true;
    }

    static uint32_t get_cseq(const char *request)
    {
        const char *cseq = strstr(request, "CSeq:");
        return cseq == nullptr ? 0 : strtoul(cseq + 5, nullptr, 10);
    }

    bool send_response(Client &client, uint32_t cseq,
                       const char *status, const char *headers = "",
                       const char *body = "")
    {
        char response[MAX_RESPONSE_SIZE];
        const uint16_t body_len = strlen(body);
        const int len = hal.util->snprintf(
            response, sizeof(response),
            "RTSP/1.0 %s\r\nCSeq: %u\r\nServer: ArduPilot-SITL-MT11\r\n%s"
            "Content-Length: %u\r\n\r\n%s",
            status, unsigned(cseq), headers, unsigned(body_len), body);
        if (len <= 0 || len >= (int)sizeof(response)) {
            return false;
        }
        return client.output->write((const uint8_t *)response, len) == unsigned(len);
    }

    int8_t stream_from_uri(const char *uri) const
    {
        if (strstr(uri, "/video1") != nullptr) {
            return 0;
        }
        if (strstr(uri, "/video2") != nullptr) {
            return 1;
        }
        return -1;
    }

    bool setup_udp(Client &client, uint8_t client_index,
                   const char *transport)
    {
        const char *client_port = strstr(transport, "client_port=");
        if (client_port == nullptr) {
            return false;
        }
        unsigned rtp_port = 0;
        if (sscanf(client_port + 12, "%u", &rtp_port) != 1 ||
            rtp_port == 0 || rtp_port > UINT16_MAX) {
            return false;
        }

        delete client.rtp_socket;
        client.rtp_socket = NEW_NOTHROW SocketAPM_native(true);
        if (client.rtp_socket == nullptr) {
            return false;
        }
        client.rtp_socket->reuseaddress();
        const uint16_t first_port = 10000U + 16U * _instance +
                                    2U * client_index;
        for (uint8_t i = 0; i < 8; i++) {
            client.server_rtp_port = first_port + 2U * i;
            if (client.rtp_socket->bind("127.0.0.1",
                                        client.server_rtp_port)) {
                client.client_rtp_port = rtp_port;
                client.transport = Transport::UDP;
                return true;
            }
        }
        delete client.rtp_socket;
        client.rtp_socket = nullptr;
        return false;
    }

    bool handle_request(Client &client, uint8_t client_index,
                        const char *request, uint64_t wall_time_us)
    {
        char method[16];
        char uri[256];
        if (sscanf(request, "%15s %255s", method, uri) != 2) {
            return false;
        }
        const uint32_t cseq = get_cseq(request);

        if (strcmp(method, "OPTIONS") == 0) {
            return send_response(
                client, cseq, "200 OK",
                "Public: OPTIONS, DESCRIBE, SETUP, PLAY, PAUSE, TEARDOWN, GET_PARAMETER\r\n");
        }

        if (strcmp(method, "DESCRIBE") == 0) {
            const int8_t stream_index = stream_from_uri(uri);
            if (stream_index < 0) {
                return send_response(client, cseq, "404 Not Found");
            }
            client.stream_index = stream_index;
            const Media &media = _media[stream_index];
            char sdp[1024];
            const int sdp_len = hal.util->snprintf(
                sdp, sizeof(sdp),
                "v=0\r\no=- 0 0 IN IP4 127.0.0.1\r\n"
                "s=MT11 %s SITL\r\nc=IN IP4 127.0.0.1\r\nt=0 0\r\n"
                "a=control:*\r\na=range:npt=0-\r\n"
                "m=video 0 RTP/AVP 96\r\na=rtpmap:96 H264/90000\r\n"
                "a=fmtp:96 packetization-mode=1;profile-level-id=%s;"
                "sprop-parameter-sets=%s,%s\r\na=control:trackID=0\r\n",
                stream_index == 0 ? "Visible" : "Thermal",
                media.profile_level_id, media.sps, media.pps);
            if (sdp_len <= 0 || sdp_len >= (int)sizeof(sdp)) {
                return false;
            }
            char headers[384];
            hal.util->snprintf(headers, sizeof(headers),
                               "Content-Type: application/sdp\r\n"
                               "Content-Base: rtsp://127.0.0.1:%u/video%u/\r\n",
                               unsigned(_port), unsigned(stream_index + 1));
            return send_response(client, cseq, "200 OK", headers, sdp);
        }

        if (strcmp(method, "SETUP") == 0) {
            const int8_t stream_index = stream_from_uri(uri);
            const char *transport = strstr(request, "Transport:");
            if (stream_index < 0 || transport == nullptr) {
                return send_response(client, cseq, "400 Bad Request");
            }
            client.stream_index = stream_index;
            client.ssrc = 0x4d543100U + 16U * _instance + client_index;
            char headers[384];
            if (strstr(transport, "RTP/AVP/TCP") != nullptr ||
                strstr(transport, "interleaved=") != nullptr) {
                unsigned rtp_channel = 0;
                unsigned rtcp_channel = 1;
                const char *interleaved = strstr(transport, "interleaved=");
                if (interleaved != nullptr) {
                    IGNORE_RETURN(sscanf(interleaved + 12, "%u-%u",
                                         &rtp_channel, &rtcp_channel));
                }
                if (rtp_channel > UINT8_MAX || rtcp_channel > UINT8_MAX) {
                    return send_response(client, cseq,
                                         "461 Unsupported Transport");
                }
                client.transport = Transport::TCP;
                client.interleaved_channel = rtp_channel;
                hal.util->snprintf(
                    headers, sizeof(headers),
                    "Transport: RTP/AVP/TCP;unicast;interleaved=%u-%u;ssrc=%08X\r\n"
                    "Session: %u;timeout=60\r\n",
                    rtp_channel, rtcp_channel, unsigned(client.ssrc),
                    unsigned(client.session_id));
            } else if (setup_udp(client, client_index, transport)) {
                hal.util->snprintf(
                    headers, sizeof(headers),
                    "Transport: RTP/AVP;unicast;client_port=%u-%u;"
                    "server_port=%u-%u;ssrc=%08X\r\n"
                    "Session: %u;timeout=60\r\n",
                    unsigned(client.client_rtp_port),
                    unsigned(client.client_rtp_port + 1U),
                    unsigned(client.server_rtp_port),
                    unsigned(client.server_rtp_port + 1U),
                    unsigned(client.ssrc), unsigned(client.session_id));
            } else {
                return send_response(client, cseq,
                                     "461 Unsupported Transport");
            }
            return send_response(client, cseq, "200 OK", headers);
        }

        if (strcmp(method, "PLAY") == 0) {
            if (client.transport == Transport::NONE) {
                return send_response(client, cseq, "455 Method Not Valid");
            }
            client.playing = true;
            client.frame_offset = 0;
            client.next_frame_us = wall_time_us;
            char headers[384];
            hal.util->snprintf(
                headers, sizeof(headers),
                "Session: %u\r\nRange: npt=0.000-\r\n"
                "RTP-Info: url=rtsp://127.0.0.1:%u/video%u/trackID=0;"
                "seq=%u;rtptime=%u\r\n",
                unsigned(client.session_id), unsigned(_port),
                unsigned(client.stream_index + 1U), unsigned(client.sequence),
                unsigned(client.timestamp));
            return send_response(client, cseq, "200 OK", headers);
        }

        if (strcmp(method, "PAUSE") == 0) {
            client.playing = false;
            char headers[64];
            hal.util->snprintf(headers, sizeof(headers), "Session: %u\r\n",
                               unsigned(client.session_id));
            return send_response(client, cseq, "200 OK", headers);
        }

        if (strcmp(method, "GET_PARAMETER") == 0 ||
            strcmp(method, "SET_PARAMETER") == 0) {
            char headers[64];
            hal.util->snprintf(headers, sizeof(headers), "Session: %u\r\n",
                               unsigned(client.session_id));
            return send_response(client, cseq, "200 OK", headers);
        }

        if (strcmp(method, "TEARDOWN") == 0) {
            client.closing = true;
            return send_response(client, cseq, "200 OK");
        }

        return send_response(client, cseq, "405 Method Not Allowed");
    }

    bool read_requests(Client &client, uint8_t client_index,
                       uint64_t wall_time_us)
    {
        if (client.output->space() < MAX_RESPONSE_SIZE) {
            return true;
        }
        if (client.control->pollin(0)) {
            const uint16_t available = sizeof(client.request) -
                                       client.request_len - 1U;
            if (available == 0) {
                return false;
            }
            const ssize_t received = client.control->recv(
                client.request + client.request_len, available, 0);
            if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
                return true;
            }
            if (received <= 0) {
                return false;
            }
            client.request_len += received;
            client.request[client.request_len] = 0;
        }

        while (client.request_len > 0 && !client.closing &&
               client.output->space() >= MAX_RESPONSE_SIZE) {
            if ((uint8_t)client.request[0] == '$') {
                if (client.request_len < 4) {
                    return true;
                }
                const uint16_t packet_len =
                    ((uint8_t)client.request[2] << 8) |
                    (uint8_t)client.request[3];
                const uint32_t total_len = uint32_t(packet_len) + 4U;
                if (total_len >= sizeof(client.request)) {
                    return false;
                }
                if (client.request_len < total_len) {
                    return true;
                }
                memmove(client.request, client.request + total_len,
                        client.request_len - total_len);
                client.request_len -= total_len;
                client.request[client.request_len] = 0;
                continue;
            }

            const char *header_end = strstr(client.request, "\r\n\r\n");
            if (header_end == nullptr) {
                return true;
            }
            uint16_t total_len = header_end - client.request + 4U;
            const char *content_length = strstr(client.request,
                                                 "Content-Length:");
            if (content_length != nullptr && content_length < header_end) {
                const unsigned long body_len = strtoul(content_length + 15,
                                                       nullptr, 10);
                if (body_len >= sizeof(client.request) - total_len) {
                    return false;
                }
                total_len += body_len;
            }
            if (total_len >= sizeof(client.request)) {
                return false;
            }
            if (client.request_len < total_len) {
                return true;
            }

            char request[4096];
            memcpy(request, client.request, total_len);
            request[total_len] = 0;
            memmove(client.request, client.request + total_len,
                    client.request_len - total_len);
            client.request_len -= total_len;
            client.request[client.request_len] = 0;
            if (!handle_request(client, client_index, request,
                                wall_time_us)) {
                return false;
            }
        }
        return true;
    }

    bool send_rtp_packet(Client &client, const uint8_t *payload,
                         uint16_t payload_len, bool marker)
    {
        uint8_t packet[12 + RTP_MAX_PAYLOAD];
        packet[0] = 0x80;
        packet[1] = 96 | (marker ? 0x80 : 0);
        packet[2] = client.sequence >> 8;
        packet[3] = client.sequence;
        packet[4] = client.timestamp >> 24;
        packet[5] = client.timestamp >> 16;
        packet[6] = client.timestamp >> 8;
        packet[7] = client.timestamp;
        packet[8] = client.ssrc >> 24;
        packet[9] = client.ssrc >> 16;
        packet[10] = client.ssrc >> 8;
        packet[11] = client.ssrc;
        memcpy(packet + 12, payload, payload_len);
        client.sequence++;

        const uint16_t packet_len = payload_len + 12U;
        if (client.transport == Transport::UDP) {
            return client.rtp_socket != nullptr &&
                   client.rtp_socket->sendto(packet, packet_len,
                                             "127.0.0.1",
                                             client.client_rtp_port) == packet_len;
        }

        uint8_t interleaved[4 + sizeof(packet)];
        interleaved[0] = '$';
        interleaved[1] = client.interleaved_channel;
        interleaved[2] = packet_len >> 8;
        interleaved[3] = packet_len;
        memcpy(interleaved + 4, packet, packet_len);
        return client.output->write(interleaved, packet_len + 4U) ==
               packet_len + 4U;
    }

    bool send_nal(Client &client, const uint8_t *nal, uint32_t nal_len,
                  bool marker)
    {
        if (nal_len <= RTP_MAX_PAYLOAD) {
            return send_rtp_packet(client, nal, nal_len, marker);
        }

        const uint8_t nal_header = nal[0];
        uint32_t offset = 1;
        while (offset < nal_len) {
            uint8_t payload[RTP_MAX_PAYLOAD];
            const uint16_t fragment_len = MIN(
                uint32_t(RTP_MAX_PAYLOAD - 2U), nal_len - offset);
            payload[0] = (nal_header & 0xe0) | 28;
            payload[1] = nal_header & 0x1f;
            if (offset == 1) {
                payload[1] |= 0x80;
            }
            const bool last_fragment = offset + fragment_len >= nal_len;
            if (last_fragment) {
                payload[1] |= 0x40;
            }
            memcpy(payload + 2, nal + offset, fragment_len);
            if (!send_rtp_packet(client, payload, fragment_len + 2U,
                                 marker && last_fragment)) {
                return false;
            }
            offset += fragment_len;
        }
        return true;
    }

    bool send_frame(Client &client)
    {
        const Media &media = _media[client.stream_index];
        const uint8_t *const end = media.data + media.size;
        const uint8_t *frame_start = media.data + client.frame_offset;
        uint8_t prefix_len = 0;
        frame_start = find_start_code(frame_start, end, prefix_len);
        if (frame_start == nullptr) {
            client.frame_offset = 0;
            return true;
        }

        const uint8_t *next_frame = nullptr;
        const uint8_t *scan = frame_start + prefix_len + 1U;
        while (scan < end) {
            uint8_t next_prefix_len = 0;
            const uint8_t *start = find_start_code(scan, end,
                                                   next_prefix_len);
            if (start == nullptr) {
                break;
            }
            const uint8_t *nal = start + next_prefix_len;
            if (nal < end && (nal[0] & 0x1f) == 9) {
                next_frame = start;
                break;
            }
            scan = nal + 1U;
        }
        const uint8_t *const frame_end = next_frame == nullptr ? end : next_frame;

        scan = frame_start;
        while (scan < frame_end) {
            uint8_t nal_prefix_len = 0;
            const uint8_t *start = find_start_code(scan, frame_end,
                                                   nal_prefix_len);
            if (start == nullptr || start + nal_prefix_len >= frame_end) {
                break;
            }
            const uint8_t *nal = start + nal_prefix_len;
            uint8_t next_prefix_len = 0;
            const uint8_t *next = find_start_code(nal + 1U, frame_end,
                                                   next_prefix_len);
            const uint8_t *nal_end = next == nullptr ? frame_end : next;
            if (!send_nal(client, nal, nal_end - nal,
                          next == nullptr)) {
                return false;
            }
            scan = nal_end;
        }

        client.frame_offset = next_frame == nullptr ? 0 :
                              next_frame - media.data;
        client.timestamp += 90000U / 30U;
        return true;
    }

    Media _media[2] {};
    Client _clients[MAX_CLIENTS] {};
    SocketAPM_native *_listener {};
    uint8_t _instance;
    uint16_t _port;
    bool _running {};
};

void MT11::update(const class Aircraft &aircraft)
{
    if (_rtsp_server == nullptr) {
        _rtsp_server = NEW_NOTHROW MT11_RTSPServer(aircraft.get_instance(),
                                                  _instance);
    }
    if (_rtsp_server != nullptr) {
        _rtsp_server->update(aircraft.get_wall_time_us());
    }
    MAVLinkGimbalv2::update(aircraft);
    MAVLinkCamV2::update(aircraft);
}

bool MT11::get_video_stream_information(
    uint8_t stream_id,
    mavlink_video_stream_information_t &info) const
{
    if (stream_id < 1 || stream_id > get_video_stream_count()) {
        return false;
    }

    const bool thermal = stream_id == 2;
    info.type = VIDEO_STREAM_TYPE_RTSP;
    info.flags = 0;
    if (_rtsp_server != nullptr && _rtsp_server->running()) {
        info.flags |= VIDEO_STREAM_STATUS_FLAGS_RUNNING;
    }
    if (thermal) {
        info.flags |= VIDEO_STREAM_STATUS_FLAGS_THERMAL;
    }
    info.framerate = 30.0f;
    info.resolution_h = thermal ? 1280 : 1920;
    info.resolution_v = thermal ? 720 : 1080;
    info.bitrate = 4096000;
    info.rotation = 0;
    info.hfov = thermal ? 24 : 88;
    info.encoding = VIDEO_STREAM_ENCODING_H264;
    strncpy_noterm(info.name, thermal ? "Thermal" : "Visible", sizeof(info.name));
    hal.util->snprintf(info.uri, sizeof(info.uri),
                       "rtsp://127.0.0.1:%u/video%u",
                       _rtsp_server == nullptr ? 8554U : _rtsp_server->port(),
                       (unsigned)stream_id);
    return true;
}

}  // namespace SITL

#endif  // AP_SIM_MT11_ENABLED
