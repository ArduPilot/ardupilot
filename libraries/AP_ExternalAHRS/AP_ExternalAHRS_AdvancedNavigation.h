/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
   Support for serial connected Advanced Navigation devices.

   Website: https://www.advancednavigation.com/
   Advanced Navigation packet Protocol (ANPP): https://docs.advancednavigation.com/certus/ANPP/Advanced%20Navigation%20Packet.htm
*/

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_ADNAV_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_DAL/AP_DAL.h>

/*
    Advanced Navigation Packet Protocol (ANPP) Header

    https://docs.advancednavigation.com/certus/ANPP/AboutPacket.htm
*/
struct PACKED AN_PacketHeader
{
    uint8_t lrc;    // Header LRC (Longitudinal Redundancy Check)
    uint8_t id;     // Packet ID
    uint8_t length; // Packet Length
    uint16_t crc;   // Packet CRC (CRC16-CCITT)

    uint8_t calculate_lrc() const
    {
        // LRC = (PacketID + PacketLength + crc0 + crc1) ⊕ 0xFF + 1
        const uint8_t *bytes = reinterpret_cast<const uint8_t *>(this);

        return ((bytes[1] + bytes[2] + bytes[3] + bytes[4]) ^ 0xFF) + 1;
    }

    bool check_lrc() const
    {
        return lrc == calculate_lrc();
    }

    bool check_crc(uint8_t *buffer, size_t size, uint16_t index) const
    {
        // Packet CRC is a CRC16-CCITT with a starting value of 0xFFFF
        return crc == crc16_ccitt(&buffer[index], length, 0xFFFF);
    }

    void update(uint8_t header_id, uint8_t payload_length, uint8_t *buffer)
    {
        length = payload_length;

        crc = crc16_ccitt(buffer, payload_length, 0xFFFF);
        id = header_id;

        lrc = calculate_lrc();
    }
};

/*
    Advanced Navigation Packet Protocol (ANPP) Payload

    https://docs.advancednavigation.com/certus/ANPP/AboutPacket.htm
*/
struct PACKED AN_PacketPayload
{
public:
    static constexpr unsigned AN_MAXIMUM_PACKET_SIZE = 255;

    void update(uint8_t *buffer, uint8_t length)
    {
        memcpy(base, buffer, length);
    }

private:
    uint8_t base[AN_MAXIMUM_PACKET_SIZE];
};

/*
    Advanced Navigation Packet Protocol (ANPP) Packet

    https://docs.advancednavigation.com/certus/ANPP/AboutPacket.htm
*/
struct PACKED AN_Packet
{
public:
    AN_Packet() {};
    AN_Packet(uint8_t id, uint8_t *buffer, uint8_t length)
    {
        payload.update(buffer, length);
        header.update(id, length, buffer);
    }
    AN_Packet(AN_Packet *packet)
    {
        this->copy(packet);
    }

    ~AN_Packet() {};

    void copy(AN_Packet *packet)
    {
        this->header = packet->header;
        this->payload = packet->payload;
    }

    void load(uint8_t *buffer, uint8_t length)
    {
        memcpy(this->base(), buffer, length);
    }

    uint8_t *base()
    {
        return &header.lrc;
    }

    size_t size() const
    {
        return sizeof(AN_PacketHeader) + header.length;
    }

    uint8_t id()
    {
        return header.id;
    }

    AN_PacketHeader header;
    AN_PacketPayload payload;
};

/*
    Advanced Navigation Packet Protocol (ANPP) Decoder

    The class reads the available buffer from the provide UART and will then
    process its own buffer into the provided ANPP packet
*/
class AP_ExternalAHRS_AdvancedNavigation_Decoder
{
public:
    friend class AP_ExternalAHRS_AdvancedNavigation;

    bool decode(AN_Packet *packet);

    ssize_t read(AP_HAL::UARTDriver *uart);

    bool is_complete() const
    {
        return _complete;
    }

private:
    static constexpr unsigned AN_DECODE_BUFFER_SIZE = 2 * (sizeof(AN_Packet::payload) + sizeof(AN_Packet::header));

    uint8_t _buffer[AN_DECODE_BUFFER_SIZE];
    uint16_t _buffer_length{0};
    bool _complete{false};
};

/*
    Advanced Navigation External ExternalAHRS Interface
*/
class AP_ExternalAHRS_AdvancedNavigation : public AP_ExternalAHRS_backend
{
public:
    AP_ExternalAHRS_AdvancedNavigation(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    int8_t get_port(void) const override;
    const char *get_name() const override;
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    void update() override;

protected:
    uint8_t num_gps_sensors(void) const override
    {
        return 1;
    }

private:
    AP_ExternalAHRS_AdvancedNavigation_Decoder _decoder;

    AN_Packet packet;
    AN_Packet last_satellites_packet;
    AN_Packet last_velocity_standard_deviation_packet;
    AN_Packet last_raw_gnss_packet;

    AP_DAL &dal;

    uint16_t _current_rate;
    uint16_t _system_status;
    uint16_t _filter_status;

    bool _rates_set{false};
    bool _gnss_disable;

    AP_HAL::UARTDriver *_uart;
    HAL_Semaphore _sem;

    uint32_t _baudrate;
    int8_t _port_num;

    uint32_t _last_pkt_ms;
    uint32_t _last_state_pkt_ms;
    uint32_t _last_device_info_pkt_ms;
    uint32_t _last_pkt_rate_message_sent_ms;
    uint32_t _last_raw_gnss_pkt_ms;
    uint32_t _last_raw_sensors_pkt_ms;
    uint32_t _last_logged_state_pkt_ms;
    uint32_t _last_logged_raw_gnss_pkt_ms;
    uint32_t _last_logged_raw_sensors_pkt_ms;
    uint32_t _device_id{0};
    uint32_t _hardware_rev;

    void update_thread();

    bool send_packet(AN_Packet &packet);
    bool send_packet_timer_period(uint16_t timer_period);
    bool send_packet_request(uint8_t count, uint8_t id, ...);
    bool send_packet_period_request(uint8_t count, ...);
    bool set_filter_options(bool gnss_en, uint8_t vehicle_type, bool permanent = false);

    bool get_packets(void);
    bool configure_device(void);

    bool has_gnss(void) const;
    bool has_baro(void) const;
    bool has_compass(void) const;

    void handle_packet();
    void handle_system_state_packet();
    void handle_acknowledge_packet();
    void handle_device_info_packet();
    void handle_velocity_standard_deviation_packet();
    void handle_raw_sensors_packet();
    void handle_raw_gnss_packet();
    void handle_satellites_packet();
};

#endif // AP_EXTERNAL_AHRS_ADNAV_ENABLED