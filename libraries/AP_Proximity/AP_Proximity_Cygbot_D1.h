#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_CYGBOT_ENABLED

#include "AP_Proximity_Backend_Serial.h"

#define CYGBOT_MAX_MSG_SIZE            350
#define CYGBOT_PACKET_HEADER_0         0x5A
#define CYGBOT_PACKET_HEADER_1         0x77
#define CYGBOT_PACKET_HEADER_2         0xFF
#define CYGBOT_PAYLOAD_HEADER 		   0x01
#define CYGBOT_2D_START_ANGLE 		   -60.0f  // Starting 2-D horizontal angle of distances received in payload
#define CYGBOT_2D_ANGLE_STEP		   0.75f   // Angle step size of each distance received. Starts from CYGBOT_2D_START_ANGLE

#define CYGBOT_TIMEOUT_MS              500    // Driver will report "unhealthy" if valid sensor readings not received within this many ms
#define CYGBOT_INIT_TIMEOUT_MS         1000   // Timeout this many ms after init
#define CYGBOT_MAX_RANGE_M             7.0f   // max range of the sensor in meters
#define CYGBOT_MIN_RANGE_M             0.2f   // min range of the sensor in meters

class AP_Proximity_Cygbot_D1 : public AP_Proximity_Backend_Serial
{

public:

    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    // update the state of the sensor
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max_m() const override { return CYGBOT_MAX_RANGE_M; }
    float distance_min_m() const override { return CYGBOT_MIN_RANGE_M; }

private:

    // send message to the sensor to start streaming 2-D data
    void send_sensor_start();

    // read bytes from the sensor
    void read_sensor_data();

    // parse one byte from the sensor. Return false on error.
    bool parse_byte(uint8_t data);

    // parse payload, to pick out distances, and feed them to the correct faces
    void parse_payload();

    // Checksum
    uint8_t calc_checksum(uint8_t *buff, int buffSize);

    // reset all variables and flags
    void reset();

    // expected bytes from the sensor
    enum PacketList {
        Header1 = 0,
        Header2,
        Header3,
        Length1,
        Length2,
        Payload_Header,
        Payload_Data,
        CheckSum
    } _parse_state;

    struct {
        uint8_t payload_len_flags_low;          // low byte for payload size
        uint8_t payload_len_flags_high;         // high byte for payload size
        uint16_t payload_len;                   // latest message expected payload length
        uint16_t payload_counter;              // counter of the number of payload bytes received
        uint8_t payload[CYGBOT_MAX_MSG_SIZE];   // payload
    } _msg;

    bool _initialized;
    uint32_t _last_init_ms;                 // system time of last sensor init
    uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor

    AP_Proximity_Temp_Boundary _temp_boundary; // temporary boundary to store incoming payload

};

#endif // AP_PROXIMITY_CYGBOT_ENABLED
