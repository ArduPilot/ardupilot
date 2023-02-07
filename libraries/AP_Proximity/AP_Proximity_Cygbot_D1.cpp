#include "AP_Proximity_Cygbot_D1.h"

#if HAL_PROXIMITY_ENABLED && AP_PROXIMITY_CYGBOT_ENABLED

// update the state of the sensor
void AP_Proximity_Cygbot_D1::update()
{
    if (!_initialized) {
        send_sensor_start();
        _temp_boundary.reset();
        _initialized = true;
        _last_init_ms = AP_HAL::millis();
    }

    if ((AP_HAL::millis() - _last_init_ms) < CYGBOT_INIT_TIMEOUT_MS) {
        // just initialized
        set_status(AP_Proximity::Status::NoData);
        return;
    }

    // read data
    read_sensor_data();

    if (AP_HAL::millis() - _last_distance_received_ms < CYGBOT_TIMEOUT_MS) {
        set_status(AP_Proximity::Status::Good);
    } else {
        // long time since we received any valid sensor data
        // try sending the sensor the "send data" message
        _initialized = false;
        set_status(AP_Proximity::Status::NoData);
    }
}

// send message to the sensor to start streaming 2-D data
void AP_Proximity_Cygbot_D1::send_sensor_start()
{
    // this message corresponds to "start message"
    const uint8_t packet_start_2d[8] = { CYGBOT_PACKET_HEADER_0, CYGBOT_PACKET_HEADER_1, CYGBOT_PACKET_HEADER_2, 0x02, 0x00, 0x01, 0x00, 0x03 };
    _uart->write(packet_start_2d, 8);
}

void AP_Proximity_Cygbot_D1::read_sensor_data()
{
    uint32_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        uint8_t byte = _uart->read();
        if (!parse_byte(byte)) {
            // reset
            reset();
        }
    }
}

// parse one byte from the sensor. Return false on error.
// Message format is: header1 + header2 + header3 + length1 + length2 + PayloadCommand + checksum
bool AP_Proximity_Cygbot_D1::parse_byte(uint8_t data)
{
    switch (_parse_state) {
    case Header1:
        if (data == CYGBOT_PACKET_HEADER_0) {
            _parse_state = Header2;
            return true;
        }

        return false;

    case Header2:
        if (data == CYGBOT_PACKET_HEADER_1) {
            _parse_state = Header3;
            return true;
        }
        return false;

    case Header3:
        if (data == CYGBOT_PACKET_HEADER_2) {
            _parse_state = Length1;
            return true;
        }
        return false;

    case Length1:
        _msg.payload_len_flags_low = data;
        _parse_state = Length2;
        return true;

    case Length2:
        _msg.payload_len_flags_high = data;
        _msg.payload_len = UINT16_VALUE(data, _msg.payload_len_flags_low);
        if (_msg.payload_len > CYGBOT_MAX_MSG_SIZE) {
            return false;
        }
        _parse_state = Payload_Header;
        return true;

    case Payload_Header:
        if (data == CYGBOT_PAYLOAD_HEADER) {
            _parse_state = Payload_Data;
            _msg.payload_counter = 1;
            _msg.payload[_msg.payload_counter] = data;
            return true;
        }
        return false;

    case Payload_Data:
        if (_msg.payload_counter < (_msg.payload_len)) {
            _msg.payload_counter++;
            _msg.payload[_msg.payload_counter] = data;
            return true;
        }
        _parse_state = CheckSum;
        FALLTHROUGH;

    case CheckSum: {
        const uint8_t checksum_num = calc_checksum(_msg.payload, _msg.payload_len);
        if (data != checksum_num) {
            return false;
        }
        // checksum is valid, parse payload
        _last_distance_received_ms = AP_HAL::millis();
        parse_payload();
        _temp_boundary.update_3D_boundary(state.instance, frontend.boundary);
        reset();
        return true;
    }
    break;

    default:
        return false;
    }

    return false;
}

// parse payload, to pick out distances, and feed them to the correct faces
void AP_Proximity_Cygbot_D1::parse_payload()
{
    // current horizontal angle in the payload
    float sampled_angle = CYGBOT_2D_START_ANGLE;

    // start from second byte as first byte is part of the header
    for (uint16_t i = 2; i < _msg.payload_len; i += 2) {
        const float corrected_angle = correct_angle_for_orientation(sampled_angle);
        const uint16_t distance_mm = UINT16_VALUE(_msg.payload[i], _msg.payload[i+1]);
        float distance_m = distance_mm * 0.001f;
        if (distance_m > distance_min() && distance_m < distance_max()) {
            if (ignore_reading(corrected_angle, distance_m)) {
                // ignore this angle
                sampled_angle += CYGBOT_2D_ANGLE_STEP;
                continue;
            }
            // convert angle to face
            const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(corrected_angle);

            // push face to temp boundary
            _temp_boundary.add_distance(face, corrected_angle, distance_m);
            // push to OA_DB
            database_push(corrected_angle, distance_m);
        }
        // increment sampled angle
        sampled_angle += CYGBOT_2D_ANGLE_STEP;
    }
}

// Checksum
uint8_t AP_Proximity_Cygbot_D1::calc_checksum(uint8_t *buff, int buffSize)
{
    uint8_t check_sum_num = 0;
    check_sum_num ^= _msg.payload_len_flags_high;
    check_sum_num ^= _msg.payload_len_flags_low;
    for (uint16_t i = 0; i <= buffSize; i++) {
        check_sum_num ^= buff[i];
    }
    return check_sum_num;
}

// reset all variables and flags
void AP_Proximity_Cygbot_D1::reset()
{
    _parse_state = Header1;
    _msg.payload_counter = 0;
    _msg.payload_len = 0;
    _temp_boundary.reset();
}

#endif // HAL_PROXIMITY_ENABLED
