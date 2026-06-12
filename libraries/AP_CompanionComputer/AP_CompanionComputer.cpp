#include "AP_CompanionComputer.h"
#include <AP_SerialManager/AP_SerialManager.h>

const AP_Param::GroupInfo AP_CompanionComputer::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable Companion computer
    // @Description: Enable communication with companion computer
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_CompanionComputer, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PORT
    // @DisplayName: instance of companion computer Serial Port
    // @Description: The nth serial port instance found starting from serial0 that uses serial protocol 50
    // @Values: 0:instance0, 1:instance1
    // @User: Advanced
    AP_GROUPINFO("PORT", 2, AP_CompanionComputer, _port_index, 0),

    AP_GROUPEND
};

AP_CompanionComputer::AP_CompanionComputer() :
    _rx_state(RxState::WAITING_HEADER1),
    _rx_count(0),
    _uart(nullptr)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
    _rx_buffer.fill(0);
}

void AP_CompanionComputer::init()
{
    if (!_enable) {
        return;
    }

    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_2CC, _port_index.get());
    if (_uart != nullptr) {
        _uart->begin(115200, 512, 128);
    }
}

void AP_CompanionComputer::update()
{
    if (!_enable || _uart == nullptr) {
        return;
    }

    while (_uart->available() > 0) {
        uint8_t byte;
        if (_uart->read(&byte, 1)) {
            process_received_data(byte);
        }
    }
}

void AP_CompanionComputer::process_received_data(uint8_t oneByte)
{
    const uint32_t now = AP_HAL::millis();

    switch (_rx_state) {
    case RxState::WAITING_HEADER1:
        if (oneByte == COMPANION_FRAME_HEADER1) {
            _rx_state = RxState::WAITING_HEADER2;
            _rx_start_time = now;
            _rx_count = 0;
        }
        break;

    case RxState::WAITING_HEADER2:
        if (oneByte == COMPANION_FRAME_HEADER2) {
            _rx_state = RxState::WAITING_SOURCE;
            _rx_buffer[_rx_count++] = COMPANION_FRAME_HEADER1;
            _rx_buffer[_rx_count++] = COMPANION_FRAME_HEADER2;
        } else {
            _rx_state = RxState::WAITING_HEADER1;
        }
        break;

    case RxState::WAITING_SOURCE:
        if (oneByte != COMPANION_CMD_SOURCE_NCU) {
            _rx_state = RxState::WAITING_HEADER1;
            _rx_count = 0;
            break;
        }
        _rx_state = RxState::WAITING_TYPE;
        _rx_buffer[_rx_count++] = oneByte;
        break;

    case RxState::WAITING_TYPE:
        if (oneByte <= NCU_CMD_POSITION) {
            _rx_state = RxState::WAITING_LENGTH;
            _rx_buffer[_rx_count++] = oneByte;
            _cmd_type = oneByte;
        } else {
            _rx_state = RxState::WAITING_HEADER1;
        }
        break;

    case RxState::WAITING_LENGTH:
        if (oneByte > NCU_RX_MAX_DATA_LEN) {
            _rx_state = RxState::WAITING_HEADER1;
            _rx_count = 0;
            break;
        }
        _rx_state = RxState::RECEIVING_DATA;
        _data_len = oneByte;
        _rx_buffer[_rx_count++] = oneByte;
        break;

    case RxState::RECEIVING_DATA:
        if (now - _rx_start_time > PACKET_TIMEOUT_MS) {
            _rx_state = RxState::WAITING_HEADER1;
            _rx_count = 0;
            break;
        }

        if (_rx_count >= _rx_buffer.size()) {
            _rx_state = RxState::WAITING_HEADER1;
            _rx_count = 0;
            break;
        }

        _rx_buffer[_rx_count++] = oneByte;

        if (_rx_count >= (_data_len + 7)) {
            if (validate_packet()) {
                handle_valid_packet();
            }
            _rx_state = RxState::WAITING_HEADER1;
            _rx_count = 0;
        }
        break;
    }
}

void AP_CompanionComputer::handle_valid_packet()
{
    switch (_cmd_type) {
    case NCU_CMD_SPEED_CTRL:
    case NCU_CMD_TURN:
    case NCU_CMD_PARAM_WRITE:
    case NCU_CMD_PARAM_READ:
    case NCU_CMD_SYSTEM_CTRL:
    case NCU_CMD_POSITION:
        break;
    default:
        break;
    }
}

bool AP_CompanionComputer::validate_packet() const
{
    const uint8_t calculated_checksum = calculate_checksum(_rx_buffer.data(), _rx_count - 2);
    return (calculated_checksum == _rx_buffer[_rx_count - 2]) &&
           (_rx_buffer[_rx_count - 1] == COMPANION_END_SIGN);
}

uint8_t AP_CompanionComputer::calculate_checksum(const uint8_t *data, uint8_t len) const
{
    uint8_t sum = 0;
    for (uint8_t i = 2; i < len; i++) {
        sum += data[i];
    }
    return sum & 0xFF;
}

void AP_CompanionComputer::send_data()
{
}

AP_CompanionComputer *AP_CompanionComputer::_singleton;

namespace AP {

AP_CompanionComputer &companioncomputer()
{
    return *AP_CompanionComputer::get_singleton();
}

}
