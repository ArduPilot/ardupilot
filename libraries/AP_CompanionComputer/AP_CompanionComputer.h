#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include "AP_CompanionComputer_config.h"

class AP_CompanionComputer {
public:
    AP_CompanionComputer();

    /* Do not allow copies */
    AP_CompanionComputer(const AP_CompanionComputer &other) = delete;
    AP_CompanionComputer &operator=(const AP_CompanionComputer&) = delete;

    static AP_CompanionComputer *get_singleton() { return _singleton; }

    void init();
    void update();
    void send_data();

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_CompanionComputer *_singleton;

    AP_Int8 _enable;
    AP_Int8 _port_index;

    AP_HAL::UARTDriver *_uart;

    enum class RxState {
        WAITING_HEADER1,
        WAITING_HEADER2,
        WAITING_SOURCE,
        WAITING_TYPE,
        WAITING_LENGTH,
        RECEIVING_DATA
    } _rx_state;

    std::array<uint8_t, COMPANION_RECV_TOTAL_LENGTH> _rx_buffer;
    uint8_t _rx_count;
    uint32_t _rx_start_time;

    uint8_t _cmd_type;
    uint8_t _data_len;

    void process_received_data(uint8_t oneByte);
    void handle_valid_packet();
    uint8_t calculate_checksum(const uint8_t *data, uint8_t len) const;
    bool validate_packet() const;
};

namespace AP {
    AP_CompanionComputer &companioncomputer();
};
