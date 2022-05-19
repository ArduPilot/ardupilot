

#include "AP_ESC_APDHVPro.h"

AP_ESC_APDHVPro::AP_ESC_APDHVPro(AP_ESC &frontend) : AP_ESC_Backend(frontend) {
    port = frontend.hal.serial(frontend.serial_port_num);
    port->begin(frontend.serial_port_baud);
    // port->set_options(AP_HAL::UARTDriver::OPTION_PULLDOWN_RX);
}

bool AP_ESC_APDHVPro::update() {
    bool status = false;
    if (!port) {
        return status;
    }

    // // we expect at least 50ms idle between frames
    // bool frame_gap = (now - last_read_ms) > 10;

    uint32_t size = port->available();
    if (size >= ESC_PACKET_SIZE) {
        if (size > ESC_PACKET_SIZE * 2) {
            // port->discard_input();
        }
        else {
            int bytes_read = port->read(raw_buffer, size);

            if (read_ESC_telemetry_data(bytes_read)) {
                last_read_ms = AP_HAL::native_millis();
                copy_to_frontend();

                status = true;
            }
        }
    }

    return status;
}

bool AP_ESC_APDHVPro::read_ESC_telemetry_data(uint32_t bytes_read) {
    bool status = false;

    uint32_t count = 0;

    if (bytes_read < ESC_PACKET_SIZE) return false;
    else count = ((bytes_read < BUFFER_LOOP_SIZE) ? bytes_read : BUFFER_LOOP_SIZE);

    for (int i = ESC_PACKET_SIZE - 2; i < count; i++) {
        if (max_buffer[i] == 255 && max_buffer[i + 1] == 255) {
            for (int j = 0; j < ESC_PACKET_SIZE; j++) {
                raw_buffer[j] = max_buffer[i - ESC_PACKET_SIZE + j];
            }

            status = true;
            break;
        }
    }
    
    return status;
}

bool AP_ESC_APDHVPro::parse_ESC_telemetry_data() {
    bool status = false;

    int check_fletch = check_flectcher16();

    decoded.voltage = (uint16_t)((raw_buffer[1] << 8) + raw_buffer[0]);
    // Temperature values needs computation login implementation
    decoded.temperature = (uint16_t)((raw_buffer[3] << 8) + raw_buffer[2]);
    decoded.bus_current = (uint16_t)((raw_buffer[5] << 8) + raw_buffer[4]) / 12.5;
    decoded.reserved1 = (uint16_t)((raw_buffer[7] << 8) + raw_buffer[6]);
    decoded.rpm = (uint32_t)((raw_buffer[11] << 24) + (raw_buffer[10] << 16) + (raw_buffer[9] << 8) + raw_buffer[8]) / 2;
    decoded.input_duty = (uint16_t)((raw_buffer[13] << 8) + raw_buffer[12]) / 10;
    decoded.motor_duty = (uint16_t)((raw_buffer[15] << 8) + raw_buffer[14]) / 10;
    decoded.status = raw_buffer[16];
    decoded.reserved2 = raw_buffer[17];
    decoded.checksum = (uint16_t)((raw_buffer[19] << 8) + raw_buffer[18]);

    return status;
}

int AP_ESC_APDHVPro::check_flectcher16() {
    int fCCRC16;
    int i;
    int c0 = 0;
    int c1 = 0;

    // Calculate checksum intermediate bytesUInt16
    for (i = 0; i < 18; i++) { //Check only first 18 bytes, skip crc bytes
        c0 = (int)(c0 + ((int)raw_buffer[i])) % 255;
        c1 = (int)(c1 + c0) % 255;
    }
    // Assemble the 16-bit checksum value
    fCCRC16 = ( c1 << 8 ) | c0;
    return (int)fCCRC16;
}
