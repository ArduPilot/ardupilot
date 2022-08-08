
#include "AP_Periph.h"
#include "AP_ESC_APDHVPro.h"


extern const AP_HAL::HAL &hal;

AP_ESC_APDHVPro::AP_ESC_APDHVPro(void){
    
}

void AP_ESC_APDHVPro::init(AP_HAL::UARTDriver *_uart) {

    port = _uart;
    port->begin(57600);

    hal.serial(1)->begin(115200, 128, 256);
}

bool AP_ESC_APDHVPro::update() {
    bool status = false;

    if (!port) {
        return status;
    }

    uint32_t size = port->available();
    if (size >= ESC_PACKET_SIZE) {

        if (size > ESC_PACKET_SIZE * 2) {
        }
        else {
            int bytes_read = port->read(max_buffer, size);

            if (read_ESC_telemetry_data(bytes_read)) {
                last_read_ms = AP_HAL::native_millis();

                parse_ESC_telemetry_data();

                status = true;
            }
        }
    }

    return status;
}

bool AP_ESC_APDHVPro::read_ESC_telemetry_data(uint32_t bytes_read) {
    bool status = false;

    if (bytes_read < ESC_PACKET_SIZE)
        return status;

    for (int i = 0; i < ESC_PACKET_SIZE; i++) {
        raw_buffer[i] = max_buffer[i];
    }

    status = true;

    return status;
}


bool AP_ESC_APDHVPro::parse_ESC_telemetry_data() {
    bool status = false;

    int check_fletch = check_flectcher16();

    decoded.voltage = (uint16_t)(((raw_buffer[1] << 8) + raw_buffer[0])/100);
    
    float rntc = (TEMPERATURE_MAX_RESOLUTION/(float)((raw_buffer[3] << 8) + raw_buffer[2])) - 1;
    float temperature1 = (logF((SERIESRESISTOR / rntc) / (float)NOMINAL_RESISTANCE)) / BCOEFFICIENT;
    decoded.temperature = (uint16_t)(1 / ((1 / ((float)NOMINAL_RESISTANCE + (float)273.15)) + temperature1)) - (float)273.15;

    decoded.bus_current = (uint16_t)(((raw_buffer[5] << 8) + raw_buffer[4]) / CURRENT_COEFFICIENT);
    decoded.reserved1 = (uint16_t)((raw_buffer[7] << 8) + raw_buffer[6]);
    decoded.rpm = ((uint32_t)((raw_buffer[11] << 24) + (raw_buffer[10] << 16) + (raw_buffer[9] << 8) + raw_buffer[8])) / POLECOUNT;
    decoded.input_duty = (uint16_t)((raw_buffer[13] << 8) + raw_buffer[12]) / 10;
    decoded.motor_duty = (uint16_t)((raw_buffer[15] << 8) + raw_buffer[14]) / 10;
    decoded.status = raw_buffer[16];
    decoded.reserved2 = raw_buffer[17];
    decoded.checksum = (uint16_t)((raw_buffer[19] << 8) + raw_buffer[18]);

    // Currently, checksum code is not mandated
    if (check_fletch == (int)decoded.checksum) { 
        status = true;
    }

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
