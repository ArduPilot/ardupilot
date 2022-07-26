
#include "AP_Periph.h"
#include "AP_ESC_APDHVPro.h"


extern const AP_HAL::HAL &hal;

AP_ESC_APDHVPro::AP_ESC_APDHVPro(void){
    
}

void AP_ESC_APDHVPro::init(AP_HAL::UARTDriver *_uart) {
    // uart = _uart;
    // uart->begin(19200);
    // uart->set_options(AP_HAL::UARTDriver::OPTION_PULLDOWN_RX);

    port = _uart;
    port->begin(57600);

    hal.serial(0)->begin(115200, 128, 256);
    hal.serial(1)->begin(115200, 128, 256);
    hal.serial(2)->begin(115200, 128, 256);
    hal.serial(3)->begin(115200, 128, 256);
    hal.serial(4)->begin(115200, 128, 256);
    // port->set_options(AP_HAL::UARTDriver::OPTION_PULLDOWN_RX);
}

bool AP_ESC_APDHVPro::update() {
    bool status = false;

    if (!port) {
        return status;
    }

    uint32_t size = port->available();
    if (size >= ESC_PACKET_SIZE) {

        if (size > ESC_PACKET_SIZE * 2) {
            // port->discard_input();
        }
        else {
            int bytes_read = port->read(max_buffer, size);

            if (read_ESC_telemetry_data(bytes_read)) {
                last_read_ms = AP_HAL::native_millis();

                parse_ESC_telemetry_data();

                // static int debug_count1 = 0;
                // if (debug_count1++ > 25) {
                //     can_printf("APD ESC Telem : V:%d, T:%d, I:%d", (int)decoded.voltage, (int)decoded.temperature, (int)decoded.bus_current);
                //     can_printf("APD ESC Telem : R:%d, S:%d, C:%d", (int)decoded.rpm, (int)decoded.status, (int)decoded.checksum);

                //     debug_count1 = 0;
                // }

                status = true;
            }
        }
    }

    return status;
}

bool AP_ESC_APDHVPro::read_ESC_telemetry_data(uint32_t bytes_read) {
    bool status = false;

    // uint32_t count = 0;

    if (bytes_read < ESC_PACKET_SIZE)
        return status;
    // else count = ((bytes_read < BUFFER_LOOP_SIZE) ? bytes_read : BUFFER_LOOP_SIZE);

    // // static int debug_count = 0;
    // // if (debug_count++ > 25) {
    // //     can_printf("APD ESC Telem : update() %d %d %d %d %d %d %d %d %d %d %d", (int)max_buffer[0], (int)max_buffer[1], (int)max_buffer[2], (int)max_buffer[3], (int)max_buffer[4], (int)max_buffer[5], (int)max_buffer[6], (int)max_buffer[7], (int)max_buffer[8], (int)max_buffer[9], (int)max_buffer[10]);
    // //     can_printf("APD ESC Telem : update() %d %d %d %d %d %d %d %d %d %d %d", (int)max_buffer[11], (int)max_buffer[12], (int)max_buffer[13], (int)max_buffer[14], (int)max_buffer[15], (int)max_buffer[16], (int)max_buffer[17], (int)max_buffer[18], (int)max_buffer[19], (int)max_buffer[20], (int)max_buffer[21]);

    // //     debug_count = 0;
    // // }

    // for (int i = ESC_PACKET_SIZE - 2; i < count; i++) {
    //     if (max_buffer[i] == 255 && max_buffer[i + 1] == 255) {
    //         for (int j = 0; j < ESC_PACKET_SIZE; j++) {
    //             raw_buffer[j] = max_buffer[i - ESC_PACKET_SIZE + j + 2];
    //         }

    //         status = true;
    //         break;
    //     }
    // }

    for (int i = 0; i < ESC_PACKET_SIZE; i++) {
        raw_buffer[i] = max_buffer[i];
    }

    status = true;

    // static int debug_count = 0;
    // if (debug_count++ > 25) {
    //     can_printf("APD Telem : %d %d %d %d %d %d %d %d %d %d %d", (int)raw_buffer[0], (int)raw_buffer[1], (int)raw_buffer[2], (int)raw_buffer[3], (int)raw_buffer[4], (int)raw_buffer[5], (int)raw_buffer[6], (int)raw_buffer[7], (int)raw_buffer[8], (int)raw_buffer[9], (int)raw_buffer[10]);
    //     can_printf("APD Telem : %d %d %d %d %d %d %d %d %d %d %d", (int)raw_buffer[11], (int)raw_buffer[12], (int)raw_buffer[13], (int)raw_buffer[14], (int)raw_buffer[15], (int)raw_buffer[16], (int)raw_buffer[17], (int)raw_buffer[18], (int)raw_buffer[19], (int)raw_buffer[20], (int)raw_buffer[21]);

    //     debug_count = 0;
    // }

    return status;
}

#define SERIESRESISTOR 10000
#define NOMINAL_RESISTANCE 10000
#define NOMINAL_TEMPERATURE 25
#define BCOEFFICIENT 3455

#define TEMPERATURE_MAX_RESOLUTION 4096


bool AP_ESC_APDHVPro::parse_ESC_telemetry_data() {
    bool status = false;

    int check_fletch = check_flectcher16();

    decoded.voltage = (uint16_t)(((raw_buffer[1] << 8) + raw_buffer[0])/100);
    // Temperature values needs computation later implementation
    float temp1 = (TEMPERATURE_MAX_RESOLUTION/(float)((raw_buffer[3] << 8) + raw_buffer[2])) - 1;
    float temperature = (SERIESRESISTOR / temp1) / NOMINAL_RESISTANCE;

    decoded.temperature = (uint16_t)((raw_buffer[3] << 8) + raw_buffer[2]);
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
    // else can_printf("APD Telem checksum: %d, %d", check_fletch, (int)decoded.checksum);

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
