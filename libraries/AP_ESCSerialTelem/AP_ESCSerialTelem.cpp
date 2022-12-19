/*
  ESC Telemetry for Hobbywing Pro 80A HV ESC.

  This protocol only allows for one ESC per UART RX line, so using a
  CAN node per ESC works well.
 */
#include "AP_ESCSerialTelem.h"
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#if AP_ESC_SERIAL_TELEM_ENABLED

extern const AP_HAL::HAL& hal;

#define TELEM_HEADER 0x9B
#define TELEM_LEN    0x16

// constructor
AP_ESCSerialTelem::AP_ESCSerialTelem(void)
{
}

void AP_ESCSerialTelem::init(AP_HAL::UARTDriver *_uart)
{
    uart = _uart;
    uart->begin(19200);
    uart->set_options(AP_HAL::UARTDriver::OPTION_PULLDOWN_RX);
}

/*
  update ESC telemetry
 */
bool AP_ESCSerialTelem::update()
{
    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }

    // we expect at least 50ms idle between frames
    uint32_t now = AP_HAL::native_millis();
    bool frame_gap = (now - last_read_ms) > 10;

    last_read_ms = now;

    // don't read too much in one loop to prevent too high CPU load
    if (n > 500) {
        n = 500;
    }
    if (len == 0 && !frame_gap) {
        uart->discard_input();
        return false;
    }

    if (frame_gap) {
        len = 0;
    }

    bool ret = false;

    while (n--) {
        uint8_t b = uart->read();
        //hal.console->printf("t=%u 0x%02x\n", now, b);
        if (len == 0 && b != TELEM_HEADER) {
            continue;
        }
        if (len == 1 && b != TELEM_LEN) {
            continue;
        }
        uint8_t *buf = (uint8_t *)&pkt;
        buf[len++] = b;
        if (len == sizeof(pkt)) {
            ret = parse_packet();
            len = 0;
        }
    }
    return ret;
}

static uint16_t calc_crc(const uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0;
    while (len--) {
        crc += *buf++;
    }
    return crc;
}

static const struct {
    uint8_t adc_temp;
    uint8_t temp_C;
} temp_table[] = {
    { 241, 	0}, 	{ 240, 	1}, 	{ 239, 	2}, 	{ 238, 	3}, 	{ 237, 	4}, 	{ 236, 	5}, 	{ 235, 	6}, 	{ 234, 	7}, 	{ 233, 	8}, 	{ 232, 	9},
    { 231, 	10}, 	{ 230, 	11}, 	{ 229, 	12}, 	{ 228, 	13}, 	{ 227, 	14}, 	{ 226, 	15}, 	{ 224, 	16}, 	{ 223, 	17}, 	{ 222, 	18}, 	{ 220, 	19},
    { 219, 	20}, 	{ 217, 	21}, 	{ 216, 	22}, 	{ 214, 	23}, 	{ 213, 	24}, 	{ 211, 	25}, 	{ 209, 	26}, 	{ 208, 	27}, 	{ 206, 	28}, 	{ 204, 	29},
    { 202, 	30}, 	{ 201, 	31}, 	{ 199, 	32}, 	{ 197, 	33}, 	{ 195, 	34}, 	{ 193, 	35}, 	{ 191, 	36}, 	{ 189, 	37}, 	{ 187, 	38}, 	{ 185, 	39},
    { 183, 	40}, 	{ 181, 	41}, 	{ 179, 	42}, 	{ 177, 	43}, 	{ 174, 	44}, 	{ 172, 	45}, 	{ 170, 	46}, 	{ 168, 	47}, 	{ 166, 	48}, 	{ 164, 	49},
    { 161, 	50}, 	{ 159, 	51}, 	{ 157, 	52}, 	{ 154, 	53}, 	{ 152, 	54}, 	{ 150, 	55}, 	{ 148, 	56}, 	{ 146, 	57}, 	{ 143, 	58}, 	{ 141, 	59},
    { 139, 	60}, 	{ 136, 	61}, 	{ 134, 	62}, 	{ 132, 	63}, 	{ 130, 	64}, 	{ 128, 	65}, 	{ 125, 	66}, 	{ 123, 	67}, 	{ 121, 	68}, 	{ 119, 	69},
    { 117, 	70}, 	{ 115, 	71}, 	{ 113, 	72}, 	{ 111, 	73}, 	{ 109, 	74}, 	{ 106, 	75}, 	{ 105, 	76}, 	{ 103, 	77}, 	{ 101, 	78}, 	{ 99, 	79},
    { 97, 	80}, 	{ 95, 	81}, 	{ 93, 	82}, 	{ 91, 	83}, 	{ 90, 	84}, 	{ 88, 	85}, 	{ 85, 	86}, 	{ 84, 	87}, 	{ 82, 	88}, 	{ 81, 	89},
    { 79, 	90}, 	{ 77, 	91}, 	{ 76, 	92}, 	{ 74, 	93}, 	{ 73, 	94}, 	{ 72, 	95}, 	{ 69, 	96}, 	{ 68, 	97}, 	{ 66, 	98}, 	{ 65, 	99},
    { 64, 	100}, 	{ 62, 	101}, 	{ 62, 	102}, 	{ 61, 	103}, 	{ 59, 	104}, 	{ 58, 	105}, 	{ 56, 	106}, 	{ 54, 	107}, 	{ 54, 	108}, 	{ 53, 	109},
    { 51, 	110}, 	{ 51, 	111}, 	{ 50, 	112}, 	{ 48, 	113}, 	{ 48, 	114}, 	{ 46, 	115}, 	{ 46, 	116}, 	{ 44, 	117}, 	{ 43, 	118}, 	{ 43, 	119},
    { 41, 	120}, 	{ 41, 	121}, 	{ 39, 	122}, 	{ 39, 	123}, 	{ 39, 	124}, 	{ 37, 	125}, 	{ 37, 	126}, 	{ 35, 	127}, 	{ 35, 	128}, 	{ 33, 	129},
};

uint8_t AP_ESCSerialTelem::temperature_decode(uint8_t temp_raw) const
{
    for (uint8_t i=0; i<ARRAY_SIZE(temp_table); i++) {
        if (temp_table[i].adc_temp <= temp_raw) {
            return temp_table[i].temp_C;
        }
    }
    return 130U;
}

/*
  parse packet
 */
bool AP_ESCSerialTelem::parse_packet(void)
{
    uint16_t crc = calc_crc((uint8_t *)&pkt, sizeof(pkt)-2);
    if (crc != pkt.crc) {
        return false;
    }

    decoded.counter = be32toh(pkt.counter);
    decoded.throttle_req = be16toh(pkt.throttle_req);
    decoded.throttle = be16toh(pkt.throttle);
    decoded.rpm = be16toh(pkt.rpm) * 5.0 / 7.0; // scale from eRPM to RPM
    decoded.voltage = be16toh(pkt.voltage) * 0.1;
    decoded.phase_current = int16_t(be16toh(pkt.phase_current)) * 0.01;
    decoded.current = int16_t(be16toh(pkt.current)) * 0.01;
    decoded.mos_temperature = temperature_decode(pkt.mos_temperature);
    decoded.cap_temperature = temperature_decode(pkt.cap_temperature);
    decoded.status = be16toh(pkt.status);
    if (decoded.status != 0) {
        decoded.error_count++;
    }

    return true;
}

#if AP_ESC_SERIAL_TELEM_ESC_TELEM_BACKEND_ENABLED
const AP_Param::GroupInfo AP_ESCSerialTelem_ESCTelem_Backend::var_info[] = {
    // @Param: MASK
    // @DisplayName: HobbyWingESC Channel Bitmask
    // @Description: Mask of which channels are returning data on serial ports
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MASK",  1, AP_ESCSerialTelem_ESCTelem_Backend, channel_mask, 0),

    AP_GROUPEND
};

// constructor
AP_ESCSerialTelem_ESCTelem_Backend::AP_ESCSerialTelem_ESCTelem_Backend(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_ESCSerialTelem_ESCTelem_Backend::init()
{
    for (uint8_t i=0; i<32 && num_backends<ARRAY_SIZE(backends); i++) {
        // work out our servo channel offset.  First instance of this
        // class gets the channel corresponding to the first bit set in
        // the mask etc
        if (!((uint32_t)channel_mask & (1U<<i))) {
            continue;
        }
        // see if there's another serial port configured for the correct protocol:
        AP_HAL::UARTDriver *uart { AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_HobbyWingESC, num_backends) };
        if (uart == nullptr) {
            break;
        }
        backends[num_backends] = new AP_ESCSerialTelem();
        if (backends[num_backends] == nullptr) {
            break;
        }
        backends[num_backends]->init(uart);
        servo_channel[num_backends++] = i+1;
    }
}

void AP_ESCSerialTelem_ESCTelem_Backend::update_telemetry()
{
    for (uint8_t i=0; i<num_backends; i++) {
        AP_ESCSerialTelem &backend = *backends[i];
        if (!backend.update()) {
            continue;
        }
        const auto &decoded = backend.get_telem();
        update_rpm(servo_channel[i], decoded.rpm, 0);

        const TelemetryData t {
            .temperature_cdeg = int16_t(decoded.mos_temperature * 100),
            .voltage = float(decoded.voltage) * 0.01f,
            .current = float(decoded.current) * 0.01f,
        };
        update_telem_data(servo_channel[i], t,
                          AP_ESC_Telem_Backend::TelemetryType::CURRENT
                          | AP_ESC_Telem_Backend::TelemetryType::VOLTAGE
                          | AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
    }
}
#endif  // AP_ESC_SERIAL_TELEM_ESC_TELEM_ENABLED

#endif  // AP_ESC_SERIAL_TELEM_ENABLED

