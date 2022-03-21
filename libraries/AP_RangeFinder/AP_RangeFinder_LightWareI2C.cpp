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
#include "AP_RangeFinder_LightWareI2C.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

#define LIGHTWARE_DISTANCE_READ_REG 0
#define LIGHTWARE_LOST_SIGNAL_TIMEOUT_READ_REG 22
#define LIGHTWARE_LOST_SIGNAL_TIMEOUT_WRITE_REG 23
#define LIGHTWARE_TIMEOUT_REG_DESIRED_VALUE 20      // number of lost signal confirmations for legacy protocol only

#define LIGHTWARE_OUT_OF_RANGE_ADD_CM   100

static const size_t lx20_max_reply_len_bytes = 32;
static const size_t lx20_max_expected_stream_reply_len_bytes = 14;

#define stream_the_median_distance_to_the_first_return "ldf,0"
#define stream_the_raw_distance_to_the_first_return    "ldf,1"
#define stream_the_signal_strength_first_return        "lhf"
#define stream_the_raw_distance_to_the_last_return     "ldl,1"
#define stream_the_signal_strength_last_return         "lhl"
#define stream_the_level_of_background_noise           "ln"

/* Data streams from the LiDAR can include any sf20 LiDAR measurement.
 * A request to stream the desired measurement is made on a 20Hz basis and
 * on the next 20Hz service 50ms later, the result is read and a streaming
 * request is made for the next desired measurement in the sequence.
 * Results are generally available from the LiDAR within 10mS of request.
 */
#define STREAM1_VAL stream_the_raw_distance_to_the_last_return
#define STREAM2_VAL stream_the_signal_strength_last_return
#define STREAM3_VAL stream_the_raw_distance_to_the_first_return
#define STREAM4_VAL stream_the_signal_strength_first_return
#define STREAM5_VAL stream_the_level_of_background_noise
static const char *parse_stream_id[NUM_SF20_DATA_STREAMS] = { // List of stream identifier strings. Must match init_stream_id[].
    STREAM1_VAL ":"
};

static const char *init_stream_id[NUM_SF20_DATA_STREAMS] = {// List of stream initialization strings. Must match parse_stream_id[].
    "$1" STREAM1_VAL "\r\n"
};

static const uint8_t streamSequence[] = { 0 }; // List of 0 based stream Ids that determine the LiDAR values collected.

static const uint8_t numStreamSequenceIndexes = sizeof(streamSequence)/sizeof(streamSequence[0]);

AP_RangeFinder_LightWareI2C::AP_RangeFinder_LightWareI2C(RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params,
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_RangeFinder_Backend(_state, _params)
    , _dev(std::move(dev)) {}

/*
   Detects if a Lightware rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_LightWareI2C::detect(RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params,
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_LightWareI2C *sensor
        = new AP_RangeFinder_LightWareI2C(_state, _params, std::move(dev));

    if (!sensor) {
        return nullptr;
    }

    WITH_SEMAPHORE(sensor->_dev->get_semaphore());
    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

/**
 * Wrapper function over #transfer() to write a sequence of bytes to
 * device. No values are read.
 */
bool AP_RangeFinder_LightWareI2C::write_bytes(uint8_t *write_buf_u8, uint32_t len_u8)
{
    return _dev->transfer(write_buf_u8, len_u8, NULL, 0);
}

/**
 * Disables "address tagging" in the sf20 response packets.
 */
void AP_RangeFinder_LightWareI2C::sf20_disable_address_tagging()
{
    sf20_send_and_expect("#CT,0\r\n", "ct:0");
}

/*
  send a native command and check for an expected reply
 */
bool AP_RangeFinder_LightWareI2C::sf20_send_and_expect(const char* send_msg, const char* expected_reply)
{
    const size_t expected_reply_len = strlen(expected_reply);
    uint8_t rx_bytes[expected_reply_len + 1];
    memset(rx_bytes, 0, sizeof(rx_bytes));

    if ((expected_reply_len > lx20_max_reply_len_bytes) ||
        (expected_reply_len < 2)) {
        return false;
    }

    if (!write_bytes((uint8_t*)send_msg,
                     strlen(send_msg))) {
        return false;
    }

    if (!sf20_wait_on_reply(rx_bytes)) {
        return false;
    }

    if ((rx_bytes[0] != expected_reply[0]) ||
        (rx_bytes[1] != expected_reply[1]) ) {
        return false;
    }

    for (uint8_t i=0; i<10; i++) {
        if (_dev->read(rx_bytes, expected_reply_len)) {
            break;
        }
        // give a bit of time for the remaining bytes to be available
        hal.scheduler->delay(1);
    }

    return memcmp(rx_bytes, expected_reply, expected_reply_len) == 0;
}

/*
  send a native command and fill a reply into a buffer. Used for
  version string
 */
void AP_RangeFinder_LightWareI2C::sf20_get_version(const char* send_msg, const char *reply_prefix, char reply[15])
{
    const size_t expected_reply_len = 16;
    uint8_t rx_bytes[expected_reply_len + 1];
    memset(rx_bytes, 0, sizeof(rx_bytes));

    if (!write_bytes((uint8_t*)send_msg, strlen(send_msg))) {
        return;
    }

    if (!sf20_wait_on_reply(rx_bytes)) {
        return;
    }

    if ((rx_bytes[0] != uint8_t(reply_prefix[0])) ||
        (rx_bytes[1] != uint8_t(reply_prefix[1])) ) {
        return;
    }

    for (uint8_t i=0; i<10; i++) {
        if (_dev->read(rx_bytes, expected_reply_len)) {
            break;
        }
        // give a bit of time for the remaining bytes to be available
        hal.scheduler->delay(1);
    }
    memcpy(reply, &rx_bytes[2], 14);
}

/* Driver first attempts to initialize the sf20.
 * If for any reason this fails, the driver attempts to initialize the legacy LightWare LiDAR.
 * If this fails, the driver returns false indicating no LightWare LiDAR is present.
 */
bool AP_RangeFinder_LightWareI2C::init()
{
    if (sf20_init()) {
        DEV_PRINTF("Found SF20 native Lidar\n");
        return true;
    }
    if (legacy_init()) {
        DEV_PRINTF("Found SF20 legacy Lidar\n");
        return true;
    }
    DEV_PRINTF("SF20 not found\n");
    return false;
}

/*
  initialise lidar using legacy 16 bit protocol
 */
bool AP_RangeFinder_LightWareI2C::legacy_init()
{
    union {
        be16_t be16_val;
        uint8_t bytes[2];
    } timeout;

    // Retrieve lost signal timeout register
    const uint8_t read_reg = LIGHTWARE_LOST_SIGNAL_TIMEOUT_READ_REG;
    if (!_dev->transfer(&read_reg, 1, timeout.bytes, 2)) {
        return false;
    }

    // Check lost signal timeout register against desired value and write it if it does not match
    if (be16toh(timeout.be16_val) != LIGHTWARE_TIMEOUT_REG_DESIRED_VALUE) {
        timeout.be16_val = htobe16(LIGHTWARE_TIMEOUT_REG_DESIRED_VALUE);
        const uint8_t send_buf[3] = {LIGHTWARE_LOST_SIGNAL_TIMEOUT_WRITE_REG, timeout.bytes[0], timeout.bytes[1]};
        if (!_dev->transfer(send_buf, sizeof(send_buf), nullptr, 0)) {
            return false;
        }
    }

    // call timer() at 20Hz
    _dev->register_periodic_callback(50000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_LightWareI2C::legacy_timer, void));

    return true;
}

/*
  initialise using newer text based protocol
 */
bool AP_RangeFinder_LightWareI2C::sf20_init()
{
    // version strings for reporting
    char version[15] {};

    sf20_get_version("?P\r\n", "p:", version);

    if (version[0]) {
        DEV_PRINTF("SF20 Lidar version %s\n", version);
    }

    // Makes sure that "address tagging" is turned off.
    // Address tagging starts every response with "0x66".
    // Turns off Address Tagging just in case it was previously left on in the non-volatile configuration.
    sf20_disable_address_tagging();
    // Disconnect the servo (if applicable)
    sf20_send_and_expect("#SC,0\r\n", "sc:0");

    // Change the power consumption:
    // 0 = power off
    // 1 = power on
    // As of 7/10/17 sw and fw version 1.0 the "#E,1" command does not seem to be supported.
    // When it is supported the expected response would be "e:1".

    // Changes the number of lost signal confirmations: 1 [1..250].
    if (!sf20_send_and_expect("#LC,20\r\n", "lc:20")) {
        return false;
    }

#if 0
    // This location in the code may be uncommented to do a one time change of the devices address.
    // It should be commented out again and immediately reloaded to the pixhawk after the device has
    // been modified by this initialization process.
    // Address change to 0x65 = 101
    write_bytes((uint8_t*)"#CI,0x65\r\n",10);
    _dev->set_address(0x65);
    uint8_t rx_bytes[lx20_max_reply_len_bytes + 1];
    sf20_wait_on_reply(rx_bytes);
    // Save the comm settings
    if (!sf20_send_and_expect("%C\r\n", "%c:")) {
        return false;
    }
#endif

#if 0
    /*
      this can be used to change the laser encoding pattern
      Changes the laser encoding pattern: 3 (Random A) [0..4].
    */
    if (!sf20_send_and_expect("#LE,3\r\n", "le:3")) {
        return false;
    }
#endif

    // Sets datum offset [-10.00 ... 10.00].
    if (!sf20_send_and_expect("#LO,0.00\r\n", "lo:0.00")) {
        return false;
    }

    // Changes to a new measuring mode (update rate):
    //    1 = 388 readings per second
    //    2 = 194 readings per second
    //    3 = 129 readings per second
    //    4 = 97 readings per second
    //    5 = 78 readings per second
    //    6 = 65 readings per second
    //    7 = 55 readings per second
    //    8 = 48 readings per second
    if (!sf20_send_and_expect("#LM,7\r\n", "lm:7")) {
        return false;
    }

    // Changes the gain boost value:
    //     Adjustment range = -20.00 ... 5.00
    if (!sf20_send_and_expect("#LB,0.00\r\n", "lb:0.00")) {
        return false;
    }

    // Switches distance streaming on or off:
    // 0 = off
    // 1 = on
    if (!sf20_send_and_expect("#SU,1\r\n", "su:1")) {
        return false;
    }

    // Changes the laser state:
    //    0 = laser is off
    //    1 = laser is running
    if (!sf20_send_and_expect("#LF,1\r\n", "lf:1")) {
        return false;
    }

    // Requests the measurement specified in the first stream.
    write_bytes((uint8_t*)init_stream_id[0], strlen(init_stream_id[0]));

    // call timer() at 20Hz
    _dev->register_periodic_callback(50000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_LightWareI2C::sf20_timer, void));

    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_LightWareI2C::legacy_get_reading(float &reading_m)
{
    be16_t val;

    const uint8_t read_reg = LIGHTWARE_DISTANCE_READ_REG;

    // read the high and low byte distance registers
    if (_dev->transfer(&read_reg, 1, (uint8_t *)&val, sizeof(val))) {
        int16_t signed_val = int16_t(be16toh(val));
        if (signed_val < 0) {
            // some lidar firmwares will return 65436 for out of range
            reading_m = uint16_t(max_distance_cm() + LIGHTWARE_OUT_OF_RANGE_ADD_CM) * 0.01f;
        } else {
            reading_m = uint16_t(signed_val) * 0.01f;
        }
        return true;
    }
    return false;
}

// read - return last value measured by sf20 sensor
bool AP_RangeFinder_LightWareI2C::sf20_get_reading(float &reading_m)
{
    // Parses up to 5 ASCII streams for LiDAR data.
    // If a parse fails, the stream measurement is not updated until it is successfully read in the future.
    uint8_t stream[lx20_max_expected_stream_reply_len_bytes+1]; // Maximum response length for a stream ie "ldf,0:40.99" is 11 characters

    uint8_t i = streamSequence[currentStreamSequenceIndex];
    size_t num_processed_chars = 0;

    /* Reads the LiDAR value requested during the last interrupt. */
    if (!_dev->read(stream, sizeof(stream))) {
        return false;
    }
    stream[lx20_max_expected_stream_reply_len_bytes] = 0;

    if (!sf20_parse_stream(stream, &num_processed_chars, parse_stream_id[i], sf20_stream_val[i])) {
        return false;
    }

    if (i==0) {
        reading_m = sf20_stream_val[0] * 0.01f;
    }

    // Increment the stream sequence
    currentStreamSequenceIndex++;
    if (currentStreamSequenceIndex >= numStreamSequenceIndexes) {
        currentStreamSequenceIndex = 0;
    }
    i = streamSequence[currentStreamSequenceIndex];

    // Request the next stream in the sequence from the SF20
    write_bytes((uint8_t*)init_stream_id[i], strlen(init_stream_id[i]));

    return true;
}

bool AP_RangeFinder_LightWareI2C::sf20_parse_stream(uint8_t *stream_buf,
        size_t *p_num_processed_chars,
        const char *string_identifier,
        uint16_t &val)
{
    size_t string_identifier_len = strlen(string_identifier);
    for (uint32_t i = 0 ; i < string_identifier_len ; i++) {
        if (stream_buf[*p_num_processed_chars] != string_identifier[i]) {
            return false;
        }
        (*p_num_processed_chars)++;
    }

    /*
      special case for being beyond maximum range, we receive a message like this:
      ldl,1:-1.00
      we will return max distance
     */
    if (strncmp((const char *)&stream_buf[*p_num_processed_chars], "-1.00", 5) == 0) {
        val = uint16_t(max_distance_cm() + LIGHTWARE_OUT_OF_RANGE_ADD_CM);
        (*p_num_processed_chars) += 5;
        return true;
    }

    /* Number is always returned in hundredths. So 6.33 is returned as 633. 6.3 is returned as 630.
     * 6 is returned as 600.
     * Extract number in format 6.33 or 123.99 (meters to be converted to centimeters).
     * Percentages such as 100 (percent), are returned as 10000.
     */
    uint32_t final_multiplier = 100;
    bool decrement_multiplier = false;
    bool number_found = false;
    uint16_t accumulator = 0;
    uint16_t digit_u16 = (uint16_t)stream_buf[*p_num_processed_chars];
    while ((((digit_u16 <= '9') &&
             (digit_u16 >= '0')) ||
            (digit_u16 == '.')) &&
           (*p_num_processed_chars < lx20_max_reply_len_bytes)) {
        (*p_num_processed_chars)++;
        if (decrement_multiplier) {
            final_multiplier /=10;
        }
        if (digit_u16 == '.') {
            decrement_multiplier = true;
            digit_u16 = (uint16_t)stream_buf[*p_num_processed_chars];
            continue;
        }
        number_found = true;
        accumulator *= 10;
        accumulator += digit_u16 - '0';
        digit_u16 = (uint16_t)stream_buf[*p_num_processed_chars];
    }

    accumulator *= final_multiplier;
    val = accumulator;
    return number_found;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_LightWareI2C::update(void)
{
    // nothing to do - its all done in the timer()
}

void AP_RangeFinder_LightWareI2C::legacy_timer(void)
{
    if (legacy_get_reading(state.distance_m)) {
        // update range_valid state based on distance measured
        update_status();
    } else {
        set_status(RangeFinder::Status::NoData);
    }
}

void AP_RangeFinder_LightWareI2C::sf20_timer(void)
{
    if (sf20_get_reading(state.distance_m)) {
        // update range_valid state based on distance measured
        update_status();
    } else {
        set_status(RangeFinder::Status::NoData);
    }
}

// Only for use during init as this blocks while waiting for the SF20 to be ready.
bool AP_RangeFinder_LightWareI2C::sf20_wait_on_reply(uint8_t *rx_two_byte)
{
    // Waits for a non-zero first byte while repeatedly reading 16 bits.
    // This is used after a read command to allow the sf20 time to provide the result.
    uint32_t start_time_ms = AP_HAL::millis();
    const uint32_t max_wait_time_ms = 50;

    while (AP_HAL::millis() - start_time_ms < max_wait_time_ms) {
        if (!_dev->read(rx_two_byte, 2)) {
            hal.scheduler->delay(1);
            continue;
        }
        if (rx_two_byte[0] != 0) {
            // normal exit
            return true;
        }
    }
    return false;
}
