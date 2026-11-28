/*
  u-blox driver tests. Messages are fed through the driver's own read(), from a
  test UART that also records what the driver sends back to the receiver.
 */
#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/UARTDriver.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_GPS/AP_GPS_UBLOX.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <RC_Channel/RC_Channel.h>

#include <vector>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();
GCS_Dummy _gcs;
// vehicle code normally supplies this; the GPS driver never uses it
const struct AP_Param::GroupInfo RC_Channels::var_info[] = { AP_GROUPEND };

static AP_GPS gps;

#if AP_GPS_UBLOX_ENABLED

class AP_GPS_Test
{
public:
    static AP_GPS::Params &params() { return gps.params[0]; }
    static AP_GPS::GPS_State &state() { return gps.state[0]; }
    static void set_auto_config(int8_t v) { gps._auto_config.set(v); }
    static void set_raw_data(int8_t v) { gps._raw_data.set(v); }
};

/*
  a UART holding the bytes the receiver sends, and collecting what the
  driver writes back
 */
class TestUart : public AP_HAL::UARTDriver {
public:
    std::vector<uint8_t> rx;
    size_t rx_pos = 0;
    std::vector<uint8_t> tx;

    bool is_initialized() override { return true; }
    bool tx_pending() override { return false; }
    uint32_t txspace() override { return 4096; }

protected:
    uint32_t _available() override { return rx.size() - rx_pos; }
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override {}
    void _end() override {}
    void _flush() override {}
    size_t _write(const uint8_t *buffer, size_t size) override
    {
        tx.insert(tx.end(), buffer, buffer + size);
        return size;
    }
    ssize_t _read(uint8_t *buf, uint16_t count) override
    {
        size_t n = MIN((size_t)count, rx.size() - rx_pos);
        memcpy(buf, &rx[rx_pos], n);
        rx_pos += n;
        return n;
    }
    bool _discard_input() override
    {
        rx_pos = rx.size();
        return true;
    }
};

// a UBX frame with its Fletcher checksum
static std::vector<uint8_t> ubx_frame(uint8_t msg_class, uint8_t msg_id, const std::vector<uint8_t> &payload)
{
    std::vector<uint8_t> f { 0xb5, 0x62, msg_class, msg_id,
                             uint8_t(payload.size() & 0xff), uint8_t(payload.size() >> 8) };
    f.insert(f.end(), payload.begin(), payload.end());
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < f.size(); i++) {
        ck_a += f[i];
        ck_b += ck_a;
    }
    f.push_back(ck_a);
    f.push_back(ck_b);
    return f;
}

// payload lengths of the UBX frames of one class/id the driver sent
static std::vector<uint16_t> sent_payload_lengths(const std::vector<uint8_t> &tx, uint8_t msg_class, uint8_t msg_id)
{
    std::vector<uint16_t> lengths;
    for (size_t i = 0; i + 6 <= tx.size(); ) {
        if (tx[i] != 0xb5 || tx[i+1] != 0x62) {
            i++;
            continue;
        }
        const uint16_t len = tx[i+4] | (tx[i+5] << 8);
        if (tx[i+2] == msg_class && tx[i+3] == msg_id) {
            lengths.push_back(len);
        }
        i += 6 + len + 2;
    }
    return lengths;
}

class UBloxTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        // no configuration traffic of the driver's own, only replies to what it receives
        AP_GPS_Test::set_auto_config(0);
        AP_GPS_Test::set_raw_data(0);
        AP_GPS_Test::params().gnss_mode.set(0);
        AP_GPS_Test::state().instance = 0;
    }

    // drivers are always heap allocated, as in AP_GPS::_detect_instance()
    AP_GPS_UBLOX *make()
    {
        return NEW_NOTHROW AP_GPS_UBLOX(gps, AP_GPS_Test::params(), AP_GPS_Test::state(),
                                        &uart, AP_GPS::GPS_ROLE_NORMAL);
    }

    void feed(AP_GPS_UBLOX &ublox, const std::vector<uint8_t> &bytes)
    {
        uart.rx.insert(uart.rx.end(), bytes.begin(), bytes.end());
        for (uint8_t i = 0; i < 16 && uart.available() > 0; i++) {
            ublox.read();
        }
    }

    TestUart uart;
};

// UBX-CFG-GNSS: msgVer, numTrkChHw, numTrkChUse, numConfigBlocks, then
// numConfigBlocks blocks of {gnssId, resTrkCh, maxTrkCh, reserved1, flags[4]}
static std::vector<uint8_t> cfg_gnss_payload(uint8_t num_blocks, uint8_t blocks_present)
{
    std::vector<uint8_t> p { 0, 32, 32, num_blocks };
    for (uint8_t i = 0; i < blocks_present; i++) {
        const uint8_t block[8] { uint8_t(i), 4, 8, 0, 0x01, 0x00, 0x01, 0x01 };
        p.insert(p.end(), block, block + sizeof(block));
    }
    return p;
}

// the driver answers a GNSS config that differs from GPS_GNSS_MODE with its
// corrected copy; a well formed 7 block message is still handled that way
TEST_F(UBloxTest, cfg_gnss_answers_well_formed_config)
{
    AP_GPS_Test::params().gnss_mode.set(0x41);  // GPS + GLONASS
    AP_GPS_UBLOX *ublox = make();
    ASSERT_NE(ublox, nullptr);
    feed(*ublox, ubx_frame(0x06, 0x3E, cfg_gnss_payload(7, 7)));
    const auto sent = sent_payload_lengths(uart.tx, 0x06, 0x3E);
    ASSERT_EQ(sent.size(), 1U);
    EXPECT_EQ(sent[0], 4 + 7 * 8);
    delete ublox;
}

// numConfigBlocks is taken from the wire. The driver must not walk (and write
// to) more blocks than the message carries, nor echo them back to the receiver
TEST_F(UBloxTest, cfg_gnss_block_count_beyond_payload_is_ignored)
{
    AP_GPS_Test::params().gnss_mode.set(0x41);
    AP_GPS_UBLOX *ublox = make();
    ASSERT_NE(ublox, nullptr);
    feed(*ublox, ubx_frame(0x06, 0x3E, cfg_gnss_payload(255, 0)));
    EXPECT_EQ(sent_payload_lengths(uart.tx, 0x06, 0x3E).size(), 0U);
    delete ublox;
}

// a complete message with more blocks than the receive buffer holds
TEST_F(UBloxTest, cfg_gnss_more_blocks_than_supported_is_ignored)
{
    AP_GPS_Test::params().gnss_mode.set(0x41);
    AP_GPS_UBLOX *ublox = make();
    ASSERT_NE(ublox, nullptr);
    feed(*ublox, ubx_frame(0x06, 0x3E, cfg_gnss_payload(8, 8)));
    EXPECT_EQ(sent_payload_lengths(uart.tx, 0x06, 0x3E).size(), 0U);
    delete ublox;
}

#if UBLOX_RXM_RAW_LOGGING && HAL_LOGGING_ENABLED
static AP_Logger logger;
static AP_UInt32 log_bitmask;
static const struct LogStructure log_structure[] = {
    LOG_COMMON_STRUCTURES,
};

static void start_logging()
{
    static bool started;
    if (!started) {
        started = true;
        log_bitmask.set(UINT32_MAX);
        logger.init(log_bitmask, log_structure, ARRAY_SIZE(log_structure));
        logger.set_vehicle_armed(true);
        gps.set_log_gps_bit(1);
    }
}

/*
  RXM-RAWX and RXM-RAW carry their measurement count in the payload. With
  GPS_RAW_DATA set the driver logs one record per measurement; a count larger
  than the measurements received used to be read past the receive buffer and
  the driver object (heap-buffer-overflow under --asan, as in the unit-tests-asan
  CI job). These must log at most what was received.
 */
TEST_F(UBloxTest, rxm_rawx_count_beyond_payload)
{
    start_logging();
    AP_GPS_Test::set_raw_data(1);
    AP_GPS_UBLOX *ublox = make();
    ASSERT_NE(ublox, nullptr);
    // rcvTow(8) week(2) leapS(1) numMeas(1) recStat(1) reserved(3), no measurements
    std::vector<uint8_t> p(16, 0);
    p[11] = 255;
    feed(*ublox, ubx_frame(0x02, 0x15, p));
    EXPECT_EQ(uart.rx_pos, uart.rx.size());
    delete ublox;
}

TEST_F(UBloxTest, rxm_raw_count_beyond_payload)
{
    start_logging();
    AP_GPS_Test::set_raw_data(1);
    AP_GPS_UBLOX *ublox = make();
    ASSERT_NE(ublox, nullptr);
    // iTOW(4) week(2) numSV(1) reserved1(1), no satellites
    std::vector<uint8_t> p(8, 0);
    p[6] = 255;
    feed(*ublox, ubx_frame(0x02, 0x10, p));
    EXPECT_EQ(uart.rx_pos, uart.rx.size());
    delete ublox;
}
#endif  // UBLOX_RXM_RAW_LOGGING && HAL_LOGGING_ENABLED

#endif  // AP_GPS_UBLOX_ENABLED

AP_GTEST_MAIN()
