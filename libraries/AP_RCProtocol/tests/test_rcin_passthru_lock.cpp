/*
  test that the RC input code leaves an added RC UART alone while another
  user (e.g. SERIAL_PASS serial passthrough) has the port locked
 */
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#include <AP_CRSF/AP_CRSF_Protocol.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS_Dummy.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// minimal vehicle singletons the decoder needs, as in RCProtocolTest
class RC_Channel_Test : public RC_Channel {};

class RC_Channels_Test : public RC_Channels
{
public:
    RC_Channel_Test obj_channels[NUM_RC_CHANNELS];

    const RC_Channel_Test *channel(const uint8_t chan) const override
    {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }
    RC_Channel_Test *channel(const uint8_t chan) override
    {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

protected:
    int8_t flight_mode_channel_number() const override
    {
        return 5;
    }
};

#define RC_CHANNELS_SUBCLASS RC_Channels_Test
#define RC_CHANNEL_SUBCLASS RC_Channel_Test

#include <RC_Channel/RC_Channels_VarInfo.h>

static RC_Channels_Test rchannels;
static AP_SerialManager serial_manager;
// protocol detection announces itself via gcs()
static GCS_Dummy _gcs;

// a UART that records the serial format it is given. On hardware,
// changing parity or stop bits restarts the USART (sdStop/sdStart on
// ChibiOS), losing any bytes in flight
class MockUART final : public AP_HAL::UARTDriver
{
public:
    uint32_t reconfigs;
    uint32_t parity_calls;
    uint32_t stop_bits_calls;
    uint8_t parity_set;
    uint8_t stop_bits = 1;
    uint8_t rx[1024];
    uint16_t rx_len;

    bool is_initialized() override { return true; }
    bool tx_pending() override { return false; }
    uint32_t txspace() override { return 1024; }

    void configure_parity(uint8_t v) override {
        reconfigs++;
        parity_calls++;
        parity_set = v;
    }
    void set_stop_bits(int n) override {
        reconfigs++;
        stop_bits_calls++;
        stop_bits = n;
    }
    bool set_options(uint16_t options) override {
        reconfigs++;
        _last_options = options;
        return true;
    }
    bool rx_inverted() const { return (get_options() & OPTION_RXINV) != 0; }

    // queue bytes for the RC code to read
    void inject_rx(const uint8_t *buf, uint16_t len) {
        len = MIN(len, uint16_t(sizeof(rx) - rx_len));
        memcpy(&rx[rx_len], buf, len);
        rx_len += len;
    }

protected:
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override {}
    size_t _write(const uint8_t *buffer, size_t size) override { return size; }
    ssize_t _read(uint8_t *buffer, uint16_t count) override {
        count = MIN(count, rx_len);
        memcpy(buffer, rx, count);
        memmove(rx, &rx[count], rx_len - count);
        rx_len -= count;
        return count;
    }
    void _end() override {}
    void _flush() override {}
    uint32_t _available() override { return rx_len; }
    bool _discard_input() override { rx_len = 0; return true; }
};

// SERIAL_PASS's lock key
static const uint32_t passthru_key = 0x3256AB9F;

// ArduPilot objects rely on zeroed allocation, so create them with new
struct Harness {
    AP_RCProtocol &rc = *new AP_RCProtocol();
    MockUART &uart = *new MockUART();
    // start a second past a 32-bit micros() boundary well ahead of now:
    // time then only moves forward (the Linux HAL will not move the clock
    // back) and no timestamp is zero, which the RC code treats as never
    uint64_t t_us = (((AP_HAL::micros64() >> 32) + 2) << 32) + 1000000;

    Harness() {
        rc.init();
        rc.add_uart(&uart);
    }
    ~Harness() {
        uart.lock_port(0, 0);
        delete &uart;
        delete &rc;
    }
    // run the RC input update at 1kHz for the given time, calling fn each ms
    template <typename F>
    void run_ms(uint32_t ms, F fn) {
        for (uint32_t i = 0; i < ms; i++) {
            t_us += 1000;
            hal.scheduler->stop_clock(t_us);
            fn(i);
            rc.update();
        }
    }
    void run_ms(uint32_t ms) {
        run_ms(ms, [](uint32_t) {});
    }
    // lock the port as SERIAL_PASS does, and give the RC code one update
    // to hand it over. It must leave the port 8N1 with no inversion,
    // only restarting it to change the parity or stop bits a search
    // config left
    void lock() {
        const uint8_t parity_before = uart.parity_set;
        const uint8_t stop_bits_before = uart.stop_bits;
        uart.parity_calls = 0;
        uart.stop_bits_calls = 0;
        uart.lock_port(passthru_key, passthru_key);
        run_ms(1);
        EXPECT_EQ(uart.parity_set, 0);
        EXPECT_EQ(uart.stop_bits, 1);
        EXPECT_FALSE(uart.rx_inverted());
        EXPECT_EQ(uart.parity_calls, parity_before == 0 ? 0U : 1U);
        EXPECT_EQ(uart.stop_bits_calls, stop_bits_before == 1 ? 0U : 1U);
        uart.reconfigs = 0;
    }
};

// CRSF telemetry is only created while disarmed, and needs a whole
// vehicle (AHRS, sensors) to run; keep it out of a test
struct Armed {
    Armed() { hal.util->set_soft_armed(true); }
    ~Armed() { hal.util->set_soft_armed(false); }
};

// one CRSF RC channels frame with all channels centred
static void crsf_frame(uint8_t frame[26])
{
    frame[0] = AP_CRSF_Protocol::CRSF_ADDRESS_FLIGHT_CONTROLLER;
    frame[1] = 24;  // type + payload + crc
    frame[2] = AP_CRSF_Protocol::CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    // 16 channels of 11 bits at 992 (1500us)
    uint32_t bits = 0;
    uint8_t nbits = 0, o = 3;
    for (uint8_t i = 0; i < 16; i++) {
        bits |= uint32_t(992) << nbits;
        nbits += 11;
        while (nbits >= 8) {
            frame[o++] = bits & 0xFF;
            bits >>= 8;
            nbits -= 8;
        }
    }
    frame[25] = crc8_dvb_s2_update(0, &frame[2], 23);
}

// no receiver detected yet (an ELRS receiver with no transmitter link
// sends nothing, so this is the usual way to flash one): the RC code is
// cycling through serial configs, including inverted and 8E2 ones. When
// the port is taken, at any point in that cycle, it must be handed over in
// 8N1 and then not touched until it is released
TEST(RCInPassthru, searching_port_handed_over)
{
    // every ms through a whole cycle of configs
    for (uint32_t lock_ms = 1; lock_ms <= 7000; lock_ms++) {
        SCOPED_TRACE(testing::Message() << "locked at " << lock_ms << "ms");
        Harness s;
        s.run_ms(lock_ms);
        s.lock();
        // longer than the RC code's 1s config change
        s.run_ms(1500);
        EXPECT_EQ(s.uart.reconfigs, 0U);

        // configured again as soon as the port is released
        s.uart.lock_port(0, 0);
        s.run_ms(1);
        EXPECT_GT(s.uart.reconfigs, 0U);
    }
}

// the receiver was detected as CRSF before passthrough was enabled: with
// no frames arriving the RC code re-applies the CRSF config every second,
// but must not while the port is locked, and RC input resumes once the
// port is released
TEST(RCInPassthru, detected_port_left_alone)
{
    Armed armed;
    Harness s;
    uint8_t frame[26];
    crsf_frame(frame);
    // frames at 250Hz until the RC code has found CRSF
    s.run_ms(8000, [&](uint32_t i) {
        if (i % 4 == 0) {
            s.uart.inject_rx(frame, sizeof(frame));
        }
    });
    ASSERT_EQ(s.rc.protocol_detected(), AP_RCProtocol::CRSF);
    // clear the new input flag from the frames above
    s.rc.new_input();

    s.lock();
    s.run_ms(5000);
    EXPECT_EQ(s.uart.reconfigs, 0U);
    EXPECT_FALSE(s.rc.new_input());

    // configured again as soon as the port is released
    s.uart.lock_port(0, 0);
    s.run_ms(1);
    EXPECT_GT(s.uart.reconfigs, 0U);
    bool got_input = false;
    s.run_ms(2000, [&](uint32_t i) {
        if (i % 4 == 0) {
            s.uart.inject_rx(frame, sizeof(frame));
        }
        got_input |= s.rc.new_input();
    });
    EXPECT_TRUE(got_input) << "no RC input after release";
    EXPECT_EQ(s.rc.protocol_detected(), AP_RCProtocol::CRSF);
}

AP_GTEST_MAIN()
