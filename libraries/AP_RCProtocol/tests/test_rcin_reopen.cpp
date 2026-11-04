/*
  test that RC input takes its UART back after another thread has opened
  it, as serial passthrough does
 */
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#include <AP_SBusOut/AP_SBusOut.h>
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

// a UART that, like ChibiOS, only lets the thread that last called begin()
// read it. The test plays both threads: begin() from the RC code makes it
// the owner, taken_by_other_thread() is another thread's begin(). The SBUS
// receiver on it is only heard with the line set up for SBUS
class MockUART final : public AP_HAL::UARTDriver
{
public:
    bool owned;
    uint32_t begins;
    uint32_t baud;
    uint8_t rx[1024];
    uint16_t rx_len;

    bool is_initialized() override { return true; }
    bool tx_pending() override { return false; }
    uint32_t txspace() override { return 1024; }
    bool is_owned_by_current_thread() const override { return owned; }

    void taken_by_other_thread() { owned = false; }

    // 100000 baud, even parity, 2 stop bits
    bool line_is_sbus() const {
        return baud == 100000 && parity == 2 && _stop_bits == 2;
    }

    // queue bytes for the RC code to read
    void inject_rx(const uint8_t *buf, uint16_t len) {
        len = MIN(len, uint16_t(sizeof(rx) - rx_len));
        memcpy(&rx[rx_len], buf, len);
        rx_len += len;
    }

protected:
    void _begin(uint32_t b, uint16_t rxSpace, uint16_t txSpace) override {
        owned = true;
        begins++;
        if (b != 0) {
            baud = b;
        }
    }
    void _configure_parity(uint8_t v) override { parity = v; }
    size_t _write(const uint8_t *buffer, size_t size) override { return size; }
    ssize_t _read(uint8_t *buffer, uint16_t count) override {
        if (!owned) {
            return 0;
        }
        if (!line_is_sbus()) {
            // nothing that decodes as SBUS
            rx_len = 0;
            return 0;
        }
        count = MIN(count, rx_len);
        memcpy(buffer, rx, count);
        memmove(rx, &rx[count], rx_len - count);
        rx_len -= count;
        return count;
    }
    void _end() override {}
    void _flush() override {}
    uint32_t _available() override { return owned ? rx_len : 0; }
    bool _discard_input() override { rx_len = 0; return true; }
};

// ArduPilot objects rely on zeroed allocation, so create them with new
struct Harness {
    AP_RCProtocol &rc = *new AP_RCProtocol();
    MockUART &uart = *new MockUART();
    // start a second past a 32-bit micros() boundary well ahead of now:
    // time then only moves forward and no timestamp is zero
    uint64_t t_us = (((AP_HAL::micros64() >> 32) + 2) << 32) + 1000000;
    uint8_t frame[25];

    Harness() {
        rc.init();
        rc.add_uart(&uart);
        uint16_t values[8];
        for (uint8_t i = 0; i < 8; i++) {
            values[i] = 1500;
        }
        AP_SBusOut::sbus_format_frame(values, 8, frame);
    }
    ~Harness() {
        uart.lock_port(0, 0);
        delete &uart;
        delete &rc;
    }
    // run the RC input update at 1kHz with an SBUS frame every 10ms,
    // returning true if a new RC input was seen
    bool run_ms(uint32_t ms) {
        bool got_input = false;
        for (uint32_t i = 0; i < ms; i++) {
            t_us += 1000;
            hal.scheduler->stop_clock(t_us);
            if (i % 10 == 0) {
                uart.inject_rx(frame, sizeof(frame));
            }
            rc.update();
            got_input |= rc.new_input();
        }
        return got_input;
    }
};

// CRSF telemetry is only created while disarmed, and needs a whole
// vehicle; keep it out of the test
struct Armed {
    Armed() { hal.util->set_soft_armed(true); }
    ~Armed() { hal.util->set_soft_armed(false); }
};

// an SBUS receiver that has been detected is never searched for again, so
// after serial passthrough has opened its port from another thread the RC
// code read nothing until a reboot. It now opens the port again itself
TEST(RCInReopen, SBUSRecoversAfterPassthrough)
{
    Armed armed;
    Harness s;
    // long enough for the search to reach the SBUS config
    s.run_ms(8000);
    ASSERT_EQ(s.rc.protocol_detected(), AP_RCProtocol::SBUS);
    EXPECT_TRUE(s.run_ms(100));

    // passthrough takes the port from its own thread, then lets it go
    const uint32_t key = 0x3256AB9F;
    s.uart.lock_port(key, key);
    s.uart.taken_by_other_thread();
    s.run_ms(2000);
    s.uart.lock_port(0, 0);

    // RC input opens it again and carries on, well before any timeout
    const uint32_t begins = s.uart.begins;
    EXPECT_TRUE(s.run_ms(200)) << "no RC input after passthrough";
    EXPECT_GT(s.uart.begins, begins);
    EXPECT_EQ(s.rc.protocol_detected(), AP_RCProtocol::SBUS);
}

// when passthrough ends it opens the port again at SERIALn_BAUD from the
// main thread, which is the wrong baud rate for SBUS. Opening it again
// from RC input puts back the line the receiver was detected with
TEST(RCInReopen, SBUSLineRestoredAfterPassthrough)
{
    Armed armed;
    Harness s;
    s.run_ms(8000);
    ASSERT_EQ(s.rc.protocol_detected(), AP_RCProtocol::SBUS);

    const uint32_t key = 0x3256AB9F;
    s.uart.lock_port(key, key);
    s.uart.taken_by_other_thread();
    s.run_ms(2000);
    s.uart.lock_port(0, 0);
    // passthrough's own restore, from another thread
    s.uart.begin(115200);
    s.uart.taken_by_other_thread();
    ASSERT_FALSE(s.uart.line_is_sbus());

    EXPECT_TRUE(s.run_ms(200)) << "no RC input after passthrough";
    EXPECT_TRUE(s.uart.line_is_sbus());
    EXPECT_EQ(s.uart.baud, 100000U);
}

// while the port is locked nothing is opened, as the lock refuses it
TEST(RCInReopen, NotReopenedWhileLocked)
{
    Armed armed;
    Harness s;
    s.run_ms(8000);
    ASSERT_EQ(s.rc.protocol_detected(), AP_RCProtocol::SBUS);
    const uint32_t key = 0x3256AB9F;
    s.uart.lock_port(key, key);
    s.uart.taken_by_other_thread();
    const uint32_t begins = s.uart.begins;
    s.run_ms(2000);
    EXPECT_EQ(s.uart.begins, begins);
    EXPECT_FALSE(s.uart.owned);
}

AP_GTEST_MAIN()
