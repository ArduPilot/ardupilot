#include <AP_gtest.h>

#include <AP_RangeFinder/AP_RangeFinder_USD1_Serial.h>
#include <AP_HAL/UARTDriver.h>
#include <AP_HAL/utility/OwnPtr.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

class DummyUart : public AP_HAL::UARTDriver {
public:
    bool is_initialized() override { return true; }
    bool tx_pending() override { return false; }
    uint32_t txspace() override { return 128; }

    void set_data(const uint8_t *data, uint16_t len) {
        _rx_buf = data;
        _rx_len = len;
        _rx_pos = 0;
    }

protected:
    uint32_t _available() override { return _rx_len - _rx_pos; }
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override {}
    void _end() override {}
    void _flush() override {}
    size_t _write(const uint8_t *buffer, size_t size) override { return size; }
    ssize_t _read(uint8_t *buf, uint16_t count) override {
        uint16_t n = 0;
        while (n < count && _rx_pos < _rx_len) {
            buf[n++] = _rx_buf[_rx_pos++];
        }
        return n;
    }
    bool _discard_input() override { return false; }

public:
    bool read(uint8_t &b) override {
        if (_rx_pos < _rx_len) {
            b = _rx_buf[_rx_pos++];
            return true;
        }
        return false;
    }

private:
    const uint8_t *_rx_buf = nullptr;
    uint16_t _rx_len = 0;
    uint16_t _rx_pos = 0;
};

class AP_RangeFinder_USD1_Test {
public:
    RangeFinder::RangeFinder_State state;
    AP_RangeFinder_Params params;
    AP_RangeFinder_USD1_Serial driver{state, params};
    DummyUart uart;

    AP_RangeFinder_USD1_Test() {
        driver.uart = &uart;
    }

    bool detect_version(const uint8_t *data, uint16_t len) {
        uart.set_data(data, len);
        return driver.detect_version();
    }

    bool get_reading(const uint8_t *data, uint16_t len, float &reading_m) {
        uart.set_data(data, len);
        return driver.get_reading(reading_m);
    }
};

TEST(USD1, Version0_SingleFrame)
{
    AP_HAL::OwnPtr<AP_RangeFinder_USD1_Test> test{NEW_NOTHROW AP_RangeFinder_USD1_Test};
    ASSERT_NE(test.get(), nullptr);

    // Detect version 0 (recurring 0x48 header on 4th byte)
    const uint8_t v0_detect[] = { 0x48, 0x10, 0x20, 0x48 };
    EXPECT_TRUE(test->detect_version(v0_detect, sizeof(v0_detect)));

    // Single 3-byte Version 0 frame: header 0x48, payload 0x05, 0x0A
    // Reading = (0x0A & 0x7F)*128 + (0x05 & 0x7F) = 1280 + 5 = 1285
    // Distance in meters = (1285 * 0.01) * 2.5 = 32.125m
    const uint8_t v0_frame[] = { 0x48, 0x05, 0x0A };
    float reading_m = 0.0f;
    EXPECT_TRUE(test->get_reading(v0_frame, sizeof(v0_frame), reading_m));
    EXPECT_NEAR(reading_m, 32.125f, 0.001f);
}

TEST(USD1, Version0_MultipleFrames)
{
    AP_HAL::OwnPtr<AP_RangeFinder_USD1_Test> test{NEW_NOTHROW AP_RangeFinder_USD1_Test};
    ASSERT_NE(test.get(), nullptr);

    const uint8_t v0_detect[] = { 0x48, 0x10, 0x20, 0x48 };
    EXPECT_TRUE(test->detect_version(v0_detect, sizeof(v0_detect)));

    // Two 3-byte Version 0 frames in one stream
    // Frame 1: 0x48, 0x05, 0x0A -> 32.125m
    // Frame 2: 0x48, 0x0A, 0x0A -> (10 * 128 + 10) * 0.01 * 2.5 = 1290 * 0.025 = 32.25m
    // Average = (32.125 + 32.25) / 2 = 32.1875m
    const uint8_t v0_two_frames[] = { 0x48, 0x05, 0x0A, 0x48, 0x0A, 0x0A };
    float reading_m = 0.0f;
    EXPECT_TRUE(test->get_reading(v0_two_frames, sizeof(v0_two_frames), reading_m));
    EXPECT_NEAR(reading_m, 32.1875f, 0.001f);
}

TEST(USD1, Version1_SixByteFrame)
{
    AP_HAL::OwnPtr<AP_RangeFinder_USD1_Test> test{NEW_NOTHROW AP_RangeFinder_USD1_Test};
    ASSERT_NE(test.get(), nullptr);

    // Detect version 1 (header 0xFE, version 0x01)
    const uint8_t v1_detect[] = { 0xFE, 0x01 };
    EXPECT_TRUE(test->detect_version(v1_detect, sizeof(v1_detect)));

    // 6-byte Version 1 frame:
    // header: 0xFE
    // version: 0x01
    // distance_low: 0x20
    // distance_high: 0x05 -> distance = 5*256 + 32 = 1312 cm -> 13.12m
    // spare: 0x00
    // checksum: (0x01 + 0x20 + 0x05 + 0x00) & 0xFF = 0x26
    const uint8_t v1_frame[] = { 0xFE, 0x01, 0x20, 0x05, 0x00, 0x26 };
    float reading_m = 0.0f;
    EXPECT_TRUE(test->get_reading(v1_frame, sizeof(v1_frame), reading_m));
    EXPECT_NEAR(reading_m, 13.12f, 0.001f);
}

AP_GTEST_MAIN()
