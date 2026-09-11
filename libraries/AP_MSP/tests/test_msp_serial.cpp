#include <AP_gtest.h>

#include <AP_MSP/AP_MSP_config.h>

#if HAL_MSP_ENABLED

#include <AP_HAL/UARTDriver.h>
#include <AP_Math/AP_Math.h>
#include <AP_MSP/msp.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// a uart which truncates writes to the space available, as the ChibiOS and
// Linux backends do, and which records what made it onto the wire
class DummyUart: public AP_HAL::UARTDriver {
public:
    bool is_initialized() override { return true; }
    bool tx_pending() override { return _tx_pending; }
    uint32_t txspace() override { return _txspace; }

    void setup(uint32_t space, bool pending) {
        _txspace = space;
        _tx_pending = pending;
        written = 0;
    }

    uint32_t written;

protected:
    uint32_t _available() override { return 0; }
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override {}
    void _end() override {}
    void _flush() override {}
    ssize_t _read(uint8_t *buf, uint16_t count) override { return 0; }
    bool _discard_input() override { return false; }

    size_t _write(const uint8_t *buffer, size_t size) override {
        const size_t accepted = MIN(size_t(_txspace), size);
        _txspace -= accepted;
        written += accepted;
        return accepted;
    }

private:
    uint32_t _txspace;
    bool _tx_pending;
};

static DummyUart test_uart;

// a frame is 6 bytes of header, 4 of payload and 1 of checksum
static uint32_t send_frame(uint32_t txspace, bool tx_pending)
{
    const uint8_t hdr[6] {};
    const uint8_t data[4] {};
    const uint8_t crc[1] {};

    test_uart.setup(txspace, tx_pending);

    MSP::msp_port_t port {};
    port.uart = &test_uart;

    return MSP::msp_serial_send_frame(&port, hdr, sizeof(hdr), data, sizeof(data), crc, sizeof(crc));
}

TEST(MSPSerial, SendFrameFits)
{
    // an idle port with room takes the whole frame
    EXPECT_EQ(11U, send_frame(11, false));
    EXPECT_EQ(11U, test_uart.written);

    // so does a busy port which still has room for it
    EXPECT_EQ(11U, send_frame(64, true));
    EXPECT_EQ(11U, test_uart.written);
}

TEST(MSPSerial, SendFrameDoesNotFit)
{
    // a busy port without room must drop the frame rather than write part of
    // it: writes do not block, so a partial write corrupts the stream
    EXPECT_EQ(0U, send_frame(10, true));
    EXPECT_EQ(0U, test_uart.written);

    // and a frame larger than the buffer can never be written whole
    EXPECT_EQ(0U, send_frame(10, false));
    EXPECT_EQ(0U, test_uart.written);

    // a port with no space at all is the same case
    EXPECT_EQ(0U, send_frame(0, true));
    EXPECT_EQ(0U, test_uart.written);
}

AP_GTEST_MAIN()

#else

AP_GTEST_MAIN()

#endif  // HAL_MSP_ENABLED
