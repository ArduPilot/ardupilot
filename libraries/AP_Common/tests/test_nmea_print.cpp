#include <AP_gtest.h>
#include <AP_HAL/UARTDriver.h>
#include <AP_Common/NMEA.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class DummyUart: public AP_HAL::UARTDriver {
public:
    bool is_initialized() override { return true; };
    bool tx_pending() override { return false; };
    uint32_t txspace() override { return _txspace; };

    void set_txspace(uint32_t space) {
        _txspace = space;
    }
    uint32_t _txspace;

protected:
    uint32_t _available() override { return 1; };
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override {  };
    void _end() override {  };
    void _flush() override {  };
    size_t _write(const uint8_t *buffer, size_t size) override { return 1; };
    ssize_t _read(uint8_t *buf, uint16_t count) override { return 0; };
    bool _discard_input() override { return false; }
};

static DummyUart test_uart;


TEST(NMEA, Printf)
{
    // test not enough space
    test_uart.set_txspace(2);
    EXPECT_FALSE(nmea_printf(&test_uart, "TEST"));
    // normal test
    test_uart.set_txspace(9);
    EXPECT_TRUE(nmea_printf(&test_uart, "TEST"));
}

AP_GTEST_MAIN()
