#include <AP_gtest.h>
#include <AP_HAL/UARTDriver.h>
#include <AP_Common/NMEA.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
static uint32_t count = 0;

void *malloc(size_t size) {
    if (count == 1) {
        return nullptr;
    }
    return calloc(size, 1);
}

// THAT IS UGLY HACK BUT IT WORKS ... it is just used to make print_vprintf return negative value.
class BufferPrinter : public AP_HAL::BetterStream {
public:
    BufferPrinter(char* str, size_t size)  :
    _offs(0), _str(str), _size(size)  {}

    size_t write(uint8_t c) override { return 1; }
    size_t write(const uint8_t *buffer, size_t size) override { return 1; }

    size_t _offs;
    char* const  _str;
    const size_t _size;

    uint32_t available() override { return 0; }
    bool read(uint8_t &c) override { return false; };
    uint32_t txspace() override { return 0; }
    bool discard_input() override { return false; }
};

void print_vprintf(AP_HAL::BetterStream *s, const char *fmt, va_list ap);
void print_vprintf(AP_HAL::BetterStream *s, const char *fmt, va_list ap) {
    BufferPrinter* p = static_cast<BufferPrinter*>(s);
    count++;
    if (count < 3) {
        p->_offs = 4;
        return;
    }
    p->_offs = -1;
    return;
}

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
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override {  };
    ssize_t _read(uint8_t *buf, uint16_t size) override { return 0; };
    void _end() override {  };
    void _flush() override {  };
    uint32_t _available() override { return 1; };
    size_t _write(const uint8_t *buffer, size_t size) override { return 1; };
    bool _discard_input() override { return false; }
};

static DummyUart test_uart;

TEST(NMEA, VAPrintf)
{
    // test Malloc failure
    EXPECT_FALSE(nmea_printf(&test_uart, "test"));
    // test second vsnprintf failure;
    EXPECT_FALSE(nmea_printf(&test_uart, "test"));
    // test first vsnprintf failure;
    EXPECT_FALSE(nmea_printf(&test_uart, "test"));
}

AP_GTEST_MAIN()
