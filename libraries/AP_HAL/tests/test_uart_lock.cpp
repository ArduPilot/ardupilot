#include <AP_gtest.h>
#include <AP_HAL/HAL.h>
#include <AP_HAL/UARTDriver.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  a UART that counts how often the backend line setting methods are
  called, to check a locked port can only be changed with its key
 */
class LineCountUart: public AP_HAL::UARTDriver {
public:
    bool is_initialized() override { return true; }
    bool tx_pending() override { return false; }
    uint32_t txspace() override { return 0; }

    uint32_t parity_calls;
    uint32_t stop_bits_calls;
    uint32_t options_calls;
    uint32_t flow_control_calls;
    uint8_t last_parity;
    int last_stop_bits;

    uint32_t total_calls() const {
        return parity_calls + stop_bits_calls + options_calls + flow_control_calls;
    }

protected:
    uint32_t _available() override { return 0; }
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override {}
    void _end() override {}
    void _flush() override {}
    size_t _write(const uint8_t *buffer, size_t size) override { return size; }
    ssize_t _read(uint8_t *buf, uint16_t count) override { return 0; }
    bool _discard_input() override { return true; }

    void _configure_parity(uint8_t v) override {
        parity_calls++;
        last_parity = v;
        parity = v;
    }
    void _set_stop_bits(int n) override {
        stop_bits_calls++;
        last_stop_bits = n;
        _stop_bits = n;
    }
    bool _set_options(uint16_t options) override {
        options_calls++;
        _last_options = options;
        return true;
    }
    void _set_flow_control(enum flow_control flow_control_setting) override {
        flow_control_calls++;
    }
};

static const uint32_t key = 0x12345678;

// the line can be changed on an unlocked port
TEST(UARTDriverLock, UnlockedPortCanBeChanged)
{
    static LineCountUart uart;  // static so it starts zeroed, like a real port
    uart.configure_parity(2);
    uart.set_stop_bits(2);
    EXPECT_TRUE(uart.set_options(AP_HAL::UARTDriver::OPTION_RXINV));
    uart.set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    EXPECT_EQ(uart.total_calls(), 4U);
    EXPECT_EQ(uart.get_parity(), 2U);
    EXPECT_EQ(uart.last_stop_bits, 2);
    EXPECT_EQ(uart.get_options(), uint16_t(AP_HAL::UARTDriver::OPTION_RXINV));
}

// another user of a locked port can't change its line, as on some
// boards that restarts the UART and loses the lock holder's data
TEST(UARTDriverLock, LockedPortIgnoresChanges)
{
    static LineCountUart uart;  // static so it starts zeroed, like a real port
    ASSERT_TRUE(uart.lock_port(key, key));
    uart.configure_parity(2);
    uart.set_stop_bits(2);
    EXPECT_FALSE(uart.set_options(AP_HAL::UARTDriver::OPTION_RXINV));
    uart.set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    EXPECT_EQ(uart.total_calls(), 0U);
    EXPECT_EQ(uart.get_parity(), 0U);
    EXPECT_EQ(uart.get_options(), 0U);

    // a wrong key doesn't help either
    uart.configure_parity_locked(2, key + 1);
    uart.set_stop_bits_locked(2, key + 1);
    EXPECT_FALSE(uart.set_options_locked(AP_HAL::UARTDriver::OPTION_RXINV, key + 1));
    EXPECT_EQ(uart.total_calls(), 0U);
}

// the lock holder can change the line with its key
TEST(UARTDriverLock, LockHolderCanChangeLine)
{
    static LineCountUart uart;  // static so it starts zeroed, like a real port
    ASSERT_TRUE(uart.lock_port(key, key));
    uart.configure_parity_locked(1, key);
    uart.set_stop_bits_locked(2, key);
    EXPECT_TRUE(uart.set_options_locked(AP_HAL::UARTDriver::OPTION_RXINV, key));
    EXPECT_EQ(uart.parity_calls, 1U);
    EXPECT_EQ(uart.stop_bits_calls, 1U);
    EXPECT_EQ(uart.options_calls, 1U);
    EXPECT_EQ(uart.get_parity(), 1U);
    EXPECT_EQ(uart.last_stop_bits, 2);
    EXPECT_EQ(uart.get_options(), uint16_t(AP_HAL::UARTDriver::OPTION_RXINV));
}

// the stop bits last set are reported, so a lock holder can put them back,
// and a refused change leaves them alone
TEST(UARTDriverLock, StopBitsReported)
{
    static LineCountUart uart;  // static so it starts zeroed, like a real port
    EXPECT_EQ(uart.get_stop_bits(), 1U);
    uart.set_stop_bits(2);
    EXPECT_EQ(uart.get_stop_bits(), 2U);
    ASSERT_TRUE(uart.lock_port(key, key));
    uart.set_stop_bits(1);
    EXPECT_EQ(uart.get_stop_bits(), 2U);
    uart.set_stop_bits_locked(1, key);
    EXPECT_EQ(uart.get_stop_bits(), 1U);
    ASSERT_TRUE(uart.lock_port(0, 0));
}

// once the port is unlocked other users can change it again
TEST(UARTDriverLock, UnlockAllowsChangesAgain)
{
    static LineCountUart uart;  // static so it starts zeroed, like a real port
    ASSERT_TRUE(uart.lock_port(key, key));
    uart.configure_parity(2);
    EXPECT_EQ(uart.parity_calls, 0U);
    ASSERT_TRUE(uart.lock_port(0, 0));
    uart.configure_parity(2);
    uart.set_stop_bits(2);
    EXPECT_TRUE(uart.set_options(0));
    uart.set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    EXPECT_EQ(uart.total_calls(), 4U);
    EXPECT_EQ(uart.get_parity(), 2U);
}

AP_GTEST_MAIN()
