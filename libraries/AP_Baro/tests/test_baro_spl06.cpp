#include <AP_gtest.h>

#include <AP_Baro/AP_Baro_SPL06.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <cstring>
#include <memory>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
AP_Vehicle& vehicle = *AP_Vehicle::get_singleton();

#if AP_BARO_SPL06_ENABLED

class SPL06Device : public AP_HAL::Device
{
public:
    SPL06Device() : Device(BUS_TYPE_I2C)
    {
        registers[0x0D] = 0x11;
        registers[0x08] = 0xC0;
        // c0=40, c1=2, c00=100000, c10=100, c01=20, c11=30,
        // c20=10, c21=40, c30=1, c31=50, c40=60.
        const uint8_t coeffs[] = {2, 0x80, 2, 0x18, 0x6A, 0, 0, 100,
                                  0, 20, 0, 30, 0, 10, 0, 40, 0, 1, 3, 0x20, 60
                                 };
        memcpy(&registers[0x10], coeffs, sizeof(coeffs));
        // Scaled pressure and temperature are both 1 at 8x oversampling.
        registers[0x00] = 0x78;
        registers[0x03] = 0x78;
    }

    bool set_speed(Speed) override
    {
        return true;
    }

    bool transfer(const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len) override
    {
        if (fail_reads) {
            return false;
        }
        if (send_len == 1 && recv != nullptr && send[0] + recv_len <= sizeof(registers)) {
            if (send[0] >= 0x10) {
                if (fail_coeffs) {
                    return false;
                }
                coeff_bytes += recv_len;
            }
            memcpy(recv, &registers[send[0]], recv_len);
            return true;
        }
        if (send_len == 2 && recv_len == 0 && send[0] < sizeof(registers)) {
            registers[send[0]] = send[1];
            return true;
        }
        return false;
    }

    AP_HAL::Semaphore *get_semaphore() override
    {
        return &semaphore;
    }

    PeriodicHandle register_periodic_callback(uint32_t period_usec, PeriodicCb cb) override
    {
        period = period_usec;
        callback = cb;
        return this;
    }

    bool adjust_periodic_callback(PeriodicHandle, uint32_t) override
    {
        return false;
    }

    uint8_t registers[0x25] {};
    uint32_t coeff_bytes = 0;
    uint32_t period = 0;
    bool fail_reads = false;
    bool fail_coeffs = false;
    PeriodicCb callback;

private:
    HAL_Semaphore semaphore;
};

struct SensorCase {
    uint8_t chip_id;
    uint16_t options;
    uint8_t devtype;
    uint8_t coeff_bytes;
    float pressure;
    float half_pressure;
};

class SPL06Test : public ::testing::TestWithParam<SensorCase> {};

TEST_P(SPL06Test, DetectConfigureAndCompensate)
{
    std::unique_ptr<AP_Baro> baro(NEW_NOTHROW AP_Baro());
    std::unique_ptr<SPL06Device> dev(NEW_NOTHROW SPL06Device());
    const auto test = GetParam();
    dev->registers[0x0D] = test.chip_id;
    ASSERT_TRUE(AP_Param::set_object_value(baro.get(), AP_Baro::var_info, "_OPTIONS", test.options));
    std::unique_ptr<AP_Baro_Backend> backend(AP_Baro_SPL06::probe(*baro, *dev));
    ASSERT_NE(backend, nullptr);
    EXPECT_EQ((dev->get_bus_id() >> 16) & 0xFF, test.devtype);
    EXPECT_EQ(dev->coeff_bytes, test.coeff_bytes);
    ASSERT_TRUE(dev->callback);
    if (dev->period == 31250) {
        EXPECT_EQ(dev->registers[0x06], 0x53);
        EXPECT_EQ(dev->registers[0x07], test.chip_id == 0x10 ? 0xD3 : 0x53);
        EXPECT_EQ(dev->registers[0x08], 7);
    } else {
        EXPECT_EQ(dev->period, 20000U);
        EXPECT_EQ(dev->registers[0x06], test.devtype == 0x19 ? 0x53 : 0x03);
        EXPECT_EQ(dev->registers[0x07], test.chip_id == 0x10 ? 0x83 : (test.devtype == 0x19 ? 0x53 : 0x03));
    }
    // Three callbacks cover temperature and pressure in command mode too.
    for (uint8_t i = 0; i < 3; i++) {
        dev->callback();
    }
    backend->update();
    EXPECT_FLOAT_EQ(baro->get_pressure(0), test.pressure);
    EXPECT_FLOAT_EQ(baro->get_temperature(0), 22.0f);

    dev->registers[0x00] = 0x3C;
    dev->registers[0x03] = 0x3C;
    // Command mode only refreshes temperature once per second.
    for (uint8_t i = 0; i < 51; i++) {
        dev->callback();
    }
    backend->update();
    for (uint8_t i = 0; i < 3; i++) {
        dev->callback();
    }
    backend->update();
    EXPECT_FLOAT_EQ(baro->get_pressure(0), test.half_pressure);
    EXPECT_FLOAT_EQ(baro->get_temperature(0), 21.0f);
}

INSTANTIATE_TEST_SUITE_P(Sensors, SPL06Test, ::testing::Values(
                             SensorCase{0x10, 0, 0x0C, 18, 100201, 100075.125f},
                             SensorCase{0x10, 2, 0x0C, 18, 100201, 100075.125f},
                             SensorCase{0x11, 0, 0x16, 21, 100311, 100085.125f},
                             SensorCase{0x11, 1, 0x16, 21, 100311, 100085.125f},
                             SensorCase{0x11, 2, 0x19, 21, 100251, 100081.375f},
                             SensorCase{0x11, 3, 0x19, 21, 100251, 100081.375f}));

TEST(SPL06, DefaultOptions)
{
    std::unique_ptr<AP_Baro> baro(NEW_NOTHROW AP_Baro());
    uint16_t expected = 0;
#ifdef AP_BARO_OPTIONS_DEFAULT
    expected = AP_BARO_OPTIONS_DEFAULT;
#elif defined(AP_BARO_OPTION_SPA06_AS_SPL07)
    expected = AP_BARO_OPTION_SPA06_AS_SPL07 << 1;
#endif
    EXPECT_EQ(baro->option_enabled(AP_Baro::Options::TreatSPA06AsSPL07), (expected & 2) != 0);
    EXPECT_EQ(baro->option_enabled(AP_Baro::Options::TreatMS5611AsMS5607), (expected & 1) != 0);
}

TEST(SPL06, RejectUnknownChip)
{
    std::unique_ptr<AP_Baro> baro(NEW_NOTHROW AP_Baro());
    std::unique_ptr<SPL06Device> dev(NEW_NOTHROW SPL06Device());
    dev->registers[0x0D] = 0xFF;
    std::unique_ptr<AP_Baro_Backend> backend(AP_Baro_SPL06::probe(*baro, *dev));
    EXPECT_EQ(backend, nullptr);
    EXPECT_EQ(dev->coeff_bytes, 0U);
    EXPECT_FALSE(dev->callback);
}

TEST(SPL06, RejectReadFailure)
{
    std::unique_ptr<AP_Baro> baro(NEW_NOTHROW AP_Baro());
    std::unique_ptr<SPL06Device> dev(NEW_NOTHROW SPL06Device());
    dev->fail_reads = true;
    std::unique_ptr<AP_Baro_Backend> backend(AP_Baro_SPL06::probe(*baro, *dev));
    EXPECT_EQ(backend, nullptr);
    EXPECT_FALSE(dev->callback);
}

TEST(SPL06, RejectCalibrationReadFailure)
{
    std::unique_ptr<AP_Baro> baro(NEW_NOTHROW AP_Baro());
    std::unique_ptr<SPL06Device> dev(NEW_NOTHROW SPL06Device());
    dev->fail_coeffs = true;
    std::unique_ptr<AP_Baro_Backend> backend(AP_Baro_SPL06::probe(*baro, *dev));
    EXPECT_EQ(backend, nullptr);
    EXPECT_FALSE(dev->callback);
}

#endif // AP_BARO_SPL06_ENABLED

AP_GTEST_MAIN()
