#include <AP_gtest.h>

#include <SITL/SIM_Battery.h>
#include <AP_Math/AP_Math.h>

// This is required because SITL::Battery uses AP_HAL::micros64() internally, even though these tests do not.
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// These values are arbitrary: if any reasonable value (e.g. 1 to 100 V) causes a test failure, something is wrong.
constexpr float max_voltage = 10.0f;
constexpr float higher_than_max_voltage = max_voltage + 1.2f;

// These values are arbitrary, need only be 'small' and 'large' compared to each other.
constexpr float small_capacity_Ah = 1.0f;
constexpr float large_capacity_Ah = 8.0f;

// These values are arbitrary, need only be 'low' and 'high' compared to each other.
constexpr float low_resistance_ohm = 0.005f;
constexpr float high_resistance_ohm = 0.04f;

// This can be any value, in operation it would come from AP_HAL::micros64().
constexpr uint64_t initial_us = 0;

class BatteryTest : public testing::Test {
protected:
    BatteryTest() {
        small_battery.setup(small_capacity_Ah, low_resistance_ohm, max_voltage);
        small_battery.init_voltage(max_voltage);
        small_battery.init_capacity(small_capacity_Ah);
        large_battery.setup(large_capacity_Ah, low_resistance_ohm, max_voltage);
        large_battery.init_voltage(max_voltage);
        large_battery.init_capacity(large_capacity_Ah);
        // Recall that capacity==0 means unlimited.
        infinite_battery.setup(0.0f, low_resistance_ohm, max_voltage);
        infinite_battery.init_voltage(max_voltage);
        infinite_battery.init_capacity(0.0f);

        small_high_resistance_battery.setup(small_capacity_Ah, high_resistance_ohm, max_voltage);
        small_high_resistance_battery.init_voltage(max_voltage);
        small_high_resistance_battery.init_capacity(small_capacity_Ah);
        // Recall that capacity==0 means unlimited.
        infinite_high_resistance_battery.setup(0.0f, high_resistance_ohm, max_voltage);
        infinite_high_resistance_battery.init_voltage(max_voltage);
        infinite_high_resistance_battery.init_capacity(0.0f);
    }

    struct BattAndObservations {
        SITL::Battery* batt;
        float min_observed_voltage;
        float final_observed_voltage;
    };

    SITL::Battery small_battery;
    SITL::Battery large_battery;
    SITL::Battery infinite_battery;
    SITL::Battery small_high_resistance_battery;
    SITL::Battery infinite_high_resistance_battery;

    enum class BattKey {
        Small = 0,
        Large,
        Infinite,
        SmallHighResist,
        InfiniteHighResist
    };

    BattAndObservations batteries_and_data[5] = {
        { &small_battery, -1.0f, -2.0f },
        { &large_battery, -1.0f, -2.0f },
        { &infinite_battery, -1.0f, -2.0f },
        { &small_high_resistance_battery, -1.0f, -2.0f },
        { &infinite_high_resistance_battery, -1.0f, -2.0f },
    };

    // These are just syntactic sugar.
    size_t small = static_cast<size_t>(BattKey::Small);
    size_t large = static_cast<size_t>(BattKey::Large);
    size_t infinite = static_cast<size_t>(BattKey::Infinite);
    size_t small_high_resist = static_cast<size_t>(BattKey::SmallHighResist);
    size_t infinite_high_resist = static_cast<size_t>(BattKey::InfiniteHighResist);
};

TEST_F(BatteryTest, EnergyConsumption)
{
    constexpr float current_amp = 25.0f;
    constexpr float test_duration_sec = 60.0f;
    constexpr float first_half = test_duration_sec / 2.0f;
    constexpr float dt_sec = 0.01f;

    for (auto& b_and_d : batteries_and_data) {
        SITL::Battery& battery = std::ref(*b_and_d.batt);
        float& min_observed_voltage = b_and_d.min_observed_voltage;

        const float initial_voltage = battery.get_voltage();
        min_observed_voltage = initial_voltage;
        const float initial_temperature = battery.get_temperature();
        float prev_temperature = initial_temperature;

        for (float t = 0.0f; t <= test_duration_sec; t+=dt_sec) {
            const uint64_t now_us = initial_us + static_cast<uint64_t>(t * 1e6);

            // Consume battery energy (or not)
            if (t > 0.0f && t < first_half) {
                battery.consume_energy(current_amp, now_us);
            } else {
                battery.consume_energy(0.0f, now_us);
            }

            // Confirm temperature rise
            if (is_zero(t)) {
                EXPECT_FLOAT_EQ(battery.get_temperature(), initial_temperature);
            } else {
                // Regardless of whether temp is rising (first half of test) or cooling (second half),
                // it should be somewhat higher than the initial temp.
                // (This test does not set an expectation on how much higher.)
                EXPECT_GT(battery.get_temperature(), initial_temperature);
            }

            // Confirm temperature-change direction
            if (t > 0.0f) {
                if (t < first_half) {
                    // First half: consuming => temperature increasing
                    // (Note: the test parameters are tuned so that the consumption half ends before temp reaches steady-state.)
                    EXPECT_GT(battery.get_temperature(), prev_temperature);
                } else {
                    // Second half: resting => temperature decreasing
                    // (Note: the test parameters are tuned so that the test ends before temp returns to steady-state.)
                    EXPECT_LT(battery.get_temperature(), prev_temperature);
                }
            }
            prev_temperature = battery.get_temperature();

            // Confirm voltage drop
            if (is_zero(t) || is_zero(battery.get_capacity())) {
                EXPECT_FLOAT_EQ(battery.get_voltage(), initial_voltage);
            } else {
                // During consumption, both voltage sag and capacity-loss contribute to voltage < initial.
                // During rest, voltage sag will disappear but capacity-loss still means voltage < initial.
                EXPECT_LT(battery.get_voltage(), initial_voltage);
            }

            // Confirm voltage drop works as expected.
            const float observed_voltage = battery.get_voltage();
            if (is_zero(t) || is_zero(battery.get_capacity())) {
                EXPECT_FLOAT_EQ(observed_voltage, initial_voltage);
                EXPECT_FLOAT_EQ(observed_voltage, min_observed_voltage);
            }
            else if (t <= first_half) {
                // During consumption, voltage should always be lower than before.
                EXPECT_LE(observed_voltage, min_observed_voltage);
                // And it should definitely be less than the initial voltage.
                EXPECT_LE(observed_voltage, initial_voltage);
            } else {
                // After consumption, voltage will rise back from lowest value to resting value
                EXPECT_GE(observed_voltage, min_observed_voltage);
                // But it will always be less than initial, because some charge was depleted
                EXPECT_LE(observed_voltage, initial_voltage);
            }
            min_observed_voltage = MIN(observed_voltage, min_observed_voltage);
        }
        b_and_d.final_observed_voltage = battery.get_voltage();
    }

    // Smaller capacity => more voltage loss
    EXPECT_LT(batteries_and_data[small].final_observed_voltage,
              batteries_and_data[large].final_observed_voltage);
    EXPECT_LT(batteries_and_data[small].min_observed_voltage,
              batteries_and_data[large].min_observed_voltage);

    EXPECT_LT(batteries_and_data[large].final_observed_voltage,
              batteries_and_data[infinite].final_observed_voltage);
    EXPECT_LT(batteries_and_data[large].min_observed_voltage,
              batteries_and_data[infinite].min_observed_voltage);

    // Infinite battery => no voltage loss
    EXPECT_FLOAT_EQ(batteries_and_data[infinite].final_observed_voltage, max_voltage);
    EXPECT_FLOAT_EQ(batteries_and_data[infinite].min_observed_voltage, max_voltage);
    EXPECT_FLOAT_EQ(batteries_and_data[infinite_high_resist].final_observed_voltage, max_voltage);
    EXPECT_FLOAT_EQ(batteries_and_data[infinite_high_resist].min_observed_voltage, max_voltage);

    // Higher resistance (with same current + time) => more voltage sag
    EXPECT_LT(batteries_and_data[small_high_resist].min_observed_voltage,
              batteries_and_data[small].min_observed_voltage);
    // But not for infinite batteries, of course
    EXPECT_FLOAT_EQ(batteries_and_data[infinite_high_resist].min_observed_voltage,
                    batteries_and_data[infinite].min_observed_voltage);
    // And higher resistance does not impact resting voltage
    EXPECT_FLOAT_EQ(batteries_and_data[small_high_resist].final_observed_voltage,
                    batteries_and_data[small].final_observed_voltage);
    EXPECT_FLOAT_EQ(batteries_and_data[infinite_high_resist].final_observed_voltage,
                    batteries_and_data[infinite].final_observed_voltage);
}

namespace {
void use_some_energy(SITL::Battery& battery) {
    constexpr float current_amp = 25.0f;
    constexpr float current_duration_sec = 60.0f;
    constexpr float dt_sec = 0.01f;

    for (float t = 0.0f; t <= current_duration_sec; t+=dt_sec) {
        battery.consume_energy(current_amp, initial_us + static_cast<uint64_t>(t * 1e6));
    }
};
} // namespace

TEST_F(BatteryTest, Resetting)
{
    for (auto& b_and_d : batteries_and_data) {
        SITL::Battery& battery = std::ref(*b_and_d.batt);

        const float initial_voltage = battery.get_voltage();

        // Show that voltage-based reset works.
        use_some_energy(battery);
        if (is_positive(battery.get_capacity())) {
            // Show that some voltage has been lost
            EXPECT_LT(battery.get_voltage(), initial_voltage);
            // Reset to initial value using voltage
            battery.init_voltage(initial_voltage);
            // Show it worked
            EXPECT_FLOAT_EQ(battery.get_voltage(), initial_voltage);
        }

        // Show that capacity-based reset does not (instantly) impact voltage.
        use_some_energy(battery);
        if (is_positive(battery.get_capacity())) {
            // Show that some voltage has been lost
            const float observed_voltage = battery.get_voltage();
            EXPECT_LT(observed_voltage, initial_voltage);
            // Reset to initial capacity
            battery.init_capacity(battery.get_capacity());
            // Show voltage did not change
            EXPECT_FLOAT_EQ(battery.get_voltage(), observed_voltage);
        }

        // Show that reset-to-partial-voltage works.
        const float partial_voltage = 0.8f * max_voltage;
        battery.init_voltage(partial_voltage);
        EXPECT_FLOAT_EQ(battery.get_voltage(), partial_voltage);

        // Show that setting higher-than-max voltage has no effect
        battery.init_voltage(higher_than_max_voltage);
        EXPECT_LT(battery.get_voltage(), higher_than_max_voltage);
        EXPECT_FLOAT_EQ(battery.get_voltage(), max_voltage);

        // Show that switching from limited to unlimited capacity (or vice versa) works
        if (is_zero(battery.get_capacity())) {
            battery.init_capacity(small_capacity_Ah);
            EXPECT_FLOAT_EQ(battery.get_voltage(), max_voltage);
            EXPECT_FLOAT_EQ(battery.get_capacity(), small_capacity_Ah);
        } else {
            battery.init_capacity(0.0f);
            EXPECT_FLOAT_EQ(battery.get_voltage(), max_voltage);
            EXPECT_FLOAT_EQ(battery.get_capacity(), 0.0f);
        }
        use_some_energy(battery);
        // Show that now-unlimited batteries do not lose voltage, and now-limited ones do.
        if (is_zero(battery.get_capacity())) {
            EXPECT_FLOAT_EQ(battery.get_voltage(), max_voltage);
        } else {
            EXPECT_LT(battery.get_voltage(), max_voltage);
        }
        // (Don't add further tests here, batteries no longer match their names.)
    }
}

AP_GTEST_MAIN()
