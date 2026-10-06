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

// A negative resistance passed to maybe_reset() means "leave the
// existing resistance unchanged".  This is also the default
constexpr float keep_resistance = -1.0f;

// This can be any value, in operation it would come from AP_HAL::micros64().
constexpr uint64_t initial_us = 0;

// This is arbitrary
constexpr float ambient_temperature_degC = 0.0f;

class BatteryTest : public testing::Test {
protected:
    BatteryTest() {
        small_battery.setup(small_capacity_Ah, low_resistance_ohm, max_voltage, ambient_temperature_degC);
        large_battery.setup(large_capacity_Ah, low_resistance_ohm, max_voltage, ambient_temperature_degC);
        // Recall that capacity==0 means unlimited.
        infinite_battery.setup(0.0f, low_resistance_ohm, max_voltage, ambient_temperature_degC);

        small_high_resistance_battery.setup(small_capacity_Ah, high_resistance_ohm, max_voltage, ambient_temperature_degC);
        // Recall that capacity==0 means unlimited.
        infinite_high_resistance_battery.setup(0.0f, high_resistance_ohm, max_voltage, ambient_temperature_degC);
    }

    struct BattAndObservations {
        SITL::Battery& batt;
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
        { small_battery, -1.0f, -2.0f },
        { large_battery, -1.0f, -2.0f },
        { infinite_battery, -1.0f, -2.0f },
        { small_high_resistance_battery, -1.0f, -2.0f },
        { infinite_high_resistance_battery, -1.0f, -2.0f },
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
        SITL::Battery& battery = b_and_d.batt;
        float& min_observed_voltage = b_and_d.min_observed_voltage;

        const float initial_voltage = battery.get_voltage();
        min_observed_voltage = initial_voltage;
        const float initial_temperature_degC = battery.get_temperature_degC();
        float prev_temperature_degC = initial_temperature_degC;

        for (float t = 0.0f; t <= test_duration_sec; t+=dt_sec) {
            const uint64_t now_us = initial_us + static_cast<uint64_t>(t * 1e6);

            // Consume battery energy (or not)
            if (t >= 0.0f && t < first_half) {
                battery.consume_energy(current_amp, now_us);
            } else {
                battery.consume_energy(0.0f, now_us);
            }

            // Confirm temperature rise
            if (is_zero(t)) {
                EXPECT_FLOAT_EQ(battery.get_temperature_degC(), initial_temperature_degC);
            } else {
                // Regardless of whether temp is rising (first half of test) or cooling (second half),
                // it should be somewhat higher than the initial temp.
                // (This test does not set an expectation on how much higher.)
                EXPECT_GT(battery.get_temperature_degC(), initial_temperature_degC);
            }

            // Confirm temperature-change direction
            if (t > 0.0f) {
                if (t < first_half) {
                    // First half: consuming => temperature increasing
                    // (Note: the test parameters are tuned so that the consumption half ends before temp reaches steady-state.)
                    EXPECT_GT(battery.get_temperature_degC(), prev_temperature_degC);
                } else {
                    // Second half: resting => temperature decreasing
                    // (Note: the test parameters are tuned so that the test ends before temp returns to steady-state.)
                    EXPECT_LT(battery.get_temperature_degC(), prev_temperature_degC);
                }
                prev_temperature_degC = battery.get_temperature_degC();
            }

            // Confirm voltage drop works as expected.
            const float observed_voltage = battery.get_voltage();
            if (is_zero(t)) {
                EXPECT_FLOAT_EQ(observed_voltage, initial_voltage);
                EXPECT_FLOAT_EQ(observed_voltage, min_observed_voltage);
            }
            else if (t <= first_half) {
                // During consumption, voltage will at least sag below initial voltage.
                EXPECT_LT(observed_voltage, initial_voltage);
                if (!battery.capacity_is_unlimited()) {
                    // Finite-capacity batteries will also be losing total voltage.
                    EXPECT_LT(observed_voltage, min_observed_voltage);
                } else {
                    // Unlimited-capacity batteries stop losing voltage at steady-state sag.
                    EXPECT_LE(observed_voltage, min_observed_voltage);
                }
            } else {
                // After consumption, voltage will rise back from lowest value to resting value.
                EXPECT_GT(observed_voltage, min_observed_voltage);
                if (!battery.capacity_is_unlimited()) {
                    // For finite-capacity batteries, it will be strictly less than initial, because some charge was depleted.
                    EXPECT_LT(observed_voltage, initial_voltage);
                } else {
                    // Unlimited-capacity batteries recover completely from the sag back to initial.
                    EXPECT_LE(observed_voltage, initial_voltage);
                }
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

    // Infinite battery => no resting voltage loss
    EXPECT_FLOAT_EQ(batteries_and_data[infinite].final_observed_voltage, max_voltage);
    EXPECT_LT(batteries_and_data[infinite].min_observed_voltage, max_voltage);
    EXPECT_FLOAT_EQ(batteries_and_data[infinite_high_resist].final_observed_voltage, max_voltage);
    EXPECT_LT(batteries_and_data[infinite_high_resist].min_observed_voltage, max_voltage);

    // Higher resistance (with same current + time) => more voltage sag
    EXPECT_LT(batteries_and_data[small_high_resist].min_observed_voltage,
              batteries_and_data[small].min_observed_voltage);
    // Higher resistance does not impact resting voltage
    EXPECT_FLOAT_EQ(batteries_and_data[small_high_resist].final_observed_voltage,
                    batteries_and_data[small].final_observed_voltage);
    EXPECT_FLOAT_EQ(batteries_and_data[infinite_high_resist].final_observed_voltage,
                    batteries_and_data[infinite].final_observed_voltage);
}

TEST_F(BatteryTest, MaximumDeltaTime)
{
    // For this test, value must be larger than SIM::Battery's maximum permissible dt.
    constexpr float dt_sec = 0.11f;

    constexpr float current_amp = 25.0f; // This value is arbitrary

    for (auto& b_and_d : batteries_and_data) {
        SITL::Battery& battery = b_and_d.batt;
        const float initial_voltage = battery.get_voltage();
        const float initial_temperature_degC = battery.get_temperature_degC();

        for (float t = 0.0f; t <= dt_sec * 10; t+=dt_sec) {
            const uint64_t now_us = initial_us + static_cast<uint64_t>(t * 1e6);

            // Attempt to consume battery energy (but does not work because dt is too large)
            battery.consume_energy(current_amp, now_us);

            // Confirm no voltage or temperature change (because energy-consumption did not work)
            EXPECT_FLOAT_EQ(battery.get_voltage(), initial_voltage);
            EXPECT_FLOAT_EQ(battery.get_temperature_degC(), initial_temperature_degC);
        }
    }
}

TEST_F(BatteryTest, RestingVoltage)
{
    constexpr float current_amp = 25.0f;
    constexpr float consumption_duration_sec = 10.0f;
    constexpr float long_enough_for_steady_state_sec = 12.0f;
    constexpr float steady_state_diff_threshold_voltage = 1e-3f;
    constexpr float dt_sec = 0.01f;

    for (auto& b_and_d : batteries_and_data) {
        SITL::Battery& battery = b_and_d.batt;
        float& min_observed_voltage = b_and_d.min_observed_voltage;
        const float initial_voltage = battery.get_voltage();
        min_observed_voltage = initial_voltage;

        // The consumption period.
        for (float t = 0.0f; t < consumption_duration_sec; t+=dt_sec) {
            const uint64_t now_us = initial_us + static_cast<uint64_t>(t * 1e6);
            battery.consume_energy(current_amp, now_us);
            const float observed_voltage = battery.get_voltage();
            if (!is_zero(t)) {
                EXPECT_LT(observed_voltage, initial_voltage);
            }
            min_observed_voltage = MIN(observed_voltage, min_observed_voltage);
        }

        // Show that battery's voltage is at least lower than the steady-state threshold.
        // (This is most meaningful for unlimited-capacity batteries, but always true.)
        EXPECT_LT(battery.get_voltage(), initial_voltage - steady_state_diff_threshold_voltage);

        // The rest period.
        constexpr float one_dt_after_consumption = consumption_duration_sec + dt_sec;
        for (float t = one_dt_after_consumption; t <= long_enough_for_steady_state_sec; t+=dt_sec) {
            const uint64_t now_us = initial_us + static_cast<uint64_t>(t * 1e6);
            battery.consume_energy(0.0f, now_us);
            EXPECT_GT(battery.get_voltage(), min_observed_voltage);
        }

        // Show that infinite-capacity battery returns to initial voltage but finite does not.
        if (battery.capacity_is_unlimited()) {
            EXPECT_NEAR(battery.get_voltage(), initial_voltage, steady_state_diff_threshold_voltage);
        } else {
            EXPECT_LT(battery.get_voltage(), initial_voltage - steady_state_diff_threshold_voltage);
        }

    }
}

namespace {
void use_some_energy(SITL::Battery& battery, float rest_duration_sec = 0.0f) {
    constexpr float current_amp = 25.0f;
    constexpr float consume_energy_duration_sec = 60.0f;
    const float total_duration_sec = consume_energy_duration_sec + rest_duration_sec;
    constexpr float dt_sec = 0.01f;

    for (float t = 0.0f; t <= total_duration_sec; t+=dt_sec) {
        const uint64_t now_us = initial_us + static_cast<uint64_t>(t * 1e6);
        if (t <= consume_energy_duration_sec) {
            battery.consume_energy(current_amp, now_us);
        }
        else {
            battery.consume_energy(0.0f, now_us);
        }
    }
};
} // namespace

TEST_F(BatteryTest, Resetting)
{
    for (auto& b_and_d : batteries_and_data) {
        SITL::Battery& battery = b_and_d.batt;

        // Show that resetting to some new voltage works.
        const float partial_voltage = 0.8f * max_voltage;
        battery.maybe_reset(partial_voltage, battery.get_capacity());
        EXPECT_FLOAT_EQ(battery.get_voltage(), partial_voltage);

        // Show that resetting to zero & negative voltages works.
        for (auto& voltage : {0.0f, -0.5f}) {
            battery.maybe_reset(voltage, battery.get_capacity());
            EXPECT_FLOAT_EQ(battery.get_voltage(), voltage);
        }

        // Show that resetting to the initial voltage works.
        battery.maybe_reset(max_voltage, battery.get_capacity());
        EXPECT_FLOAT_EQ(battery.get_voltage(), max_voltage);

        // Show that attempting a reset without changing any batt params is a no-op.
        use_some_energy(battery);
        if (!battery.capacity_is_unlimited()) {
            // Show that some voltage has been lost
            float observed = battery.get_voltage();
            EXPECT_LT(observed, max_voltage);

            battery.maybe_reset(max_voltage, battery.get_capacity());
            // Show reset did nothing
            EXPECT_FLOAT_EQ(battery.get_voltage(), observed);
        }

        // Show that resetting to higher-than-max voltage resets to max, but not higher
        battery.maybe_reset(higher_than_max_voltage, battery.get_capacity());
        EXPECT_FLOAT_EQ(battery.get_voltage(), max_voltage);
        EXPECT_LT(battery.get_voltage(), higher_than_max_voltage);

        // Show that switching from limited to unlimited capacity (or vice versa) works
        if (battery.capacity_is_unlimited()) {
            battery.maybe_reset(max_voltage, small_capacity_Ah);
            EXPECT_FLOAT_EQ(battery.get_voltage(), max_voltage);
            EXPECT_FLOAT_EQ(battery.get_capacity(), small_capacity_Ah);
        } else {
            battery.maybe_reset(max_voltage, 0.0f);
            EXPECT_FLOAT_EQ(battery.get_voltage(), max_voltage);
            EXPECT_FLOAT_EQ(battery.get_capacity(), 0.0f);
        }
        use_some_energy(battery, 30.0f);
        // Show that now-unlimited batteries do not lose voltage, and now-limited ones do.
        if (battery.capacity_is_unlimited()) {
            EXPECT_FLOAT_EQ(battery.get_voltage(), max_voltage);
        } else {
            EXPECT_LT(battery.get_voltage(), max_voltage);
        }
        // (Don't add further tests here, batteries no longer match their names.)
    }
}

TEST_F(BatteryTest, ResistanceOverride)
{
    // Unlimited capacity keeps the resting voltage pinned at max, so any drop in
    // the observed voltage is purely sag (current * resistance).
    SITL::Battery battery;
    // The initial resistance is arbitrary, as this test will vary it as-needed.
    battery.setup(0.0f, high_resistance_ohm, max_voltage, ambient_temperature_degC);

    constexpr float current_amp = 25.0f;
    constexpr float dt_sec = 0.01f;
    constexpr float settle_sec = 5.0f;  // many time-constants of the 10Hz voltage filter
    uint64_t now_us = initial_us;

    // Apply the requested resistance, then drive a constant current long enough for
    // the voltage filter to settle, and return the resulting sagged voltage.
    auto sagged_voltage = [&](float resistance_ohm) -> float {
        battery.maybe_reset(max_voltage, battery.get_capacity(), resistance_ohm);
        for (float t = 0.0f; t < settle_sec; t += dt_sec) {
            now_us += static_cast<uint64_t>(dt_sec * 1e6);
            battery.consume_energy(current_amp, now_us);
        }
        return battery.get_voltage();
    };

    // Zero resistance disables sag entirely.
    EXPECT_FLOAT_EQ(sagged_voltage(0.0f), max_voltage);

    // A negative resistance leaves the previous resistance (still zero) in place.
    EXPECT_FLOAT_EQ(sagged_voltage(keep_resistance), max_voltage);

    // A positive resistance sags the voltage by current * resistance...
    const float low_sag = sagged_voltage(low_resistance_ohm);
    EXPECT_NEAR(low_sag, max_voltage - current_amp * low_resistance_ohm, 1e-3f);

    // ...and a higher resistance sags more.
    const float high_sag = sagged_voltage(high_resistance_ohm);
    EXPECT_LT(high_sag, low_sag);
    EXPECT_NEAR(high_sag, max_voltage - current_amp * high_resistance_ohm, 1e-3f);
}

AP_GTEST_MAIN()
