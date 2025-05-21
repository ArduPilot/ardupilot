#include <AP_gtest.h>

#include <SITL/SIM_Aircraft.h>
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace SITL;

// Dummy class to access protected functions through a public interface for unit tests
class dummy : public SITL::Aircraft {

public:
    dummy(const char *frame_str) : SITL::Aircraft(frame_str) {}

    float filtered_servo_angle_pub(const struct sitl_input &input, uint8_t idx);

    // Implement required pure virtual functions
    void update(const struct sitl_input &input) override
    {
        // Do nothing
    }

    float perpendicular_distance_to_rangefinder_surface() const override
    {
        return 0.0;  // Return a default value
    }

    bool on_ground() const override
    {
        return true; // Assume it's on the ground by default
    }
};

float dummy::filtered_servo_angle_pub(const struct sitl_input &input, uint8_t idx)
{
    return dummy::filtered_servo_angle(input, idx);
}

const struct servo_test_values {
    sitl_input input;
    uint8_t idx;
    float expected_angle;
} servo_test_data[] = {
    {{1100}, 0, -0.8},
    {{1190}, 0, -0.62},
    {{1500}, 0, 0.0},
    {{1642}, 0, 0.28},
    {{1900}, 0, 0.8},
    {{0}, 0, 0.0}
};

TEST(Aircraft, filtered_servo_angle)
{
    dummy aircraft("plane");  
    float accuracy = 0.01;

    for (auto elem : servo_test_data) {
        float result = aircraft.filtered_servo_angle_pub(elem.input, elem.idx);
        EXPECT_NEAR(result, elem.expected_angle, accuracy);
    }
}

AP_GTEST_MAIN()
