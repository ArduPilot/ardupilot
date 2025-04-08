/*
  test that SBUS decoding matches SBUS encoding
 */
#include <AP_gtest.h>
#include <AP_SBusOut/AP_SBusOut.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#include <AP_RCProtocol/AP_RCProtocol_SBUS.h>

#define SBUS_RANGE_MIN 200
#define SBUS_RANGE_MAX 1800
#define SBUS_RANGE_RANGE (SBUS_RANGE_MAX - SBUS_RANGE_MIN)

#define SBUS_TARGET_MIN 1000
#define SBUS_TARGET_MAX 2000
#define SBUS_TARGET_RANGE (SBUS_TARGET_MAX - SBUS_TARGET_MIN)

// this is 875
#define SBUS_SCALE_OFFSET (SBUS_TARGET_MIN - ((SBUS_TARGET_RANGE * SBUS_RANGE_MIN / SBUS_RANGE_RANGE)))

TEST(SBUSEncodeDecode, test_sbus_encode_decode)
{
    const uint8_t num_channels = 8;
    uint16_t values_in[num_channels];
    uint16_t values_out[num_channels];
    uint8_t frame[25];

    for (uint16_t v=875;v<2155; v++) {
        for (uint8_t i=0; i<num_channels; i++) {
            values_in[i] = v;
        }
        AP_SBusOut::sbus_format_frame(values_in, num_channels, frame);
        uint16_t num_values = 0;
        bool sbus_failsafe=false;
        AP_RCProtocol_SBUS::sbus_decode(frame, values_out, &num_values, sbus_failsafe, num_channels);
        EXPECT_EQ(sbus_failsafe, values_in[0] == 875);
        for (uint8_t i=0; i<num_channels; i++) {
            if (values_in[i] != values_out[i]) {
                EXPECT_EQ(values_in[i], values_out[i]);
            }
        }
    }
}


AP_GTEST_MAIN()
int hal = 0;
