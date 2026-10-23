#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LWI2C_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_I2C.h"

#define NUM_SF20_DATA_STREAMS 1

class AP_RangeFinder_LightWareI2C : public AP_RangeFinder_Backend_I2C
{

public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          class AP_HAL::I2CDevice &dev) {
        // this will free the object if configuration fails:
        return configure(NEW_NOTHROW AP_RangeFinder_LightWareI2C(_state, _params, dev));
    }

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override
    {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    float sf20_stream_val[NUM_SF20_DATA_STREAMS];
    int currentStreamSequenceIndex = 0;

    // constructor
    using AP_RangeFinder_Backend_I2C::AP_RangeFinder_Backend_I2C;

    bool write_bytes(uint8_t *write_buf_u8, uint32_t len_u8);
    void sf20_disable_address_tagging();
    bool sf20_send_and_expect(const char* send, const char* expected_reply);
    bool sf20_set_lost_signal_confirmations();
    void sf20_get_version(const char* send_msg, const char *reply_prefix, char *reply, uint8_t reply_len);
    bool sf20_wait_on_reply(uint8_t *rx_two_bytes);
    bool init() override;
    bool legacy_init();
    bool sf20_init();
    void sf20_init_streamRecovery();
    void legacy_timer();
    void sf20_timer();

    // get a reading
    bool legacy_get_reading(float &reading_m);
    bool sf20_get_reading(float &reading_m);
    bool sf20_parse_stream(uint8_t *stream_buf,
                           size_t *p_num_processed_chars,
                           const char *string_identifier,
                           float &val);
};

#endif  // AP_RANGEFINDER_LWI2C_ENABLED
