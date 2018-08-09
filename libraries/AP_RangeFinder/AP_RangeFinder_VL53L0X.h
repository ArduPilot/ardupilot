#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

class AP_RangeFinder_VL53L0X : public AP_RangeFinder_Backend
{

public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update(void) override;

    static void set_addr(AP_HAL::OwnPtr<AP_HAL::I2CDevice> &dev, uint8_t addr);

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    // constructor
    AP_RangeFinder_VL53L0X(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool init();
    void timer();

    // check sensor ID
    bool check_id(void);

    // get a reading
    bool get_reading(uint16_t &reading_cm);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    uint8_t read_register(uint8_t reg);
    uint16_t read_register16(uint8_t reg);

    void write_register(uint8_t reg, uint8_t value);
    void write_register16(uint8_t reg, uint16_t value);

    struct SequenceStepEnables {
        bool tcc:1, msrc:1, dss:1, pre_range:1, final_range:1;
    };

    struct SequenceStepTimeouts {
        uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

        uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
        uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
    };

    struct RegData {
        uint8_t reg;
        uint8_t value;
    };

    static const RegData tuning_data[];
    
    enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange };
    
    bool get_SPAD_info(uint8_t * count, bool *type_is_aperture);
    void getSequenceStepEnables(SequenceStepEnables * enables);
    uint32_t getMeasurementTimingBudget(void);
    void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);
    uint8_t getVcselPulsePeriod(vcselPeriodType type);
    uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
    uint16_t decodeTimeout(uint16_t reg_val);
    bool setMeasurementTimingBudget(uint32_t budget_us);
    uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
    uint16_t encodeTimeout(uint16_t timeout_mclks);
    bool performSingleRefCalibration(uint8_t vhv_init_byte);
    void start_continuous(void);
    
    uint8_t stop_variable;
    uint32_t measurement_timing_budget_us;
    uint32_t start_ms;

    uint32_t sum_mm;
    uint32_t counter;
};
