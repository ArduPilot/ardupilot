#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_ICP201XX_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

class AP_Baro_ICP201XX : public AP_Baro_Backend
{
public:
    void update() override;

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::Device &dev);

private:
    AP_Baro_ICP201XX(AP_Baro &baro, AP_HAL::Device &dev);

    bool init();
    void dummy_reg();
    bool read_reg(uint8_t reg, uint8_t *buf, uint8_t len);
    bool read_reg(uint8_t reg, uint8_t *val);
    bool write_reg(uint8_t reg, uint8_t val);
    bool mode_select(uint8_t mode);
    bool read_otp_data(uint8_t addr, uint8_t cmd, uint8_t *val);
    bool get_sensor_data(float *pressure, float *temperature);
    void soft_reset();
    bool boot_sequence();
    bool configure();
    void wait_read();
    bool flush_fifo();
    void timer();

    uint8_t instance;

    AP_HAL::Device *dev;

    // accumulation structure, protected by _sem
    struct {
        float tsum;
        float psum;
        uint32_t count;
    } accum;

    // time last read command was sent
    uint32_t last_measure_us;

    enum class OP_MODE : uint8_t {
		OP_MODE0 = 0,   /* Mode 0: Bw:6.25 Hz ODR: 25Hz */
		OP_MODE1,       /* Mode 1: Bw:30 Hz ODR: 120Hz */
		OP_MODE2,       /* Mode 2: Bw:10 Hz ODR: 40Hz */
		OP_MODE3,       /* Mode 3: Bw:0.5 Hz ODR: 2Hz */
		OP_MODE4,       /* Mode 4: User configurable Mode */
	} _op_mode{OP_MODE::OP_MODE2};

	enum class FIFO_READOUT_MODE : uint8_t {
		FIFO_READOUT_MODE_PRES_TEMP = 0,   /* Pressure and temperature as pair and address wraps to the start address of the Pressure value ( pressure first ) */
		FIFO_READOUT_MODE_TEMP_ONLY = 1,   /* Temperature only reporting */
		FIFO_READOUT_MODE_TEMP_PRES = 2,   /* Pressure and temperature as pair and address wraps to the start address of the Temperature value ( Temperature first ) */
		FIFO_READOUT_MODE_PRES_ONLY = 3    /* Pressure only reporting */
	} _fifo_readout_mode{FIFO_READOUT_MODE::FIFO_READOUT_MODE_PRES_TEMP};

	enum class POWER_MODE : uint8_t {
		POWER_MODE_NORMAL = 0,  /* Normal Mode: Device is in standby and goes to active mode during the execution of a measurement */
		POWER_MODE_ACTIVE = 1   /* Active Mode: Power on DVDD and enable the high frequency clock */
	} _power_mode{POWER_MODE::POWER_MODE_NORMAL};

	enum MEAS_MODE : uint8_t {
		MEAS_MODE_FORCED_TRIGGER = 0, /* Force trigger mode based on icp201xx_forced_meas_trigger_t **/
		MEAS_MODE_CONTINUOUS = 1   /* Continuous measurements based on selected mode ODR settings*/
	} _meas_mode{MEAS_MODE::MEAS_MODE_CONTINUOUS};

	enum FORCED_MEAS_TRIGGER : uint8_t {
		FORCE_MEAS_STANDBY = 0,			/* Stay in Stand by */
		FORCE_MEAS_TRIGGER_FORCE_MEAS = 1	/* Trigger for forced measurements */
	} _forced_meas_trigger{FORCED_MEAS_TRIGGER::FORCE_MEAS_STANDBY};
};

#endif  // AP_BARO_ICP201XX_ENABLED 
