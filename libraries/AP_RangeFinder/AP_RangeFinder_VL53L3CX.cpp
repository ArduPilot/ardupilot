/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  driver for ST VL53L3CX lidar
 */
#include "AP_RangeFinder_VL53L3CX.h"

#if AP_RANGEFINDER_VL53L3CX_ENABLED

#include <utility>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

static const uint8_t MEASUREMENT_TIME_MS = 50;

#ifndef HAL_RANGEFINDER_VL53L3CX_XSHUT_FORWARD
#ifdef HAL_RANGEFINDER_VL53L3CX_XSHUT_FRONT
#define HAL_RANGEFINDER_VL53L3CX_XSHUT_FORWARD HAL_RANGEFINDER_VL53L3CX_XSHUT_FRONT
#else
#define HAL_RANGEFINDER_VL53L3CX_XSHUT_FORWARD -1
#endif
#endif

#ifndef HAL_RANGEFINDER_VL53L3CX_XSHUT_DOWN
#ifdef HAL_RANGEFINDER_VL53L3CX_XSHUT_BOTTOM
#define HAL_RANGEFINDER_VL53L3CX_XSHUT_DOWN HAL_RANGEFINDER_VL53L3CX_XSHUT_BOTTOM
#else
#define HAL_RANGEFINDER_VL53L3CX_XSHUT_DOWN -1
#endif
#endif

#if HAL_RANGEFINDER_VL53L3CX_XSHUT_FORWARD >= 0 && HAL_RANGEFINDER_VL53L3CX_XSHUT_DOWN >= 0
#define VL53L3CX_HAS_XSHUT 1
#else
#define VL53L3CX_HAS_XSHUT 0
#endif

#ifndef HAL_RANGEFINDER_VL53L3CX_INT_FORWARD
#ifdef HAL_RANGEFINDER_VL53L3CX_INT_FRONT
#define HAL_RANGEFINDER_VL53L3CX_INT_FORWARD HAL_RANGEFINDER_VL53L3CX_INT_FRONT
#else
#define HAL_RANGEFINDER_VL53L3CX_INT_FORWARD -1
#endif
#endif

#ifndef HAL_RANGEFINDER_VL53L3CX_INT_DOWN
#ifdef HAL_RANGEFINDER_VL53L3CX_INT_BOTTOM
#define HAL_RANGEFINDER_VL53L3CX_INT_DOWN HAL_RANGEFINDER_VL53L3CX_INT_BOTTOM
#else
#define HAL_RANGEFINDER_VL53L3CX_INT_DOWN -1
#endif
#endif

#if VL53L3CX_HAS_XSHUT
static bool xshut_pins_configured;
static bool vl53l3cx_instance_active;
static bool vl53l3cx_multiple_reported;
#endif

/**
 * VL53LX target range statuses accepted as usable measurements.
 */
enum class RangeStatus : uint8_t {
    Valid = VL53LX_RANGESTATUS_RANGE_VALID,
    ValidNoWrapCheckFail = VL53LX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL,
    ValidMergedPulse = VL53LX_RANGESTATUS_RANGE_VALID_MERGED_PULSE,
};

/**
 * Construct a VL53L3CX rangefinder backend.
 *
 * @param _state frontend state owned by AP_RangeFinder.
 * @param _params parameter set for this rangefinder instance.
 * @param _dev I2C device used to communicate with the sensor.
 */
AP_RangeFinder_VL53L3CX::AP_RangeFinder_VL53L3CX(RangeFinder::RangeFinder_State &_state,
                                                 AP_RangeFinder_Params &_params,
                                                 AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev)
    : AP_RangeFinder_Backend(_state, _params)
    , dev(std::move(_dev))
    , sensor(dev.get())
    , i2c_address(0)
    , sum_mm(0)
    , counter(0)
{
}

/**
 * Probe and initialise a VL53L3CX backend on an I2C device.
 *
 * Board-specific XSHUT pins, when defined, are prepared before reading the
 * sensor ID so StampFly can select one of its same-address sensors.
 *
 * @param _state frontend state owned by AP_RangeFinder.
 * @param _params parameter set for this rangefinder instance.
 * @param dev I2C device to probe.
 * @param address 7-bit I2C address to use when XSHUT selection is not active.
 * @return new backend on success, nullptr on failure.
 */
AP_RangeFinder_Backend *AP_RangeFinder_VL53L3CX::detect(RangeFinder::RangeFinder_State &_state,
                                                        AP_RangeFinder_Params &_params,
                                                        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                                        uint8_t address)
{
#if VL53L3CX_HAS_XSHUT
    if (vl53l3cx_instance_active) {
        if (!vl53l3cx_multiple_reported) {
            printf("VL53L3CX: multiple rangefinder instances are not supported\n");
            vl53l3cx_multiple_reported = true;
        }
        return nullptr;
    }
#endif

    if (!dev) {
        return nullptr;
    }

#if VL53L3CX_HAS_XSHUT
    address = VL53L3CX_I2C_ADDR_DEFAULT;
    const Rotation orientation = (Rotation)_params.orientation.get();
    if (orientation != ROTATION_PITCH_270 && orientation != ROTATION_NONE) {
        printf("VL53L3CX: unsupported orientation %u for XSHUT selection (use 25=down or 0=forward)\n",
               unsigned(_params.orientation.get()));
        return nullptr;
    }
    const bool select_down_xshut = (orientation == ROTATION_PITCH_270);
#else
    const bool select_down_xshut = true;
#endif

    AP_RangeFinder_VL53L3CX *sensor =
        NEW_NOTHROW AP_RangeFinder_VL53L3CX(_state, _params, std::move(dev));

    if (sensor == nullptr) {
        return nullptr;
    }
    sensor->i2c_address = address;

    sensor->dev->get_semaphore()->take_blocking();

    if (!prepare_xshut(select_down_xshut)) {
        sensor->dev->get_semaphore()->give();
        delete sensor;
        return nullptr;
    }

    if (!sensor->check_id() || !sensor->init(address)) {
        sensor->dev->get_semaphore()->give();
        delete sensor;
        return nullptr;
    }

#if VL53L3CX_HAS_XSHUT
    vl53l3cx_instance_active = true;
#endif
    sensor->dev->get_semaphore()->give();

    return sensor;
}

/**
 * Return the I2C address that should be used for the probe.
 *
 * StampFly XSHUT selection keeps the selected sensor at the default address.
 *
 * @param address configured 7-bit I2C address.
 * @return address to pass to the AP_HAL I2C device manager.
 */
uint8_t AP_RangeFinder_VL53L3CX::probe_address(uint8_t address)
{
#if VL53L3CX_HAS_XSHUT
    return VL53L3CX_I2C_ADDR_DEFAULT;
#else
    return address;
#endif
}

/**
 * Prepare board-specific XSHUT and interrupt pins.
 *
 * When paired XSHUT pins are defined, only the selected StampFly sensor is
 * enabled and the other same-address sensor is held in reset.
 *
 * @param select_down_xshut true to enable the down/bottom sensor, false for
 *        the forward/front sensor.
 * @return true when no board-specific XSHUT setup failed.
 */
bool AP_RangeFinder_VL53L3CX::prepare_xshut(bool select_down_xshut)
{
#if !VL53L3CX_HAS_XSHUT
    (void)select_down_xshut;
#endif
#if VL53L3CX_HAS_XSHUT
    if (!xshut_pins_configured) {
        hal.gpio->pinMode(HAL_RANGEFINDER_VL53L3CX_XSHUT_FORWARD, HAL_GPIO_OUTPUT);
        hal.gpio->pinMode(HAL_RANGEFINDER_VL53L3CX_XSHUT_DOWN, HAL_GPIO_OUTPUT);
#if HAL_RANGEFINDER_VL53L3CX_INT_FORWARD >= 0
        hal.gpio->pinMode(HAL_RANGEFINDER_VL53L3CX_INT_FORWARD, HAL_GPIO_INPUT);
#endif
#if HAL_RANGEFINDER_VL53L3CX_INT_DOWN >= 0
        hal.gpio->pinMode(HAL_RANGEFINDER_VL53L3CX_INT_DOWN, HAL_GPIO_INPUT);
#endif
        hal.gpio->write(HAL_RANGEFINDER_VL53L3CX_XSHUT_FORWARD, 0);
        hal.gpio->write(HAL_RANGEFINDER_VL53L3CX_XSHUT_DOWN, 0);
        hal.scheduler->delay(10);
        xshut_pins_configured = true;
    }

    hal.gpio->write(HAL_RANGEFINDER_VL53L3CX_XSHUT_FORWARD, select_down_xshut ? 0 : 1);
    hal.gpio->write(HAL_RANGEFINDER_VL53L3CX_XSHUT_DOWN, select_down_xshut ? 1 : 0);
    hal.scheduler->delay(100);
#endif
    return true;
}

/**
 * Read one byte from a 16-bit VL53L3CX register.
 *
 * @param reg register address.
 * @param value output byte read from the register.
 * @return true if the I2C transfer completed successfully.
 */
bool AP_RangeFinder_VL53L3CX::read_register(uint16_t reg, uint8_t &value)
{
    const uint8_t reg_buf[2] {
        uint8_t(reg >> 8),
        uint8_t(reg & 0xFF)
    };
    return dev->transfer(reg_buf, sizeof(reg_buf), &value, 1);
}

/**
 * Write one byte to a 16-bit VL53L3CX register.
 *
 * @param reg register address.
 * @param value byte to write.
 * @return true if the I2C transfer completed successfully.
 */
bool AP_RangeFinder_VL53L3CX::write_register(uint16_t reg, uint8_t value)
{
    const uint8_t buf[3] {
        uint8_t(reg >> 8),
        uint8_t(reg & 0xFF),
        value
    };
    return dev->transfer(buf, sizeof(buf), nullptr, 0);
}

/**
 * Perform a soft reset of the sensor.
 *
 * @return true when the reset register writes completed successfully.
 */
bool AP_RangeFinder_VL53L3CX::reset(void)
{
    if (!write_register(VL53LX_SOFT_RESET, 0x00)) {
        return false;
    }
    hal.scheduler->delay(2);
    if (!write_register(VL53LX_SOFT_RESET, 0x01)) {
        return false;
    }
    hal.scheduler->delay(100);
    return true;
}

/**
 * Verify that the connected device reports the expected VL53L3CX ID.
 *
 * @return true when model and module ID registers match VL53L3CX.
 */
bool AP_RangeFinder_VL53L3CX::check_id(void)
{
    uint8_t model_id = 0;
    uint8_t module_type = 0;

    if (!read_register(0x010F, model_id) ||
        !read_register(0x0110, module_type)) {
        return false;
    }

    if (model_id != 0xEA || module_type != 0xAA) {
        return false;
    }

    printf("Detected VL53L3CX[0x%02x] on bus 0x%x\n", unsigned(i2c_address), unsigned(dev->get_bus_id()));
    return true;
}

/**
 * Configure the ST API wrapper and start periodic ranging.
 *
 * @param address [in] 7-bit I2C address used by the selected sensor.
 * @retval true when sensor initialisation and measurement start succeeded.
 * @retval false when startup is unsafe after a watchdog reset, sensor reset
 *         fails, ST API setup fails, or periodic measurement cannot be started.
 */
bool AP_RangeFinder_VL53L3CX::init(uint8_t address)
{
    if (hal.util->was_watchdog_armed()) {
        return false;
    }

    dev->set_retries(3);
    if (!reset()) {
        printf("VL53L3CX[0x%02x]: reset failed\n", unsigned(i2c_address));
        return false;
    }

    sensor.begin();

    const uint8_t st_api_addr = address << 1;
    VL53LX_Error status = sensor.InitSensor(st_api_addr);
    if (status != VL53LX_ERROR_NONE) {
        printf("VL53L3CX[0x%02x]: InitSensor failed status=%d\n", unsigned(i2c_address), int(status));
        return false;
    }

    status = sensor.VL53LX_SetDistanceMode(VL53LX_DISTANCEMODE_LONG);
    if (status != VL53LX_ERROR_NONE) {
        printf("VL53L3CX[0x%02x]: SetDistanceMode failed status=%d\n", unsigned(i2c_address), int(status));
        return false;
    }

    status = sensor.VL53LX_SetMeasurementTimingBudgetMicroSeconds(40000);
    if (status != VL53LX_ERROR_NONE) {
        printf("VL53L3CX[0x%02x]: SetTimingBudget failed status=%d\n", unsigned(i2c_address), int(status));
        return false;
    }

    status = sensor.VL53LX_ClearInterruptAndStartMeasurement();
    if (status != VL53LX_ERROR_NONE) {
        printf("VL53L3CX[0x%02x]: InitialStart failed status=%d\n", unsigned(i2c_address), int(status));
        return false;
    }
    hal.scheduler->delay(100);

    status = sensor.VL53LX_StartMeasurement();
    if (status != VL53LX_ERROR_NONE) {
        printf("VL53L3CX[0x%02x]: StartMeasurement failed status=%d\n", unsigned(i2c_address), int(status));
        return false;
    }
    hal.scheduler->delay(MEASUREMENT_TIME_MS);

    dev->register_periodic_callback(MEASUREMENT_TIME_MS * 1000,
                                    FUNCTOR_BIND_MEMBER(&AP_RangeFinder_VL53L3CX::timer, void));

    return true;
}

/**
 * Check whether a VL53LX target status represents a usable range.
 *
 * @param status [in] ST API range status for one target.
 * @retval true when the target range should be accepted.
 * @retval false when the target status represents an invalid or unusable range.
 */
bool AP_RangeFinder_VL53L3CX::range_status_ok(uint8_t status)
{
    switch (static_cast<RangeStatus>(status)) {
    case RangeStatus::Valid:
    case RangeStatus::ValidNoWrapCheckFail:
    case RangeStatus::ValidMergedPulse:
        return true;
    }

    return false;
}

/**
 * Read the latest multi-target measurement and choose the nearest valid range.
 *
 * @param reading_mm [in/out] nearest valid target range in millimetres.
 * @retval true when a valid range was read.
 * @retval false when data is not ready, ST API read/clear fails, or no valid target range is present in the measurement.
 */
bool AP_RangeFinder_VL53L3CX::get_reading(uint16_t &reading_mm)
{
    uint8_t ready = 0;
    VL53LX_Error status = sensor.VL53LX_GetMeasurementDataReady(&ready);
    if (status != VL53LX_ERROR_NONE) {
        return false;
    }
    if (ready == 0) {
        return false;
    }

    VL53LX_MultiRangingData_t ranging_data;
    status = sensor.VL53LX_GetMultiRangingData(&ranging_data);
    if (status != VL53LX_ERROR_NONE) {
        sensor.VL53LX_ClearInterruptAndStartMeasurement();
        return false;
    }

    bool got_reading = false;
    uint16_t nearest_mm = UINT16_MAX;
    uint8_t object_count = ranging_data.NumberOfObjectsFound;
    if (object_count > VL53LX_MAX_RANGE_RESULTS) {
        object_count = VL53LX_MAX_RANGE_RESULTS;
    }

    for (uint8_t i = 0; i < object_count; i++) {
        const VL53LX_TargetRangeData_t &range = ranging_data.RangeData[i];
        if (!range_status_ok(range.RangeStatus) || range.RangeMilliMeter <= 0) {
            continue;
        }
        const uint16_t range_mm = uint16_t(range.RangeMilliMeter);
        if (range_mm < nearest_mm) {
            nearest_mm = range_mm;
            got_reading = true;
        }
    }

    status = sensor.VL53LX_ClearInterruptAndStartMeasurement();
    if (status != VL53LX_ERROR_NONE) {
        return false;
    }

    if (!got_reading) {
        return false;
    }

    reading_mm = nearest_mm;
    return true;
}

/**
 * Periodic callback used to accumulate valid sensor readings.
 */
void AP_RangeFinder_VL53L3CX::timer(void)
{
    uint16_t range_mm = 0;
    if (get_reading(range_mm) && (range_mm * 0.001f) <= max_distance()) {
        WITH_SEMAPHORE(_sem);
        sum_mm += range_mm;
        counter++;
    }
}

/**
 * Publish accumulated readings to the AP_RangeFinder frontend state.
 */
void AP_RangeFinder_VL53L3CX::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (counter > 0) {
        state.distance_m = (sum_mm * 0.001f) / counter;
        state.last_reading_ms = AP_HAL::millis();
        update_status();
        sum_mm = 0;
        counter = 0;
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        set_status(RangeFinder::Status::NoData);
    }
}

// Build the vendor VL53L3CX API in this translation unit so the
// STMicroelectronics source files can stay under AP_RangeFinder_VL53L3CX/.
#include "AP_RangeFinder_VL53L3CX/vl53lx_class.cpp"

#endif  // AP_RANGEFINDER_VL53L3CX_ENABLED
