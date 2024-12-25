
#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_ISM330DHCX.h"

#include <inttypes.h>
#include <utility>


#include <GCS_MAVLink/GCS.h>

const extern AP_HAL::HAL& hal;


///////

#define ISM330DHCX_ADDR_FUNC_CFG_ACCESS     0x01
#define ISM330DHCX_ADDR_PIN_CTRL            0x02
#define ISM330DHCX_ADDR_FIFO_CTRL1          0x07
#define ISM330DHCX_ADDR_FIFO_CTRL2          0x08
#define ISM330DHCX_ADDR_FIFO_CTRL3          0x09
#define ISM330DHCX_ADDR_FIFO_CTRL4          0x0A
#define ISM330DHCX_ADDR_COUNTER_BDR_REG1    0x0B
#define ISM330DHCX_ADDR_COUNTER_BDR_REG2    0x0C
#define ISM330DHCX_ADDR_INT1_CTRL           0x0D
#define ISM330DHCX_ADDR_INT2_CTRL           0x0E
#define ISM330DHCX_ADDR_WHO_AM_I            0x0F

#define ISM330DHCX_ADDR_CTRL1_XL            0x10
#define ISM330DHCX_ADDR_CTRL2_G             0x11
#define ISM330DHCX_ADDR_CTRL3_C             0x12
#define ISM330DHCX_ADDR_CTRL4_C             0x13
#define ISM330DHCX_ADDR_CTRL5_C             0x14
#define ISM330DHCX_ADDR_CTRL6_C             0x15
#define ISM330DHCX_ADDR_CTRL7_G             0x16
#define ISM330DHCX_ADDR_CTRL8_XL            0x17
#define ISM330DHCX_ADDR_CTRL9_XL            0x18
#define ISM330DHCX_ADDR_CTRL10_C            0x19


#define ISM330DHCX_ADDR_ALL_INT_SRC         0x1A
#define ISM330DHCX_ADDR_WAKE_UP_SRC         0x1B
#define ISM330DHCX_ADDR_TAP_SRC             0x1C
#define ISM330DHCX_ADDR_D6D_SRC             0x1D
#define ISM330DHCX_ADDR_STATUS_REG          0x1E

#define ISM330DHCX_ADDR_OUT_TEMP_L          0x20
#define ISM330DHCX_ADDR_OUT_TEMP_H          0x21

#define ISM330DHCX_ADDR_OUTX_L_G            0x22
#define ISM330DHCX_ADDR_OUTX_H_G            0x23
#define ISM330DHCX_ADDR_OUTY_L_G            0x24
#define ISM330DHCX_ADDR_OUTY_H_G            0x25
#define ISM330DHCX_ADDR_OUTZ_L_G            0x26
#define ISM330DHCX_ADDR_OUTZ_H_G            0x27
#define ISM330DHCX_ADDR_OUTX_L_A            0x28
#define ISM330DHCX_ADDR_OUTX_H_A            0x29
#define ISM330DHCX_ADDR_OUTY_L_A            0x2A
#define ISM330DHCX_ADDR_OUTY_H_A            0x2B
#define ISM330DHCX_ADDR_OUTZ_L_A            0x2C
#define ISM330DHCX_ADDR_OUTZ_H_A            0x2D

#define ISM330DHCX_ADDR_EMB_FUNC_STATUS_MAINPAGE            0x35
#define ISM330DHCX_ADDR_FSM_STATUS_A_MAINPAGE               0x36
#define ISM330DHCX_ADDR_FSM_STATUS_B_MAINPAGE               0x37
#define ISM330DHCX_ADDR_MLC_STATUS_MAINPAGE                 0x38
#define ISM330DHCX_ADDR_STATUS_MASTER_MAINPAGE              0x39
#define ISM330DHCX_ADDR_FIFO_STATUS1                        0x3A
#define ISM330DHCX_ADDR_FIFO_STATUS2                        0x3B
#define ISM330DHCX_ADDR_TIMESTAMP0_REG                      0x40
#define ISM330DHCX_ADDR_TIMESTAMP1_REG                      0x41
#define ISM330DHCX_ADDR_TIMESTAMP2_REG                      0x42
#define ISM330DHCX_ADDR_TIMESTAMP3_REG                      0x43
#define ISM330DHCX_ADDR_TAP_CFG0                            0x56
#define ISM330DHCX_ADDR_TAP_CFG1                            0x57
#define ISM330DHCX_ADDR_TAP_CFG2                            0x58
#define ISM330DHCX_ADDR_TAP_THS_6D                          0x59
#define ISM330DHCX_ADDR_INT_DUR2                            0x5A
#define ISM330DHCX_ADDR_WAKE_UP_THS                         0x5B
#define ISM330DHCX_ADDR_WAKE_UP_DUR                         0x5C
#define ISM330DHCX_ADDR_FREE_FALL                           0x5D
#define ISM330DHCX_ADDR_MD1_CFG                             0x5E
#define ISM330DHCX_ADDR_MD2_CFG                             0x5F

#define ISM330DHCX_ADDR_INTERNAL_FREQ_FINE                  0x63
#define ISM330DHCX_ADDR_INT_OIS                             0x6F


#define ISM330DHCX_ADDR_INT_OIS                             0x6F
#define ISM330DHCX_ADDR_CTRL1_OIS                           0x70
#define ISM330DHCX_ADDR_CTRL2_OIS                           0x71
#define ISM330DHCX_ADDR_CTRL3_OIS                           0x72
#define ISM330DHCX_ADDR_X_OFS_USR                           0x73
#define ISM330DHCX_ADDR_Y_OFS_USR                           0x74
#define ISM330DHCX_ADDR_Z_OFS_USR                           0x75

#define ISM330DHCX_ADDR_FIFO_DATA_OUT_TAG                   0x78
#define ISM330DHCX_ADDR_FIFO_DATA_OUT_X_L                   0x79
#define ISM330DHCX_ADDR_FIFO_DATA_OUT_X_H                   0x7A
#define ISM330DHCX_ADDR_FIFO_DATA_OUT_Y_L                   0x7B
#define ISM330DHCX_ADDR_FIFO_DATA_OUT_Y_H                   0x7C
#define ISM330DHCX_ADDR_FIFO_DATA_OUT_Z_L                   0x7D
#define ISM330DHCX_ADDR_FIFO_DATA_OUT_Z_H                   0x7E




#define ISM330DHCX_ADDR_OUTZ_H_A            0x2D
#define ISM330DHCX_ADDR_OUTZ_H_A            0x2D
#define ISM330DHCX_ADDR_OUTZ_H_A            0x2D
#define ISM330DHCX_ADDR_OUTZ_H_A            0x2D
#define ISM330DHCX_ADDR_OUTZ_H_A            0x2D
#define ISM330DHCX_ADDR_OUTZ_H_A            0x2D




//////////

#define ISM330DHCX_VALUE_WHO_AM_I           0x6B
#define ISM330DHCX_ACCELEROMETER_SCALE_M_S    (2.0f / 256.0f)


///////

#define ISM330DHCX_GYRO_SCALE_R_S (DEG_TO_RAD * 70.0f * 0.001f)


// constructor
AP_InertialSensor_ISM330DHCX::AP_InertialSensor_ISM330DHCX(AP_InertialSensor &imu,
                                                    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                                    enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev)), _rotation(rotation)
{
}


AP_InertialSensor_ISM330DHCX::~AP_InertialSensor_ISM330DHCX()
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_ISM330DHCX::probe(AP_InertialSensor &imu,
                                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                                            enum Rotation rotation)
{


    if (!dev){
        return nullptr;
    }
    AP_InertialSensor_ISM330DHCX *sensor
        = NEW_NOTHROW AP_InertialSensor_ISM330DHCX(imu, std::move(dev), rotation);
    if (!sensor || !sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}


bool AP_InertialSensor_ISM330DHCX::_accel_init()
{
    _dev->get_semaphore()->take_blocking();

    // Init the accelerometer
    uint8_t data = 0;
    
    _dev->read_registers(ISM330DHCX_ADDR_WHO_AM_I, &data, 1);
    if (data != ISM330DHCX_VALUE_WHO_AM_I) {
        AP_HAL::panic("AP_InertialSensor_ISM303DHCX: could not find accelerometer sensor");
    }


    uint8_t fifo_ctrl3 = 0x66; //417Hz for both accel and gyro
    _dev->write_register(ISM330DHCX_ADDR_FIFO_CTRL3, fifo_ctrl3);

    hal.scheduler->delay(5);

    _dev->write_register(ISM330DHCX_ADDR_FIFO_CTRL4, 0x06); // Continuous mode: if the FIFO is full, the new sample overwrites the older one;
    hal.scheduler->delay(5);


    _dev->write_register(ISM330DHCX_ADDR_CTRL1_XL, 0x6C);  //416 Hz  without filtering

    hal.scheduler->delay(5);

    _dev->write_register(ISM330DHCX_ADDR_CTRL2_G, 0x6C); // 
    hal.scheduler->delay(5);

    _dev->write_register(ISM330DHCX_ADDR_CTRL3_C, 0x04); //

    //Filter configuration

    uint8_t ctrl_reg7g = 0x44; //0100 0100
    _dev->write_register(ISM330DHCX_ADDR_CTRL7_G, ctrl_reg7g);

    hal.scheduler->delay(5);



    // Set up the filter desired
    uint16_t filter_hz = _accel_filter_cutoff();
    _set_filter_frequency(filter_hz);

    _dev->get_semaphore()->give();
    
    return true;
}

bool AP_InertialSensor_ISM330DHCX::_gyro_init()
{

    return true;
}

bool AP_InertialSensor_ISM330DHCX::_init_sensor(void)
{
    
    _accel_init();

    _gyro_init();

    return true;
}

/*
  startup the sensor
 */
void AP_InertialSensor_ISM330DHCX::start(void)
{
    if (!_imu.register_gyro(gyro_instance, 800, _dev->get_bus_id_devtype(DEVTYPE_INS_ISM330DHCX))) {
        return;
    }

    if (!_imu.register_accel(accel_instance, 800, _dev->get_bus_id_devtype(DEVTYPE_INS_ISM330DHCX))) {
        return;
    }

    set_accel_orientation(accel_instance, _rotation);
    set_gyro_orientation(gyro_instance, _rotation);
    // start the timer process to read samples
    _dev->register_periodic_callback(1250, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ISM330DHCX::_accumulate_accel, void));
}


/*
  set the filter frequency
 */
void AP_InertialSensor_ISM330DHCX::_set_filter_frequency(uint8_t filter_hz)
{
    _accel_filter.set_cutoff_frequency(400, filter_hz);
    _gyro_filter.set_cutoff_frequency(400, filter_hz);
}


/*
  copy filtered data to the frontend
 */
bool AP_InertialSensor_ISM330DHCX::update(void)
{
    update_gyro(gyro_instance);
    update_accel(accel_instance);

    return true;
}

// Accumulate values from accels and gyros
void AP_InertialSensor_ISM330DHCX::_accumulate_gyro (void)
{

}

void AP_InertialSensor_ISM330DHCX::_accumulate_accel (void)
{
    uint8_t num_samples_available;
    uint8_t fifo_status = 0;
    //uint8_t fifo_tag = 0;
    uint8_t status = 0;

    // Read accelerometer FIFO to find out how many samples are available
    _dev->read_registers(ISM330DHCX_ADDR_FIFO_STATUS1,
                         &fifo_status, 1);
    num_samples_available = fifo_status;

    _dev->read_registers(ISM330DHCX_ADDR_STATUS_REG,
                         &status, 1);

    // read the samples and apply the filter
    if ((num_samples_available > 0) || (status)) {
        if (status & 0x01) {
            int16_t accbuf[3];
            if (_dev->read_registers(ISM330DHCX_ADDR_OUTX_L_A, (uint8_t *)accbuf, sizeof(accbuf))) {
                Vector3f accel = Vector3f(accbuf[0], -accbuf[1], -accbuf[2]);
                // Adjust for chip scaling to get m/s/s
                //accel *= ISM330DHCX_ACCELEROMETER_SCALE_M_S;
                float scale = 9.8f/4096.0f;
                //float scale = 6 / 65536.0f; //32768.0f;
                accel *= scale;
                _rotate_and_correct_accel(accel_instance, accel);

                _notify_new_accel_raw_sample(accel_instance, accel);
            }

        } 
        if (status & 0x02) {
            int16_t girobuf[3];
            if (_dev->read_registers(ISM330DHCX_ADDR_OUTX_L_G, (uint8_t *)girobuf, sizeof(girobuf))) {
                Vector3f gyro = Vector3f(girobuf[0], -girobuf[1], -girobuf[2]);
                // Adjust for chip scaling to get radians/sec
                float _gyro_scale = ISM330DHCX_GYRO_SCALE_R_S;

                gyro *= _gyro_scale;

                _rotate_and_correct_gyro(gyro_instance, gyro);
                _notify_new_gyro_raw_sample(gyro_instance, gyro);
            }
        }

        if (status & 0x04) {
            uint16_t temp;
            if (_dev->read_registers(ISM330DHCX_ADDR_OUT_TEMP_L, (uint8_t *)&temp, sizeof(temp))) {
             //   _imu.set_temperature(temp);
            }
        }

    } 
}

#ifdef ISM330DHCX_DEBUG
/* dump all config registers - used for debug */
void AP_InertialSensor_ISM330DHCX::_dump_registers(void)
{
    hal.console->printf("ISM330DHCX registers:\n");
    hal.console->printf("Gyroscope registers:\n");
//    const uint8_t first = OUT_TEMP_L_XM;
//    const uint8_t last = ACT_DUR;
//    for (uint8_t reg=first; reg<=last; reg++) {
//        uint8_t v = _register_read_g(reg);
//        hal.console->printf("%02x:%02x ", (unsigned)reg, (unsigned)v);
//        if ((reg - (first-1)) % 16 == 0) {
//            hal.console->printf("\n");
//        }
//    }
    hal.console->printf("\n");

    hal.console->printf("Accelerometer and Magnetometers registers:\n");
//    for (uint8_t reg=first; reg<=last; reg++) {
//        uint8_t v = _register_read_xm(reg);
//        hal.console->printf("%02x:%02x ", (unsigned)reg, (unsigned)v);
//        if ((reg - (first-1)) % 16 == 0) {
//            hal.console->printf("\n");
//        }
//    }
    hal.console->printf("\n");

}
#endif
