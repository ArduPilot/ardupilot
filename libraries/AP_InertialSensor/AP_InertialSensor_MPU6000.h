/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_MPU6000_H__
#define __AP_INERTIAL_SENSOR_MPU6000_H__

#include <string.h>
#include <stdint.h>

#include "../AP_PeriodicProcess/AP_PeriodicProcess.h"
#include "../AP_Math/AP_Math.h"
#include "AP_InertialSensor.h"

#define DMP_FIFO_BUFFER_SIZE 72        // DMP FIFO buffer size

// DMP memory
extern const uint8_t        dmpMem[8][16][16] PROGMEM;

class AP_InertialSensor_MPU6000 : public AP_InertialSensor
{
public:

    AP_InertialSensor_MPU6000( uint8_t cs_pin );

    uint16_t            init( AP_PeriodicProcess * scheduler );
    static void         dmp_init(); // Initialise MPU6000's DMP
    static void         dmp_reset();    // Reset the DMP (required for changes in gains or offsets to take effect)

    /* Concrete implementation of AP_InertialSensor functions: */
    bool                update();
    bool                new_data_available();
    float               gx();
    float               gy();
    float               gz();
    void                get_gyros( float * );
    float               ax();
    float               ay();
    float               az();
    void                get_accels( float * );
    void                get_sensors( float * );
    float               temperature();
    uint32_t            sample_time();
    void                reset_sample_time();
    float               get_gyro_drift_rate();

    // set_gyro_offsets - updates gyro offsets in mpu6000 registers
    static void         set_gyro_offsets_scaled(float offX, float offY, float offZ);
    static void         set_gyro_offsets(int16_t offsetX, int16_t offsetY, int16_t offsetZ);

    // set_accel_offsets - updates accel offsets in DMP registers
    static void         set_accel_offsets_scaled(float offX, float offY, float offZ);
    static void         set_accel_offsets(int16_t offsetX, int16_t offsetY, int16_t offsetZ);

private:

    static void                 read(uint32_t);
    static void                 data_interrupt(void);
    static uint8_t              register_read( uint8_t reg );
    static void                 register_write( uint8_t reg, uint8_t val );
    static void                 hardware_init();

    Vector3f                    _gyro;
    Vector3f                    _accel;
    float                       _temp;

    uint32_t                    _last_sample_micros;

    float                       _temp_to_celsius( uint16_t );

    static const float          _accel_scale;
    static const float          _gyro_scale;

    static const uint8_t        _gyro_data_index[3];
    static const int8_t         _gyro_data_sign[3];

    static const uint8_t        _accel_data_index[3];
    static const int8_t         _accel_data_sign[3];

    static const uint8_t        _temp_data_index;

    /* TODO deprecate _cs_pin */
    static uint8_t              _cs_pin;

    // ensure we can't initialise twice
    bool                        _initialised;

    // dmp related methods and parameters
    static void                 dmp_register_write(uint8_t bank, uint8_t address, uint8_t num_bytes, uint8_t data[]); // Method to write multiple bytes into dmp registers.  Requires a "bank"
    static void                 dmp_set_rate(uint8_t rate); // set DMP output rate (see constants)

public:
    static Quaternion           quaternion;             // holds the 4 quaternions representing attitude taken directly from the DMP

    static bool                 FIFO_ready();                   // returns true if new attitude data is available in FIFO buffer
    static void                 FIFO_reset();                   // clear attitude data from FIFO buffer
    static void                 FIFO_getPacket();       // get latest attitude data from FIFO buffer
    static void                 dmp_set_gyro_calibration();
    static void                 dmp_set_accel_calibration();
    static void                 dmp_apply_endian_accel();
    static void                 dmp_set_mpu_sensors();  // To configure for SIX_AXIS output
    static void                 dmp_set_bias_from_no_motion(); // Turn on bias from no motion
    static void                 dmp_set_bias_none();            // Turn off internal bias correction (we will use this and we handle the gyro bias correction externally)
    static void                 dmp_set_fifo_interrupt();
    static void                 dmp_send_quaternion();  // Send quaternion data to FIFO
    static void                 dmp_send_gyro();                // Send gyro data to FIFO
    static void                 dmp_send_accel();       // Send accel data to FIFO
    static void                 dmp_set_fifo_rate(uint8_t rate); // This functions defines the rate at wich attitude data is send to FIFO.  Rate: 0 => SAMPLE_RATE(ex:200Hz), 1=> SAMPLE_RATE/2 (ex:100Hz), 2=> SAMPLE_RATE/3 (ex:66Hz)
    static void                 dmp_set_sensor_fusion_accel_gain(uint8_t gain); // This function defines the weight of the accel on the sensor fusion. Default value is 0x80. The official invensense name is inv_key_0_96 (?)
    static void                 dmp_load_mem();                 // Load initial memory values into DMP memory banks

    static uint8_t              _received_packet[DMP_FIFO_BUFFER_SIZE]; // FIFO packet buffer
    static uint8_t              _fifoCountH;                    // high byte of number of elements in fifo buffer
    static uint8_t              _fifoCountL;                    // low byte of number of elements in fifo buffer

    static bool                 _dmp_initialised;
};

#endif // __AP_INERTIAL_SENSOR_MPU6000_H__
