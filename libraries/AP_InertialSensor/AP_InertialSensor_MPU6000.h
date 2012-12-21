/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_MPU6000_H__
#define __AP_INERTIAL_SENSOR_MPU6000_H__

#include <stdint.h>
#include <AP_HAL.h>
#include <AP_Math.h>
#include <AP_Progmem.h>
#include "AP_InertialSensor.h"

#define MPU6000_CS_PIN       53        // APM pin connected to mpu6000's chip select pin
#define DMP_FIFO_BUFFER_SIZE 72        // DMP FIFO buffer size

// DMP memory
extern const uint8_t        dmpMem[8][16][16] PROGMEM;

class AP_InertialSensor_MPU6000 : public AP_InertialSensor
{
public:

    AP_InertialSensor_MPU6000();

    static void         dmp_init(); // Initialise MPU6000's DMP
    static void         dmp_reset();    // Reset the DMP (required for changes in gains or offsets to take effect)

    /* Concrete implementation of AP_InertialSensor functions: */
    bool                update();
    bool                new_data_available();
    float               temperature();
    float               get_gyro_drift_rate();

    // push_gyro_offsets_to_dmp - updates gyro offsets in mpu6000 registers
    void                push_gyro_offsets_to_dmp();
    void                set_dmp_gyro_offsets(int16_t offsetX, int16_t offsetY, int16_t offsetZ);

    // push_accel_offsets_to_dmp - updates accel offsets in DMP registers
    void                push_accel_offsets_to_dmp();
    void                set_dmp_accel_offsets(int16_t offsetX, int16_t offsetY, int16_t offsetZ);

    // num_samples_available - get number of samples read from the sensors
    uint16_t            num_samples_available();

    // get_delta_time returns the time period in seconds overwhich the sensor data was collected
    uint32_t            get_delta_time_micros();

protected:
    uint16_t                    _init_sensor( Sample_rate sample_rate );

private:

    static void                 read_data();
    static void                 data_interrupt(void);
    static uint8_t              register_read( uint8_t reg );
    static void                 register_write( uint8_t reg, uint8_t val );
    void                        hardware_init(Sample_rate sample_rate);

    static AP_HAL::SPIDeviceDriver *_spi;
    static AP_HAL::Semaphore *_spi_sem;

    // set to true when data interrupt from MPU6k fires
    static volatile bool		_data_ready;
    
    float                       _temp;

    float                       _temp_to_celsius( uint16_t );

    static const float          _gyro_scale;

    static const uint8_t        _gyro_data_index[3];
    static const int8_t         _gyro_data_sign[3];

    static const uint8_t        _accel_data_index[3];
    static const int8_t         _accel_data_sign[3];

    static const uint8_t        _temp_data_index;

    // ensure we can't initialise twice
    bool                        _initialised;
    static int16_t              _mpu6000_product_id;

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
