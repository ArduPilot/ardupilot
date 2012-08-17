#ifndef AP_OPTICALFLOW_ADNS3080_H
#define AP_OPTICALFLOW_ADNS3080_H

#include "AP_OpticalFlow.h"

// default pin settings
#define ADNS3080_CHIP_SELECT 34  // PC3
#define ADNS3080_RESET       0   // reset pin is unattached by default

// orientations for ADNS3080 sensor
#define AP_OPTICALFLOW_ADNS3080_PINS_FORWARD ROTATION_YAW_180
#define AP_OPTICALFLOW_ADNS3080_PINS_FORWARD_RIGHT ROTATION_YAW_135
#define AP_OPTICALFLOW_ADNS3080_PINS_RIGHT ROTATION_YAW_90
#define AP_OPTICALFLOW_ADNS3080_PINS_BACK_RIGHT ROTATION_YAW_45
#define AP_OPTICALFLOW_ADNS3080_PINS_BACK ROTATION_NONE
#define AP_OPTICALFLOW_ADNS3080_PINS_BACK_LEFT ROTATION_YAW_315
#define AP_OPTICALFLOW_ADNS3080_PINS_LEFT ROTATION_YAW_270
#define AP_OPTICALFLOW_ADNS3080_PINS_FORWARD_LEFT ROTATION_YAW_225

// field of view of ADNS3080 sensor lenses
#define AP_OPTICALFLOW_ADNS3080_08_FOV 0.202458  // 11.6 degrees

// scaler - value returned when sensor is moved equivalent of 1 pixel
#define AP_OPTICALFLOW_ADNS3080_SCALER  1.1

// ADNS3080 hardware config
#define ADNS3080_PIXELS_X                 30
#define ADNS3080_PIXELS_Y                 30
#define ADNS3080_CLOCK_SPEED                      24000000

// Register Map for the ADNS3080 Optical OpticalFlow Sensor
#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60

// Configuration Bits
#define ADNS3080_LED_MODE_ALWAYS_ON        0x00
#define ADNS3080_LED_MODE_WHEN_REQUIRED    0x01

#define ADNS3080_RESOLUTION_400                 400
#define ADNS3080_RESOLUTION_1600                1600

// Extended Configuration bits
#define ADNS3080_SERIALNPU_OFF  0x02

#define ADNS3080_FRAME_RATE_MAX         6469
#define ADNS3080_FRAME_RATE_MIN         2000

class AP_OpticalFlow_ADNS3080 : public AP_OpticalFlow
{
private:
    // bytes to store SPI settings
    byte        orig_spi_settings_spcr;
    byte        orig_spi_settings_spsr;

    // save and restore SPI settings
    byte        backup_spi_settings();
    byte        restore_spi_settings();

public:
    int         _cs_pin; // pin used for chip select
    int         _reset_pin; // pin used for chip reset
    bool        _motion; // true if there has been motion
    bool        _overflow; // true if the x or y data buffers overflowed

public:
    AP_OpticalFlow_ADNS3080(int cs_pin = ADNS3080_CHIP_SELECT, int reset_pin = ADNS3080_RESET);
    bool        init(bool initCommAPI = true); // parameter controls whether I2C/SPI interface is initialised (set to false if other devices are on the I2C/SPI bus and have already initialised the interface)
    byte        read_register(byte address);
    void        write_register(byte address, byte value);
    void        reset();      // reset sensor by holding a pin high (or is it low?) for 10us.
    bool        update();     // read latest values from sensor and fill in x,y and totals, return true on successful read

    // ADNS3080 specific features
    bool        motion() {
        if( _motion ) { _motion = false; return true; }else{ return false; }
    }                                                                                                           // return true if there has been motion since the last time this was called

    void                disable_serial_pullup();

    bool                get_led_always_on();         // returns true if LED is always on, false if only on when required
    void                set_led_always_on( bool alwaysOn ); // set parameter to true if you want LED always on, otherwise false for only when required

    int                 get_resolution();                                       // returns resolution (either 400 or 1600 counts per inch)
    void                set_resolution(int resolution); // set parameter to 400 or 1600 counts per inch

    bool                get_frame_rate_auto();           // get_frame_rate_auto - return true if frame rate is set to "auto", false if manual
    void                set_frame_rate_auto(bool auto_frame_rate); // set_frame_rate_auto(bool) - set frame rate to auto (true), or manual (false)

    unsigned int        get_frame_period();                                     // get_frame_period -
    void                set_frame_period(unsigned int period);

    unsigned int        get_frame_rate();
    void                set_frame_rate(unsigned int rate);

    bool                get_shutter_speed_auto();                       // get_shutter_speed_auto - returns true if shutter speed is adjusted automatically, false if manual
    void                set_shutter_speed_auto(bool auto_shutter_speed); // set_shutter_speed_auto - set shutter speed to auto (true), or manual (false)

    unsigned int        get_shutter_speed();
    void                set_shutter_speed(unsigned int shutter_speed);

    void                clear_motion(); // will cause the x,y, dx, dy, and the sensor's motion registers to be cleared

    void                print_pixel_data(Stream *serPort); // dumps a 30x30 image to the Serial port
};

#endif
