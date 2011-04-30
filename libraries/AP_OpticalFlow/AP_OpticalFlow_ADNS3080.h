#ifndef AP_OPTICALFLOW_ADNS3080_H
#define AP_OPTICALFLOW_ADNS3080_H

#include "AP_OpticalFlow.h"
#include "HardwareSerial.h"

// We use Serial Port 2 in SPI Mode
#define AP_SPI_DATAIN      50    // MISO  // PB3
#define AP_SPI_DATAOUT     51    // MOSI  // PB2
#define AP_SPI_CLOCK       52    // SCK   // PB1
#define ADNS3080_CHIP_SELECT 32    // PC5 
#define ADNS3080_RESET       33    // PC6

// ADNS3080 hardware config
#define ADNS3080_PIXELS_X                 30
#define ADNS3080_PIXELS_Y                 30
#define ADNS3080_CLOCK_SPEED			  24000000

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

#define ADNS3080_LED_MODE_ALWAYS_ON        0x00
#define ADNS3080_LED_MODE_WHEN_REQUIRED    0x01

#define ADNS3080_RESOLUTION_400			400
#define ADNS3080_RESOLUTION_1200		1200

#define ADNS3080_FRAME_RATE_MAX         6469
#define ADNS3080_FRAME_RATE_MIN         2000



class AP_OpticalFlow_ADNS3080 : public AP_OpticalFlow
{
  private:
    // bytes to store SPI settings
    byte orig_spi_settings_spcr;
	byte orig_spi_settings_spsr;
	
	// save and restore SPI settings
	byte backup_spi_settings();
	byte restore_spi_settings();
	
	boolean _motion;  	// true if there has been motion
	
  public:
	AP_OpticalFlow_ADNS3080();  // Constructor
	void init(boolean initCommAPI = true); // parameter controls whether I2C/SPI interface is initialised (set to false if other devices are on the I2C/SPI bus and have already initialised the interface)
	byte read_register(byte address);
	void write_register(byte address, byte value);
	void reset();         // reset sensor by holding a pin high (or is it low?) for 10us.
	int read();           // read latest values from sensor and fill in x,y and totals, return OPTICALFLOW_SUCCESS on successful read
	
	// ADNS3080 specific features
	boolean motion() { if( _motion ) { _motion = false; return true; }else{ return false; } }			// return true if there has been motion since the last time this was called
	
	boolean get_led_always_on();                    // returns true if LED is always on, false if only on when required
	void set_led_always_on( boolean alwaysOn );     // set parameter to true if you want LED always on, otherwise false for only when required
	
	int get_resolution();							// returns resolution (either 400 or 1200 counts per inch)
	void set_resolution(int resolution);            // set parameter to 400 or 1200 counts per inch
	
	boolean get_frame_rate_auto();                      // get_frame_rate_auto - return true if frame rate is set to "auto", false if manual
	void set_frame_rate_auto(boolean auto_frame_rate);  // set_frame_rate_auto(boolean) - set frame rate to auto (true), or manual (false)

	unsigned int get_frame_period();					// get_frame_period - 
	void set_frame_period(unsigned int period);

	unsigned int get_frame_rate();
	void set_frame_rate(unsigned int rate);
	
	boolean get_shutter_speed_auto();                   		// get_shutter_speed_auto - returns true if shutter speed is adjusted automatically, false if manual
	void set_shutter_speed_auto(boolean auto_shutter_speed); 	// set_shutter_speed_auto - set shutter speed to auto (true), or manual (false)

    unsigned int get_shutter_speed();
	unsigned int set_shutter_speed(unsigned int shutter_speed);

	void clear_motion();  // will cause the x,y, dx, dy, and the sensor's motion registers to be cleared
	
	int print_pixel_data(HardwareSerial *serPort); // dumps a 30x30 image to the Serial port
};

#endif
