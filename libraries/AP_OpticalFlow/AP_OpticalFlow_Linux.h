/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef AP_OpticalFlow_Linux_H
#define AP_OpticalFlow_Linux_H

#include "OpticalFlow.h"

#include <AP_Math.h>

/* Configuration Constants */
#define I2C_FLOW_ADDRESS 		0x42	///< 7-bit address. 8-bit address is 0x84, range 0x42 - 0x49

/* PX4FLOW Registers addresses */
#define PX4FLOW_REG			0x16	///< Measure Register 22

#define PX4FLOW_CONVERSION_INTERVAL	100000	///< in microseconds! 20000 = 50 Hz 100000 = 10Hz
#define PX4FLOW_I2C_MAX_BUS_SPEED	400000	///< 400 KHz maximum speed

typedef  struct i2c_frame
{
    uint16_t frame_count;
    int16_t pixel_flow_x_sum;
    int16_t pixel_flow_y_sum;
    int16_t flow_comp_m_x;
    int16_t flow_comp_m_y;
    int16_t qual;
    int16_t gyro_x_rate;
    int16_t gyro_y_rate;
    int16_t gyro_z_rate;
    uint8_t gyro_range;
    uint8_t sonar_timestamp;
    int16_t ground_distance;
} i2c_frame;

#define I2C_FRAME_SIZE (sizeof(i2c_frame))


typedef struct i2c_integral_frame
{
    uint16_t frame_count_since_last_readout;
    int16_t pixel_flow_x_integral;
    int16_t pixel_flow_y_integral;
    int16_t gyro_x_rate_integral;
    int16_t gyro_y_rate_integral;
    int16_t gyro_z_rate_integral;
    uint32_t integration_timespan;
    uint32_t sonar_timestamp;
    uint16_t ground_distance;
    int16_t gyro_temperature;
    uint8_t qual;
} i2c_integral_frame;

#define I2C_INTEGRAL_FRAME_SIZE (sizeof(i2c_integral_frame))

/**
 * Optical flow in NED body frame in SI units.
 *
 * @see http://en.wikipedia.org/wiki/International_System_of_Units
 */
struct optical_flow_s {

	uint64_t timestamp;		/**< in microseconds since system start  */
	uint8_t sensor_id;		/**< id of the sensor emitting the flow value */
	float pixel_flow_x_integral; /**< accumulated optical flow in radians around x axis */
	float pixel_flow_y_integral; /**< accumulated optical flow in radians around y axis */
	float gyro_x_rate_integral;	 /**< accumulated gyro value in radians around x axis */
	float gyro_y_rate_integral;  /**< accumulated gyro value in radians around y axis */
	float gyro_z_rate_integral;   /**< accumulated gyro value in radians around z axis */
	float ground_distance_m;	 /**< Altitude / distance to ground in meters */
	uint32_t integration_timespan; /**<accumulation timespan in microseconds     */
	uint32_t time_since_last_sonar_update;/**< time since last sonar update in microseconds */
	uint16_t frame_count_since_last_readout;/**< number of accumulated frames in timespan */
	int16_t gyro_temperature;/**< 	Temperature * 100 in centi-degrees Celsius */
	uint8_t	quality;		/**< Average of quality of accumulated frames, 0: bad quality, 255: maximum quality */
};

class AP_OpticalFlow_Linux : public OpticalFlow_backend
{
public:
    /// constructor
    AP_OpticalFlow_Linux(OpticalFlow &_frontend);

    // init - initialise the sensor
    void init();

    // update - read latest values from sensor and fill in x,y and totals.
    void update(void);    

private:
	int 	read(optical_flow_s* report);
    void    print(optical_flow_s* report);    
	struct 	i2c_frame f;
	struct 	i2c_integral_frame f_integral;	
};

#endif
