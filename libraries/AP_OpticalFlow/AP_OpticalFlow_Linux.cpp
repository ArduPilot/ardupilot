/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *       AP_OpticalFlow_Linux.cpp - ardupilot library for the PX4Flow sensor.
 *          inspired by the PX4Firmware code.
 *      
 *       @author: Víctor Mayoral Vilches
 *
 */

#include <AP_HAL.h>
#include "OpticalFlow.h"

#define DEBUG 1
#define RAW_READ 0

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
extern const AP_HAL::HAL& hal;

AP_OpticalFlow_Linux::AP_OpticalFlow_Linux(OpticalFlow &_frontend) : 
  OpticalFlow_backend(_frontend)
{}


void AP_OpticalFlow_Linux::init(void)
{
  uint8_t buff[22];

  // get pointer to i2c bus semaphore
  AP_HAL::Semaphore *i2c_sem = hal.i2c->get_semaphore();

  // take i2c bus sempahore
  if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
      hal.scheduler->panic(PSTR("PX4FLOW: unable to get semaphore"));
  }

  // to be sure this is not a ll40ls Lidar (which can also be on
  // 0x42) we check if a I2C_FRAME_SIZE byte transfer works from address
  // 0. The ll40ls gives an error for that, whereas the flow
  // happily returns some data
  uint8_t val[I2C_FRAME_SIZE];
  if (hal.i2c->readRegisters(I2C_FLOW_ADDRESS, 0, I2C_FRAME_SIZE, val))
    hal.scheduler->panic(PSTR("ll40ls Lidar"));

  i2c_sem->give();

}

int AP_OpticalFlow_Linux::read(optical_flow_s* report)
{
  // get pointer to i2c bus semaphore
  AP_HAL::Semaphore *i2c_sem = hal.i2c->get_semaphore();

  // take i2c bus sempahore
  if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
      hal.scheduler->panic(PSTR("PX4FLOW: unable to get semaphore"));
  }


  uint8_t val[I2C_FRAME_SIZE + I2C_INTEGRAL_FRAME_SIZE] = { 0 };

#if RAW_READ
  hal.console->printf_P(PSTR("PX4FLOW: RAW_READ"));
  // Send the command to begin a measurement
  uint8_t cmd = PX4FLOW_REG;
  if (hal.i2c->write(I2C_FLOW_ADDRESS, 1, &cmd)){    
    hal.console->printf_P(PSTR("PX4FLOW: Error while beginning a measurement"));
    i2c_sem->give();
    return 0;
  }

  // Perform the reading
  if (PX4FLOW_REG == 0x00) {
    if (hal.i2c->read(I2C_FLOW_ADDRESS, I2C_FRAME_SIZE + I2C_INTEGRAL_FRAME_SIZE, val)){
      hal.console->printf_P(PSTR("PX4FLOW: Error while reading"));
      i2c_sem->give();
      return 0;
    }
  }

  if (PX4FLOW_REG == 0x16) {
    if (hal.i2c->read(I2C_FLOW_ADDRESS, I2C_INTEGRAL_FRAME_SIZE, val)){
      hal.console->printf_P(PSTR("PX4FLOW: Error while reading"));
      i2c_sem->give();
      return 0;
    }
  }

#else
  // Perform the writing and reading in a single command
  if (PX4FLOW_REG == 0x00) {
    if (hal.i2c->readRegisters(I2C_FLOW_ADDRESS, PX4FLOW_REG, I2C_FRAME_SIZE + I2C_INTEGRAL_FRAME_SIZE, val)){
      hal.console->printf_P(PSTR("PX4FLOW: Error while reading"));
      i2c_sem->give();
      return 0;
    }
  }

  if (PX4FLOW_REG == 0x16) {
    if (hal.i2c->readRegisters(I2C_FLOW_ADDRESS, PX4FLOW_REG, I2C_INTEGRAL_FRAME_SIZE, val)){
      hal.console->printf_P(PSTR("PX4FLOW: Error while reading"));
      i2c_sem->give();
      return 0;
    }
  }
#endif


  if (PX4FLOW_REG == 0) {
    memcpy(&f, val, I2C_FRAME_SIZE);
    memcpy(&f_integral, &(val[I2C_FRAME_SIZE]), I2C_INTEGRAL_FRAME_SIZE);
  }

  if (PX4FLOW_REG == 0x16) {
    memcpy(&f_integral, val, I2C_INTEGRAL_FRAME_SIZE);
  }

  // report->timestamp = hrt_absolute_time();
  report->pixel_flow_x_integral = static_cast<float>(f_integral.pixel_flow_x_integral) / 10000.0f;//convert to radians
  report->pixel_flow_y_integral = static_cast<float>(f_integral.pixel_flow_y_integral) / 10000.0f;//convert to radians
  report->frame_count_since_last_readout = f_integral.frame_count_since_last_readout;
  report->ground_distance_m = static_cast<float>(f_integral.ground_distance) / 1000.0f;//convert to meters
  report->quality = f_integral.qual; //0:bad ; 255 max quality
  report->gyro_x_rate_integral = static_cast<float>(f_integral.gyro_x_rate_integral) / 10000.0f; //convert to radians
  report->gyro_y_rate_integral = static_cast<float>(f_integral.gyro_y_rate_integral) / 10000.0f; //convert to radians
  report->gyro_z_rate_integral = static_cast<float>(f_integral.gyro_z_rate_integral) / 10000.0f; //convert to radians
  report->integration_timespan = f_integral.integration_timespan; //microseconds
  report->time_since_last_sonar_update = f_integral.sonar_timestamp;//microseconds
  report->gyro_temperature = f_integral.gyro_temperature;//Temperature * 100 in centi-degrees Celsius

  report->sensor_id = 0;

  hal.console->printf_P(PSTR("PX4FLOW measurement: ground_distance_m: %f\n"), report->ground_distance_m);

/*
  // rotate measurements according to parameter
  float zeroval = 0.0f;
  rotate_3f(_sensor_rotation, report.pixel_flow_x_integral, report.pixel_flow_y_integral, zeroval); 
*/

  i2c_sem->give();
  return 1;

}

// update - read latest values from sensor and fill in x,y and totals.
void AP_OpticalFlow_Linux::update(void)
{
    struct optical_flow_s report;
    
    // read the report from the sensor
    read(&report);

    // process
    struct OpticalFlow::OpticalFlow_state state;
    state.device_id = report.sensor_id;
    state.surface_quality = report.quality;
    if (report.integration_timespan > 0) {
        const Vector2f flowScaler = _flowScaler();
        float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
        float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;
        float integralToRate = 1e6f / float(report.integration_timespan);
        state.flowRate.x = flowScaleFactorX * integralToRate * float(report.pixel_flow_x_integral); // rad/sec measured optically about the X sensor axis
        state.flowRate.y = flowScaleFactorY * integralToRate * float(report.pixel_flow_y_integral); // rad/sec measured optically about the Y sensor axis
        state.bodyRate.x = integralToRate * float(report.gyro_x_rate_integral); // rad/sec measured inertially about the X sensor axis
        state.bodyRate.y = integralToRate * float(report.gyro_y_rate_integral); // rad/sec measured inertially about the Y sensor axis
    } else {
        state.flowRate.zero();
        state.bodyRate.zero();
    }

#if DEBUG
    hal.console->printf_P(PSTR("PX4FLOW print: sensor_id: %d\n"), state.device_id);
    hal.console->printf_P(PSTR("PX4FLOW print: surface_quality: %d\n"), state.surface_quality);
    hal.console->printf_P(PSTR("PX4FLOW print: flowRate.x: %d\n"), state.flowRate.x);
    hal.console->printf_P(PSTR("PX4FLOW print: flowRate.y: %d\n"), state.flowRate.y);
    hal.console->printf_P(PSTR("PX4FLOW print: bodyRate.x: %d\n"), state.bodyRate.x);
    hal.console->printf_P(PSTR("PX4FLOW print: bodyRate.y: %d\n"), state.bodyRate.y);
#endif
    _update_frontend(state);
}



#endif // CONFIG_HAL_BOARD == HAL_BOARD_LINUX
