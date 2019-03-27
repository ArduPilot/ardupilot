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
#pragma once

#include <linux/videodev2.h>

#include <AP_HAL/OpticalFlow.h>
#include <AP_Math/AP_Math.h>

#include "AP_HAL_Linux.h"
#include "CameraSensor.h"
#include "Flow_PX4.h"
#include "PWM_Sysfs.h"
#include "VideoIn.h"
#include "AP_HAL/utility/RingBuffer.h"

namespace Linux {

class GyroSample {
public:
    Vector2f gyro;
    uint64_t time_us;
};

class OpticalFlow_Onboard : public AP_HAL::OpticalFlow {
public:
    void init() override;
    bool read(AP_HAL::OpticalFlow::Data_Frame& frame) override;
    void push_gyro(float gyro_x, float gyro_y, float dt) override;
    void push_gyro_bias(float gyro_bias_x, float gyro_bias_y) override;

private:
    void _run_optflow();
    static void *_read_thread(void *arg);
    void _get_integrated_gyros(uint64_t timestamp, GyroSample &gyro);
    VideoIn* _videoin;
    VideoIn::Frame _last_video_frame;
    PWM_Sysfs_Base* _pwm;
    CameraSensor* _camerasensor;
    Flow_PX4* _flow;
    pthread_t _thread;
    pthread_mutex_t _mutex;
    bool _initialized;
    bool _data_available;
    bool _crop_by_software;
    bool _shrink_by_software;
    uint32_t _camera_output_width;
    uint32_t _camera_output_height;
    uint32_t _width;
    uint32_t _height;
    uint32_t _format;
    uint32_t _bytesperline;
    uint32_t _sizeimage;
    float _pixel_flow_x_integral;
    float _pixel_flow_y_integral;
    float _gyro_x_integral;
    float _gyro_y_integral;
    uint64_t _integration_timespan;
    uint8_t _surface_quality;
    Vector2f _last_gyro_rate;
    Vector2f _gyro_bias;
    Vector2f _integrated_gyro;
    uint64_t _last_integration_time;
    ObjectBuffer<GyroSample> *_gyro_ring_buffer;
};

}
