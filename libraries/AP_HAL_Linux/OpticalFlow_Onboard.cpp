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

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
#include "OpticalFlow_Onboard.h"

#include <fcntl.h>
#include <linux/v4l2-mediabus.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include "CameraSensor_Mt9v117.h"
#include "GPIO.h"
#include "PWM_Sysfs.h"
#include "AP_HAL/utility/RingBuffer.h"

#define OPTICAL_FLOW_ONBOARD_RTPRIO 11
static const unsigned int OPTICAL_FLOW_GYRO_BUFFER_LEN = 400;

extern const AP_HAL::HAL& hal;

using namespace Linux;

void OpticalFlow_Onboard::init()
{
    uint32_t top, left;
    uint32_t crop_width, crop_height;
    uint32_t memtype = V4L2_MEMORY_MMAP;
    unsigned int nbufs = 0;
    int ret;
    pthread_attr_t attr;
    struct sched_param param = {
        .sched_priority = OPTICAL_FLOW_ONBOARD_RTPRIO
    };

    if (_initialized) {
        return;
    }

    _videoin = NEW_NOTHROW VideoIn;
    const char* device_path = HAL_OPTFLOW_ONBOARD_VDEV_PATH;
    memtype = V4L2_MEMORY_MMAP;
    nbufs = HAL_OPTFLOW_ONBOARD_NBUFS;
    _width = HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH;
    _height = HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT;
    crop_width = HAL_OPTFLOW_ONBOARD_CROP_WIDTH;
    crop_height = HAL_OPTFLOW_ONBOARD_CROP_HEIGHT;
    top = 0;
    /* make the image square by cropping to YxY, removing the lateral edges */
    left = (HAL_OPTFLOW_ONBOARD_SENSOR_WIDTH -
            HAL_OPTFLOW_ONBOARD_SENSOR_HEIGHT) / 2;

    if (device_path == nullptr ||
        !_videoin->open_device(device_path, memtype)) {
        AP_HAL::panic("OpticalFlow_Onboard: couldn't open "
                      "video device");
    }

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
    _pwm = NEW_NOTHROW PWM_Sysfs_Bebop(BEBOP_CAMV_PWM);
    _pwm->init();
    _pwm->set_freq(BEBOP_CAMV_PWM_FREQ);
    _pwm->enable(true);

    _camerasensor = NEW_NOTHROW CameraSensor_Mt9v117(HAL_OPTFLOW_ONBOARD_SUBDEV_PATH,
                                             hal.i2c_mgr->get_device(0, 0x5D),
                                             MT9V117_QVGA,
                                             BEBOP_GPIO_CAMV_NRST,
                                             BEBOP_CAMV_PWM_FREQ);
    if (!_camerasensor->set_format(HAL_OPTFLOW_ONBOARD_SENSOR_WIDTH,
                                   HAL_OPTFLOW_ONBOARD_SENSOR_HEIGHT,
                                   V4L2_MBUS_FMT_UYVY8_2X8)) {
        AP_HAL::panic("OpticalFlow_Onboard: couldn't set subdev fmt");
    }
    _format = V4L2_PIX_FMT_NV12;
#endif

    if (!_videoin->set_format(&_width, &_height, &_format, &_bytesperline,
                              &_sizeimage)) {
        AP_HAL::panic("OpticalFlow_Onboard: couldn't set video format");
    }

    if (_format != V4L2_PIX_FMT_NV12 && _format != V4L2_PIX_FMT_GREY &&
        _format != V4L2_PIX_FMT_YUYV) {
        AP_HAL::panic("OpticalFlow_Onboard: format not supported");
    }

    if (_width == HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH &&
        _height == HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT) {
        _shrink_by_software = false;
    } else {
        /* here we store the actual camera output width and height to use
         * them later on to software shrink each frame. */
        _shrink_by_software = true;
        _camera_output_width = _width;
        _camera_output_height = _height;

        /* we set these values here in order to the calculations be correct
         * (such as PX4 init) even though we shrink each frame later on. */
        _width = HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH;
        _height = HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT;
        _bytesperline = HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH;
    }

    if (_videoin->set_crop(left, top, crop_width, crop_height)) {
        _crop_by_software = false;
    } else {
        _crop_by_software = true;

        if (!_shrink_by_software) {
            /* here we store the actual camera output width and height to use
             * them later on to software crop each frame. */
            _camera_output_width = _width;
            _camera_output_height = _height;

            /* we set these values here in order to the calculations be correct
             * (such as PX4 init) even though we crop each frame later on. */
            _width = HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH;
            _height = HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT;
            _bytesperline = HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH;
        }
    }

    if (!_videoin->allocate_buffers(nbufs)) {
        AP_HAL::panic("OpticalFlow_Onboard: couldn't allocate video buffers");
    }

    _videoin->prepare_capture();

    /* Use px4 algorithm for optical flow */
    _flow = NEW_NOTHROW Flow_PX4(_width, _bytesperline,
                         HAL_FLOW_PX4_MAX_FLOW_PIXEL,
                         HAL_FLOW_PX4_BOTTOM_FLOW_FEATURE_THRESHOLD,
                         HAL_FLOW_PX4_BOTTOM_FLOW_VALUE_THRESHOLD);

    /* Create the thread that will be waiting for frames
     * Initialize thread and mutex */
    ret = pthread_mutex_init(&_mutex, nullptr);
    if (ret != 0) {
        AP_HAL::panic("OpticalFlow_Onboard: failed to init mutex");
    }

    ret = pthread_attr_init(&attr);
    if (ret != 0) {
        AP_HAL::panic("OpticalFlow_Onboard: failed to init attr");
    }
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &param);
    ret = pthread_create(&_thread, &attr, _read_thread, this);
    if (ret != 0) {
        AP_HAL::panic("OpticalFlow_Onboard: failed to create thread");
    }

    _gyro_ring_buffer = NEW_NOTHROW ObjectBuffer<GyroSample>(OPTICAL_FLOW_GYRO_BUFFER_LEN);
    if (_gyro_ring_buffer != nullptr && _gyro_ring_buffer->get_size() == 0) {
        // allocation failed
        delete _gyro_ring_buffer;
        _gyro_ring_buffer = nullptr;
    }

    _initialized = true;
}

bool OpticalFlow_Onboard::read(AP_HAL::OpticalFlow::Data_Frame& frame)
{
    bool ret;

    pthread_mutex_lock(&_mutex);
    if (!_data_available) {
        ret = false;
        goto end;
    }
    frame.pixel_flow_x_integral = _pixel_flow_x_integral;
    frame.pixel_flow_y_integral = _pixel_flow_y_integral;
    frame.gyro_x_integral = _gyro_x_integral;
    frame.gyro_y_integral = _gyro_y_integral;
    frame.delta_time = _integration_timespan;
    frame.quality = _surface_quality;
    _integration_timespan = 0;
    _pixel_flow_x_integral = 0;
    _pixel_flow_y_integral = 0;
    _gyro_x_integral = 0;
    _gyro_y_integral = 0;
    _data_available = false;
    ret = true;
end:
    pthread_mutex_unlock(&_mutex);
    return ret;
}

void OpticalFlow_Onboard::push_gyro(float gyro_x, float gyro_y, float dt)
{
    GyroSample sample;
    struct timespec ts;

    if (!_gyro_ring_buffer) {
        return;
    }

    clock_gettime(CLOCK_MONOTONIC, &ts);
    _integrated_gyro.x += (gyro_x - _gyro_bias.x) * dt;
    _integrated_gyro.y += (gyro_y - _gyro_bias.y) * dt;
    sample.gyro = _integrated_gyro;
    sample.time_us = ts.tv_sec*1000000ULL + ts.tv_nsec/1000ULL;

    _gyro_ring_buffer->push(sample);
}

void OpticalFlow_Onboard::_get_integrated_gyros(uint64_t timestamp, GyroSample &gyro)
{
    GyroSample integrated_gyro_at_time = {};
    unsigned int retries = 0;

    // pop all samples prior to frame time
    while (_gyro_ring_buffer->pop(integrated_gyro_at_time) &&
            integrated_gyro_at_time.time_us < timestamp &&
            retries++ < OPTICAL_FLOW_GYRO_BUFFER_LEN);
    gyro = integrated_gyro_at_time;
}

void OpticalFlow_Onboard::push_gyro_bias(float gyro_bias_x, float gyro_bias_y)
{
    _gyro_bias.x = gyro_bias_x;
    _gyro_bias.y = gyro_bias_y;
}

void *OpticalFlow_Onboard::_read_thread(void *arg)
{
    OpticalFlow_Onboard *optflow_onboard = (OpticalFlow_Onboard *) arg;

    optflow_onboard->_run_optflow();
    return nullptr;
}

void OpticalFlow_Onboard::_run_optflow()
{
    GyroSample gyro_sample;
    Vector2f flow_rate;
    VideoIn::Frame video_frame;
    uint32_t convert_buffer_size = 0, output_buffer_size = 0;
    uint32_t crop_left = 0, crop_top = 0;
    uint32_t shrink_scale = 0, shrink_width = 0, shrink_height = 0;
    uint32_t shrink_width_offset = 0, shrink_height_offset = 0;
    uint8_t *convert_buffer = nullptr, *output_buffer = nullptr;
    uint8_t qual;

    if (_format == V4L2_PIX_FMT_YUYV) {
        if (_shrink_by_software || _crop_by_software) {
            convert_buffer_size = _camera_output_width * _camera_output_height;
        } else {
            convert_buffer_size = _width * _height;
        }

        convert_buffer = (uint8_t *)calloc(1, convert_buffer_size);
        if (!convert_buffer) {
            AP_HAL::panic("OpticalFlow_Onboard: couldn't allocate conversion buffer");
        }
    }

    if (_shrink_by_software || _crop_by_software) {
        output_buffer_size = HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH *
            HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT;

        output_buffer = (uint8_t *)calloc(1, output_buffer_size);
        if (!output_buffer) {
            if (convert_buffer) {
                free(convert_buffer);
            }

            AP_HAL::panic("OpticalFlow_Onboard: couldn't allocate crop buffer");
        }
    }

    if (_shrink_by_software) {
        if (_camera_output_width > _camera_output_height) {
            shrink_scale = (uint32_t) _camera_output_height /
                HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT;
        } else {
            shrink_scale = (uint32_t) _camera_output_width /
                HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH;
        }

        shrink_width = HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH * shrink_scale;
        shrink_height = HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT * shrink_scale;

        shrink_width_offset = (_camera_output_width - shrink_width) / 2;
        shrink_height_offset = (_camera_output_height - shrink_height) / 2;
    } else if (_crop_by_software) {
        crop_left = _camera_output_width / 2 -
           HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH / 2;
        crop_top = _camera_output_height / 2 -
           HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT / 2;
    }

    while(true) {
        /* wait for next frame to come */
        if (!_videoin->get_frame(video_frame)) {
            if (convert_buffer) {
               free(convert_buffer);
            }

            if (output_buffer) {
               free(output_buffer);
            }

            AP_HAL::panic("OpticalFlow_Onboard: couldn't get frame");
        }

        if (_format == V4L2_PIX_FMT_YUYV) {
            VideoIn::yuyv_to_grey((uint8_t *)video_frame.data,
                convert_buffer_size * 2, convert_buffer);

            memset(video_frame.data, 0, convert_buffer_size * 2);
            memcpy(video_frame.data, convert_buffer, convert_buffer_size);
        }

        if (_shrink_by_software) {
            /* shrink_8bpp() will shrink a selected area using the offsets,
             * therefore, we don't need the crop. */
            VideoIn::shrink_8bpp((uint8_t *)video_frame.data, output_buffer,
                                 _camera_output_width, _camera_output_height,
                                 shrink_width_offset, shrink_width,
                                 shrink_height_offset, shrink_height,
                                 shrink_scale, shrink_scale);
            memset(video_frame.data, 0, _camera_output_width * _camera_output_height);
            memcpy(video_frame.data, output_buffer, output_buffer_size);
        } else if (_crop_by_software) {
            VideoIn::crop_8bpp((uint8_t *)video_frame.data, output_buffer,
                               _camera_output_width,
                               crop_left, HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH,
                               crop_top, HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT);

            memset(video_frame.data, 0, _camera_output_width * _camera_output_height);
            memcpy(video_frame.data, output_buffer, output_buffer_size);
        }

        /* if it is at least the second frame we receive
         * since we have to compare 2 frames */
        if (_last_video_frame.data == nullptr) {
            _last_video_frame = video_frame;
            continue;
        }

        /* read the integrated gyro data */
        _get_integrated_gyros(video_frame.timestamp, gyro_sample);

#ifdef OPTICALFLOW_ONBOARD_RECORD_VIDEO
        int fd = open(OPTICALFLOW_ONBOARD_VIDEO_FILE, O_CLOEXEC | O_CREAT | O_WRONLY
                | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP |
                S_IWGRP | S_IROTH | S_IWOTH);
	    if (fd != -1) {
	        write(fd, video_frame.data, _sizeimage);
#ifdef OPTICALFLOW_ONBOARD_RECORD_METADATAS
            struct PACKED {
                uint32_t timestamp;
                float x;
                float y;
                float z;
            } metas = { video_frame.timestamp, rate_x, rate_y, rate_z};
            write(fd, &metas, sizeof(metas));
#endif
	        close(fd);
        }
#endif

        /* compute gyro data and video frames
         * get flow rate to send it to the opticalflow driver
         */
        qual = _flow->compute_flow((uint8_t*)_last_video_frame.data,
                                   (uint8_t *)video_frame.data,
                                   video_frame.timestamp -
                                   _last_video_frame.timestamp,
                                   &flow_rate.x, &flow_rate.y);

        /* fill data frame for upper layers */
        pthread_mutex_lock(&_mutex);
        _pixel_flow_x_integral += flow_rate.x /
                                  HAL_FLOW_PX4_FOCAL_LENGTH_MILLIPX;
        _pixel_flow_y_integral += flow_rate.y /
                                  HAL_FLOW_PX4_FOCAL_LENGTH_MILLIPX;
        _integration_timespan += video_frame.timestamp -
                                 _last_video_frame.timestamp;
        _gyro_x_integral       += (gyro_sample.gyro.x - _last_gyro_rate.x) *
                                  (video_frame.timestamp - _last_video_frame.timestamp) /
                                  (gyro_sample.time_us - _last_integration_time);
        _gyro_y_integral       += (gyro_sample.gyro.y - _last_gyro_rate.y) /
                                  (gyro_sample.time_us - _last_integration_time) *
                                  (video_frame.timestamp - _last_video_frame.timestamp);
        _surface_quality = qual;
        _data_available = true;
        pthread_mutex_unlock(&_mutex);

        /* give the last frame back to the video input driver */
        _videoin->put_frame(_last_video_frame);
        _last_integration_time = gyro_sample.time_us;
        _last_video_frame = video_frame;
        _last_gyro_rate = gyro_sample.gyro;
    }

    if (convert_buffer) {
        free(convert_buffer);
    }

    if (output_buffer) {
        free(output_buffer);
    }
}
#endif
