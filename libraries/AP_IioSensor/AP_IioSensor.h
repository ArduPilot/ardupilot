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

#include <AP_HAL/AP_HAL_Boards.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <iio.h>
#include <vector>

using namespace std;

class AP_Iio_Channel
{
public:
    AP_Iio_Channel(const char *name);
    int get_data(double *values);
    const char *get_name() { return _name; };
    friend class AP_Iio_Sensor;
private:
    const char *_name;
    double _val;
    double _scale;
    double _offset;
    const struct iio_data_format *_iio_fmt;
    struct iio_channel *_iio_chan;
    const struct iio_buffer *_iio_buf;
protected:
    int init(struct iio_device *dev, struct iio_buffer *iio_buf);
    void disable();
    void set_buf(const struct iio_buffer *buf) { _iio_buf = buf; };
};

class AP_Iio_Sensor
{
public:
    AP_Iio_Sensor(const char *name, vector<AP_Iio_Channel*> *channels,
                  long long sampling_freq, unsigned int buf_count);
    ~AP_Iio_Sensor();
    int init();
    int read();
private:
    const char *_name;
    double _sampling_freq;
    unsigned int _buf_count;
    struct iio_device *_iio_dev;
    struct iio_context *_iio_ctx;
    struct iio_buffer *_iio_buf;
    vector<AP_Iio_Channel*> *_channels;
};
#endif
