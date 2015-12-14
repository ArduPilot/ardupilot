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
#include "CameraSensor.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/v4l2-subdev.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace Linux;

bool CameraSensor::set_format(uint32_t width, uint32_t height, uint32_t format)
{
    struct v4l2_subdev_format fmt;
    int ret, fd;

    fd = open(_device_path, O_RDWR);
    if (fd < 0) {
        return false;
    }

    memset(&fmt, 0, sizeof(fmt));
    fmt.pad = 0;
    fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    fmt.format.width = width;
    fmt.format.height = height;
    fmt.format.code = format;

    ret = ioctl(fd, VIDIOC_SUBDEV_S_FMT, &fmt);
    if (ret < 0) {
        return false;
    }

    return true;
}
