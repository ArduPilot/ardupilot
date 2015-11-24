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
#ifndef __VIDEOIN_H__
#define __VIDEOIN_H__

#include "AP_HAL_Linux.h"
#include <linux/videodev2.h>

struct buffer {
    unsigned int size;
    void *mem;
};

class Linux::VideoIn {
public:
    /* This structure implements the fields of the v4l2_pix_format struct 
     * that are considered useful for an optical flow application along
     * with the v4l2_buffer fields timestamp and sequence*/
    class Frame {
    friend class VideoIn;
    public:
        uint32_t timestamp = 0;
        uint32_t sequence = 0;
        void *data = NULL;
    private:
        uint32_t buf_index;
    };

    bool get_frame(Frame &frame);
    void put_frame(Frame &frame);
    void set_device_path(const char* path);
    void init();
    bool open_device(const char *device_path, uint32_t memtype);
    bool allocate_buffers(uint32_t nbufs);
    bool set_format(uint32_t *width, uint32_t *height, uint32_t *format,
                    uint32_t *bytesperline, uint32_t *sizeimage);
    bool set_crop(uint32_t left, uint32_t top,
                  uint32_t width, uint32_t height);
    void prepare_capture();

private:
    void _queue_buffer(int index);
    bool _set_streaming(bool enable);
    bool _dequeue_frame(Frame &frame);
    uint32_t _timeval_to_us(struct timeval& tv);
    int _fd = -1;
    struct buffer *_buffers = NULL;
    unsigned int _nbufs = 0;
    bool _streaming = false;
    uint32_t _width = 0;
    uint32_t _height = 0;
    uint32_t _format = 0;
    uint32_t _bytesperline = 0;
    uint32_t _sizeimage = 0;
    uint32_t _memtype = V4L2_MEMORY_MMAP;
};
#endif
