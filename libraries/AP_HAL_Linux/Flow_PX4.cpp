/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Petri Tanskanen <tpetri@inf.ethz.ch>
 *            Lorenz Meier <lm@inf.ethz.ch>
 *            Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 *   Modified to fit the APM framework by:
 *          Julien BERAUD <julien.beraud@parrot.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
#include "Flow_PX4.h"

#include <cmath>
#include <stdio.h>
#include <stdlib.h>

extern const AP_HAL::HAL& hal;

using namespace Linux;

Flow_PX4::Flow_PX4(uint32_t width, uint32_t bytesperline,
                   uint32_t max_flow_pixel,
                   float bottom_flow_feature_threshold,
                   float bottom_flow_value_threshold) :
    _width(width),
    _search_size(max_flow_pixel),
    _bytesperline(bytesperline),
    _bottom_flow_feature_threshold(bottom_flow_feature_threshold),
    _bottom_flow_value_threshold(bottom_flow_value_threshold)
{
    /* _pixlo is _search_size + 1 because if we need to evaluate
     * the subpixels up/left of the first pixel, the index
     * will be equal to _pixlo - _search_size -1
     * idem if we need to evaluate the subpixels down/right
     * the index will be equal to _pixhi + _search_size + 1
     * which needs to remain inferior to _width - 1
     */
    _pixlo = _search_size + 1;
    _pixhi = _width - 1 - (_search_size + 1);
    /* 1 block is of size 2*_search_size + 1 + 1 pixel on each
     * side for subpixel calculation.
     * So _num_blocks = _width / (2 * _search_size + 3)
     */
    _num_blocks = _width / (2 * _search_size + 3);
    _pixstep = ceilf(((float)(_pixhi - _pixlo)) / _num_blocks);
}

/**
 * @brief Compute the average pixel gradient of all horizontal and vertical
 *        steps
 *
 * @param image ...
 * @param offX x coordinate of upper left corner of 8x8 pattern in image
 * @param offY y coordinate of upper left corner of 8x8 pattern in image
 */
static inline uint32_t compute_diff(uint8_t *image, uint16_t offx, uint16_t offy,
                                    uint16_t row_size, uint8_t window_size)
{
    /* calculate position in image buffer */
    /* we calc only the 4x4 pattern */
    uint16_t off = (offy + 2) * row_size + (offx + 2);
    uint32_t acc = 0;
    unsigned int i;

    for (i = 0; i < window_size; i++) {
        /* accumulate differences between line1/2, 2/3, 3/4 for 4 pixels
         * starting at offset off
         */
        acc += abs(image[off + i] - image[off + i + row_size]);
        acc += abs(image[off + i + row_size] - image[off + i + 2 * row_size]);
        acc += abs(image[off + i + 2 * row_size] -
                   image[off + i + 3 * row_size]);

        /* accumulate differences between col1/2, 2/3, 3/4 for 4 pixels starting
         * at off
         */
        acc += abs(image[off + row_size * i] - image[off + row_size * i + 1]);
        acc += abs(image[off + row_size * i + 1] -
                   image[off + row_size * i + 2]);
        acc += abs(image[off + row_size * i + 2] -
                   image[off + row_size * i + 3]);
    }

    return acc;
}

/**
 * @brief Compute SAD of two pixel windows.
 *
 * @param image1 ...
 * @param image2 ...
 * @param off1X x coordinate of upper left corner of pattern in image1
 * @param off1Y y coordinate of upper left corner of pattern in image1
 * @param off2X x coordinate of upper left corner of pattern in image2
 * @param off2Y y coordinate of upper left corner of pattern in image2
 */
static inline uint32_t compute_sad(uint8_t *image1, uint8_t *image2,
                                   uint16_t off1x, uint16_t off1y,
                                   uint16_t off2x, uint16_t off2y,
                                   uint16_t row_size, uint16_t window_size)
{
    /* calculate position in image buffer
     * off1 for image1 and off2 for image2
     */
    uint16_t off1 = off1y * row_size + off1x;
    uint16_t off2 = off2y * row_size + off2x;
    unsigned int i,j;
    uint32_t acc = 0;

    for (i = 0; i < window_size; i++) {
        for (j = 0; j < window_size; j++) {
            acc += abs(image1[off1 + i + j*row_size] -
                       image2[off2 + i + j*row_size]);
        }
    }
    return acc;
}

/**
 * @brief Compute SAD distances of subpixel shift of two pixel patterns.
 *
 * @param image1 ...
 * @param image2 ...
 * @param off1X x coordinate of upper left corner of pattern in image1
 * @param off1Y y coordinate of upper left corner of pattern in image1
 * @param off2X x coordinate of upper left corner of pattern in image2
 * @param off2Y y coordinate of upper left corner of pattern in image2
 * @param acc array to store SAD distances for shift in every direction
 */
static inline uint32_t compute_subpixel(uint8_t *image1, uint8_t *image2,
                                        uint16_t off1x, uint16_t off1y,
                                        uint16_t off2x, uint16_t off2y,
                                        uint32_t *acc, uint16_t row_size,
                                        uint16_t window_size)
{
    /* calculate position in image buffer */
    uint16_t off1 = off1y * row_size + off1x; // image1
    uint16_t off2 = off2y * row_size + off2x; // image2
    uint8_t sub[8];
    uint16_t i, j, k;

    memset(acc, 0, window_size * sizeof(uint32_t));

    for (i = 0; i < window_size; i++) {
        for (j = 0; j < window_size; j++) {
            /* the 8 s values are from following positions for each pixel (X):
             *  + - + - + - +
             *  +   5   7   +
             *  + - + 6 + - +
             *  +   4 X 0   +
             *  + - + 2 + - +
             *  +   3   1   +
             *  + - + - + - +
             */

            /* subpixel 0 is the mean value of base pixel and
             * the pixel on the right, subpixel 1 is the mean
             * value of base pixel, the pixel on the right,
             * the pixel down from it, and the pixel down on
             * the right. etc...
             */
            sub[0] = (image2[off2 + i + j*row_size] +
                      image2[off2 + i + 1 + j*row_size])/2;

            sub[1] = (image2[off2 + i + j*row_size] +
                      image2[off2 + i + 1 + j*row_size] +
                      image2[off2 + i + (j+1)*row_size] +
                      image2[off2 + i + 1 + (j+1)*row_size])/4;

            sub[2] = (image2[off2 + i + j*row_size] +
                      image2[off2 + i + 1 + (j+1)*row_size])/2;

            sub[3] = (image2[off2 + i + j*row_size] +
                      image2[off2 + i - 1 + j*row_size] +
                      image2[off2 + i - 1 + (j+1)*row_size] +
                      image2[off2 + i + (j+1)*row_size])/4;

            sub[4] = (image2[off2 + i + j*row_size] +
                      image2[off2 + i - 1 + (j+1)*row_size])/2;

            sub[5] = (image2[off2 + i + j*row_size] +
                      image2[off2 + i - 1 + j*row_size] +
                      image2[off2 + i - 1 + (j-1)*row_size] +
                      image2[off2 + i + (j-1)*row_size])/4;

            sub[6] = (image2[off2 + i + j*row_size] +
                      image2[off2 + i + (j-1)*row_size])/2;

            sub[7] = (image2[off2 + i + j*row_size] +
                      image2[off2 + i + 1 + j*row_size] +
                      image2[off2 + i + (j-1)*row_size] +
                      image2[off2 + i + 1 + (j-1)*row_size])/4;

            for (k = 0; k < 8; k++) {
                acc[k] += abs(image1[off1 + i + j*row_size] - sub[k]);
            }
        }
    }

    return 0;
}

uint8_t Flow_PX4::compute_flow(uint8_t *image1, uint8_t *image2,
                               uint32_t delta_time, float *pixel_flow_x,
                               float *pixel_flow_y)
{
    /* constants */
    const int16_t winmin = -_search_size;
    const int16_t winmax = _search_size;
    uint16_t i, j;
    uint32_t acc[2*_search_size];
    int8_t dirsx[_num_blocks*_num_blocks];
    int8_t dirsy[_num_blocks*_num_blocks];
    uint8_t subdirs[_num_blocks*_num_blocks];
    float meanflowx = 0.0f;
    float meanflowy = 0.0f;
    uint16_t meancount = 0;
    float histflowx = 0.0f;
    float histflowy = 0.0f;

    /* iterate over all patterns
     */
    for (j = _pixlo; j < _pixhi; j += _pixstep) {
        for (i = _pixlo; i < _pixhi; i += _pixstep) {
            /* test pixel if it is suitable for flow tracking */
            uint32_t diff = compute_diff(image1, i, j, (uint16_t) _bytesperline,
                                         _search_size);
            if (diff < _bottom_flow_feature_threshold) {
                continue;
            }

            uint32_t dist = 0xFFFFFFFF; // set initial distance to "infinity"
            int8_t sumx = 0;
            int8_t sumy = 0;
            int8_t ii, jj;

            for (jj = winmin; jj <= winmax; jj++) {
                for (ii = winmin; ii <= winmax; ii++) {
                    uint32_t temp_dist = compute_sad(image1, image2, i, j,
                                                     i + ii, j + jj,
                                                     (uint16_t)_bytesperline,
                                                     2 * _search_size);
                    if (temp_dist < dist) {
                        sumx = ii;
                        sumy = jj;
                        dist = temp_dist;
                    }
                }
            }

            /* acceptance SAD distance threshold */
            if (dist < _bottom_flow_value_threshold) {
                meanflowx += (float)sumx;
                meanflowy += (float) sumy;

                compute_subpixel(image1, image2, i, j, i + sumx, j + sumy,
                                 acc, (uint16_t) _bytesperline,
                                 2 * _search_size);
                uint32_t mindist = dist; // best SAD until now
                uint8_t mindir = 8; // direction 8 for no direction
                for (uint8_t k = 0; k < 2 * _search_size; k++) {
                    if (acc[k] < mindist) {
                        // SAD becomes better in direction k
                        mindist = acc[k];
                        mindir = k;
                    }
                }
                dirsx[meancount] = sumx;
                dirsy[meancount] = sumy;
                subdirs[meancount] = mindir;
                meancount++;
            }
        }
    }

    /* evaluate flow calculation */
    if (meancount > _num_blocks * _num_blocks / 2) {
        meanflowx /= meancount;
        meanflowy /= meancount;

        /* use average of accepted flow values */
        uint32_t meancount_x = 0;
        uint32_t meancount_y = 0;

        for (uint16_t h = 0; h < meancount; h++) {
            float subdirx = 0.0f;
            if (subdirs[h] == 0 || subdirs[h] == 1 || subdirs[h] == 7) {
                subdirx = 0.5f;
            }
            if (subdirs[h] == 3 || subdirs[h] == 4 || subdirs[h] == 5) {
                subdirx = -0.5f;
            }
            histflowx += (float)dirsx[h] + subdirx;
            meancount_x++;

            float subdiry = 0.0f;
            if (subdirs[h] == 5 || subdirs[h] == 6 || subdirs[h] == 7) {
                subdiry = -0.5f;
            }
            if (subdirs[h] == 1 || subdirs[h] == 2 || subdirs[h] == 3) {
                subdiry = 0.5f;
            }
            histflowy += (float)dirsy[h] + subdiry;
            meancount_y++;
        }

        histflowx /= meancount_x;
        histflowy /= meancount_y;

        *pixel_flow_x = histflowx;
        *pixel_flow_y = histflowy;
    } else {
        *pixel_flow_x = 0.0f;
        *pixel_flow_y = 0.0f;
        return 0;
    }

    /* calc quality */
    uint8_t qual = (uint8_t)(meancount * 255 / (_num_blocks*_num_blocks));

    return qual;
}

#endif
