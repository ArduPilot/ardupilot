/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_Filter_config.h"
#include "NotchFilter.h"

#if AP_FILTER_ENABLED

class AP_Filter {
public:
    enum class FilterType : uint8_t {
        FILTER_NONE            = 0,
        FILTER_NOTCH           = 1,
    };   

    AP_Filter(FilterType type);

    void init();

    virtual bool setup_notch_filter(NotchFilterFloat& filter, float sample_rate) { return false; }

    FilterType _type;
};

struct AP_Filter_params {
public:
    AP_Filter_params();

    static const struct AP_Param::GroupInfo var_info[];

    AP_Enum<AP_Filter::FilterType> _type;
};

struct AP_NotchFilter_params : public AP_Filter {
public:
    AP_NotchFilter_params();

    bool setup_notch_filter(NotchFilterFloat& filter, float sample_rate) override;

    static const struct AP_Param::GroupInfo var_info[];

    AP_Float _center_freq_hz;
    AP_Float _quality;
    AP_Float _attenuation_dB;
};

class AP_Filters {
public:
    AP_Filters();

    CLASS_NO_COPY(AP_Filters);

    static AP_Filters *get_singleton(void) { return singleton; }

    void init();
    // 1Hz update to process config changes
    void update();

    static const struct AP_Param::GroupInfo var_info[];

    static const struct AP_Param::GroupInfo *backend_var_info[AP_FILTER_NUM_FILTERS];

    AP_Filter* get_filter(uint8_t filt_num);

private:
    AP_Filter* filters[AP_FILTER_NUM_FILTERS];
    AP_Filter_params params[AP_FILTER_NUM_FILTERS];

    static AP_Filters *singleton;
};

namespace AP {
    AP_Filters &filters();
};

#endif // AP_FILTER_ENABLED

