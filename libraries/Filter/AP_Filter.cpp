#include "AP_Filter_config.h"

#if AP_FILTER_ENABLED

#include "AP_Filter.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/AP_HAL_Boards.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Filters::var_info[] = {

#if AP_FILTER_NUM_FILTERS >= 1
    // @Group: 1_
    // @Path: AP_Filter_params.cpp
    AP_SUBGROUPINFO(params[0], "1_", 2, AP_Filters, AP_Filter_params),
    // @Group: 1_
    // @Path: AP_NotchFilter_params.cpp
    AP_SUBGROUPVARPTR(filters[0], "1_", 3, AP_Filters, backend_var_info[0]),
#endif
#if AP_FILTER_NUM_FILTERS >= 2
    // @Group: 2_
    // @Path: AP_Filter_params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 4, AP_Filters, AP_Filter_params),
    // @Group: 2_
    // @Path: AP_NotchFilter_params.cpp
    AP_SUBGROUPVARPTR(filters[1], "2_", 5, AP_Filters, backend_var_info[1]),
#endif
#if AP_FILTER_NUM_FILTERS >= 3
    // @Group: 3_
    // @Path: AP_Filter_params.cpp
    AP_SUBGROUPINFO(params[2], "3_", 6, AP_Filters, AP_Filter_params),
    // @Group: 3_
    // @Path: AP_NotchFilter_params.cpp
    AP_SUBGROUPVARPTR(filters[2], "3_", 7, AP_Filters, backend_var_info[2]),
#endif
#if AP_FILTER_NUM_FILTERS >= 4
    // @Group: 4_
    // @Path: AP_Filter_params.cpp
    AP_SUBGROUPINFO(params[3], "4_", 8, AP_Filters, AP_Filter_params),
    // @Group: 4_
    // @Path: AP_NotchFilter_params.cpp
    AP_SUBGROUPVARPTR(filters[3], "4_", 9, AP_Filters, backend_var_info[3]),
#endif
#if AP_FILTER_NUM_FILTERS >= 5
    // @Group: 5_
    // @Path: AP_Filter_params.cpp
    AP_SUBGROUPINFO(params[4], "5_", 10, AP_Filters, AP_Filter_params),
    // @Group: 5_
    // @Path: AP_NotchFilter_params.cpp
    AP_SUBGROUPVARPTR(filters[4], "5_", 11, AP_Filters, backend_var_info[4]),
#endif
#if AP_FILTER_NUM_FILTERS >= 6
    // @Group: 6_
    // @Path: AP_Filter_params.cpp
    AP_SUBGROUPINFO(params[5], "6_", 12, AP_Filters, AP_Filter_params),
    // @Group: 6_
    // @Path: AP_NotchFilter_params.cpp
    AP_SUBGROUPVARPTR(filters[5], "6_", 13, AP_Filters, backend_var_info[5]),
#endif
#if AP_FILTER_NUM_FILTERS >= 7
    // @Group: 7_
    // @Path: AP_Filter_params.cpp
    AP_SUBGROUPINFO(params[6], "7_", 14, AP_Filters, AP_Filter_params),
    // @Group: 7_
    // @Path: AP_NotchFilter_params.cpp
    AP_SUBGROUPVARPTR(filters[6], "7_", 15, AP_Filters, backend_var_info[6]),
#endif
#if AP_FILTER_NUM_FILTERS >= 8
    // @Group: 8_
    // @Path: AP_Filter_params.cpp
    AP_SUBGROUPINFO(params[7], "8_", 16, AP_Filters, AP_Filter_params),
    // @Group: 8_
    // @Path: AP_NotchFilter_params.cpp
    AP_SUBGROUPVARPTR(filters[7], "8_", 17, AP_Filters, backend_var_info[7]),
#endif
    AP_GROUPEND
};

const AP_Param::GroupInfo *AP_Filters::backend_var_info[AP_FILTER_NUM_FILTERS];

AP_Filter::AP_Filter(FilterType type):
    _type(type)
{
}

AP_Filters::AP_Filters()
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (singleton != nullptr) {
        AP_HAL::panic("AP_Filters must be singleton");
    }
#endif
    singleton = this;
}

void AP_Filters::init()
{
    update();
}

// 1Hz update to process config changes
void AP_Filters::update()
{
    if (hal.util->get_soft_armed()) {
        return;
    }

    // make sure all filters are allocated
    for (uint8_t i = 0; i < AP_FILTER_NUM_FILTERS; i++) {
        bool update = false;
        switch (AP_Filter::FilterType(params[i]._type)) {
            case AP_Filter::FilterType::FILTER_NONE:
                break;
            case AP_Filter::FilterType::FILTER_NOTCH:
                if (filters[i] == nullptr) {
                    filters[i] = NEW_NOTHROW AP_NotchFilter_params();
                    backend_var_info[i] = AP_NotchFilter_params::var_info;
                    update = true;
                }
                break;
            default:
                return;
        }

        if (update) {
            AP_Param::load_object_from_eeprom(filters[i], backend_var_info[i]);
            AP_Param::invalidate_count();
        }
    }
}

AP_Filter* AP_Filters::get_filter(uint8_t index)
{
    if (index >= AP_FILTER_NUM_FILTERS) {
        return nullptr;
    }

    return filters[index-1];
}

// singleton instance
AP_Filters *AP_Filters::singleton;

namespace AP {

AP_Filters &filters()
{
    return *AP_Filters::get_singleton();
}

}

#endif // AP_FILTER_ENABLED
