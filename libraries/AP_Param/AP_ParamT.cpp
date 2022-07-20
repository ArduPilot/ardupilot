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

#include "AP_Param.h"
#include <AP_Math/AP_Math.h>

// Param type template functions

// set a parameter that is an ENABLE param
template<typename T, ap_var_type PT>
void AP_ParamT<T, PT>::set_enable(const T &v) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    if (v != _value) {
#pragma GCC diagnostic pop
        invalidate_count();
    }
    _value = v;
}

// Sets if the parameter is unconfigured
template<typename T, ap_var_type PT>
void AP_ParamT<T, PT>::set_default(const T &v) {
#if AP_PARAM_DEFAULTS_ENABLED
    add_default(this, (float)v);
#endif
    if (!configured()) {
        set(v);
    }
}

// Sets parameter and default
template<typename T, ap_var_type PT>
void AP_ParamT<T, PT>::set_and_default(const T &v) {
#if AP_PARAM_DEFAULTS_ENABLED
    add_default(this, (float)v);
#endif
    set(v);
}

// Value setter - set value, tell GCS
template<typename T, ap_var_type PT>
void AP_ParamT<T, PT>::set_and_notify(const T &v) {
// We do want to compare each value, even floats, since it being the same here
// is the result of previously setting it.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    if (v != _value) {
#pragma GCC diagnostic pop
        set(v);
        notify();
    }
}

// Combined set and save
template<typename T, ap_var_type PT>
void AP_ParamT<T, PT>::set_and_save(const T &v) {
    bool force = fabsf((float)(_value - v)) < FLT_EPSILON;
    set(v);
    save(force);
}

// Combined set and save, but only does the save if the value if
// different from the current ram value, thus saving us a
// scan(). This should only be used where we have not set() the
// value separately, as otherwise the value in EEPROM won't be
// updated correctly.
template<typename T, ap_var_type PT>
void AP_ParamT<T, PT>::set_and_save_ifchanged(const T &v) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    if (v == _value) {
#pragma GCC diagnostic pop
        return;
    }
    set(v);
    save(true);
}

// AP_ParamT types can implement AP_Param::cast_to_float
template<typename T, ap_var_type PT>
float AP_ParamT<T, PT>::cast_to_float(void) const {
    return (float)_value;
}

template class AP_ParamT<float, AP_PARAM_FLOAT>;
template class AP_ParamT<int8_t, AP_PARAM_INT8>;
template class AP_ParamT<int16_t, AP_PARAM_INT16>;
template class AP_ParamT<int32_t, AP_PARAM_INT32>;

// Value setter - set value, tell GCS
template<typename T, ap_var_type PT>
void AP_ParamV<T, PT>::set_and_notify(const T &v) {
    if (v != _value) {
        set(v);
        notify();
    }
}

    /// Combined set and save
template<typename T, ap_var_type PT>
void AP_ParamV<T, PT>::set_and_save(const T &v) {
    bool force = (_value != v);
    set(v);
    save(force);
}

// Combined set and save, but only does the save if the value is
// different from the current ram value, thus saving us a
// scan(). This should only be used where we have not set() the
// value separately, as otherwise the value in EEPROM won't be
// updated correctly.
template<typename T, ap_var_type PT>
void AP_ParamV<T, PT>::set_and_save_ifchanged(const T &v) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    if (_value == v) {
#pragma GCC diagnostic pop
        return;
    }
    set(v);
    save(true);
}

template class AP_ParamV<Vector3f, AP_PARAM_VECTOR3F>;
