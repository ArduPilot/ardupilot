# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Firmware C-library float conversion, with an end offset instead of a pointer."""

from native_templates import render


def specification(first_id):
    return dict(id=first_id, name='string.strtof',
                arguments=[dict(type='nul-free-bytes-max255-zero-pad256', direction='in')],
                result=dict(value='f32-le', consumed='u16-le'),
                policy='v2:libc-strtof,may-allocate,no-scheduler-lock,platform-c-number-prefix,'
                       'no-conversion-is-zero-consumed,allow-nonfinite,ignore-errno,'
                       'platform-rounding')


def cpp(_method):
    return '''static int32_t binding_string_strtof(const uint8_t *input, uint32_t,
                                    uint8_t *output, uint32_t)
{
    const char *text = BindingWire::read_cstring(input);
    if (text == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    char *end;
    const float value = strtof(text, &end);
    BindingWire::write_float(output, value);
    BindingWire::write(output + 4, uint16_t(end - text), 2);
    return AP_MODULE_STATUS_OK;
}
'''.splitlines()


def rust(method, notice):
    return notice + render(
        'rust/strtof.rs',
        method_id=method['id'],
    )
