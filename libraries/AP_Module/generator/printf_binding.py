# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Experimental typed packets for the HAL's shared printf formatter."""

from native_templates import render


def specification(first_id):
    return dict(id=first_id, name='string.format_printf',
                arguments=[dict(type='format:nul-free-bytes-max255-zero-pad256,count:u8,'
                                     'tags:u8[8],values:le-u64[8],strings:nul-terminated-pool256,'
                                     'tags:1=i32,2=u32,3=i64,4=u64,5=f64,6=string-offset-u16', direction='in')],
                result='length:u16-le,text:nul-free-utf8-max255-zero-terminated256',
                policy='v1:hal-snprintf-args,no-allocation,max8args,checked-format-types,'
                       'd/i/u/o/x/X/c/s/f/F/e/E/g/G,ll-integers,width<=255,precision<=60,'
                       'no-star/no-positional/no-p/no-n,ascii-nonzero-c,overflow-is-invalid,'
                       'firmware-float32-rendering')


def rust(method, notice):
    return notice + render(
        'rust/printf.rs',
        method_id=method['id'],
    )
