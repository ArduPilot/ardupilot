#include "float16.h"
/*
  float16 implementation

  algorithm with thanks to libcanard:
  https://github.com/dronecan/libcanard
*/


float Float16_t::get(void) const
{
    union FP32 {
        uint32_t u;
        float f;
    };
    const union FP32 magic = { (254UL - 15UL) << 23U };
    const union FP32 was_inf_nan = { (127UL + 16UL) << 23U };
    union FP32 out;

    out.u = (v16 & 0x7FFFU) << 13U;
    out.f *= magic.f;
    if (out.f >= was_inf_nan.f) {
        out.u |= 255UL << 23U;
    }
    out.u |= (v16 & 0x8000UL) << 16U;

    return out.f;
}

void Float16_t::set(float value)
{
    union FP32
    {
        uint32_t u;
        float f;
    };

    const union FP32 f32inf = { 255UL << 23U };
    const union FP32 f16inf = { 31UL << 23U };
    const union FP32 magic = { 15UL << 23U };
    const uint32_t sign_mask = 0x80000000UL;
    const uint32_t round_mask = 0xFFFFF000UL;

    union FP32 in;
    in.f = value;
    uint32_t sign = in.u & sign_mask;
    in.u ^= sign;

    v16 = 0;

    if (in.u >= f32inf.u)
    {
        v16 = (in.u > f32inf.u) ? (uint16_t)0x7FFFU : (uint16_t)0x7C00U;
    }
    else
    {
        in.u &= round_mask;
        in.f *= magic.f;
        in.u -= round_mask;
        if (in.u > f16inf.u)
        {
            in.u = f16inf.u;
        }
        v16 = (uint16_t)(in.u >> 13U);
    }

    v16 |= (uint16_t)(sign >> 16U);
}
