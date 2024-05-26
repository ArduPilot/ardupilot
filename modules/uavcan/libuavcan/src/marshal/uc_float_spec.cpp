/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/marshal/float_spec.hpp>
#include <uavcan/build_config.hpp>
#include <cmath>

namespace uavcan
{

#if !UAVCAN_USE_EXTERNAL_FLOAT16_CONVERSION

union Fp32
{
    uint32_t u;
    float f;
};

/*
 * IEEE754Converter
 */
uint16_t IEEE754Converter::nativeIeeeToHalf(float value)
{
    /*
     * https://gist.github.com/rygorous/2156668
     * Public domain, by Fabian "ryg" Giesen
     */
    const Fp32 f32infty = { 255U << 23 };
    const Fp32 f16infty = { 31U << 23 };
    const Fp32 magic = { 15U << 23 };
    const uint32_t sign_mask = 0x80000000U;
    const uint32_t round_mask = ~0xFFFU;

    Fp32 in;
    uint16_t out;

    in.f = value;

    uint32_t sign = in.u & sign_mask;
    in.u ^= sign;

    if (in.u >= f32infty.u) /* Inf or NaN (all exponent bits set) */
    {
        /* NaN->sNaN and Inf->Inf */
        out = (in.u > f32infty.u) ? 0x7FFFU : 0x7C00U;
    }
    else /* (De)normalized number or zero */
    {
        in.u &= round_mask;
        in.f *= magic.f;
        in.u -= round_mask;
        if (in.u > f16infty.u)
        {
            in.u = f16infty.u; /* Clamp to signed infinity if overflowed */
        }

        out = uint16_t(in.u >> 13); /* Take the bits! */
    }

    out = uint16_t(out | (sign >> 16));

    return out;
}

float IEEE754Converter::halfToNativeIeee(uint16_t value)
{
    /*
     * https://gist.github.com/rygorous/2144712
     * Public domain, by Fabian "ryg" Giesen
     */
    const Fp32 magic = { (254U - 15U) << 23 };
    const Fp32 was_infnan = { (127U + 16U) << 23 };
    Fp32 out;

    out.u = (value & 0x7FFFU) << 13;   /* exponent/mantissa bits */
    out.f *= magic.f;                  /* exponent adjust */
    if (out.f >= was_infnan.f)         /* make sure Inf/NaN survive */
    {
        out.u |= 255U << 23;
    }
    out.u |= (value & 0x8000U) << 16;  /* sign bit */

    return out.f;
}

#endif // !UAVCAN_USE_EXTERNAL_FLOAT16_CONVERSION

}
