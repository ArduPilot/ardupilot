// SPDX-License-Identifier: GPL-3.0-or-later
//! Pure-Rust float helpers for no_std (core has no libm).
//!
//! cbrtf, cosf, acosf, logf, expf, floorf, ceilf and the kernels are ports of
//! musl libc (MIT, derived from FreeBSD/fdlibm, Copyright Sun Microsystems).
//! sqrtf/sqrt are exact integer square roots, so they are correctly rounded.

const fn pow2(e: i32) -> f64 {
    f64::from_bits(((1023 + e) as u64) << 52)
}

// musl-style hex float constant: m * 2^e
const fn hf(m: u64, e: i32) -> f64 {
    m as f64 * pow2(e)
}

fn isqrt_rem(n: u128) -> (u128, bool) {
    let mut op = n;
    let mut res = 0u128;
    let mut one = 1u128 << 126;
    while one > op {
        one >>= 2;
    }
    while one != 0 {
        if op >= res + one {
            op -= res + one;
            res = (res >> 1) + one;
        } else {
            res >>= 1;
        }
        one >>= 2;
    }
    (res, op != 0)
}

// Correctly rounded sqrt of a finite positive value given as mantissa bits
// `m` (with `p` bits of precision, implicit bit included) and unbiased
// exponent `e` of the leading bit; returns (biased exponent, mantissa) packed.
fn sqrt_bits(mut m: u64, mut e: i32, p: u32, bias: i32) -> u64 {
    while m >> (p - 1) == 0 {
        m <<= 1;
        e -= 1;
    }
    let ee = e - (p as i32 - 1); // x = m * 2^ee
    let k = if (ee - (p as i32 + 3)) & 1 == 0 { p + 3 } else { p + 4 };
    let (s, inexact) = isqrt_rem((m as u128) << k);
    let mut q = (s >> 2) as u64;
    let round = (s >> 1) & 1 == 1;
    let sticky = s & 1 == 1 || inexact;
    if round && (sticky || q & 1 == 1) {
        q += 1;
    }
    let exp = (ee - k as i32) / 2 + 2 + p as i32 - 1 + bias;
    ((exp as u64) << (p - 1)) + (q - (1 << (p - 1)))
}

pub fn sqrtf(x: f32) -> f32 {
    let u = x.to_bits();
    if x.is_nan() || x == 0.0 || x == f32::INFINITY {
        return x;
    }
    if u >> 31 != 0 {
        return f32::NAN;
    }
    let be = (u >> 23) as i32;
    let (m, e) = if be == 0 { (u as u64, -126) } else { (((u & 0x7f_ffff) | 0x80_0000) as u64, be - 127) };
    f32::from_bits(sqrt_bits(m, e, 24, 127) as u32)
}

pub fn sqrt(x: f64) -> f64 {
    let u = x.to_bits();
    if x.is_nan() || x == 0.0 || x == f64::INFINITY {
        return x;
    }
    if u >> 63 != 0 {
        return f64::NAN;
    }
    let be = (u >> 52) as i32;
    let (m, e) = if be == 0 {
        (u, -1022)
    } else {
        ((u & 0xf_ffff_ffff_ffff) | (1 << 52), be - 1023)
    };
    f64::from_bits(sqrt_bits(m, e, 53, 1023))
}

/// hypotf as in glibc: double-precision sum of squares, rounded once.
pub fn hypotf(x: f32, y: f32) -> f32 {
    if x.is_infinite() || y.is_infinite() {
        return f32::INFINITY;
    }
    let (x, y) = (x as f64, y as f64);
    sqrt(x * x + y * y) as f32
}

pub fn floorf(x: f32) -> f32 {
    let mut u = x.to_bits();
    let e = ((u >> 23) & 0xff) as i32 - 0x7f;
    if e >= 23 {
        return x;
    }
    if e >= 0 {
        let m = 0x007f_ffffu32 >> e;
        if u & m == 0 {
            return x;
        }
        if u >> 31 != 0 {
            u += m;
        }
        u &= !m;
    } else if u >> 31 == 0 {
        u = 0;
    } else if u << 1 != 0 {
        return -1.0;
    }
    f32::from_bits(u)
}

pub fn ceilf(x: f32) -> f32 {
    let mut u = x.to_bits();
    let e = ((u >> 23) & 0xff) as i32 - 0x7f;
    if e >= 23 {
        return x;
    }
    if e >= 0 {
        let m = 0x007f_ffffu32 >> e;
        if u & m == 0 {
            return x;
        }
        if u >> 31 == 0 {
            u += m;
        }
        u &= !m;
    } else if u >> 31 != 0 {
        return -0.0;
    } else if u << 1 != 0 {
        return 1.0;
    }
    f32::from_bits(u)
}

pub fn truncf(x: f32) -> f32 {
    let mut u = x.to_bits();
    let e = ((u >> 23) & 0xff) as i32 - 0x7f + 9;
    if e >= 32 {
        return x;
    }
    if e < 9 {
        return f32::from_bits(u & 0x8000_0000);
    }
    u &= !(u32::MAX >> e);
    f32::from_bits(u)
}

/// Rounds half away from zero, like C roundf.
pub fn roundf(x: f32) -> f32 {
    truncf(x + (0.5 - 0.25 * f32::EPSILON).copysign(x))
}

pub fn cbrtf(x: f32) -> f32 {
    const B1: u32 = 709958130;
    const B2: u32 = 642849266;
    let mut u = x.to_bits();
    let mut hx = u & 0x7fff_ffff;
    if hx >= 0x7f80_0000 {
        return x + x;
    }
    if hx < 0x0080_0000 {
        if hx == 0 {
            return x;
        }
        u = (x * f32::from_bits(0x4b80_0000)).to_bits(); // x * 2^24
        hx = u & 0x7fff_ffff;
        hx = hx / 3 + B2;
    } else {
        hx = hx / 3 + B1;
    }
    u &= 0x8000_0000;
    u |= hx;
    let xd = x as f64;
    let mut t = f32::from_bits(u) as f64;
    let mut r = t * t * t;
    t = t * (xd + xd + r) / (xd + r + r);
    r = t * t * t;
    t = t * (xd + xd + r) / (xd + r + r);
    t as f32
}

fn cosdf(x: f64) -> f32 {
    const C0: f64 = -hf(0x1ffffffd0c5e81, -54);
    const C1: f64 = hf(0x155553e1053a42, -57);
    const C2: f64 = -hf(0x16c087e80f1e27, -62);
    const C3: f64 = hf(0x199342e0ee5069, -68);
    let z = x * x;
    let w = z * z;
    let r = C2 + z * C3;
    (((1.0 + z * C0) + w * C1) + (w * z) * r) as f32
}

fn sindf(x: f64) -> f32 {
    const S1: f64 = -hf(0x15555554cbac77, -55);
    const S2: f64 = hf(0x111110896efbb2, -59);
    const S3: f64 = -hf(0x1a00f9e2cae774, -65);
    const S4: f64 = hf(0x16cd878c3b46a7, -71);
    let z = x * x;
    let w = z * z;
    let r = S3 + z * S4;
    let s = z * x;
    ((x + s * (S1 + z * S2)) + s * w * r) as f32
}

// Medium-size argument reduction from musl __rem_pio2f; accurate for
// |x| < 2^28*pi/2, which covers every use here.
fn rem_pio2f(x: f32) -> (i32, f64) {
    const TOINT: f64 = 1.5 / f64::EPSILON;
    const PIO4: f64 = hf(0x1921fb6, -25);
    let invpio2 = f64::from_bits(0x3FE4_5F30_6DC9_C883);
    let pio2_1 = f64::from_bits(0x3FF9_21FB_5000_0000);
    let pio2_1t = f64::from_bits(0x3E51_10B4_611A_6263);
    let xd = x as f64;
    let mut fnv = xd * invpio2 + TOINT - TOINT;
    let mut n = fnv as i32;
    let mut y = xd - fnv * pio2_1 - fnv * pio2_1t;
    if y < -PIO4 {
        n -= 1;
        fnv -= 1.0;
        y = xd - fnv * pio2_1 - fnv * pio2_1t;
    } else if y > PIO4 {
        n += 1;
        fnv += 1.0;
        y = xd - fnv * pio2_1 - fnv * pio2_1t;
    }
    (n, y)
}

pub fn cosf(x: f32) -> f32 {
    const C1PIO2: f64 = core::f64::consts::FRAC_PI_2;
    const C2PIO2: f64 = 2.0 * C1PIO2;
    const C3PIO2: f64 = 3.0 * C1PIO2;
    const C4PIO2: f64 = 4.0 * C1PIO2;
    let u = x.to_bits();
    let sign = u >> 31 != 0;
    let ix = u & 0x7fff_ffff;
    let xd = x as f64;
    if ix <= 0x3f49_0fda {
        if ix < 0x3980_0000 {
            return 1.0;
        }
        return cosdf(xd);
    }
    if ix <= 0x407b_53d1 {
        if ix > 0x4016_cbe3 {
            return -cosdf(if sign { xd + C2PIO2 } else { xd - C2PIO2 });
        }
        return if sign { sindf(xd + C1PIO2) } else { sindf(C1PIO2 - xd) };
    }
    if ix <= 0x40e2_31d5 {
        if ix > 0x40af_eddf {
            return cosdf(if sign { xd + C4PIO2 } else { xd - C4PIO2 });
        }
        return if sign { sindf(-xd - C3PIO2) } else { sindf(xd - C3PIO2) };
    }
    if ix >= 0x7f80_0000 {
        return f32::NAN;
    }
    let (n, y) = rem_pio2f(x);
    match n & 3 {
        0 => cosdf(y),
        1 => sindf(-y),
        2 => -cosdf(y),
        _ => sindf(y),
    }
}

pub fn acosf(x: f32) -> f32 {
    let pio2_hi = f32::from_bits(0x3fc9_0fda);
    let pio2_lo = f32::from_bits(0x33a2_2168);
    const PS0: f32 = 1.6666586697e-01;
    const PS1: f32 = -4.2743422091e-02;
    const PS2: f32 = -8.6563630030e-03;
    const QS1: f32 = -7.0662963390e-01;
    let r = |z: f32| {
        let p = z * (PS0 + z * (PS1 + z * PS2));
        let q = 1.0 + z * QS1;
        p / q
    };
    let hx = x.to_bits();
    let ix = hx & 0x7fff_ffff;
    if ix >= 0x3f80_0000 {
        if ix == 0x3f80_0000 {
            return if hx >> 31 != 0 { 2.0 * pio2_hi + f32::from_bits(0x0380_0000) } else { 0.0 };
        }
        return f32::NAN;
    }
    if ix < 0x3f00_0000 {
        if ix <= 0x3280_0000 {
            return pio2_hi + f32::from_bits(0x0380_0000);
        }
        return pio2_hi - (x - (pio2_lo - x * r(x * x)));
    }
    if hx >> 31 != 0 {
        let z = (1.0 + x) * 0.5;
        let s = sqrtf(z);
        let w = r(z) * s - pio2_lo;
        return 2.0 * (pio2_hi - (s + w));
    }
    let z = (1.0 - x) * 0.5;
    let s = sqrtf(z);
    let df = f32::from_bits(s.to_bits() & 0xffff_f000);
    let c = (z - df * df) / (s + df);
    let w = r(z) * s + c;
    2.0 * (df + w)
}

pub fn logf(x: f32) -> f32 {
    let ln2_hi = f32::from_bits(0x3f31_7180);
    let ln2_lo = f32::from_bits(0x3717_f7d1);
    const LG1: f32 = hf(0xaaaaaa, -24) as f32;
    const LG2: f32 = hf(0xccce13, -25) as f32;
    const LG3: f32 = hf(0x91e9ee, -25) as f32;
    const LG4: f32 = hf(0xf89e26, -26) as f32;
    let mut x = x;
    let mut ix = x.to_bits();
    let mut k: i32 = 0;
    if ix < 0x0080_0000 || ix >> 31 != 0 {
        if ix << 1 == 0 {
            return f32::NEG_INFINITY;
        }
        if ix >> 31 != 0 {
            return f32::NAN;
        }
        k -= 25;
        x *= f32::from_bits(0x4c00_0000); // 2^25
        ix = x.to_bits();
    } else if ix >= 0x7f80_0000 {
        return x;
    } else if ix == 0x3f80_0000 {
        return 0.0;
    }
    ix += 0x3f80_0000 - 0x3f35_04f3;
    k += (ix >> 23) as i32 - 0x7f;
    ix = (ix & 0x007f_ffff) + 0x3f35_04f3;
    x = f32::from_bits(ix);
    let f = x - 1.0;
    let s = f / (2.0 + f);
    let z = s * s;
    let w = z * z;
    let t1 = w * (LG2 + w * LG4);
    let t2 = z * (LG1 + w * LG3);
    let r = t2 + t1;
    let hfsq = 0.5 * f * f;
    let dk = k as f32;
    s * (hfsq + r) + dk * ln2_lo - hfsq + f + dk * ln2_hi
}

fn scalbnf(mut x: f32, mut n: i32) -> f32 {
    let p127 = f32::from_bits(0x7f00_0000);
    let pm126 = f32::from_bits(0x0080_0000);
    while n > 127 {
        x *= p127;
        n -= 127;
    }
    while n < -126 {
        x *= pm126;
        n += 126;
    }
    x * f32::from_bits(((0x7f + n) as u32) << 23)
}

pub fn expf(x: f32) -> f32 {
    let ln2hi = f32::from_bits(0x3f31_7200);
    let ln2lo = f32::from_bits(0x35bf_be8e);
    let invln2 = f32::from_bits(0x3fb8_aa3b);
    const P1: f32 = hf(0xaaaa8f, -26) as f32;
    const P2: f32 = -hf(0xb55215, -32) as f32;
    let u = x.to_bits();
    let sign = (u >> 31) as i32;
    let hx = u & 0x7fff_ffff;
    if hx >= 0x42ae_ac50 {
        if hx > 0x7f80_0000 {
            return x;
        }
        if hx >= 0x42b1_7218 && sign == 0 {
            return f32::INFINITY;
        }
        if sign != 0 && hx >= 0x42cf_f1b5 {
            return 0.0;
        }
    }
    let (k, hi, lo, x) = if hx > 0x3eb1_7218 {
        let k: i32 = if hx > 0x3f85_1592 {
            (invln2 * x + if sign != 0 { -0.5 } else { 0.5 }) as i32
        } else {
            1 - sign - sign
        };
        let hi = x - k as f32 * ln2hi;
        let lo = k as f32 * ln2lo;
        (k, hi, lo, hi - lo)
    } else if hx > 0x3900_0000 {
        (0, x, 0.0, x)
    } else {
        return 1.0 + x;
    };
    let xx = x * x;
    let c = x - xx * (P1 + xx * P2);
    let y = 1.0 + (x * c / (2.0 - c) - lo + hi);
    if k == 0 {
        y
    } else {
        scalbnf(y, k)
    }
}

/// x^y for x > 0 via expf(y*logf(x)); a few ulp, enough for colour maths.
pub fn powf(x: f32, y: f32) -> f32 {
    if y == 0.0 {
        return 1.0;
    }
    if x.is_nan() || y.is_nan() || x < 0.0 {
        return f32::NAN;
    }
    expf(y * logf(x))
}
