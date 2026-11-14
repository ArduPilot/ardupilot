// SPDX-License-Identifier: GPL-3.0-or-later
//! Numeric conventions of plane_aerobatics.lua (LUA_32BITS).

use ap_rust_sdk::{Context, Error, Quaternion, Result, Vector3f};

pub const PI: f32 = core::f32::consts::PI;

// Lua's math.rad/deg multiply by this same rounded float conversion factor.
pub fn rad(x: f32) -> f32 {
    x * (PI / 180.0)
}
pub fn deg(x: f32) -> f32 {
    x * (180.0 / PI)
}
pub fn min(a: f32, b: f32) -> f32 {
    if b < a {
        b
    } else {
        a
    }
}
pub fn max(a: f32, b: f32) -> f32 {
    if b > a {
        b
    } else {
        a
    }
}
pub fn constrain(mut x: f32, low: f32, high: f32) -> f32 {
    if x < low {
        x = low;
    }
    if x > high {
        x = high;
    }
    x
}

// Preserve the source's asymmetric epsilon comparison, including sgn(0)=-1.
pub fn sgn(x: f32) -> f32 {
    if x > 0.000001 {
        1.0
    } else if x < 0.000001 {
        -1.0
    } else {
        0.0
    }
}

pub fn wrap_360(c: &Context<'_>, x: f32) -> Result<f32> {
    let mut value = c.libm().fmodf(x, 360.0)?;
    if value < 0.0 {
        value += 360.0;
    }
    Ok(value)
}
pub fn wrap_180(c: &Context<'_>, x: f32) -> Result<f32> {
    let value = wrap_360(c, x)?;
    Ok(if value > 180.0 { value - 360.0 } else { value })
}
pub fn wrap_pi(c: &Context<'_>, x: f32) -> Result<f32> {
    Ok(rad(wrap_180(c, deg(x))?))
}
pub fn wrap_2pi(c: &Context<'_>, x: f32) -> Result<f32> {
    Ok(rad(wrap_360(c, deg(x))?))
}
pub fn lowpass_alpha(dt: f32, time_constant: f32) -> f32 {
    dt / (dt + time_constant / (PI * 2.0))
}
pub fn dcm_c_y(q: Quaternion) -> f32 {
    2.0 * (q.q3 * q.q4 + q.q1 * q.q2)
}

pub fn orient(c: &Context<'_>, roll: f32, pitch: f32, yaw: f32) -> Result<Quaternion> {
    let mut q = Quaternion::default();
    q.from_euler(c, rad(roll), rad(pitch), rad(yaw))?;
    Ok(q)
}
pub fn earth_to_body(c: &Context<'_>, q: Quaternion, mut v: Vector3f) -> Result<Vector3f> {
    q.earth_to_body(c, &mut v)?;
    Ok(v)
}
pub fn body_to_earth(c: &Context<'_>, q: Quaternion, v: Vector3f) -> Result<Vector3f> {
    earth_to_body(c, q.inverse(c)?, v)
}
pub fn vectors_to_rotation(
    c: &Context<'_>,
    mut a: Vector3f,
    mut b: Vector3f,
) -> Result<Quaternion> {
    // Reject degenerate tangents locally instead of triggering AP_Math's
    // firmware-wide INTERNAL_ERROR on a zero quaternion.
    let al = a.length(c)?;
    let bl = b.length(c)?;
    if !al.is_finite() || !bl.is_finite() || al <= 0.0 || bl <= 0.0 {
        return Err(Error::Invalid);
    }
    a.normalize(c)?;
    b.normalize(c)?;
    let dot = a.dot(c, b)?;
    let axis = a.cross(c, b)?;
    let mut q = Quaternion::new(1.0 + dot, axis.x, axis.y, axis.z);
    let length = q.length(c)?;
    if !length.is_finite() || length < f32::EPSILON {
        return Err(Error::Invalid);
    }
    q.normalize(c)?;
    Ok(q)
}
pub fn tangents_to_rate(c: &Context<'_>, a: Vector3f, b: Vector3f, dt: f32) -> Result<Vector3f> {
    if !dt.is_finite() || dt <= 0.0 || !(1.0 / dt).is_finite() {
        return Err(Error::Invalid);
    }
    let q = vectors_to_rotation(c, a, b)?;
    let mut rate = Vector3f::default();
    q.to_axis_angle(c, &mut rate)?;
    rate.scale(c, 1.0 / dt)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn lua_ordering_and_epsilon() {
        assert!(min(f32::NAN, 1.0).is_nan());
        assert!(max(f32::NAN, 1.0).is_nan());
        assert_eq!(min(1.0, f32::NAN), 1.0);
        assert_eq!(max(1.0, f32::NAN), 1.0);
        assert_eq!(sgn(0.0), -1.0);
        assert_eq!(sgn(0.000001), 0.0);
        assert_eq!(sgn(0.000002), 1.0);
        assert_eq!(constrain(2.0, 3.0, 1.0), 1.0);
    }
}
