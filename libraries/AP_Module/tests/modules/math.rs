// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Quaternion, Result, Vector3f};

ap_rust_sdk::module!(run);

fn near(a: f32, b: f32) {
    assert!((a - b).abs() < 0.00001);
}

fn run(context: &Context<'_>) -> Result {
    let x = Vector3f::new(1.0, 0.0, 0.0);
    let z = Vector3f::new(0.0, 0.0, 1.0);
    let half_pi = core::f32::consts::FRAC_PI_2;
    for _ in 0..100 {
        let mut v = Vector3f::new(3.0, 4.0, 0.0);
        assert!(Vector3f::default().is_zero(context)?);
        assert!(!v.is_zero(context)?);
        assert!(!v.is_nan(context)?);
        assert!(!v.is_inf(context)?);
        assert!(Vector3f::new(f32::NAN, 0.0, 0.0).is_nan(context)?);
        assert!(Vector3f::new(0.0, f32::INFINITY, 0.0).is_inf(context)?);
        assert_eq!(v.length(context)?, 5.0);
        assert_eq!(v.dot(context, x)?, 3.0);
        assert_eq!(v.cross(context, z)?, Vector3f::new(4.0, -3.0, 0.0));
        assert_eq!(v.scale(context, 2.0)?, Vector3f::new(6.0, 8.0, 0.0));
        assert_eq!(v.add(context, x)?.sub(context, x)?, v);
        near(v.angle(context, x)?, 0.9272952);
        let mut rotated = v;
        rotated.rotate_xy(context, half_pi)?;
        near(rotated.x, -4.0);
        near(rotated.y, 3.0);
        v.normalize(context)?;
        near(v.x, 0.6);
        near(v.y, 0.8);

        let mut q = Quaternion::default();
        assert_eq!(q, Quaternion::new(1.0, 0.0, 0.0, 0.0));
        q.from_axis_angle(context, z, half_pi)?;
        near(q.length(context)?, 1.0);
        near(q.get_euler_roll(context)?, 0.0);
        near(q.get_euler_pitch(context)?, 0.0);
        near(q.get_euler_yaw(context)?, half_pi);
        q.normalize(context)?;
        let mut axis = Vector3f::new(99.0, 99.0, 99.0);
        q.to_axis_angle(context, &mut axis)?;
        near(axis.x, 0.0);
        near(axis.y, 0.0);
        near(axis.z, half_pi);
        let mut body = x;
        q.earth_to_body(context, &mut body)?;
        near(body.x, 0.0);
        near(body.y, 1.0);
        near(body.z, 0.0);
        let identity = q.mul(context, q.inverse(context)?)?;
        near(identity.q1, 1.0);
        near(identity.q2, 0.0);
        near(identity.q3, 0.0);
        near(identity.q4, 0.0);
        q.from_euler(context, 0.1, -0.2, 0.3)?;
        near(q.get_euler_roll(context)?, 0.1);
        near(q.get_euler_pitch(context)?, -0.2);
        near(q.get_euler_yaw(context)?, 0.3);
        q.from_angular_velocity(context, z, 0.02)?;
        near(q.q4 / q.q1, 0.01);
        near(q.length(context)?, 1.0);
    }
    context.text(6, "MATH ok")
}
