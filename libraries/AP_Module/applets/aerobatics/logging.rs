// SPDX-License-Identifier: GPL-3.0-or-later
//! Preserve the Lua applet's DataFlash names, labels and encodings.
use super::{
    controller::{Input, Output},
    math::deg,
};
use ap_rust_sdk::{Context, Error, LogValue as L, Quaternion, Result, Vector3f};

fn write(c: &Context<'_>, name: &str, labels: &str, values: &[L<'_>]) -> Result {
    match c.logger().write(name, labels, None, values) {
        // Compiled-out logging must not disable the controller.
        Err(Error::Unavailable) => Ok(()),
        result => result,
    }
}

fn pose(c: &Context<'_>, name: &str, pos: Vector3f, q: Quaternion) -> Result {
    let Some(mut loc) = c.ahrs().get_origin()? else {
        return Ok(());
    };
    loc.offset(c, pos.x, pos.y)?;
    write(
        c,
        name,
        "px,py,pz,q1,q2,q3,q4,r,p,y,Lat,Lon",
        &[
            L::Float(pos.x),
            L::Float(pos.y),
            L::Float(pos.z),
            L::Float(q.q1),
            L::Float(q.q2),
            L::Float(q.q3),
            L::Float(q.q4),
            L::Float(deg(q.get_euler_roll(c)?)),
            L::Float(deg(q.get_euler_pitch(c)?)),
            L::Float(deg(q.get_euler_yaw(c)?)),
            L::LatLon(loc.lat()),
            L::LatLon(loc.lng()),
        ],
    )
}

pub fn record(c: &Context<'_>, input: Input, o: &Output) -> Result {
    if let Some((py, ly, pz, lz)) = o.lookahead_rates {
        write(
            c,
            "AELK",
            "Py,Ly,Pz,Lz",
            &[L::Float(py), L::Float(ly), L::Float(pz), L::Float(lz)],
        )?;
    }
    pose(c, "POSM", input.pos, o.measured_attitude)?;
    pose(c, "POST", o.track_pos, o.track_attitude)?;
    write(
        c,
        "AETM",
        "T,Terr,QCt,Adt",
        &[
            L::Float(o.path_t),
            L::Float(o.path_error_t),
            L::Float(o.q_change_t),
            L::Float(o.dt),
        ],
    )?;
    write(
        c,
        "AERT",
        "Cx,Cy,Cz,Px,Py,Pz,Ex,Tx,Ty,Tz,Perr,Aerr,Yff,Rofs",
        &[
            L::Float(o.correction_rates.x),
            L::Float(o.correction_rates.y),
            L::Float(o.correction_rates.z),
            L::Float(o.path_rates.x),
            L::Float(o.path_rates.y),
            L::Float(o.path_rates.z),
            L::Float(o.roll_error_rate),
            L::Float(o.rates.x),
            L::Float(o.rates.y),
            L::Float(o.rates.z),
            L::Float(o.position_error),
            L::Float(o.angle_error),
            L::Float(0.0),
            L::Float(o.rudder),
        ],
    )?;
    write(
        c,
        "AESP",
        "Targ,Curr,P,I,Total,Add",
        &[
            L::Float(o.speed_log[0]),
            L::Float(o.speed_log[1]),
            L::Float(o.speed_log[2]),
            L::Float(o.speed_log[3]),
            L::Float(o.speed_log[4]),
            L::Float(o.speed_log[5]),
        ],
    )
}
