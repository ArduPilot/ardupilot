// SPDX-License-Identifier: GPL-3.0-or-later
//! SIMPLE_GEOFENCE_SETUP handling, as AC_DroneShowManager::configure_fences().

use crate::{coords::ShowFrame, mavlink::GeofenceSetup, vehicle};
use alloc::vec::Vec;
use ap_rust_sdk::{enums::MavSeverity, Context, Location, Result};

const FENCE_TYPE_ALT_MAX: u8 = 1;
const FENCE_TYPE_CIRCLE: u8 = 2;
const FENCE_TYPE_POLYGON: u8 = 4;

// AC_Fence::Action values the fork accepts
const ACCEPTED_ACTIONS: [u8; 6] = [0, 1, 2, 3, 4, 5];

fn error(c: &Context<'_>, text: &str) -> Result<bool> {
    c.gcs().send_text(MavSeverity::ERROR, text)?;
    Ok(false)
}

fn set_param(c: &Context<'_>, name: &str, value: f32) -> Result<bool> {
    if !c.param().set_and_save(name, value)? {
        return Ok(false);
    }
    Ok(true)
}

/// Validated fence setup, still to be written after the acknowledgment.
pub struct Pending {
    points: Vec<Location>,
    mask: u8,
}

/// Validates the setup and writes the fence parameters; None if refused.
/// The polygon is written by apply() after the acknowledgment: in SITL the
/// polygon write queues "Fence Indexed OK", which must follow the ACK.
pub fn prepare(c: &Context<'_>, frame: Option<&ShowFrame>, g: &GeofenceSetup<'_>) -> Result<Option<Pending>> {
    let action = g.flags & 0x0f;
    if !ACCEPTED_ACTIONS.contains(&action) {
        error(c, "Unsupported fence action")?;
        return Ok(None);
    }
    // validate and convert the polygon before changing anything
    let mut points = Vec::new();
    if g.num_points > 0 {
        let Some(frame) = frame else {
            error(c, "Show coordinate system is not valid")?;
            return Ok(None);
        };
        points.reserve(g.num_points);
        for i in 0..g.num_points {
            let (x, y) = g.point(i);
            points.push(frame.to_global(c, [x as f32 * 100.0, y as f32 * 100.0, 0.0])?);
        }
    }
    let mut mask = 0u8;
    if g.max_altitude_dm > 0 {
        if !set_param(c, "FENCE_ALT_MAX", g.max_altitude_dm as f32 / 10.0)? {
            error(c, "FENCE_ALT_MAX parameter cannot be set")?;
            return Ok(None);
        }
        mask |= FENCE_TYPE_ALT_MAX;
    }
    if g.radius_dm > 0 {
        if !set_param(c, "FENCE_RADIUS", g.radius_dm as f32 / 10.0)? {
            error(c, "FENCE_RADIUS parameter cannot be set")?;
            return Ok(None);
        }
        mask |= FENCE_TYPE_CIRCLE;
    }
    if !set_param(c, "FENCE_ACTION", action as f32)? {
        error(c, "FENCE_ACTION parameter cannot be set")?;
        return Ok(None);
    }
    if !points.is_empty() {
        mask |= FENCE_TYPE_POLYGON;
    }
    Ok(Some(Pending { points, mask }))
}

/// Writes the polygon and enables the selected fence types.
pub fn apply(c: &Context<'_>, p: &Pending) -> Result<bool> {
    if !p.points.is_empty() && !vehicle::write_inclusion_polygon(c, &p.points)? {
        return error(c, "Failed to write polygon fence");
    }
    // the fork writes the type mask into FENCE_ENABLE, which is only an
    // on/off switch here: select the types, then enable once all are in place
    if p.mask != 0 && !set_param(c, "FENCE_TYPE", p.mask as f32)? {
        return error(c, "FENCE_TYPE parameter cannot be set");
    }
    if !set_param(c, "FENCE_ENABLE", if p.mask != 0 { 1.0 } else { 0.0 })? {
        return error(c, "FENCE_ENABLE parameter cannot be set");
    }
    Ok(true)
}
