// SPDX-License-Identifier: GPL-3.0-or-later
//! Show coordinate system: mm, X along the show orientation, Y to the left, Z up.

use crate::show::fmath::cosf;
use ap_rust_sdk::{enums::LocationAltFrame, Context, Error, Location, Result, Vector3f};

pub const ALT_ABSOLUTE: LocationAltFrame = LocationAltFrame(0);
pub const ALT_ABOVE_HOME: LocationAltFrame = LocationAltFrame(1);

#[derive(Clone, Copy, Default)]
pub struct ShowFrame {
    pub origin_lat: i32,
    pub origin_lng: i32,
    pub origin_amsl_mm: i32,
    pub amsl_valid: bool,
    cos_o: f32,
    sin_o: f32,
    pub orientation_deg: f32,
}

impl ShowFrame {
    pub fn new(lat: i32, lng: i32, amsl_mm: i32, amsl_valid: bool, orientation_deg: f32) -> Self {
        let r = orientation_deg.to_radians();
        Self {
            origin_lat: lat,
            origin_lng: lng,
            origin_amsl_mm: amsl_mm,
            amsl_valid,
            cos_o: cosf(r),
            sin_o: cosf(r - core::f32::consts::FRAC_PI_2),
            orientation_deg,
        }
    }

    /// Show-frame horizontal vector (mm) to (north, east) metres.
    pub fn rotate_to_ne(&self, x: f32, y: f32) -> (f32, f32) {
        (
            (self.cos_o * x + self.sin_o * y) / 1000.0,
            (self.sin_o * x - self.cos_o * y) / 1000.0,
        )
    }

    /// (north, east) metres to show-frame horizontal vector (mm).
    pub fn rotate_from_ne(&self, n: f32, e: f32) -> (f32, f32) {
        let (x, y) = (n * 1000.0, -e * 1000.0);
        (self.cos_o * x - self.sin_o * y, self.sin_o * x + self.cos_o * y)
    }

    /// Show yaw (deg relative to X axis) to heading in degrees.
    pub fn yaw_to_global_deg(&self, yaw: f32) -> f32 {
        self.orientation_deg + yaw
    }

    /// convert_show_to_global_coordinate()
    pub fn to_global(&self, c: &Context<'_>, p: [f32; 3]) -> Result<Location> {
        let (n, e) = self.rotate_to_ne(p[0], p[1]);
        let mut loc = Location::default();
        loc.set_lat(c, self.origin_lat)?;
        loc.set_lng(c, self.origin_lng)?;
        // the fork truncates the altitude to whole centimetres
        let alt_cm = (p[2] / 10.0) as i32;
        if self.amsl_valid {
            let cm = alt_cm as f32 + self.origin_amsl_mm as f32 / 10.0;
            loc.set_alt_m(c, cm / 100.0, ALT_ABSOLUTE)?;
        } else {
            loc.set_alt_m(c, alt_cm as f32 / 100.0, ALT_ABOVE_HOME)?;
        }
        loc.offset(c, n, e)?;
        Ok(loc)
    }

    /// Show position (mm) to NED metres from the EKF origin.
    pub fn to_ned_from_origin(&self, c: &Context<'_>, p: [f32; 3]) -> Result<Vector3f> {
        let mut loc = self.to_global(c, p)?;
        let neu = loc.get_vector_from_origin_neu_m(c)?.ok_or(Error::Unavailable)?;
        Ok(Vector3f { x: neu.x, y: neu.y, z: -neu.z })
    }

    /// Show-frame velocity/acceleration (mm/s) to NED m/s.
    pub fn vec_to_ned(&self, v: [f32; 3]) -> Vector3f {
        let (n, e) = self.rotate_to_ne(v[0], v[1]);
        Vector3f { x: n, y: e, z: -v[2] / 1000.0 }
    }

    /// convert_global_to_show_coordinate()
    pub fn from_global(&self, c: &Context<'_>, loc: &Location) -> Result<[f32; 3]> {
        let mut origin = Location::default();
        origin.set_lat(c, self.origin_lat)?;
        origin.set_lng(c, self.origin_lng)?;
        if self.amsl_valid {
            origin.set_alt_m(c, self.origin_amsl_mm as f32 / 1000.0, ALT_ABSOLUTE)?;
        } else {
            origin.set_alt_m(c, 0.0, ALT_ABOVE_HOME)?;
        }
        let mut target = *loc;
        let d = origin.get_distance_ned(c, &mut target)?;
        let (x, y) = self.rotate_from_ne(d.x, d.y);
        Ok([x, y, -d.z * 1000.0])
    }
}
