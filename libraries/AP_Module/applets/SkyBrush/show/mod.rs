// SPDX-License-Identifier: GPL-3.0-or-later
// Port of a subset of libskybrush (Copyright CollMot Robotics Ltd).
//! Skybrush show data: .skyb container, trajectories, statistics, landing
//! replacement, light programs and a show controller. Units are mm, mm/s and
//! seconds unless stated otherwise. No SDK dependency; no_std + alloc.

pub mod binary;
pub mod builder;
pub mod controller;
pub mod events;
pub mod fmath;
pub mod lights;
pub mod poly;
pub mod rth_plan;
pub mod stats;
pub mod time_axis;
pub mod trajectory;
pub mod yaw;

/// Error codes; discriminants match libskybrush `sb_error_t`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Error {
    NoMem = 1,
    Inval = 2,
    Read = 5,
    Parse = 8,
    Failure = 11,
    Unimplemented = 13,
    NoEnt = 18,
    Corrupted = 19,
    Overflow = 20,
}

pub type Result<T> = core::result::Result<T, Error>;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };

    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Vec3 { x, y, z }
    }

    pub fn scaled(self, k: f32) -> Self {
        Vec3::new(self.x * k, self.y * k, self.z * k)
    }
}
