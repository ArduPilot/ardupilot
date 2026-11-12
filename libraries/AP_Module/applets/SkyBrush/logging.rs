// SPDX-License-Identifier: GPL-3.0-or-later
//! DataFlash messages with the fork's names, labels and encodings.

use ap_rust_sdk::{Context, Error, LogValue as L, Result};

fn write(c: &Context<'_>, name: &str, labels: &str, values: &[L<'_>]) -> Result {
    match c.logger().write(name, labels, None, values) {
        // compiled-out logging must not stop the show
        Err(Error::Unavailable) => Ok(()),
        r => r,
    }
}

pub struct ShowRow {
    pub clock_ms: i32,
    pub stage: u8,
    pub scene: u8,
    pub scene_ms: i32,
    pub pos_m: [f32; 3],
    pub rgb: [u8; 3],
    pub hdist: f32,
    pub vdist: f32,
}

/// SHOW; TimeUS is added by the logger at write time.
pub fn show(c: &Context<'_>, r: &ShowRow) -> Result {
    write(
        c,
        "SHOW",
        "ClockMS,Stage,Scene,SceneMS,X,Y,Z,R,G,B,HDist,VDist",
        &[
            L::I32(r.clock_ms),
            L::U8(r.stage),
            L::U8(r.scene),
            L::I32(r.scene_ms),
            L::Float(r.pos_m[0]),
            L::Float(r.pos_m[1]),
            L::Float(r.pos_m[2]),
            L::U8(r.rgb[0]),
            L::U8(r.rgb[1]),
            L::U8(r.rgb[2]),
            L::Float(r.hdist),
            L::Float(r.vdist),
        ],
    )
}

pub struct FenceRow {
    pub geo_enabled: u8,
    pub geo_breaches: u8,
    pub geo_count: u16,
    pub hard: u8,
    pub bubble: u8,
    pub bubble_count: u16,
}

pub fn fence(c: &Context<'_>, r: &FenceRow) -> Result {
    write(
        c,
        "FNCS",
        "GeoEn,GeoB,GeoCnt,HardB,BubbleB,BubbleCnt",
        &[
            L::U8(r.geo_enabled),
            L::U8(r.geo_breaches),
            L::U16(r.geo_count),
            L::U8(r.hard),
            L::U8(r.bubble),
            L::U16(r.bubble_count),
        ],
    )
}

/// SBEV: one row per executed show event.
pub fn event(c: &Context<'_>, clock_ms: i32, scene: u8, scene_ms: i32, kind: u8, subtype: u8, payload: u32, result: u8) -> Result {
    write(
        c,
        "SBEV",
        "ClockMS,Scene,SceneMS,Type,Subtype,Payload,Result",
        &[
            L::I32(clock_ms),
            L::U8(scene),
            L::I32(scene_ms),
            L::U8(kind),
            L::U8(subtype),
            L::U32(payload),
            L::U8(result),
        ],
    )
}

/// SBSP: one row per time axis segment; Origin only on a scene's first row.
pub fn screenplay(c: &Context<'_>, seq: u8, sp: &crate::show::controller::Screenplay) -> Result {
    for (i, scene) in sp.scenes.iter().enumerate().take(256) {
        let mut origin = scene.time_axis.origin_msec() as u64;
        for (j, seg) in scene.time_axis.segments().iter().enumerate().take(256) {
            write(
                c,
                "SBSP",
                "Seq,Scene,Index,Origin,Duration,IR,FR",
                &[
                    L::U8(seq),
                    L::U8(i as u8),
                    L::U8(j as u8),
                    L::U64(origin),
                    L::U32(seg.duration_msec),
                    L::Float(seg.initial_rate),
                    L::Float(seg.final_rate),
                ],
            )?;
            origin = 0;
        }
    }
    Ok(())
}

/// CRTH: collective RTH scene created; start in show frame metres.
pub fn crth(c: &Context<'_>, clock_ms: i32, rth_start_sec: f32, start_m: [f32; 3]) -> Result {
    write(
        c,
        "CRTH",
        "ClockMS,StartMS,StartX,StartY,StartZ",
        &[
            L::I32(clock_ms),
            L::U32((rth_start_sec * 1000.0) as u32),
            L::Float(start_m[0]),
            L::Float(start_m[1]),
            L::Float(start_m[2]),
        ],
    )
}
