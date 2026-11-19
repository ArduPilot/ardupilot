// SPDX-License-Identifier: GPL-3.0-or-later
// Port of libskybrush events/events.c (Copyright CollMot Robotics Ltd).
//! Event list block (type 6): 10-byte entries {time u32 ms, type u8,
//! subtype u8, payload [4]}, sorted by time; trailing bytes are ignored.

use alloc::vec::Vec;

use super::binary::{read_u32, ShowFile, BLOCK_EVENT_LIST};
use super::{Error, Result};

pub const EVENT_TYPE_NONE: u8 = 0;
pub const EVENT_TYPE_PYRO: u8 = 1;
pub const EVENT_TYPE_LIGHT: u8 = 2;

pub const LIGHT_SET_SOURCE: u8 = 0;
pub const LIGHT_SET_PIXEL_X: u8 = 1;
pub const LIGHT_SET_PIXEL_Y: u8 = 2;

/// Pyro payload meaning "off".
pub const PYRO_OFF: u32 = u32::MAX;

const SIZE_OF_EVENT: usize = 10;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Event {
    pub time_msec: u32,
    pub kind: u8,
    pub subtype: u8,
    pub payload: [u8; 4],
}

impl Event {
    pub fn payload_u32(&self) -> u32 {
        u32::from_le_bytes(self.payload)
    }

    pub fn is_pyro_on(&self) -> bool {
        self.kind == EVENT_TYPE_PYRO && self.payload_u32() != PYRO_OFF
    }

    pub fn is_pyro_off(&self) -> bool {
        self.kind == EVENT_TYPE_PYRO && self.payload_u32() == PYRO_OFF
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct EventList {
    pub events: Vec<Event>,
}

impl EventList {
    pub fn from_bytes(buf: &[u8]) -> Result<Self> {
        let mut events = Vec::with_capacity(buf.len() / SIZE_OF_EVENT);
        let mut last = 0;
        for chunk in buf.chunks_exact(SIZE_OF_EVENT) {
            let mut off = 0;
            let time_msec = read_u32(chunk, &mut off);
            if time_msec < last {
                return Err(Error::Corrupted);
            }
            last = time_msec;
            events.push(Event {
                time_msec,
                kind: chunk[4],
                subtype: chunk[5],
                payload: [chunk[6], chunk[7], chunk[8], chunk[9]],
            });
        }
        Ok(EventList { events })
    }

    /// Missing block is `Err(NoEnt)`, which callers treat as no events.
    pub fn from_show(file: &ShowFile) -> Result<Self> {
        Self::from_bytes(file.find_first(BLOCK_EVENT_LIST)?)
    }

    pub fn len(&self) -> usize {
        self.events.len()
    }

    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }

    /// Inserts after any events with the same timestamp.
    pub fn insert(&mut self, e: Event) {
        let pos = self.events.partition_point(|x| x.time_msec <= e.time_msec);
        self.events.insert(pos, e);
    }

    /// Shifts events of one type, clamping to [0, u32::MAX], then re-sorts (stable).
    pub fn adjust_timestamps_by_type(&mut self, kind: u8, delta_msec: i32) {
        for e in self.events.iter_mut().filter(|e| e.kind == kind) {
            e.time_msec = if delta_msec >= 0 {
                e.time_msec.saturating_add(delta_msec as u32)
            } else {
                e.time_msec.saturating_sub(delta_msec.unsigned_abs())
            };
        }
        self.events.sort_by_key(|e| e.time_msec);
    }

    /// Ensures every pyro "on" event is followed by an "off" on the same
    /// channel within `time_msec`.
    pub fn add_pyro_off_events(&mut self, time_msec: u32) {
        let mut i = 0;
        while i < self.events.len() {
            let ev = self.events[i];
            if ev.is_pyro_on() {
                let off_time = ev.time_msec.saturating_add(time_msec);
                let found = (i + 1..self.events.len())
                    .find(|&j| self.events[j].is_pyro_off() && self.events[j].subtype == ev.subtype);
                let mut have_off = found.is_some();
                if let Some(j) = found {
                    if self.events[j].time_msec > off_time {
                        self.events.remove(j);
                        have_off = false;
                    }
                }
                if !have_off {
                    self.insert(Event {
                        time_msec: off_time,
                        kind: EVENT_TYPE_PYRO,
                        subtype: ev.subtype,
                        payload: PYRO_OFF.to_le_bytes(),
                    });
                }
            }
            i += 1;
        }
    }
}

fn to_msec(t: f32) -> u32 {
    if t > (u32::MAX / 1000) as f32 {
        u32::MAX
    } else {
        (t * 1000.0) as u32
    }
}

/// Hands out events in order as show time advances.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EventPlayer {
    pub current_index: usize,
}

impl EventPlayer {
    pub fn peek<'a>(&self, list: &'a EventList) -> Option<&'a Event> {
        list.events.get(self.current_index)
    }

    /// Next event due at or before `t` seconds, if any.
    pub fn next_event_not_later_than<'a>(&mut self, list: &'a EventList, t: f32) -> Option<&'a Event> {
        if t.is_nan() || t < 0.0 {
            return None;
        }
        let t_msec = to_msec(t);
        let e = list.events.get(self.current_index).filter(|e| e.time_msec <= t_msec)?;
        self.current_index += 1;
        Some(e)
    }

    /// Positions the player so that events before `t` seconds are skipped.
    pub fn seek(&mut self, list: &EventList, t: f32) {
        self.current_index = 0;
        if !t.is_nan() && t > 0.0 {
            let t_msec = to_msec(t);
            while list.events.get(self.current_index).is_some_and(|e| e.time_msec < t_msec) {
                self.current_index += 1;
            }
        }
    }
}
