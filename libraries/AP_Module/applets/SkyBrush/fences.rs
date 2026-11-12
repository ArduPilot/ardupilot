// SPDX-License-Identifier: GPL-3.0-or-later
//! Hard fence and bubble fence state machines, as AC_HardFence / AC_BubbleFence.

use crate::show::fmath::hypotf;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub enum Breach {
    #[default]
    None = 0,
    Soft = 1,
    Hard = 2,
}

pub struct HardFenceParams {
    pub enabled: bool,
    pub distance: f32,
    pub timeout: f32,
}

/// Messages the caller should send after an update.
#[derive(PartialEq, Eq)]
pub enum HardMsg {
    None,
    Breached,
    CountdownStarted,
    Resolved,
}

#[derive(Default)]
pub struct HardFence {
    pub state: Breach,
    hard_started_ms: u32,
    was_enabled: bool,
}

impl HardFence {
    /// `distance` is the distance from the breach point (f32::MAX without position);
    /// `breach_point_needed` is set when the caller must record the current location.
    pub fn update(
        &mut self,
        p: &HardFenceParams,
        breaches: u8,
        now_ms: u32,
        distance: f32,
        breach_point_needed: &mut bool,
        msg: &mut HardMsg,
    ) -> bool {
        *msg = HardMsg::None;
        if p.enabled && !self.was_enabled {
            if self.state >= Breach::Hard {
                self.hard_started_ms = now_ms;
            }
            if self.state >= Breach::Soft {
                *msg = self.state_msg(p, Breach::None);
            }
        }
        self.was_enabled = p.enabled;
        let next = match self.state {
            Breach::None if breaches != 0 => Breach::Soft,
            Breach::Soft if breaches == 0 => Breach::None,
            Breach::Soft if p.distance >= 0.0 && distance > p.distance => Breach::Hard,
            Breach::Hard if breaches == 0 => Breach::None,
            Breach::Hard if p.distance < 0.0 || distance <= p.distance => Breach::Soft,
            s => s,
        };
        if next != self.state {
            let old = self.state;
            self.state = next;
            if p.enabled {
                *msg = self.state_msg(p, old);
            }
            if next == Breach::None {
                self.hard_started_ms = 0;
            } else {
                if old == Breach::None {
                    *breach_point_needed = true;
                }
                self.hard_started_ms = if next == Breach::Hard { now_ms } else { 0 };
            }
        }
        self.should_disarm(p, now_ms)
    }

    fn state_msg(&self, p: &HardFenceParams, old: Breach) -> HardMsg {
        if self.state == Breach::Hard {
            if p.timeout > 0.0 {
                HardMsg::CountdownStarted
            } else {
                HardMsg::Breached
            }
        } else if old == Breach::Hard {
            HardMsg::Resolved
        } else {
            HardMsg::None
        }
    }

    fn should_disarm(&self, p: &HardFenceParams, now_ms: u32) -> bool {
        if self.state == Breach::None || !p.enabled || !(p.distance >= 0.0) {
            return false;
        }
        let since = if self.hard_started_ms > 0 {
            now_ms.wrapping_sub(self.hard_started_ms) as f32 / 1000.0
        } else {
            0.0
        };
        self.hard_started_ms > 0 && since >= p.timeout
    }
}

pub struct BubbleParams {
    pub enabled: bool,
    pub dxy: f32,
    pub dz: f32,
    pub timeout: f32,
    pub action: u8,
}

pub const BUBBLE_NONE: u8 = 0;
pub const BUBBLE_REPORT: u8 = 1;
pub const BUBBLE_FLASH: u8 = 2;
pub const BUBBLE_RTL: u8 = 3;
pub const BUBBLE_LAND: u8 = 4;
pub const BUBBLE_DISARM: u8 = 5;

#[derive(Default)]
pub struct BubbleFence {
    pub state: Breach,
    started_ms: u32,
    last_action_ms: u32,
    pub action_counter: u16,
}

impl BubbleFence {
    /// Returns the action to perform now (BUBBLE_NONE most of the time).
    pub fn update(&mut self, p: &BubbleParams, dist: [f32; 3], now_ms: u32) -> u8 {
        let too_far = p.enabled
            && ((p.dxy > 0.0 && hypotf(dist[0], dist[1]) > p.dxy)
                || (p.dz > 0.0 && dist[2].abs() > p.dz));
        if too_far {
            if self.state == Breach::None {
                self.state = Breach::Soft;
                self.started_ms = now_ms;
                self.last_action_ms = 0;
            }
        } else if self.state != Breach::None {
            self.state = Breach::None;
            self.started_ms = 0;
            self.last_action_ms = 0;
        }
        if self.state == Breach::Soft
            && now_ms.wrapping_sub(self.started_ms) as f32 > p.timeout * 1000.0
        {
            self.state = Breach::Hard;
            self.last_action_ms = 0;
        }
        if self.state == Breach::Hard
            && (self.last_action_ms == 0 || now_ms.wrapping_sub(self.last_action_ms) > 1000)
        {
            self.last_action_ms = now_ms;
            self.action_counter = self.action_counter.wrapping_add(1);
            return p.action;
        }
        BUBBLE_NONE
    }
}

/// One text per 5 s, as the fork's send_breach_notification_if_needed().
#[derive(Default)]
pub struct RateLimit {
    last_ms: Option<u32>,
}

impl RateLimit {
    pub fn allow(&mut self, now_ms: u32) -> bool {
        match self.last_ms {
            Some(t) if now_ms.wrapping_sub(t) <= 5000 => false,
            _ => {
                self.last_ms = Some(now_ms);
                true
            }
        }
    }
}
