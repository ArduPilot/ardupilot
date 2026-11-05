// SPDX-License-Identifier: GPL-3.0-or-later
//! Event-driven ELRS receiver state. No HAL calls, allocation or shared globals.
use crate::protocol::*;
use crate::transport::{Downlink, Uplink};
#[derive(Clone, Copy)]
pub struct Rate {
    pub wire: u8,
    pub size: usize,
    pub sf: u8,
    pub hop: u8,
    pub interval: u32,
    pub timeout: u32,
    pub scan: u32,
    pub airtime: u16,
    pub coding: u8,
    pub preamble: u8,
    pub sends: u8,
    pub flrc: bool,
}
impl Rate {
    pub fn slack(self) -> u32 {
        (self.interval as i32 - 2 * i32::from(self.airtime)).max(200) as u32
    }
}
pub const RATES: [Rate; 10] = [
    Rate {
        wire: 21,
        size: 8,
        sf: 8,
        hop: 2,
        interval: 20000,
        timeout: 4000000,
        scan: 3520000,
        airtime: 10798,
        coding: 7,
        preamble: 12,
        sends: 1,
        flrc: false,
    },
    Rate {
        wire: 23,
        size: 13,
        sf: 7,
        hop: 4,
        interval: 10000,
        timeout: 3500000,
        scan: 3520000,
        airtime: 7605,
        coding: 7,
        preamble: 12,
        sends: 1,
        flrc: false,
    },
    Rate {
        wire: 24,
        size: 8,
        sf: 7,
        hop: 4,
        interval: 6666,
        timeout: 3500000,
        scan: 2346000,
        airtime: 5871,
        coding: 7,
        preamble: 12,
        sends: 1,
        flrc: false,
    },
    Rate {
        wire: 27,
        size: 8,
        sf: 6,
        hop: 4,
        interval: 4000,
        timeout: 3000000,
        scan: 1408000,
        airtime: 3300,
        coding: 7,
        preamble: 14,
        sends: 1,
        flrc: false,
    },
    Rate {
        wire: 28,
        size: 13,
        sf: 5,
        hop: 4,
        interval: 3003,
        timeout: 2500000,
        scan: 1057000,
        airtime: 2374,
        coding: 7,
        preamble: 12,
        sends: 1,
        flrc: false,
    },
    Rate {
        wire: 29,
        size: 8,
        sf: 5,
        hop: 4,
        interval: 2000,
        timeout: 2500000,
        scan: 704000,
        airtime: 1507,
        coding: 6,
        preamble: 12,
        sends: 1,
        flrc: false,
    },
    Rate {
        wire: 32,
        size: 8,
        sf: 0,
        hop: 2,
        interval: 2000,
        timeout: 2500000,
        scan: 352000,
        airtime: 389,
        coding: 0,
        preamble: 32,
        sends: 1,
        flrc: true,
    },
    Rate {
        wire: 33,
        size: 8,
        sf: 0,
        hop: 2,
        interval: 1000,
        timeout: 2500000,
        scan: 176000,
        airtime: 389,
        coding: 0,
        preamble: 32,
        sends: 1,
        flrc: true,
    },
    Rate {
        wire: 31,
        size: 8,
        sf: 0,
        hop: 2,
        interval: 1000,
        timeout: 2500000,
        scan: 176000,
        airtime: 389,
        coding: 0,
        preamble: 32,
        sends: 2,
        flrc: true,
    },
    Rate {
        wire: 30,
        size: 8,
        sf: 0,
        hop: 2,
        interval: 1000,
        timeout: 2500000,
        scan: 176000,
        airtime: 389,
        coding: 0,
        preamble: 32,
        sends: 4,
        flrc: true,
    },
];
pub struct Tracker {
    pub state: u8,
    pub rate_index: u8,
    pub nonce: u8,
    pub mode: u8,
    pub stats: [u32; 14],
    pub channels: Channels,
    pub seen: u16,
    pub frame_sequence: u32,
    pub frame_rc: u32,
    pub generation: u32,
    pub denom: u8,
    protocol: u8,
    downlink: Downlink,
    uplink: Uplink,
    last_stats: u32,
    stats_sent: bool,
    fragment_stats: bool,
    frame_seen: u16,
    pending_full: u16,
    frame_channels: Channels,
    frame_updated: [u32; 16],
    group_rc: bool,
    group_uplink: bool,
    group_good: bool,
    scan_started: u32,
    scanning: bool,
    fhss: Fhss,
    uid: [u8; 6],
    model: u8,
    init: u16,
    expired: u16,
    next_slot: u32,
    last_valid: u32,
    last_rc: u32,
    updated: [u32; 16],
    telemetry_pending: bool,
    lq_history: [bool; 100],
    lq_position: usize,
    lq_count: u8,
    lq_good: u8,
    received: bool,
}
impl Tracker {
    pub const TIMEOUT: u32 = 4000000;
    pub fn new() -> Self {
        Self {
            state: 0,
            rate_index: 0,
            nonce: 0,
            mode: 0,
            stats: [0; 14],
            channels: Channels::new(),
            seen: 0,
            frame_sequence: 0,
            frame_rc: 0,
            generation: 0,
            denom: 1,
            protocol: 0,
            downlink: Downlink::new(),
            uplink: Uplink::new(),
            last_stats: 0,
            stats_sent: false,
            fragment_stats: true,
            frame_seen: 0,
            pending_full: 0,
            frame_channels: Channels::new(),
            frame_updated: [0; 16],
            group_rc: false,
            group_uplink: false,
            group_good: false,
            scan_started: 0,
            scanning: false,
            fhss: Fhss::new(0),
            uid: [0; 6],
            model: 255,
            init: 0,
            expired: 0,
            next_slot: 0,
            last_valid: 0,
            last_rc: 0,
            updated: [0; 16],
            telemetry_pending: false,
            lq_history: [false; 100],
            lq_position: 0,
            lq_count: 0,
            lq_good: 0,
            received: false,
        }
    }
    fn inc(&mut self, index: usize) {
        self.stats[index] = self.stats[index].wrapping_add(1);
    }
    pub fn rate(&self) -> Rate {
        RATES[usize::from(self.rate_index)]
    }
    pub fn service_limit(&self) -> u32 {
        (self.rate().interval / 2).min(2000)
    }
    pub fn output_age(&self) -> u32 {
        2 * self.rate().interval * u32::from(self.rate().sends)
    }
    pub fn full16(&self) -> bool {
        self.rate().size == 13 && self.mode == 1
    }
    pub fn start(&mut self, uid: [u8; 6], model: u8, rate: u8) -> bool {
        self.stop();
        self.rate_index = 0;
        self.stats = [0; 14];
        if !uid_is_bound(&uid) || usize::from(rate) >= RATES.len() {
            return false;
        }
        self.rate_index = rate;
        self.uid = uid;
        self.model = model;
        self.init = crc_initializer(&uid);
        self.fhss = Fhss::new(uid_seed(&uid));
        self.state = 1;
        true
    }
    pub fn stop(&mut self) {
        self.protocol = 0;
        self.reset_transport();
        self.state = 0;
        self.scanning = false;
        self.frame_seen = 0;
        self.pending_full = 0;
        self.group_rc = false;
        self.group_uplink = false;
        self.group_good = false;
        self.channels = Channels::new();
        self.uplink.reset();
        self.seen = 0;
        self.expired = 0;
        self.received = false;
        self.nonce = 0;
        self.denom = 1;
        self.telemetry_pending = false;
        self.lq_history.fill(false);
        self.lq_position = 0;
        self.lq_count = 0;
        self.lq_good = 0;
        self.fhss.set_index(0);
    }
    pub fn lose(&mut self) {
        if self.state == 0 {
            return;
        }
        if self.state == 2 {
            self.inc(6);
        }
        self.stop();
        self.state = 1;
    }
    pub fn last_input(&self) -> u32 {
        if !self.full16() {
            return self.frame_rc;
        }
        if self.frame_rc.wrapping_sub(self.frame_updated[0])
            > self.frame_rc.wrapping_sub(self.frame_updated[8])
        {
            self.frame_updated[0]
        } else {
            self.frame_updated[8]
        }
    }
    pub fn read_primary(&self, now: u32, out: &mut [u16; 4]) -> bool {
        if self.state != 2
            || self.frame_seen & 15 != 15
            || now.wrapping_sub(self.frame_rc) > self.output_age()
            || (self.full16() && now.wrapping_sub(self.last_input()) > Self::TIMEOUT)
        {
            return false;
        }
        for i in 0..4 {
            out[i] = (u32::from(self.frame_channels.crsf[i]) * 5 / 8 + 880) as u16;
        }
        true
    }
    pub fn read_channels(&self, now: u32, out: &mut [u16; 16]) -> (u8, u32) {
        let count = if self.rate().size == 8 {
            12
        } else if self.mode == 1 {
            16
        } else if self.seen & 0xf00 != 0 {
            12
        } else {
            8
        };
        let required = ((1u32 << count) - 1) as u16;
        let age = now.wrapping_sub(self.frame_rc);
        if self.state != 2 || self.frame_seen & required != required || age > self.output_age() {
            return (0, 0);
        }
        let mut remaining = self.output_age() - age;
        for i in 0..count {
            let age = now.wrapping_sub(self.frame_updated[i]);
            if age > Self::TIMEOUT {
                return (0, 0);
            }
            remaining = remaining.min(Self::TIMEOUT - age);
        }
        for i in 0..count {
            out[i] = (u32::from(self.frame_channels.crsf[i]) * 5 / 8 + 880) as u16;
        }
        (count as u8, remaining)
    }
    fn publish(&mut self) {
        if self.full16() {
            if self.pending_full != 0xffff {
                return;
            }
            self.pending_full = 0;
        }
        self.frame_sequence = self.frame_sequence.wrapping_add(1);
        self.frame_channels = self.channels;
        self.frame_seen = self.seen & !self.expired;
        self.frame_rc = self.last_rc;
        self.frame_updated = self.updated;
    }
    fn expire(&mut self, now: u32) {
        for i in 0..16 {
            if self.seen & (1 << i) != 0 && now.wrapping_sub(self.updated[i]) > Self::TIMEOUT {
                self.expired |= 1 << i;
            }
        }
        self.frame_seen &= !self.expired;
        if !self.full16() {
            return;
        }
        if now.wrapping_sub(self.updated[0]) > Self::TIMEOUT {
            self.pending_full &= 0xff00;
        }
        if now.wrapping_sub(self.updated[8]) > Self::TIMEOUT {
            self.pending_full &= 0x00ff;
        }
        if now.wrapping_sub(self.frame_rc) > self.output_age() {
            self.frame_seen = 0;
        }
    }
    fn record_lq(&mut self, good: bool) {
        self.lq_good -= u8::from(self.lq_history[self.lq_position]);
        self.lq_history[self.lq_position] = good;
        self.lq_good += u8::from(good);
        self.lq_position = (self.lq_position + 1) % 100;
        if self.lq_count < 100 {
            self.lq_count += 1;
        }
    }
    pub fn lq(&self) -> u8 {
        if self.lq_count == 0 {
            0
        } else {
            (u16::from(self.lq_good) * 100 / u16::from(self.lq_count)) as u8
        }
    }
    pub fn frequency(&self) -> u32 {
        if self.state == 2 {
            self.fhss.frequency()
        } else {
            Fhss::channel_frequency(40)
        }
    }
    pub fn hop_index(&self) -> u8 {
        self.fhss.index()
    }
    pub fn wait(&self, now: u32) -> u32 {
        if self.state != 2 {
            return self.rate().interval;
        }
        (self.next_slot.wrapping_sub(now) as i32).max(0) as u32
    }
    fn telemetry_slot(&self) -> bool {
        self.denom > 1 && self.nonce % self.denom == 0
    }
    pub fn advance(&mut self, now: u32) {
        if self.state == 1 {
            if !self.scanning {
                self.scanning = true;
                self.scan_started = now;
            } else if now.wrapping_sub(self.scan_started) >= self.rate().scan {
                self.rate_index = (self.rate_index + 1) % 10;
                self.scan_started = now;
            }
            return;
        }
        if self.state != 2 {
            return;
        }
        self.expire(now);
        if now.wrapping_sub(self.last_valid) > self.rate().timeout {
            self.lose();
            return;
        }
        if (now.wrapping_sub(self.next_slot) as i32) < 0 {
            return;
        }
        let late = now.wrapping_sub(self.next_slot);
        self.stats[8] = self.stats[8].max(late);
        if late > self.service_limit() {
            self.inc(5);
            self.lose();
            return;
        }
        self.group_uplink |= !self.telemetry_slot();
        self.group_good |= self.received && !self.telemetry_slot();
        if self.rate().sends == 1 || self.nonce % self.rate().sends == 0 {
            if self.group_uplink {
                self.record_lq(self.group_good);
            }
            if self.rate().sends > 1 && self.group_rc {
                self.publish();
            }
            self.group_rc = false;
            self.group_uplink = false;
            self.group_good = false;
        }
        self.next_slot = self.next_slot.wrapping_add(self.rate().interval);
        self.nonce = self.nonce.wrapping_add(1);
        if self.nonce % self.rate().hop == 0 {
            self.fhss.next_frequency();
        }
        self.received = false;
        self.telemetry_pending = self.telemetry_slot();
    }
    fn reset_transport(&mut self) {
        self.downlink.reset(self.rate().size);
        self.uplink.reset();
        self.generation = self.generation.wrapping_add(1);
        self.stats_sent = false;
        self.fragment_stats = true;
    }
    pub fn mavlink_mode(&self) -> bool {
        self.state == 2 && self.protocol == 1
    }
    pub fn telemetry_valid_for(&self, now: u32) -> u16 {
        self.service_limit()
            .saturating_sub(now.wrapping_sub(self.next_slot.wrapping_sub(self.rate().interval)))
            .min(
                self.output_age()
                    .saturating_sub(now.wrapping_sub(self.last_valid)),
            ) as u16
    }
    pub fn telemetry_pending(&self) -> bool {
        self.telemetry_pending
    }
    pub fn uplink_pending(&self) -> bool {
        self.mavlink_mode() && self.denom > 1 && self.uplink.pending()
    }
    pub fn downlink_pending(&self) -> bool {
        self.downlink.active()
    }
    pub fn queue(&mut self, data: &[u8]) -> bool {
        self.mavlink_mode() && self.denom > 1 && self.downlink.queue(data)
    }
    pub fn read(&mut self, out: &mut [u8; 62]) -> u8 {
        if self.mavlink_mode() && self.denom > 1 {
            self.uplink.read(out)
        } else {
            0
        }
    }
    fn confirm(&mut self, ack: bool) {
        if !self.mavlink_mode() && !self.downlink.resyncing() {
            return;
        }
        let waiting = self.downlink.awaiting_ack();
        if self.downlink.confirm(ack) {
            self.inc(10);
        }
        if waiting && self.downlink.resyncing() {
            self.inc(13);
        }
    }
    pub fn telemetry(&mut self, now: u32, packet: &mut [u8; 13], rssi: i8, snr: i8) -> bool {
        if !self.telemetry_pending {
            return false;
        }
        self.telemetry_pending = false;
        if self.state != 2
            || !self.telemetry_slot()
            || now.wrapping_sub(self.next_slot.wrapping_sub(self.rate().interval))
                > self.service_limit()
            || now.wrapping_sub(self.last_valid) > self.output_age()
        {
            return false;
        }
        let size = self.rate().size;
        if self.downlink.ready() && (self.mavlink_mode() || self.downlink.resyncing()) {
            if self.downlink.awaiting_ack()
                && !self.fragment_stats
                && self.stats_sent
                && now.wrapping_sub(self.last_stats) >= 512000
            {
                self.downlink.resync();
                self.inc(13);
            }
            let retry = self.downlink.awaiting_ack();
            if !retry {
                self.fragment_stats = !self.stats_sent
                    || now.wrapping_sub(self.last_stats) >= 512000
                    || self.downlink.resyncing();
            }
            let stats = self.fragment_stats;
            let capacity = (if size == 13 { 10 } else { 5 }) - if stats { 4 } else { 0 };
            let mut payload = [0; 10];
            let index = self.downlink.payload(&mut payload[..capacity]);
            make_downlink(
                &mut packet[..size],
                self.init,
                self.nonce,
                stats,
                index,
                &payload,
                self.uplink.ack,
                rssi,
                snr,
                self.lq(),
            );
            self.inc(9);
            if retry {
                self.inc(12);
            }
            if self.downlink.resyncing() {
                self.inc(11);
            }
            if stats {
                self.last_stats = now;
                self.stats_sent = true;
            }
            return true;
        }
        self.last_stats = now;
        self.stats_sent = true;
        make_link_stats(
            &mut packet[..size],
            self.init,
            self.nonce,
            rssi,
            snr,
            self.lq(),
            self.uplink.ack,
        );
        true
    }
    fn clear_observations(&mut self) {
        self.channels = Channels::new();
        self.seen = 0;
        self.expired = 0;
        self.frame_seen = 0;
        self.pending_full = 0;
        self.group_rc = false;
        self.group_uplink = false;
        self.group_good = false;
    }
    pub fn receive(&mut self, packet: &[u8], timestamp: u32, now: u32) {
        if self.state == 0 {
            return;
        }
        let service = now.wrapping_sub(timestamp);
        self.stats[7] = self.stats[7].max(service);
        if service > self.service_limit() {
            self.inc(5);
            self.lose();
            return;
        }
        if packet.len() != self.rate().size || !validate_packet(packet, self.init, self.nonce) {
            self.inc(4);
            return;
        }
        let kind = packet[0] & 3;
        if kind == 2 {
            let sync = match parse_sync(packet) {
                Some(s) => s,
                None => {
                    self.inc(4);
                    return;
                }
            };
            if sync.gemini
                || sync.protocol > 1
                || sync.uid4 != self.uid[4]
                || sync.uid5 != (self.uid[5] ^ (!self.model & 0x3f))
                || sync.fhss_index >= 240
            {
                self.inc(4);
                self.lose();
                return;
            }
            let proposed = match RATES.iter().position(|r| r.wire == sync.rate) {
                Some(i) => i as u8,
                None => {
                    self.inc(4);
                    self.lose();
                    return;
                }
            };
            let changed = proposed != self.rate_index;
            if changed {
                let same_flrc = self.rate().flrc && RATES[usize::from(proposed)].flrc;
                if !same_flrc {
                    self.lose();
                    self.rate_index = proposed;
                    return;
                }
                self.rate_index = proposed;
                self.clear_observations();
            }
            let mut fhss = self.fhss.clone();
            fhss.set_index(sync.fhss_index);
            if fhss.frequency() != self.frequency() {
                self.inc(4);
                self.lose();
                return;
            }
            let observations = self.state != 2
                || self.nonce != sync.nonce
                || self.fhss.index() != sync.fhss_index
                || self.mode != sync.switch_mode
                || self.protocol != sync.protocol;
            if observations {
                self.clear_observations();
            } else if self.received {
                self.inc(4);
                return;
            }
            let telemetry_changed =
                (self.denom > 1) != (telemetry_denominator(sync.telemetry_ratio) > 1);
            if observations || changed || telemetry_changed {
                self.reset_transport();
            }
            self.fhss.set_index(sync.fhss_index);
            if self.protocol != sync.protocol {
                self.uplink.reset();
            }
            self.protocol = sync.protocol;
            self.nonce = sync.nonce;
            self.mode = sync.switch_mode;
            self.denom = telemetry_denominator(sync.telemetry_ratio);
            let opportunities =
                512 * (1000000 / self.rate().interval) / u32::from(self.denom) / 1000;
            let burst = if opportunities > 256 {
                255
            } else if opportunities > 1 {
                (opportunities - 1) as u8
            } else {
                1
            };
            self.downlink.ratio(self.denom, burst);
            self.telemetry_pending = false;
            self.state = 2;
            self.inc(0);
        } else {
            if self.state != 2
                || self.telemetry_slot()
                || self.received
                || now.wrapping_sub(self.last_valid) > self.rate().timeout
            {
                self.inc(4);
                return;
            }
            let phase =
                timestamp.wrapping_sub(self.next_slot.wrapping_sub(self.rate().slack())) as i32;
            if phase < -(self.service_limit() as i32) || phase > self.service_limit() as i32 {
                self.inc(5);
                self.lose();
                return;
            }
            if kind == 1 {
                if self.mavlink_mode() {
                    self.confirm(if self.rate().size == 13 {
                        packet[0] & 4 != 0
                    } else {
                        packet[1] & 0x80 != 0
                    });
                }
                self.inc(2);
                let full = self.rate().size == 13;
                let index = if full {
                    packet[0] >> 3
                } else {
                    packet[1] & 0x7f
                };
                let completed = if self.mavlink_mode() {
                    self.uplink
                        .receive(index, &packet[if full { 1..11 } else { 2..7 }])
                } else {
                    self.uplink.receive_index(index)
                };
                if completed {
                    self.inc(3);
                }
            }
            if kind == 0 {
                self.expire(now);
                if !decode_channels(packet, self.init, self.nonce, self.mode, &mut self.channels) {
                    self.inc(4);
                    return;
                }
                self.confirm(self.channels.telemetry_ack);
                self.seen |= self.channels.updated_mask;
                self.expired &= !self.channels.updated_mask;
                if self.full16() {
                    self.pending_full |= self.channels.updated_mask;
                }
                self.inc(1);
                self.last_rc = timestamp;
                for i in 0..16 {
                    if self.channels.updated_mask & (1 << i) != 0 {
                        self.updated[i] = timestamp;
                    }
                }
                if self.rate().sends == 1 {
                    self.publish();
                } else {
                    self.group_rc = true;
                }
            }
        }
        self.received = true;
        self.last_valid = timestamp;
        self.next_slot = timestamp.wrapping_add(self.rate().slack());
    }
}
