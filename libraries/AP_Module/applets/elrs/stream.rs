// SPDX-License-Identifier: GPL-3.0-or-later
//! MAVLink framing and flow control. The host only moves bounded byte chunks.
use crate::mavlink_crc;
use crate::tracker::Tracker;
fn crc_byte(crc: u16, byte: u8) -> u16 {
    let mut tmp = byte ^ (crc as u8);
    tmp ^= tmp << 4;
    (crc >> 8) ^ ((tmp as u16) << 8) ^ ((tmp as u16) << 3) ^ ((tmp as u16) >> 4)
}
pub struct Stream {
    frame: [u8; 280],
    length: usize,
    expected: usize,
    ready: bool,
    offset: usize,
    rf: [u8; 62],
    rf_size: usize,
    rf_offset: usize,
    flow: [u8; 21],
    flow_offset: usize,
    sequence: u8,
    last_flow: u32,
    offered: usize,
    offer_flow: bool,
}
impl Stream {
    pub fn new() -> Self {
        Self {
            frame: [0; 280],
            length: 0,
            expected: 0,
            ready: false,
            offset: 0,
            rf: [0; 62],
            rf_size: 0,
            rf_offset: 0,
            flow: [0; 21],
            flow_offset: 21,
            sequence: 0,
            last_flow: 0,
            offered: 0,
            offer_flow: false,
        }
    }
    pub fn reset(&mut self) {
        self.length = 0;
        self.expected = 0;
        self.ready = false;
        self.offset = 0;
        self.rf_size = 0;
        self.rf_offset = 0;
        self.flow_offset = 21;
        self.offered = 0;
    }
    fn byte(&mut self, b: u8) {
        if self.length == 0 && b != 0xfe && b != 0xfd {
            return;
        }
        self.frame[self.length] = b;
        self.length += 1;
        let v2 = self.frame[0] == 0xfd;
        if self.length == 2 {
            self.expected = usize::from(b) + if v2 { 12 } else { 8 };
        }
        if v2 && self.length == 3 {
            if b & !1 != 0 {
                self.length = 0;
                return;
            }
            if b & 1 != 0 {
                self.expected += 13;
            }
        }
        if self.length < self.expected || self.length < 3 {
            return;
        }
        let header = if v2 { 10 } else { 6 };
        let end = header + usize::from(self.frame[1]);
        let id = if v2 {
            u32::from(self.frame[7])
                | u32::from(self.frame[8]) << 8
                | u32::from(self.frame[9]) << 16
        } else {
            u32::from(self.frame[5])
        };
        let mut crc = 0xffff;
        for &b in &self.frame[1..end] {
            crc = crc_byte(crc, b);
        }
        self.ready = mavlink_crc::extra(id).is_some_and(|extra| {
            crc = crc_byte(crc, extra);
            crc == u16::from_le_bytes([self.frame[end], self.frame[end + 1]])
        });
        if !self.ready {
            self.length = 0;
        }
        self.offset = 0;
    }
    pub fn update(
        &mut self,
        t: &mut Tracker,
        now: u32,
        ack: u8,
        space: u32,
        txspace: u32,
        active: bool,
        out: &mut [u8],
    ) {
        let ack = usize::from(ack).min(self.offered);
        if self.offer_flow {
            self.flow_offset += ack;
        } else {
            self.offset += ack;
        }
        self.offered = 0;
        if self.ready && self.offset == self.length {
            self.ready = false;
            self.length = 0;
            self.offset = 0;
        }
        // Never interleave RADIO_STATUS inside a partially delivered frame.
        if self.flow_offset == 21
            && self.offset == 0
            && now.wrapping_sub(self.last_flow) >= 10000
            && space >= 21
        {
            let credit = if !active || txspace <= 3584 {
                0
            } else {
                ((txspace - 3584) * 100 / 512).min(100) as u8
            };
            self.flow = [
                0xfd,
                9,
                0,
                0,
                self.sequence,
                255,
                68,
                109,
                0,
                0,
                0,
                0,
                0,
                0,
                255,
                0,
                credit,
                255,
                255,
                0,
                0,
            ];
            let mut crc = 0xffff;
            for &b in &self.flow[1..19] {
                crc = crc_byte(crc, b);
            }
            crc = crc_byte(crc, 185);
            self.flow[19..21].copy_from_slice(&crc.to_le_bytes());
            self.sequence = self.sequence.wrapping_add(1);
            self.flow_offset = 0;
            self.last_flow = now;
        }
        if active && !self.ready {
            if self.rf_offset == self.rf_size && t.uplink_pending() {
                self.rf_size = usize::from(t.read(&mut self.rf));
                self.rf_offset = 0;
            }
            while self.rf_offset < self.rf_size && !self.ready {
                let b = self.rf[self.rf_offset];
                self.rf_offset += 1;
                self.byte(b);
            }
        }
        let bytes = if self.flow_offset < 21 {
            self.offer_flow = true;
            &self.flow[self.flow_offset..]
        } else if self.ready {
            self.offer_flow = false;
            &self.frame[self.offset..self.length]
        } else {
            &[]
        };
        let n = bytes.len().min(62).min(space as usize);
        out[43] = n as u8;
        out[44..44 + n].copy_from_slice(&bytes[..n]);
        self.offered = n;
    }
}
