// SPDX-License-Identifier: GPL-3.0-or-later
//! Fixed-capacity ELRS transport; ported from elrs_protocol.cpp.
#[derive(Clone, Copy, PartialEq)]
enum State {
    Idle,
    WaitAck,
    ResyncPending,
    Resync,
    Pending,
    Sending,
    Final,
}
pub struct Downlink {
    state: State,
    data: [u8; 64],
    length: u8,
    offset: u8,
    index: u8,
    sent: u8,
    resync_index: u8,
    wait: u16,
    max_wait: u16,
    expected: bool,
    last_ack: bool,
    ack_known: bool,
    needs_resync: bool,
    final_sent: bool,
}
impl Downlink {
    pub fn new() -> Self {
        Self {
            state: State::Idle,
            data: [0; 64],
            length: 0,
            offset: 0,
            index: 1,
            sent: 0,
            resync_index: 127,
            wait: 0,
            max_wait: 80,
            expected: true,
            last_ack: false,
            ack_known: false,
            needs_resync: true,
            final_sent: false,
        }
    }
    pub fn reset(&mut self, size: usize) {
        self.state = State::WaitAck;
        self.length = 0;
        self.offset = 0;
        self.sent = 0;
        self.index = 1;
        self.resync_index = if size == 13 { 31 } else { 127 };
        self.wait = 0;
        self.max_wait = 80;
        self.expected = true;
        self.last_ack = false;
        self.ack_known = false;
        self.final_sent = false;
        self.needs_resync = true;
    }
    pub fn ratio(&mut self, ratio: u8, burst: u8) {
        if burst > 0 {
            self.max_wait =
                (u32::from(ratio) * (1 + u32::from(burst)) / u32::from(burst) * 20) as u16;
        }
    }
    pub fn active(&self) -> bool {
        self.length != 0
    }
    pub fn ready(&self) -> bool {
        self.state != State::Idle
    }
    pub fn awaiting_ack(&self) -> bool {
        self.state == State::Sending || (self.state == State::Final && self.final_sent)
    }
    pub fn resyncing(&self) -> bool {
        matches!(
            self.state,
            State::WaitAck | State::ResyncPending | State::Resync
        )
    }
    pub fn resync(&mut self) {
        if !self.active() {
            return;
        }
        self.offset = 0;
        self.sent = 0;
        self.index = 1;
        self.expected = !self.last_ack;
        self.wait = 0;
        self.final_sent = false;
        self.state = if self.ack_known {
            State::ResyncPending
        } else {
            State::WaitAck
        };
    }
    pub fn queue(&mut self, data: &[u8]) -> bool {
        if self.active() || data.is_empty() || data.len() > 62 {
            return false;
        }
        self.data[0] = 0x10;
        self.data[1] = data.len() as u8;
        self.data[2..2 + data.len()].copy_from_slice(data);
        self.length = (data.len() + 2) as u8;
        self.offset = 0;
        self.sent = 0;
        self.index = 1;
        self.wait = 0;
        self.final_sent = false;
        if self.state == State::Idle {
            self.state = if !self.needs_resync {
                State::Pending
            } else if self.ack_known {
                State::ResyncPending
            } else {
                State::WaitAck
            };
        }
        true
    }
    pub fn payload(&mut self, out: &mut [u8]) -> u8 {
        if out.is_empty() {
            return 0;
        }
        match self.state {
            State::WaitAck => return self.resync_index,
            State::ResyncPending | State::Resync => {
                self.state = State::Resync;
                return self.resync_index;
            }
            State::Final => {
                self.final_sent = true;
                return 0;
            }
            State::Pending | State::Sending => {}
            _ => return 0,
        }
        if self.state == State::Pending {
            self.sent = (self.length - self.offset).min(out.len().min(255) as u8);
            self.state = State::Sending;
        }
        let count = usize::from(self.sent).min(out.len());
        out[..count].copy_from_slice(
            &self.data[usize::from(self.offset)..usize::from(self.offset) + count],
        );
        if self.index > 1 && self.offset + self.sent == self.length {
            0
        } else {
            self.index
        }
    }
    pub fn confirm(&mut self, ack: bool) -> bool {
        self.last_ack = ack;
        self.ack_known = true;
        if matches!(self.state, State::Idle | State::WaitAck) {
            if self.needs_resync {
                self.expected = !ack;
            }
            if self.state == State::WaitAck {
                self.state = State::ResyncPending;
            }
            return false;
        }
        if matches!(self.state, State::Pending | State::ResyncPending)
            || (self.state == State::Final && !self.final_sent)
        {
            return false;
        }
        if ack != self.expected {
            if self.state != State::Resync {
                self.wait = self.wait.wrapping_add(1);
                if self.wait > self.max_wait {
                    self.resync();
                }
            }
            return false;
        }
        self.expected = !ack;
        self.wait = 0;
        if self.state == State::Resync {
            self.needs_resync = false;
            self.state = if self.active() {
                State::Pending
            } else {
                State::Idle
            };
            return false;
        }
        if self.state == State::Sending {
            self.offset += self.sent;
            if self.offset < self.length {
                self.index = self.index.wrapping_add(1);
                self.state = State::Pending;
                return false;
            }
            if self.index == 1 {
                self.state = State::Final;
                self.final_sent = false;
                return false;
            }
        }
        self.state = State::Idle;
        self.length = 0;
        true
    }
}
pub struct Uplink {
    next: u8,
    pub ack: bool,
    data: [u8; 65],
    length: usize,
    complete: bool,
}
impl Uplink {
    pub fn new() -> Self {
        Self {
            next: 1,
            ack: false,
            data: [0; 65],
            length: 0,
            complete: false,
        }
    }
    pub fn reset(&mut self) {
        self.next = 1;
        self.ack = false;
        self.length = 0;
        self.complete = false;
    }
    pub fn receive_index(&mut self, index: u8) -> bool {
        if index == 14 {
            self.ack = !self.ack;
            self.next = 1;
            return false;
        }
        if index == 0 && self.next > 1 {
            self.ack = !self.ack;
            self.next = 1;
            return true;
        }
        if index == self.next || (index == 1 && self.next > 1) {
            self.next = index.wrapping_add(1);
            self.ack = !self.ack;
        }
        false
    }
    pub fn receive(&mut self, index: u8, data: &[u8]) -> bool {
        if index == 14 {
            self.receive_index(index);
            self.length = 0;
            self.complete = false;
            return false;
        }
        if self.complete || (data.len() != 5 && data.len() != 10) {
            return false;
        }
        let previous = self.ack;
        let completed = self.receive_index(index);
        if previous == self.ack {
            return false;
        }
        if index == 1 {
            self.length = 0;
        }
        let count = data.len().min(65 - self.length);
        self.data[self.length..self.length + count].copy_from_slice(&data[..count]);
        self.length += count;
        if completed {
            self.complete = self.length >= 2
                && self.data[0] == 0xfd
                && self.data[1] > 0
                && self.data[1] <= 62
                && usize::from(self.data[1]) <= self.length - 2;
            if !self.complete {
                self.length = 0;
            }
        }
        completed
    }
    pub fn pending(&self) -> bool {
        self.complete
    }
    pub fn read(&mut self, out: &mut [u8; 62]) -> u8 {
        if !self.complete {
            return 0;
        }
        let length = self.data[1];
        out[..usize::from(length)].copy_from_slice(&self.data[2..2 + usize::from(length)]);
        self.complete = false;
        self.length = 0;
        length
    }
}
