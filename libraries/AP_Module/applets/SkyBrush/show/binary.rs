// SPDX-License-Identifier: GPL-3.0-or-later
// Port of libskybrush formats/binary.c, crc32.c, parsing.c (Copyright CollMot Robotics Ltd).
//! .skyb container: "skyb", version (1|2), features (v2), optional CRC32,
//! then blocks of `type u8, length u16 LE, body`.

use super::{Error, Result};

pub const BLOCK_TRAJECTORY: u8 = 1;
pub const BLOCK_LIGHT_PROGRAM: u8 = 2;
pub const BLOCK_COMMENT: u8 = 3;
pub const BLOCK_RTH_PLAN: u8 = 4;
pub const BLOCK_YAW_CONTROL: u8 = 5;
pub const BLOCK_EVENT_LIST: u8 = 6;
pub const BLOCK_LIGHT_CONTROL: u8 = 7;
pub const BLOCK_METADATA: u8 = 8;

pub const FEATURE_CRC32: u8 = 1;

const CRC_TABLE: [u32; 256] = {
    let mut t = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
            k += 1;
        }
        t[i] = c;
        i += 1;
    }
    t
};

/// ArduPilot crc_crc32 / MAVFTP CRC (reflected 0x04C11DB7, init 0, no xorout).
pub fn crc32_update(mut crc: u32, buf: &[u8]) -> u32 {
    for &b in buf {
        crc = CRC_TABLE[((crc ^ b as u32) & 0xff) as usize] ^ (crc >> 8);
    }
    crc
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Block<'a> {
    pub kind: u8,
    /// Offset of the body from the start of the file.
    pub offset: usize,
    pub body: &'a [u8],
}

#[derive(Clone, Copy, Debug)]
pub struct ShowFile<'a> {
    data: &'a [u8],
    pub version: u8,
    pub features: u8,
    first_block: usize,
}

impl<'a> ShowFile<'a> {
    /// Parses the header and validates the CRC32 if present.
    pub fn parse(data: &'a [u8]) -> Result<Self> {
        if data.len() < 5 || &data[..4] != b"skyb" {
            return Err(Error::Parse);
        }
        let version = data[4];
        if version != 1 && version != 2 {
            return Err(Error::Parse);
        }
        let mut pos = 5;
        let mut features = 0;
        if version == 2 {
            features = *data.get(5).ok_or(Error::Parse)?;
            pos = 6;
        }
        if features & FEATURE_CRC32 != 0 {
            let crc = data.get(6..10).ok_or(Error::Parse)?;
            let expected = u32::from_le_bytes([crc[0], crc[1], crc[2], crc[3]]);
            let mut c = crc32_update(0, &data[..6]);
            c = crc32_update(c, &[0; 4]);
            c = crc32_update(c, &data[10..]);
            if c != expected {
                return Err(Error::Corrupted);
            }
            pos = 10;
        }
        Ok(ShowFile { data, version, features, first_block: pos })
    }

    pub fn data(&self) -> &'a [u8] {
        self.data
    }

    /// Iterates over the blocks; a truncated block header or body yields
    /// `Err(Error::Read)` and ends the iteration.
    pub fn blocks(&self) -> Blocks<'a> {
        Blocks { data: self.data, pos: self.first_block }
    }

    /// Body of the first block of the given type.
    pub fn find_first(&self, kind: u8) -> Result<&'a [u8]> {
        for b in self.blocks() {
            let b = b?;
            if b.kind == kind {
                return Ok(b.body);
            }
        }
        Err(Error::NoEnt)
    }
}

pub struct Blocks<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Iterator for Blocks<'a> {
    type Item = Result<Block<'a>>;

    fn next(&mut self) -> Option<Self::Item> {
        let rest = self.data.get(self.pos..)?;
        if rest.is_empty() {
            return None;
        }
        self.pos = usize::MAX;
        if rest.len() < 3 {
            return Some(Err(Error::Read));
        }
        let len = u16::from_le_bytes([rest[1], rest[2]]) as usize;
        let offset = self.data.len() - rest.len() + 3;
        let body = match rest.get(3..3 + len) {
            Some(b) => b,
            None => return Some(Err(Error::Read)),
        };
        self.pos = offset + len;
        Some(Ok(Block { kind: rest[0], offset, body }))
    }
}

pub(crate) fn read_u16(buf: &[u8], off: &mut usize) -> u16 {
    let v = u16::from_le_bytes([buf[*off], buf[*off + 1]]);
    *off += 2;
    v
}

pub(crate) fn read_i16(buf: &[u8], off: &mut usize) -> i16 {
    read_u16(buf, off) as i16
}

pub(crate) fn read_u32(buf: &[u8], off: &mut usize) -> u32 {
    let v = u32::from_le_bytes([buf[*off], buf[*off + 1], buf[*off + 2], buf[*off + 3]]);
    *off += 4;
    v
}

pub(crate) fn write_u16(buf: &mut [u8], off: &mut usize, v: u16) {
    buf[*off..*off + 2].copy_from_slice(&v.to_le_bytes());
    *off += 2;
}
