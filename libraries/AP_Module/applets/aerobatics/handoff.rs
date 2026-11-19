// SPDX-License-Identifier: GPL-3.0-or-later
//! Portable snapshot, committed only after the outgoing worker finishes its tick.
use ap_rust_sdk::{Context, Error, Result, SharedState};

pub const WORDS: usize = 512;
pub const MAGIC: u32 = 0x4145524f;
pub const VERSION: u32 = 1;
pub struct Handoff<'a, 'h> {
    block: SharedState<'a, 'h>,
    pub words: [u32; WORDS],
    revision: u64,
    pub held: bool,
}
impl<'a, 'h> Handoff<'a, 'h> {
    pub fn new(c: &'a Context<'h>) -> Result<Self> {
        Ok(Self {
            block: c.shared_state(MAGIC, 2048)?,
            words: [0; WORDS],
            revision: 0,
            held: false,
        })
    }
    fn write(&mut self) -> Result<bool> {
        let mut bytes = [0u8; 2048];
        for (value, dest) in self.words.iter().zip(bytes.chunks_exact_mut(4)) {
            dest.copy_from_slice(&value.to_le_bytes());
        }
        match self.block.write(0, &bytes, self.revision)? {
            Some(revision) => {
                self.revision = revision;
                Ok(true)
            }
            None => Ok(false),
        }
    }
    /// No takeover is allowed until the previous owner explicitly yields.
    /// A failed owner therefore fails closed until all modules are restarted.
    pub fn claim(&mut self) -> Result<bool> {
        let mut bytes = [0u8; 2048];
        self.revision = self.block.read(0, &mut bytes)?;
        for (dest, source) in self.words.iter_mut().zip(bytes.chunks_exact(4)) {
            *dest = u32::from_le_bytes(source.try_into().unwrap());
        }
        if self.words[0] == 0 {
            if self.words.iter().any(|v| *v != 0) {
                return Err(Error::Invalid);
            }
            self.words[0] = MAGIC;
            self.words[1] = VERSION;
        }
        if self.words[0] != MAGIC || self.words[1] != VERSION {
            return Err(Error::Invalid);
        }
        if self.words[2] != 0 {
            return Ok(false);
        }
        self.words[2] = self.block.token();
        self.held = self.write()?;
        Ok(self.held)
    }
    pub fn yield_control(&mut self) -> Result {
        self.words[2] = 0;
        if !self.write()? {
            return Err(Error::Invalid);
        }
        self.held = false;
        Ok(())
    }
}
