// SPDX-License-Identifier: GPL-2.0-only

use std::fmt;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug)]
pub enum Error {
    Io(std::io::Error),
    InvalidMagic {
        what: &'static str,
        offset: usize,
    },
    Truncated {
        what: &'static str,
        offset: usize,
        needed: usize,
        available: usize,
    },
    InvalidOffset {
        what: &'static str,
        offset: usize,
    },
    InvalidValue {
        what: &'static str,
        value: String,
    },
    RegionNotFound(String),
    EntryNotFound(String),
    NoCbfsRegion,
    UnsupportedCompression(u32),
    Compression {
        algorithm: &'static str,
        message: String,
    },
    UnsupportedCommand(String),
}

impl Error {
    pub(crate) fn truncated(
        what: &'static str,
        offset: usize,
        needed: usize,
        available: usize,
    ) -> Self {
        Self::Truncated {
            what,
            offset,
            needed,
            available,
        }
    }
}

impl From<std::io::Error> for Error {
    fn from(value: std::io::Error) -> Self {
        Self::Io(value)
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(err) => write!(f, "I/O error: {err}"),
            Self::InvalidMagic { what, offset } => {
                write!(f, "invalid {what} magic at offset 0x{offset:x}")
            }
            Self::Truncated {
                what,
                offset,
                needed,
                available,
            } => write!(
                f,
                "truncated {what} at offset 0x{offset:x}: need {needed} bytes, have {available}"
            ),
            Self::InvalidOffset { what, offset } => {
                write!(f, "invalid {what} offset 0x{offset:x}")
            }
            Self::InvalidValue { what, value } => write!(f, "invalid {what}: {value}"),
            Self::RegionNotFound(name) => write!(f, "region not found: {name}"),
            Self::EntryNotFound(name) => write!(f, "CBFS entry not found: {name}"),
            Self::NoCbfsRegion => write!(f, "no CBFS region found"),
            Self::UnsupportedCompression(kind) => {
                write!(f, "unsupported CBFS compression algorithm: {kind}")
            }
            Self::Compression { algorithm, message } => {
                write!(f, "{algorithm} compression error: {message}")
            }
            Self::UnsupportedCommand(command) => {
                write!(f, "unsupported by the Rust implementation yet: {command}")
            }
        }
    }
}

impl std::error::Error for Error {}
