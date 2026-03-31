// SPDX-License-Identifier: GPL-2.0-only

use object::{Object, ObjectSegment};

use crate::error::{Error, Result};

/// Summary of an ELF file relevant to CBFS stage/payload builders.
#[derive(Debug, Clone)]
pub struct ElfInfo {
    entry: u64,
    segments: Vec<ElfSegment>,
}

impl ElfInfo {
    /// Parses ELF metadata with the `object` crate.
    pub fn parse(data: &[u8]) -> Result<Self> {
        let file = object::File::parse(data).map_err(|err| Error::InvalidValue {
            what: "ELF",
            value: err.to_string(),
        })?;
        let segments = file
            .segments()
            .map(|segment| ElfSegment {
                address: segment.address(),
                size: segment.size(),
                file_range: segment.file_range(),
            })
            .collect();
        Ok(Self {
            entry: file.entry(),
            segments,
        })
    }

    /// Returns the ELF entry address.
    pub fn entry(&self) -> u64 {
        self.entry
    }

    /// Returns loadable ELF segments.
    pub fn segments(&self) -> &[ElfSegment] {
        &self.segments
    }
}

/// Loadable ELF segment metadata.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ElfSegment {
    address: u64,
    size: u64,
    file_range: (u64, u64),
}

impl ElfSegment {
    /// Returns the segment load address.
    pub fn address(&self) -> u64 {
        self.address
    }

    /// Returns the in-memory segment size.
    pub fn size(&self) -> u64 {
        self.size
    }

    /// Returns the file offset and file size backing the segment.
    pub fn file_range(&self) -> (u64, u64) {
        self.file_range
    }
}
