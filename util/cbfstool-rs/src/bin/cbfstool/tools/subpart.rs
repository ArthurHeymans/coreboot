// SPDX-License-Identifier: GPL-2.0-only

use cbfstool_rs::{Error, Result};

use crate::common::{fixed_string, read_u8, read_u32};

#[derive(Debug)]
pub(crate) struct SubpartDirectory {
    pub(crate) name: String,
    pub(crate) entries: Vec<SubpartDirectoryEntry>,
}

#[derive(Debug)]
pub(crate) struct SubpartDirectoryEntry {
    pub(crate) name: String,
    pub(crate) offset: usize,
    pub(crate) length: usize,
    pub(crate) reserved: u32,
    pub(crate) compressed: bool,
}

impl SubpartDirectory {
    pub(crate) fn parse(data: &[u8]) -> Result<Self> {
        if data.len() < 16 || read_u32(data, 0, "sub-partition directory marker")? != 0x44504324 {
            return Err(Error::InvalidMagic {
                what: "sub-partition directory",
                offset: 0,
            });
        }
        let count = read_u32(data, 4, "sub-partition directory entry count")? as usize;
        let header_len = read_u8(data, 10, "sub-partition directory header length")? as usize;
        let name = fixed_string(data.get(12..16).ok_or(Error::InvalidOffset {
            what: "sub-partition directory name",
            offset: 12,
        })?);
        let entries = (0..count)
            .map(|index| {
                let offset = header_len + index * 24;
                let name_bytes = data.get(offset..offset + 12).ok_or(Error::InvalidOffset {
                    what: "sub-partition directory entry",
                    offset,
                })?;
                let offset_bytes = read_u32(data, offset + 12, "sub-partition entry offset")?;
                Ok(SubpartDirectoryEntry {
                    name: fixed_string(name_bytes),
                    offset: (offset_bytes & 0x01ff_ffff) as usize,
                    length: read_u32(data, offset + 16, "sub-partition entry length")? as usize,
                    reserved: read_u32(data, offset + 20, "sub-partition entry reserved")?,
                    compressed: ((offset_bytes >> 25) & 1) != 0,
                })
            })
            .collect::<Result<Vec<_>>>()?;
        Ok(Self { name, entries })
    }

    pub(crate) fn find(&self, name: &str) -> Option<&SubpartDirectoryEntry> {
        self.entries.iter().find(|entry| entry.name == name)
    }
}

pub(crate) fn print_subpart_directory(directory: &SubpartDirectory) {
    println!("{:<25} {}", "Name", directory.name);
    println!(
        "{:<25}{:<25}{:<25}{:<25}{:<25}{:<25}",
        "Entry #", "Name", "Offset", "Huffman Compressed?", "Length", "Rsvd"
    );
    for (index, entry) in directory.entries.iter().enumerate() {
        println!(
            "{:<25}{:<25}0x{:<23x}{:<25}0x{:<23x}0x{:<23x}",
            index + 1,
            entry.name,
            entry.offset,
            if entry.compressed { 'Y' } else { 'N' },
            entry.length,
            entry.reserved
        );
    }
}
