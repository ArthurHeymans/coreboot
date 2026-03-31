// SPDX-License-Identifier: GPL-2.0-only

use std::fs;

use cbfstool_rs::{Error, Result};

use crate::cli::{IfwiArgs, IfwiCommand};
use crate::common::{read_u16, read_u32, read_u64};
use crate::tools::subpart::{SubpartDirectory, print_subpart_directory};

pub(crate) fn ifwi_tool(args: IfwiArgs) -> Result<()> {
    let image = fs::read(&args.image)?;
    let second_lbp = match &args.command {
        IfwiCommand::Print(print) => print.second_lbp,
        IfwiCommand::Extract(extract) => extract.second_lbp,
    };
    let ifwi = IfwiImage::parse(&image, usize::from(second_lbp))?;
    match args.command {
        IfwiCommand::Print(print) => {
            print_ifwi_bpdt("BPDT", &ifwi.bpdt, ifwi.lbp_offset);
            if let Some(sbpdt) = &ifwi.sbpdt {
                print_ifwi_bpdt("S-BPDT", sbpdt, ifwi.lbp_offset);
            }
            if print.directory {
                for partition in ifwi.partitions() {
                    if ifwi_subpart_contains_dir(partition.kind)
                        && let Ok(directory) = SubpartDirectory::parse(partition.bytes)
                    {
                        print_subpart_directory(&directory);
                    }
                }
            }
        }
        IfwiCommand::Extract(extract) => {
            let kind = ifwi_subpart_type(&extract.name).ok_or(Error::InvalidValue {
                what: "IFWI sub-partition name",
                value: extract.name.clone(),
            })?;
            let partition = ifwi.partition(kind).ok_or(Error::InvalidValue {
                what: "IFWI sub-partition",
                value: format!("{} not present", extract.name),
            })?;
            if extract.directory {
                let entry_name = extract.entry.ok_or(Error::InvalidValue {
                    what: "IFWI directory entry",
                    value: "-e/--subpart-dentry is required with -d".to_owned(),
                })?;
                let directory = SubpartDirectory::parse(partition.bytes)?;
                let entry = directory.find(&entry_name).ok_or(Error::InvalidValue {
                    what: "IFWI directory entry",
                    value: format!("{entry_name} not found"),
                })?;
                let end = entry
                    .offset
                    .checked_add(entry.length)
                    .ok_or(Error::InvalidOffset {
                        what: "IFWI directory entry",
                        offset: entry.offset,
                    })?;
                let bytes = partition
                    .bytes
                    .get(entry.offset..end)
                    .ok_or(Error::InvalidOffset {
                        what: "IFWI directory entry",
                        offset: entry.offset,
                    })?;
                fs::write(extract.file, bytes)?;
            } else {
                fs::write(extract.file, partition.bytes)?;
            }
        }
    }
    Ok(())
}

struct IfwiImage<'a> {
    lbp_offset: usize,
    bpdt: IfwiBpdt,
    sbpdt: Option<IfwiBpdt>,
    data: &'a [u8],
}

#[derive(Debug)]
struct IfwiBpdt {
    header: IfwiBpdtHeader,
    entries: Vec<IfwiBpdtEntry>,
}

#[derive(Debug)]
struct IfwiBpdtHeader {
    descriptor_count: usize,
    version: u16,
    xor_checksum: u32,
    ifwi_version: u32,
    fit_tool_version: u64,
}

#[derive(Debug, Clone)]
struct IfwiBpdtEntry {
    kind: u16,
    flags: u16,
    offset: usize,
    size: usize,
}

#[derive(Debug)]
struct IfwiPartition<'a> {
    kind: u16,
    bytes: &'a [u8],
}

impl<'a> IfwiImage<'a> {
    fn parse(data: &'a [u8], target_lbp: usize) -> Result<Self> {
        let mut offset = 0;
        let mut lbp = 0;
        while offset + 4 <= data.len() {
            if read_u32(data, offset, "BPDT signature")? == 0x55aa {
                if lbp == target_lbp {
                    break;
                }
                let bpdt = IfwiBpdt::parse_at(data, offset)?;
                let size = bpdt
                    .entries
                    .iter()
                    .map(|entry| entry.offset.saturating_add(entry.size))
                    .max()
                    .unwrap_or(24);
                offset = offset.saturating_add(size);
                lbp += 1;
            } else {
                offset = offset.saturating_add(4096);
            }
        }
        if offset + 4 > data.len() {
            return Err(Error::InvalidMagic {
                what: "IFWI BPDT",
                offset: 0,
            });
        }
        let bpdt = IfwiBpdt::parse_at(data, offset)?;
        let sbpdt = bpdt
            .entries
            .iter()
            .find(|entry| entry.kind == 5 && entry.size != 0)
            .and_then(|entry| IfwiBpdt::parse_at(data, offset + entry.offset).ok());
        Ok(Self {
            lbp_offset: offset,
            bpdt,
            sbpdt,
            data,
        })
    }

    fn all_entries(&self) -> impl Iterator<Item = &IfwiBpdtEntry> {
        self.bpdt.entries.iter().chain(
            self.sbpdt
                .as_ref()
                .map(|bpdt| bpdt.entries.as_slice())
                .unwrap_or(&[])
                .iter(),
        )
    }

    fn partition(&self, kind: u16) -> Option<IfwiPartition<'a>> {
        let entry = self
            .all_entries()
            .find(|entry| entry.kind == kind && entry.size != 0)?;
        let start = self.lbp_offset.checked_add(entry.offset)?;
        let end = start.checked_add(entry.size)?;
        self.data
            .get(start..end)
            .map(|bytes| IfwiPartition { kind, bytes })
    }

    fn partitions(&self) -> Vec<IfwiPartition<'a>> {
        self.all_entries()
            .filter_map(|entry| self.partition(entry.kind))
            .collect()
    }
}

impl IfwiBpdt {
    fn parse_at(data: &[u8], offset: usize) -> Result<Self> {
        if read_u32(data, offset, "BPDT signature")? != 0x55aa {
            return Err(Error::InvalidMagic {
                what: "BPDT",
                offset,
            });
        }
        let descriptor_count = read_u16(data, offset + 4, "BPDT descriptor count")? as usize;
        let header = IfwiBpdtHeader {
            descriptor_count,
            version: read_u16(data, offset + 6, "BPDT version")?,
            xor_checksum: read_u32(data, offset + 8, "BPDT checksum")?,
            ifwi_version: read_u32(data, offset + 12, "BPDT IFWI version")?,
            fit_tool_version: read_u64(data, offset + 16, "BPDT FIT tool version")?,
        };
        let entries = (0..descriptor_count)
            .map(|index| {
                let entry = offset + 24 + index * 12;
                Ok(IfwiBpdtEntry {
                    kind: read_u16(data, entry, "BPDT entry type")?,
                    flags: read_u16(data, entry + 2, "BPDT entry flags")?,
                    offset: read_u32(data, entry + 4, "BPDT entry offset")? as usize,
                    size: read_u32(data, entry + 8, "BPDT entry size")? as usize,
                })
            })
            .collect::<Result<Vec<_>>>()?;
        Ok(Self { header, entries })
    }
}

fn print_ifwi_bpdt(name: &str, bpdt: &IfwiBpdt, lbp_offset: usize) {
    println!("{:<25} {name}", "Header");
    println!("{:<25} 0x{:<23.8x}", "Signature", 0x55aa_u32);
    println!(
        "{:<25} {:<25}",
        "Descriptor count", bpdt.header.descriptor_count
    );
    println!("{:<25} {:<25}", "BPDT Version", bpdt.header.version);
    println!("{:<25} 0x{:<23x}", "XOR checksum", bpdt.header.xor_checksum);
    println!("{:<25} 0x{:<23x}", "IFWI Version", bpdt.header.ifwi_version);
    println!(
        "{:<25} 0x{:<23x}",
        "FIT Tool Version", bpdt.header.fit_tool_version
    );
    println!("{name} entries");
    println!(
        "{:<25}{:<25}{:<25}{:<25}{:<25}{:<25}{:<25}{:<25}",
        "Entry #", "Sub-Partition", "Name", "Type", "Flags", "Offset", "Size", "File Offset"
    );
    for (index, entry) in bpdt.entries.iter().enumerate() {
        println!(
            "{:<25}{:<25}{:<25}{:<25}0x{:<23.08x}0x{:<23x}0x{:<23x}0x{:<23x}",
            index + 1,
            ifwi_subpart_name(entry.kind),
            ifwi_subpart_readable(entry.kind),
            entry.kind,
            entry.flags,
            entry.offset,
            entry.size,
            lbp_offset + entry.offset
        );
    }
}

fn ifwi_subpart_name(kind: u16) -> &'static str {
    match kind {
        0 => "SMIP",
        1 => "RBEP",
        2 => "FTPR",
        3 => "UCOD",
        4 => "IBBP",
        5 => "S_BPDT",
        6 => "OBBP",
        7 => "NFTP",
        8 => "ISHP",
        9 => "DLMP",
        10 => "IFP_OVERRIDE",
        11 => "DEBUG_TOKENS",
        12 => "UFS_PHY",
        13 => "UFS_GPP",
        14 => "PMCP",
        15 => "IUNP",
        16 => "NVM_CONFIG",
        17 => "UEP",
        18 => "UFS_RATE_B",
        _ => "UNKNOWN",
    }
}

fn ifwi_subpart_readable(kind: u16) -> &'static str {
    match kind {
        1 => "CSE_RBE",
        2 => "CSE_BUP",
        3 => "Microcode",
        4 => "Bootblock",
        5 => "S-BPDT",
        6 => "OEM boot block",
        7 => "CSE_MAIN",
        8 => "ISH",
        9 => "CSE_IDLM",
        11 => "Debug Tokens",
        12 => "UFS Phy",
        13 => "UFS GPP",
        14 => "PMC firmware",
        15 => "IUNIT",
        16 => "NVM Config",
        18 => "UFS Rate B Config",
        _ => ifwi_subpart_name(kind),
    }
}

fn ifwi_subpart_type(name: &str) -> Option<u16> {
    (0..=18).find(|kind| ifwi_subpart_name(*kind) == name)
}

fn ifwi_subpart_contains_dir(kind: u16) -> bool {
    matches!(kind, 0 | 1 | 2 | 3 | 4 | 6 | 7 | 9 | 14)
}
