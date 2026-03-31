// SPDX-License-Identifier: GPL-2.0-only

use std::fs;
use std::path::PathBuf;

use cbfstool_rs::{Error, Result};

use crate::cli::{CseFptArgs, CseFptCommand, CseLayoutArgs, CseSergerArgs, CseSergerCommand};
use crate::common::{
    hex_string, push_le_u16, push_le_u32, read_u8, read_u16, read_u32, usize_to_u32,
};
use crate::tools::subpart::{SubpartDirectory, print_subpart_directory};

pub(crate) fn cse_serger_tool(args: CseSergerArgs) -> Result<()> {
    match args.command {
        CseSergerCommand::Print(print) => {
            let image = fs::read(&args.image)?;
            let cse = CseBpdt::parse(&image)?;
            print_cse_bpdt(&cse, print.name.as_deref(), print.partition_type);
            if print.sub_partition {
                for entry in &cse.entries {
                    if let Some(bytes) = cse.entry_bytes(&image, entry)
                        && let Ok(directory) = SubpartDirectory::parse(bytes)
                    {
                        println!(
                            "\n\n * Subpart entry #{}({})",
                            entry.kind,
                            cse_subpart_readable(entry.kind)
                        );
                        print_subpart_directory(&directory);
                    }
                }
            }
            Ok(())
        }
        CseSergerCommand::Dump(dump) => {
            let image = fs::read(&args.image)?;
            let cse = CseBpdt::parse(&image)?;
            if let Some(directory) = &dump.output_dir
                && !directory.is_dir()
            {
                return Err(Error::InvalidValue {
                    what: "CSE output directory",
                    value: format!("{} is not a directory", directory.display()),
                });
            }
            let mut dumped = false;
            for entry in &cse.entries {
                if !cse_entry_matches(entry, dump.name.as_deref(), dump.partition_type) {
                    continue;
                }
                let bytes = cse.entry_bytes(&image, entry).ok_or(Error::InvalidOffset {
                    what: "CSE sub-partition",
                    offset: entry.offset,
                })?;
                let name = cse_subpart_name(entry.kind);
                let path = dump
                    .output_dir
                    .as_deref()
                    .map_or_else(|| PathBuf::from(name), |dir| dir.join(name));
                println!("Dumping {name} in {}", path.display());
                fs::write(path, bytes)?;
                dumped = true;
            }
            if dumped {
                Ok(())
            } else {
                Err(Error::InvalidValue {
                    what: "CSE sub-partition",
                    value: "no matching partition found".to_owned(),
                })
            }
        }
        CseSergerCommand::PrintLayout(layout) => {
            let bytes = create_cse_layout_bytes(&layout)?;
            print_cse_layout(&layout.version.version, &bytes)?;
            Ok(())
        }
        CseSergerCommand::CreateLayout(layout) => {
            fs::write(args.image, create_cse_layout_bytes(&layout)?).map_err(Into::into)
        }
        CseSergerCommand::CreateBpdt(version) => {
            fs::write(args.image, create_cse_bpdt_bytes(&version.version)).map_err(Into::into)
        }
    }
}

pub(crate) fn cse_fpt_tool(args: CseFptArgs) -> Result<()> {
    let image = fs::read(&args.image)?;
    let fpt = CseFpt::parse(&image)?;
    match args.command {
        CseFptCommand::Print(print) => {
            if let Some(name) = print.partition_name {
                let entry = fpt.find_entry(&name).ok_or(Error::InvalidValue {
                    what: "FPT partition",
                    value: format!("{name} not found"),
                })?;
                print_cse_fpt_entry(entry);
            } else {
                print_cse_fpt_header(&fpt.header);
                print_cse_fpt_entries(&fpt.entries);
            }
        }
        CseFptCommand::Dump(dump) => {
            let mut dumped = false;
            if let Some(directory) = &dump.output_dir
                && !directory.is_dir()
            {
                return Err(Error::InvalidValue {
                    what: "FPT output directory",
                    value: format!("{} is not a directory", directory.display()),
                });
            }
            for entry in &fpt.entries {
                if !entry.is_valid() {
                    if dump.partition_name.as_deref() == Some(entry.name()) {
                        return Err(Error::InvalidValue {
                            what: "FPT partition",
                            value: format!("{} is invalid", entry.name()),
                        });
                    }
                    continue;
                }
                if dump
                    .partition_name
                    .as_deref()
                    .is_some_and(|name| name != entry.name())
                {
                    continue;
                }
                let entry_end =
                    entry
                        .offset
                        .checked_add(entry.length)
                        .ok_or(Error::InvalidOffset {
                            what: "FPT partition",
                            offset: entry.offset,
                        })?;
                let contents = image
                    .get(entry.offset..entry_end)
                    .ok_or(Error::InvalidOffset {
                        what: "FPT partition",
                        offset: entry.offset,
                    })?;
                let output = dump
                    .output_dir
                    .as_deref()
                    .map_or_else(|| PathBuf::from(entry.name()), |dir| dir.join(entry.name()));
                println!("Dumping {} in {}", entry.name(), output.display());
                fs::write(output, contents)?;
                dumped = true;
            }
            if !dumped {
                return Err(Error::InvalidValue {
                    what: "FPT partitions",
                    value: dump.partition_name.map_or_else(
                        || "no valid partitions found".to_owned(),
                        |name| format!("{name} not found"),
                    ),
                });
            }
        }
    }
    Ok(())
}

struct CseFpt {
    header: CseFptHeader,
    entries: Vec<CseFptEntry>,
}

impl CseFpt {
    fn parse(image: &[u8]) -> Result<Self> {
        let fpt_offset = [0x10_usize, 0]
            .into_iter()
            .find(|offset| image.get(*offset..*offset + 4) == Some(b"$FPT".as_slice()))
            .ok_or(Error::InvalidMagic {
                what: "FPT",
                offset: 0,
            })?;
        let data = image.get(fpt_offset..).ok_or(Error::InvalidOffset {
            what: "FPT",
            offset: fpt_offset,
        })?;
        let header = CseFptHeader::parse(data)?;
        let entries_start = header.header_len;
        let entries_len =
            header
                .entry_count
                .checked_mul(CSE_FPT_ENTRY_LEN)
                .ok_or(Error::InvalidValue {
                    what: "FPT entries",
                    value: header.entry_count.to_string(),
                })?;
        let entries_bytes =
            data.get(entries_start..entries_start + entries_len)
                .ok_or(Error::InvalidOffset {
                    what: "FPT entries",
                    offset: fpt_offset + entries_start,
                })?;
        let entries = entries_bytes
            .chunks_exact(CSE_FPT_ENTRY_LEN)
            .map(CseFptEntry::parse)
            .collect::<Result<Vec<_>>>()?;
        Ok(Self { header, entries })
    }

    fn find_entry(&self, name: &str) -> Option<&CseFptEntry> {
        self.entries.iter().find(|entry| entry.name() == name)
    }
}

const CSE_FPT_ENTRY_LEN: usize = 32;

#[derive(Debug, Clone)]
struct CseFptHeader {
    entry_count: usize,
    version: u8,
    entry_version: u8,
    header_len: usize,
    checksum_or_redundancy: u32,
    fit_tool_version: Option<[u16; 4]>,
}

impl CseFptHeader {
    fn parse(data: &[u8]) -> Result<Self> {
        if data.get(..4) != Some(b"$FPT".as_slice()) {
            return Err(Error::InvalidMagic {
                what: "FPT",
                offset: 0,
            });
        }
        let version = read_u8(data, 8, "FPT header version")?;
        match version {
            0x20 => Self::parse_20(data),
            0x21 => Self::parse_21(data),
            _ => Err(Error::InvalidValue {
                what: "FPT header version",
                value: format!("{version:#x}"),
            }),
        }
    }

    fn parse_20(data: &[u8]) -> Result<Self> {
        let header = data.get(..32).ok_or(Error::InvalidOffset {
            what: "FPT header",
            offset: 0,
        })?;
        if header
            .iter()
            .fold(0_u8, |sum, byte| sum.wrapping_add(*byte))
            != 0
        {
            return Err(Error::InvalidValue {
                what: "FPT header checksum",
                value: format!("{:#x}", header[11]),
            });
        }
        let entry_version = read_u8(data, 9, "FPT entry version")?;
        validate_fpt_entry_version(entry_version)?;
        Ok(Self {
            entry_count: read_u32(data, 4, "FPT entry count")? as usize,
            version: 0x20,
            entry_version,
            header_len: read_u8(data, 10, "FPT header length")? as usize,
            checksum_or_redundancy: u32::from(header[11]),
            fit_tool_version: None,
        })
    }

    fn parse_21(data: &[u8]) -> Result<Self> {
        let _header = data.get(..32).ok_or(Error::InvalidOffset {
            what: "FPT header",
            offset: 0,
        })?;
        let entry_version = read_u8(data, 9, "FPT entry version")?;
        validate_fpt_entry_version(entry_version)?;
        Ok(Self {
            entry_count: read_u32(data, 4, "FPT entry count")? as usize,
            version: 0x21,
            entry_version,
            header_len: read_u8(data, 10, "FPT header length")? as usize,
            checksum_or_redundancy: read_u32(data, 20, "FPT checksum")?,
            fit_tool_version: Some([
                read_u16(data, 24, "FPT FIT tool major")?,
                read_u16(data, 26, "FPT FIT tool minor")?,
                read_u16(data, 28, "FPT FIT tool build")?,
                read_u16(data, 30, "FPT FIT tool hotfix")?,
            ]),
        })
    }
}

#[derive(Debug, Clone)]
struct CseFptEntry {
    name: String,
    offset: usize,
    length: usize,
    flags: u32,
}

impl CseFptEntry {
    fn parse(data: &[u8]) -> Result<Self> {
        let name = data
            .get(..4)
            .ok_or(Error::InvalidOffset {
                what: "FPT entry name",
                offset: 0,
            })?
            .iter()
            .copied()
            .take_while(|byte| *byte != 0)
            .map(char::from)
            .collect::<String>();
        Ok(Self {
            name,
            offset: read_u32(data, 8, "FPT entry offset")? as usize,
            length: read_u32(data, 12, "FPT entry length")? as usize,
            flags: read_u32(data, 28, "FPT entry flags")?,
        })
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn is_valid(&self) -> bool {
        self.offset != 0 && self.length != 0 && ((self.flags >> 24) & 0xff) != 0xff
    }

    fn is_code(&self) -> bool {
        self.flags & 0x7f == 0
    }
}

fn validate_fpt_entry_version(version: u8) -> Result<()> {
    if version == 0x10 {
        Ok(())
    } else {
        Err(Error::InvalidValue {
            what: "FPT entry version",
            value: format!("{version:#x}"),
        })
    }
}

fn print_cse_fpt_header(header: &CseFptHeader) {
    println!(" * FPT header");
    println!("{:<25}: $FPT", "Marker");
    println!("{:<25}: {}", "Number of entries", header.entry_count);
    println!("{:<25}: {:#x}", "Header version", header.version);
    println!("{:<25}: {:#x}", "Entry version", header.entry_version);
    println!("{:<25}: {}", "Header length", header.header_len);
    if let Some(version) = header.fit_tool_version {
        println!("{:<25}: {:#x}", "Checksum", header.checksum_or_redundancy);
        println!(
            "{:<25}: {}.{}.{}.{}({:02x}.{:02x}.{:02x}.{:02x})",
            "FIT Tool Version",
            version[0],
            version[1],
            version[2],
            version[3],
            version[0],
            version[1],
            version[2],
            version[3]
        );
    } else {
        println!(
            "{:<25}: {:#x}",
            "Header checksum", header.checksum_or_redundancy
        );
    }
}

fn print_cse_fpt_entries(entries: &[CseFptEntry]) {
    println!("\n * FPT entries");
    println!(
        "{:<25}{:<25}{:<25}{:<25}",
        "Name", "Offset", "Size", "Flags"
    );
    println!(
        "============================================================================================="
    );
    entries.iter().for_each(print_cse_fpt_entry);
    println!(
        "============================================================================================="
    );
    println!("Flags: I=invalid, V=valid, C=code, D=data");
}

fn print_cse_fpt_entry(entry: &CseFptEntry) {
    println!(
        "{:<25}0x{:<23x}0x{:<23x}{},{} ({:#010x})",
        entry.name(),
        entry.offset,
        entry.length,
        if entry.is_code() { 'C' } else { 'D' },
        if entry.is_valid() { 'V' } else { 'I' },
        entry.flags
    );
}

struct CseBpdt {
    version: String,
    entries: Vec<CseBpdtEntry>,
}

#[derive(Debug)]
struct CseBpdtEntry {
    kind: u32,
    offset: usize,
    size: usize,
}

impl CseBpdt {
    fn parse(data: &[u8]) -> Result<Self> {
        if read_u32(data, 0, "CSE BPDT signature")? != 0x55aa {
            return Err(Error::InvalidMagic {
                what: "CSE BPDT",
                offset: 0,
            });
        }
        let count = read_u16(data, 4, "CSE BPDT descriptor count")? as usize;
        let version_byte = read_u8(data, 6, "CSE BPDT version")?;
        let version = if version_byte == 2 { "1.7" } else { "1.6" }.to_owned();
        let entries = (0..count)
            .map(|index| {
                let offset = 24 + index * 12;
                Ok(CseBpdtEntry {
                    kind: read_u32(data, offset, "CSE BPDT entry type")?,
                    offset: read_u32(data, offset + 4, "CSE BPDT entry offset")? as usize,
                    size: read_u32(data, offset + 8, "CSE BPDT entry size")? as usize,
                })
            })
            .collect::<Result<Vec<_>>>()?;
        Ok(Self { version, entries })
    }

    fn entry_bytes<'a>(&self, image: &'a [u8], entry: &CseBpdtEntry) -> Option<&'a [u8]> {
        image.get(entry.offset..entry.offset.checked_add(entry.size)?)
    }
}

fn print_cse_bpdt(cse: &CseBpdt, name: Option<&str>, kind: Option<u32>) {
    println!(" * BPDT header");
    println!("{:<25} 0x{:<23.8x}", "Signature", 0x55aa_u32);
    println!("{:<25} {:<25}", "Descriptor count", cse.entries.len());
    println!(
        "{:<25} {} (Layout {})",
        "BPDT Version",
        if cse.version == "1.7" { 2 } else { 1 },
        cse.version
    );
    println!("\n * BPDT entries");
    println!(
        "{:<25}{:<25}{:<25}{:<25}{:<25}{:<25}",
        "Entry #", "Partition Name", "Human readable name", "Type", "Offset", "Size"
    );
    for (index, entry) in cse.entries.iter().enumerate() {
        if !cse_entry_matches(entry, name, kind) {
            continue;
        }
        println!(
            "{:<25}{:<25}{:<25}{:<25}0x{:<23x}0x{:<23x}",
            index + 1,
            cse_subpart_name(entry.kind),
            cse_subpart_readable(entry.kind),
            entry.kind,
            entry.offset,
            entry.size
        );
    }
}

fn cse_entry_matches(entry: &CseBpdtEntry, name: Option<&str>, kind: Option<u32>) -> bool {
    if kind.is_some_and(|expected| expected != entry.kind) {
        return false;
    }
    if let Some(name) = name {
        return cse_subpart_name(entry.kind) == name || cse_subpart_alt_name(entry.kind) == name;
    }
    true
}

fn cse_subpart_name(kind: u32) -> &'static str {
    match kind {
        0 => "SMIP",
        1 => "RBEP",
        2 => "FTPR",
        3 => "UCOD",
        4 => "IBBP",
        5 => "SBDT",
        6 => "OBBP",
        7 => "NFTP",
        8 => "ISHP",
        9 => "DLMP",
        10 => "IFPP",
        11 => "UTOK",
        12 => "UFSP",
        13 => "UFSG",
        14 => "PMCP",
        15 => "IUNP",
        16 => "NVMC",
        17 => "UEPP",
        20 => "OEMP",
        22 => "PAVP",
        23 => "IOMP",
        24 => "NPHY",
        25 => "TBTP",
        32 => "PCHC",
        _ => "UNKNOWN",
    }
}

fn cse_subpart_alt_name(kind: u32) -> &'static str {
    match kind {
        2 => "MFTP",
        _ => "",
    }
}

fn cse_subpart_readable(kind: u32) -> &'static str {
    match kind {
        0 => "OEM SMIP",
        1 => "CSE RBE",
        2 => "CSE BUP",
        3 => "Microcode",
        4 => "Initial Boot Block",
        5 => "Secondary BPDT",
        6 => "OEM Boot Block",
        7 => "CSE Main",
        8 => "ISH Firmware",
        9 => "CSE IDLM",
        10 => "IFP override",
        11 => "Debug tokens",
        12 => "UFS Phy",
        13 => "UFS GPP",
        14 => "PMC Firmware",
        15 => "IUNIT Firmware",
        16 => "NVM CFG",
        17 => "UEP",
        20 => "OEM Key Manifest",
        22 => "PAVP",
        23 => "IOM Firmware",
        24 => "NPHY Firmware",
        25 => "TBT Firmware",
        32 => "ICC Firmware",
        _ => "Unknown",
    }
}

fn create_cse_bpdt_bytes(version: &str) -> Vec<u8> {
    let mut output = Vec::with_capacity(24);
    push_le_u32(&mut output, 0x55aa);
    push_le_u16(&mut output, 0);
    if version == "1.7" {
        output.push(2);
        output.push(0);
        push_le_u32(&mut output, 0);
    } else {
        push_le_u16(&mut output, 1);
        push_le_u16(&mut output, 0);
        output.push(0);
        output.push(0);
    }
    push_le_u32(&mut output, 0);
    (0..4).for_each(|_| push_le_u16(&mut output, 0));
    output
}

fn create_cse_layout_bytes(args: &CseLayoutArgs) -> Result<Vec<u8>> {
    let mut output = Vec::new();
    output.extend_from_slice(if args.version.version == "1.7" {
        &[0xff; 16]
    } else {
        &[0; 16]
    });
    if args.version.version == "1.7" {
        push_le_u16(&mut output, 60);
        push_le_u16(&mut output, 0);
        push_le_u32(&mut output, 0);
    }
    for region in [args.data, args.bp1, args.bp2, args.bp3] {
        let (offset, size) = region.unwrap_or((0, 0));
        push_le_u32(&mut output, usize_to_u32(offset, "CSE layout offset")?);
        push_le_u32(&mut output, usize_to_u32(size, "CSE layout size")?);
    }
    if args.version.version == "1.7" {
        let (offset, size) = args.bp4.unwrap_or((0, 0));
        push_le_u32(&mut output, usize_to_u32(offset, "CSE layout offset")?);
        push_le_u32(&mut output, usize_to_u32(size, "CSE layout size")?);
        (0..4).for_each(|_| push_le_u32(&mut output, 0));
        let checksum = !crc32_ieee(0xffff_ffff, &output[16..]);
        output[20..24].copy_from_slice(&checksum.to_le_bytes());
    } else {
        (0..16).for_each(|_| push_le_u32(&mut output, 0));
        output.push(0);
    }
    Ok(output)
}

fn print_cse_layout(version: &str, data: &[u8]) -> Result<()> {
    println!(" * CSE Layout\n");
    println!("ROM Bypass: {}", hex_string(data.get(..16).unwrap_or(&[])));
    let mut offset = 16;
    if version == "1.7" {
        println!("Size: 0x{:x}", read_u16(data, offset, "CSE layout size")?);
        println!(
            "Redundancy: 0x{:x}",
            read_u16(data, offset + 2, "CSE layout redundancy")?
        );
        println!(
            "Checksum: 0x{:x}",
            read_u32(data, offset + 4, "CSE layout checksum")?
        );
        offset += 8;
    }
    for name in ["Data partition", "BP1", "BP2", "BP3"] {
        println!(
            "{name} offset: 0x{:x}",
            read_u32(data, offset, "CSE layout offset")?
        );
        println!(
            "{name} size: 0x{:x}",
            read_u32(data, offset + 4, "CSE layout size")?
        );
        offset += 8;
    }
    if version == "1.7" {
        println!(
            "BP4 offset: 0x{:x}",
            read_u32(data, offset, "CSE layout offset")?
        );
        println!(
            "BP4 size: 0x{:x}",
            read_u32(data, offset + 4, "CSE layout size")?
        );
    } else {
        println!("Checksum: 0x{:x}", data.last().copied().unwrap_or(0));
    }
    Ok(())
}

fn crc32_ieee(seed: u32, data: &[u8]) -> u32 {
    data.iter().fold(seed, |mut crc, byte| {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xedb8_8320
            } else {
                crc >> 1
            };
        }
        crc
    })
}
