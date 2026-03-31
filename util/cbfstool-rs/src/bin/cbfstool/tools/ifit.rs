// SPDX-License-Identifier: GPL-2.0-only

use std::fs;

use cbfstool_rs::{CbfsEntry, Error, FirmwareImage, Result};

use crate::cli::IfitArgs;
use crate::common::{read_u8, read_u32, read_u64, write_le_u32};
use crate::fileio::rewrite_firmware_image;

pub(crate) fn ifit_tool(args: IfitArgs) -> Result<()> {
    let mut image_bytes = fs::read(&args.file)?;
    let image = FirmwareImage::parse(&image_bytes)?;
    let region = image.region(Some(&args.region))?;
    let region_offset = region.offset();
    let region_size = region.size();
    let image_size = image.data().len();

    let mut fit_offset = args.header_offset;
    if args.set_fit_pointer {
        let name = args.name.as_deref().ok_or(Error::InvalidValue {
            what: "IFIT table name",
            value: "-F requires -n".to_owned(),
        })?;
        let cbfs = image.cbfs(Some(&args.region))?;
        let entry = cbfs
            .find_entry(name)?
            .ok_or_else(|| Error::EntryNotFound(name.to_owned()))?;
        fit_offset = Some(entry.offset() + entry.data_offset());
        let fit_address = top_aligned_region_address(region_size, fit_offset.unwrap_or(0))?;
        let pointer_offset =
            fit_pointer_region_offset(region_size, args.topswap_size.unwrap_or(0))?;
        drop(image);
        let absolute_pointer_offset = region_offset + pointer_offset;
        write_le_u32(
            &mut image_bytes,
            absolute_pointer_offset,
            fit_address,
            "FIT pointer",
        )?;
        write_le_u32(
            &mut image_bytes,
            absolute_pointer_offset + 4,
            0,
            "FIT pointer high",
        )?;
    } else {
        drop(image);
    }

    let region_bytes = image_bytes
        .get(region_offset..region_offset + region_size)
        .ok_or(Error::InvalidOffset {
            what: "IFIT region",
            offset: region_offset,
        })?;
    let fit_offset = if let Some(offset) = fit_offset {
        offset
    } else {
        region_bytes
            .windows(8)
            .position(|window| window == b"_FIT_   ")
            .ok_or(Error::InvalidMagic {
                what: "FIT",
                offset: region_offset,
            })?
    };
    let fit = FitTable::parse(region_bytes, fit_offset)?;
    if args.dump || (!args.clear_table && args.delete.is_none()) {
        print_fit_table(&fit);
    }
    if args.clear_table
        || args.delete.is_some()
        || args.add_cbfs_entry
        || args.add_region
        || args.set_fit_pointer
    {
        let table_start = region_offset + fit_offset;
        if args.clear_table {
            clear_fit_table(&mut image_bytes, table_start, fit.table_size)?;
        }
        if let Some(index) = args.delete {
            delete_fit_entry(&mut image_bytes, table_start, &fit, index)?;
        }
        if args.add_region {
            let name = args.name.as_deref().ok_or(Error::InvalidValue {
                what: "IFIT region name",
                value: "-A requires -n".to_owned(),
            })?;
            let image = FirmwareImage::parse(&image_bytes)?;
            let target_region = image.region(Some(name))?;
            let address = absolute_top_aligned_address(image_size, target_region.offset(), 0)?;
            let kind = required_fit_type(args.fit_type)?;
            add_fit_entry(
                &mut image_bytes,
                table_start,
                kind,
                address,
                0,
                required_max_entries(args.max_table_size)?,
            )?;
        }
        if args.add_cbfs_entry {
            let name = args.name.as_deref().ok_or(Error::InvalidValue {
                what: "IFIT CBFS name",
                value: "-a requires -n".to_owned(),
            })?;
            let kind = required_fit_type(args.fit_type)?;
            let max_entries = required_max_entries(args.max_table_size)?;
            let entries_to_add = {
                let image = FirmwareImage::parse(&image_bytes)?;
                let file_region_name = args.file_region.as_deref().unwrap_or(&args.region);
                let file_region = image.region(Some(file_region_name))?;
                let cbfs = image.cbfs(Some(file_region_name))?;
                let entry = cbfs
                    .find_entry(name)?
                    .ok_or_else(|| Error::EntryNotFound(name.to_owned()))?;
                let mut entries_to_add = Vec::new();
                if kind == 1 {
                    let microcodes = microcode_offsets(&entry)?;
                    for (offset, _) in microcodes {
                        let address = absolute_top_aligned_address(
                            image_size,
                            file_region.offset(),
                            entry.offset() + entry.data_offset() + offset,
                        )?;
                        entries_to_add.push((address, 0));
                    }
                } else {
                    let address = absolute_top_aligned_address(
                        image_size,
                        file_region.offset(),
                        entry.offset() + entry.data_offset(),
                    )?;
                    entries_to_add.push((
                        address,
                        u32::try_from(entry.len()).map_err(|_| Error::InvalidValue {
                            what: "FIT entry size",
                            value: entry.len().to_string(),
                        })?,
                    ));
                }
                entries_to_add
            };
            for (address, len) in entries_to_add {
                add_fit_entry(
                    &mut image_bytes,
                    table_start,
                    kind,
                    address,
                    len,
                    max_entries,
                )?;
            }
        }
        rewrite_firmware_image(&args.file, image_bytes)?;
    }
    Ok(())
}

struct FitTable {
    table_size: usize,
    entries: Vec<FitEntry>,
}

#[derive(Debug, Clone)]
struct FitEntry {
    address: u64,
    size_reserved: u32,
    kind: u8,
}

impl FitEntry {
    fn parse(data: &[u8], offset: usize) -> Result<Self> {
        Ok(Self {
            address: read_u64(data, offset, "FIT entry address")?,
            size_reserved: read_u32(data, offset + 8, "FIT entry size")?,
            kind: read_u8(data, offset + 14, "FIT entry type")? & !0x80,
        })
    }

    fn size_bytes(&self) -> usize {
        ((self.size_reserved & 0x00ff_ffff) as usize) << 4
    }
}

impl FitTable {
    fn parse(data: &[u8], offset: usize) -> Result<Self> {
        let header = FitEntry::parse(data, offset)?;
        if data.get(offset..offset + 8) != Some(b"_FIT_   ".as_slice()) {
            return Err(Error::InvalidMagic {
                what: "FIT",
                offset,
            });
        }
        let table_size = header.size_bytes();
        if table_size < 16 {
            return Err(Error::InvalidValue {
                what: "FIT table size",
                value: table_size.to_string(),
            });
        }
        let entry_count = table_size / 16 - 1;
        let entries = (0..entry_count)
            .map(|index| FitEntry::parse(data, offset + 16 + index * 16))
            .collect::<Result<Vec<_>>>()?;
        Ok(Self {
            table_size,
            entries,
        })
    }
}

fn print_fit_table(fit: &FitTable) {
    println!("\n    FIT table:");
    if fit.entries.is_empty() {
        println!("    empty\n");
        return;
    }
    println!(
        "    {:<6} {:<20} {:<16} {:<8}",
        "Index", "Type", "Addr", "Size"
    );
    for (index, entry) in fit.entries.iter().enumerate() {
        println!(
            "    {:6} {:<20} 0x{:08x}      0x{:08x}",
            index,
            fit_type_name(entry.kind),
            entry.address,
            entry.size_bytes()
        );
    }
    println!();
}

fn fit_type_name(kind: u8) -> &'static str {
    match kind {
        1 => "Microcode",
        2 => "BIOS ACM",
        4 => "Platform Boot Policy",
        7 => "BIOS Startup Module",
        8 => "TPM Policy",
        9 => "BIOS Policy",
        0x0a => "TXT Policy",
        0x0b => "Key Manifest",
        0x0c => "Boot Policy",
        0x10 => "CSE SecureBoot",
        0x2d => "TXTSX policy",
        0x2f => "JMP debug policy",
        127 => "unused",
        _ => "unknown",
    }
}

fn clear_fit_table(image: &mut [u8], table_start: usize, table_size: usize) -> Result<()> {
    let table =
        image
            .get_mut(table_start..table_start + table_size)
            .ok_or(Error::InvalidOffset {
                what: "FIT table",
                offset: table_start,
            })?;
    if table.len() > 16 {
        table[16..].fill(0);
    }
    write_le_u32(table, 8, 1, "FIT header size")?;
    table[15] = 0;
    let checksum = table[..16]
        .iter()
        .fold(0_u8, |sum, byte| sum.wrapping_add(*byte));
    table[15] = checksum.wrapping_neg();
    Ok(())
}

fn delete_fit_entry(
    image: &mut [u8],
    table_start: usize,
    fit: &FitTable,
    index: usize,
) -> Result<()> {
    if index >= fit.entries.len() {
        return Err(Error::InvalidValue {
            what: "FIT entry index",
            value: index.to_string(),
        });
    }
    let entry_offset = table_start + 16 + index * 16;
    let entry = image
        .get_mut(entry_offset..entry_offset + 16)
        .ok_or(Error::InvalidOffset {
            what: "FIT entry",
            offset: entry_offset,
        })?;
    entry.fill(0);
    entry[14] = 127;
    Ok(())
}

fn required_fit_type(fit_type: Option<u8>) -> Result<u8> {
    fit_type.ok_or(Error::InvalidValue {
        what: "FIT type",
        value: "-t is required for add operations".to_owned(),
    })
}

fn required_max_entries(max_entries: Option<usize>) -> Result<usize> {
    max_entries.ok_or(Error::InvalidValue {
        what: "FIT max table size",
        value: "-s is required for add operations".to_owned(),
    })
}

fn fit_pointer_region_offset(region_size: usize, topswap_size: usize) -> Result<usize> {
    let top_aligned_offset = topswap_size.checked_add(64).ok_or(Error::InvalidValue {
        what: "FIT pointer offset",
        value: "overflow".to_owned(),
    })?;
    region_size
        .checked_sub(top_aligned_offset)
        .ok_or(Error::InvalidValue {
            what: "FIT pointer offset",
            value: format!("top-aligned offset {top_aligned_offset:#x} outside region"),
        })
}

fn top_aligned_region_address(region_size: usize, offset: usize) -> Result<u32> {
    let top_aligned = if offset > 0 && offset < region_size {
        region_size - offset
    } else {
        region_size.checked_sub(offset).ok_or(Error::InvalidValue {
            what: "FIT address",
            value: format!("offset {offset:#x} outside region"),
        })?
    };
    top_aligned_address(top_aligned)
}

fn absolute_top_aligned_address(
    image_size: usize,
    region_offset: usize,
    offset: usize,
) -> Result<u32> {
    let absolute = region_offset
        .checked_add(offset)
        .ok_or(Error::InvalidValue {
            what: "FIT address",
            value: "offset overflow".to_owned(),
        })?;
    let top_aligned = image_size
        .checked_sub(absolute)
        .ok_or(Error::InvalidValue {
            what: "FIT address",
            value: format!("offset {absolute:#x} outside image"),
        })?;
    top_aligned_address(top_aligned)
}

fn top_aligned_address(top_aligned: usize) -> Result<u32> {
    let top_aligned = u32::try_from(top_aligned).map_err(|_| Error::InvalidValue {
        what: "FIT address",
        value: top_aligned.to_string(),
    })?;
    Ok(0_u32.wrapping_sub(top_aligned))
}

fn microcode_offsets(entry: &CbfsEntry<'_>) -> Result<Vec<(usize, usize)>> {
    const MICROCODE_HEADER_LEN: usize = 48;
    let data = entry.data();
    let mut offset = 0;
    let mut result = Vec::new();
    while data.len().saturating_sub(offset) > MICROCODE_HEADER_LEN {
        let total_size = read_u32(data, offset + 32, "microcode total size")? as usize;
        let total_size = if total_size == 0 { 2048 } else { total_size };
        if total_size < MICROCODE_HEADER_LEN || total_size > data.len().saturating_sub(offset) {
            break;
        }
        result.push((offset, total_size));
        offset = offset.checked_add(total_size).ok_or(Error::InvalidValue {
            what: "microcode offset",
            value: "overflow".to_owned(),
        })?;
    }
    Ok(result)
}

fn add_fit_entry(
    image: &mut [u8],
    table_start: usize,
    kind: u8,
    address: u32,
    len: u32,
    max_entries: usize,
) -> Result<()> {
    if !supported_fit_type(kind) {
        return Err(Error::InvalidValue {
            what: "FIT type",
            value: format!("unsupported type {kind}"),
        });
    }
    let table_size = fit_table_size(image, table_start)?;
    let entry_count = table_size / 16 - 1;
    if entry_count >= max_entries {
        return Err(Error::InvalidValue {
            what: "FIT table",
            value: "no free entries".to_owned(),
        });
    }
    let entry_offset = table_start + 16 + entry_count * 16;
    write_fit_entry(image, entry_offset, kind, address, len)?;
    write_fit_table_size(image, table_start, table_size + 16)?;
    sort_fit_entries(image, table_start)?;
    update_fit_checksum(image, table_start)
}

fn supported_fit_type(kind: u8) -> bool {
    matches!(kind, 1 | 2 | 4 | 7 | 9 | 0x0a | 0x0b | 0x0c)
}

fn write_fit_entry(
    image: &mut [u8],
    offset: usize,
    kind: u8,
    address: u32,
    len: u32,
) -> Result<()> {
    let entry = image
        .get_mut(offset..offset + 16)
        .ok_or(Error::InvalidOffset {
            what: "FIT entry",
            offset,
        })?;
    entry.fill(0);
    entry[0..8].copy_from_slice(&(address as u64).to_le_bytes());
    let size_reserved = match kind {
        7 | 9 => len >> 4,
        0x0b | 0x0c => len,
        _ => 0,
    };
    entry[8..12].copy_from_slice(&size_reserved.to_le_bytes());
    let version = match kind {
        0x0a => 1_u16,
        _ => 0x0100_u16,
    };
    entry[12..14].copy_from_slice(&version.to_le_bytes());
    entry[14] = kind;
    Ok(())
}

fn fit_table_size(image: &[u8], table_start: usize) -> Result<usize> {
    let raw = read_u32(image, table_start + 8, "FIT table size")?;
    Ok(((raw & 0x00ff_ffff) as usize) << 4)
}

fn write_fit_table_size(image: &mut [u8], table_start: usize, size: usize) -> Result<()> {
    if !size.is_multiple_of(16) {
        return Err(Error::InvalidValue {
            what: "FIT table size",
            value: size.to_string(),
        });
    }
    write_le_u32(
        image,
        table_start + 8,
        u32::try_from(size >> 4).map_err(|_| Error::InvalidValue {
            what: "FIT table size",
            value: size.to_string(),
        })?,
        "FIT header size",
    )
}

fn sort_fit_entries(image: &mut [u8], table_start: usize) -> Result<()> {
    let table_size = fit_table_size(image, table_start)?;
    let entry_count = table_size / 16 - 1;
    let entries_start = table_start + 16;
    let entries_end = entries_start + entry_count * 16;
    let entries = image
        .get_mut(entries_start..entries_end)
        .ok_or(Error::InvalidOffset {
            what: "FIT entries",
            offset: entries_start,
        })?;
    for _ in 0..entry_count {
        for index in 0..entry_count.saturating_sub(1) {
            let left_type = entries[index * 16 + 14];
            let right_type = entries[(index + 1) * 16 + 14];
            if left_type > right_type {
                for byte in 0..16 {
                    entries.swap(index * 16 + byte, (index + 1) * 16 + byte);
                }
            }
        }
    }
    Ok(())
}

fn update_fit_checksum(image: &mut [u8], table_start: usize) -> Result<()> {
    let table_size = fit_table_size(image, table_start)?;
    let table =
        image
            .get_mut(table_start..table_start + table_size)
            .ok_or(Error::InvalidOffset {
                what: "FIT table",
                offset: table_start,
            })?;
    table[15] = 0;
    let checksum = table.iter().fold(0_u8, |sum, byte| sum.wrapping_add(*byte));
    table[15] = checksum.wrapping_neg();
    Ok(())
}
