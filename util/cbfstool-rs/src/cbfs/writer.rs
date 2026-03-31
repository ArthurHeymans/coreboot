// SPDX-License-Identifier: GPL-2.0-only

use crate::cbfs::attributes::AttributeBuilder;
use crate::cbfs::{Cbfs, CbfsType, Compression};
use crate::error::{Error, Result};
use crate::format::cbfs::{
    CBFS_ALIGNMENT, CBFS_ATTRIBUTE_ALIGN, CBFS_FILE_HEADER_LEN, CBFS_HEADER_MAGIC,
    CbfsCompressionAttribute, CbfsFileHeader, CbfsHeader,
};
use crate::vboot::hash::{self, HashAlgorithm};

use zerocopy::IntoBytes;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntryPlacement {
    Any,
    Offset(usize),
    Alignment(usize),
}

pub fn add_raw_entry(region: &mut [u8], name: &str, ty: CbfsType, contents: &[u8]) -> Result<()> {
    add_entry(region, name, ty, contents, Compression::None)
}

pub fn add_entry(
    region: &mut [u8],
    name: &str,
    ty: CbfsType,
    contents: &[u8],
    compression: Compression,
) -> Result<()> {
    add_entry_with_attributes(region, name, ty, contents, compression, &[])
}

pub fn add_entry_with_attributes(
    region: &mut [u8],
    name: &str,
    ty: CbfsType,
    contents: &[u8],
    compression: Compression,
    extra_attributes: &[u8],
) -> Result<()> {
    add_entry_with_attributes_at(
        region,
        name,
        ty,
        contents,
        compression,
        extra_attributes,
        EntryPlacement::Any,
    )
}

pub fn add_entry_with_attributes_at(
    region: &mut [u8],
    name: &str,
    ty: CbfsType,
    contents: &[u8],
    compression: Compression,
    extra_attributes: &[u8],
    placement: EntryPlacement,
) -> Result<()> {
    add_entry_with_attributes_and_hash_at(
        region,
        name,
        ty,
        contents,
        compression,
        extra_attributes,
        None,
        placement,
    )
}

pub fn add_entry_with_attributes_and_hash_at(
    region: &mut [u8],
    name: &str,
    ty: CbfsType,
    contents: &[u8],
    compression: Compression,
    extra_attributes: &[u8],
    hash_algorithm: Option<HashAlgorithm>,
    placement: EntryPlacement,
) -> Result<()> {
    if !extra_attributes.len().is_multiple_of(CBFS_ATTRIBUTE_ALIGN) {
        return Err(Error::InvalidValue {
            what: "CBFS attributes",
            value: format!("length {} is not 4-byte aligned", extra_attributes.len()),
        });
    }

    let cbfs = Cbfs::parse(region)?;
    if cbfs.find_entry(name)?.is_some() {
        return Err(Error::InvalidValue {
            what: "CBFS entry",
            value: format!("duplicate name: {name}"),
        });
    }

    let (stored_compression, stored_contents) = compress_entry(compression, contents)?;
    let generated_attributes;
    let entry_attributes = if let Some(algorithm) = hash_algorithm {
        if !hash_attributes_allowed(ty) {
            return Err(Error::InvalidValue {
                what: "CBFS hash attribute",
                value: format!("hash attributes are not valid on {} entries", ty.name()),
            });
        }
        let digest = hash::digest(algorithm, &stored_contents)?;
        let mut builder = AttributeBuilder::new();
        builder.push_serialized(extra_attributes)?;
        builder.hash(algorithm, &digest)?;
        generated_attributes = builder.into_bytes();
        generated_attributes.as_slice()
    } else {
        extra_attributes
    };
    let spec = FileEntrySpec {
        name,
        ty,
        contents: &stored_contents,
        compression: stored_compression,
        decompressed_size: contents.len(),
        extra_attributes: entry_attributes,
    };
    let header_len = file_header_len(name, stored_compression, entry_attributes.len());
    let empty_header_len = file_header_len("", Compression::None, 0);
    let align = cbfs.alignment();
    let entries = cbfs.entries().collect::<Result<Vec<_>>>()?;
    let Some((entry_start, entry_end, write_start)) = entries.iter().find_map(|entry| {
        if entry.entry_type() != CbfsType::NULL && entry.entry_type() != CbfsType::DELETED {
            return None;
        }
        let entry_start = entry.offset();
        let entry_end = entry_aligned_end(entry, align).ok()?;
        let write_start = placement_start(placement, entry_start, entry_end, empty_header_len)?;
        let data_end = write_start
            .checked_add(header_len)?
            .checked_add(stored_contents.len())?;
        let trailing_start = data_end.next_multiple_of(align);
        if trailing_start == entry_end || trailing_start + empty_header_len <= entry_end {
            Some((entry_start, entry_end, write_start))
        } else {
            None
        }
    }) else {
        return Err(Error::InvalidValue {
            what: "CBFS free space",
            value: format!("no empty entry large enough for {name}"),
        });
    };

    if write_start > entry_start {
        write_empty_entry(region, entry_start, write_start)?;
    }
    write_file_entry(region, write_start, spec)?;
    let data_end = write_start + header_len + stored_contents.len();
    let trailing_start = data_end.next_multiple_of(align);
    if trailing_start < entry_end {
        write_empty_entry(region, trailing_start, entry_end)?;
    }
    Ok(())
}

pub fn remove_entry(region: &mut [u8], name: &str) -> Result<()> {
    let cbfs = Cbfs::parse(region)?;
    let align = cbfs.alignment();
    let entries = cbfs.entries().collect::<Result<Vec<_>>>()?;
    let Some(entry) = entries.iter().find(|entry| entry.name() == name) else {
        return Err(Error::EntryNotFound(name.to_owned()));
    };
    let start = entry.offset();
    let end = (start + entry.data_offset() + entry.len()).next_multiple_of(align);
    write_empty_entry(region, start, end)
}

pub fn compact_region(region: &mut [u8]) -> Result<()> {
    let cbfs = Cbfs::parse(region)?;
    let align = cbfs.alignment();
    let entries = cbfs.entries().collect::<Result<Vec<_>>>()?;
    let mut kept_entries = Vec::new();
    for entry in entries {
        if is_empty_entry(entry.entry_type()) {
            continue;
        }
        let end = entry_data_end(&entry)?;
        let bytes = region
            .get(entry.offset()..end)
            .ok_or(Error::InvalidOffset {
                what: "CBFS entry",
                offset: entry.offset(),
            })?
            .to_vec();
        kept_entries.push(bytes);
    }

    write_entries_to_empty_region(region, align, &kept_entries, 0)
}

pub fn copy_region(source: &[u8], destination: &mut [u8]) -> Result<()> {
    let source_cbfs = Cbfs::parse(source)?;
    let align = source_cbfs.alignment();
    let entries = source_cbfs.entries().collect::<Result<Vec<_>>>()?;
    let mut copied_entries = Vec::new();
    for entry in entries {
        if is_empty_entry(entry.entry_type()) || entry.entry_type() == CbfsType::CBFSHEADER {
            continue;
        }
        let end = entry_data_end(&entry)?;
        let bytes = source
            .get(entry.offset()..end)
            .ok_or(Error::InvalidOffset {
                what: "CBFS entry",
                offset: entry.offset(),
            })?
            .to_vec();
        copied_entries.push(bytes);
    }

    write_entries_to_empty_region(destination, align, &copied_entries, 4)
}

pub fn expand_region(region: &mut [u8]) -> Result<()> {
    let cbfs = Cbfs::parse(region)?;
    let align = cbfs.alignment();
    let entries = cbfs.entries().collect::<Result<Vec<_>>>()?;
    let Some(last_entry) = entries.last() else {
        return Err(Error::InvalidMagic {
            what: "CBFS file",
            offset: 0,
        });
    };

    let start = if is_empty_entry(last_entry.entry_type()) {
        last_entry.offset()
    } else {
        entry_aligned_end(last_entry, align)?
    };
    if start + file_header_len("", Compression::None, 0) <= region.len() {
        write_empty_entry(region, start, region.len())?;
    }
    Ok(())
}

pub fn truncate_region(region: &mut [u8]) -> Result<usize> {
    let cbfs = Cbfs::parse(region)?;
    let align = cbfs.alignment();
    let entries = cbfs.entries().collect::<Result<Vec<_>>>()?;
    let Some(last_entry) = entries.last() else {
        return Ok(0);
    };

    let size = if is_empty_entry(last_entry.entry_type()) {
        last_entry.offset()
    } else {
        entry_aligned_end(last_entry, align)?
    };
    if is_empty_entry(last_entry.entry_type()) {
        let trailing = region.get_mut(size..).ok_or(Error::InvalidOffset {
            what: "CBFS truncate",
            offset: size,
        })?;
        trailing.fill(0xff);
    }
    Ok(size)
}

pub fn create_empty_region(region: &mut [u8]) -> Result<()> {
    region.fill(0xff);
    write_empty_entry(region, 0, region.len())
}

/// Serializes a CBFS master header for a region.
pub fn master_header_bytes(region_offset: usize, region_size: usize) -> Result<Vec<u8>> {
    const CBFS_HEADER_VERSION2: u32 = 0x3131_3132;
    const CBFS_ARCHITECTURE_UNKNOWN: u32 = 0xffff_ffff;

    let rom_size = region_offset
        .checked_add(region_size)
        .ok_or(Error::InvalidValue {
            what: "CBFS master header romsize",
            value: "offset plus size overflows".to_owned(),
        })?;
    let header = CbfsHeader::from_fields([
        CBFS_HEADER_MAGIC,
        CBFS_HEADER_VERSION2,
        checked_u32(rom_size, "CBFS master header romsize")?,
        4,
        checked_u32(CBFS_ALIGNMENT, "CBFS master header alignment")?,
        checked_u32(region_offset, "CBFS master header offset")?,
        CBFS_ARCHITECTURE_UNKNOWN,
        0,
    ]);
    Ok(header.as_bytes().to_vec())
}

/// Returns the little-endian tail pointer for a CBFS master header.
pub fn relative_master_header_pointer(region_size: usize, header_offset: usize) -> Result<u32> {
    if region_size < 4 || header_offset >= region_size {
        return Err(Error::InvalidValue {
            what: "CBFS master header pointer",
            value: format!("header offset {header_offset:#x} outside region size {region_size:#x}"),
        });
    }
    let distance = checked_u32(region_size - header_offset, "CBFS master header pointer")?;
    Ok(0_u32.wrapping_sub(distance))
}

fn write_entries_to_empty_region(
    region: &mut [u8],
    align: usize,
    entries: &[Vec<u8>],
    reserved_tail_len: usize,
) -> Result<()> {
    let writable_end = region
        .len()
        .checked_sub(reserved_tail_len)
        .ok_or(Error::InvalidValue {
            what: "CBFS destination region",
            value: format!("smaller than reserved tail length {reserved_tail_len}"),
        })?;
    let mut cursor = 0_usize;
    let mut positions = Vec::with_capacity(entries.len());
    for (index, entry) in entries.iter().enumerate() {
        let end = cursor
            .checked_add(entry.len())
            .ok_or(Error::InvalidOffset {
                what: "CBFS entry",
                offset: cursor,
            })?;
        if end > writable_end {
            return Err(Error::InvalidValue {
                what: "CBFS destination region",
                value: "not large enough for copied entries".to_owned(),
            });
        }
        positions.push((cursor, end));
        cursor = end.next_multiple_of(align);
        if cursor > region.len() && index + 1 != entries.len() {
            return Err(Error::InvalidValue {
                what: "CBFS destination region",
                value: "not large enough for entry alignment".to_owned(),
            });
        }
    }

    region[..writable_end].fill(0xff);
    for (entry, (start, end)) in entries.iter().zip(positions) {
        let destination = region.get_mut(start..end).ok_or(Error::InvalidValue {
            what: "CBFS destination region",
            value: "not large enough for copied entries".to_owned(),
        })?;
        destination.copy_from_slice(entry);
    }

    if cursor + file_header_len("", Compression::None, 0) <= writable_end {
        write_empty_entry(region, cursor, writable_end)?;
    }
    Ok(())
}

fn placement_start(
    placement: EntryPlacement,
    entry_start: usize,
    entry_end: usize,
    empty_header_len: usize,
) -> Option<usize> {
    let start = match placement {
        EntryPlacement::Any => entry_start,
        EntryPlacement::Offset(offset) => offset,
        EntryPlacement::Alignment(alignment) => {
            if alignment == 0 {
                return None;
            }
            entry_start.next_multiple_of(alignment)
        }
    };
    if start < entry_start || start >= entry_end {
        return None;
    }
    if start != entry_start && start.saturating_sub(entry_start) < empty_header_len {
        return None;
    }
    Some(start)
}

fn is_empty_entry(ty: CbfsType) -> bool {
    ty == CbfsType::NULL || ty == CbfsType::DELETED
}

pub fn hash_attributes_allowed(ty: CbfsType) -> bool {
    !matches!(
        ty,
        CbfsType::BOOTBLOCK | CbfsType::CBFSHEADER | CbfsType::INTEL_FIT | CbfsType::AMDFW
    )
}

fn entry_data_end(entry: &crate::cbfs::CbfsEntry<'_>) -> Result<usize> {
    entry
        .offset()
        .checked_add(entry.data_offset())
        .and_then(|start| start.checked_add(entry.len()))
        .ok_or(Error::InvalidOffset {
            what: "CBFS entry",
            offset: entry.offset(),
        })
}

fn entry_aligned_end(entry: &crate::cbfs::CbfsEntry<'_>, align: usize) -> Result<usize> {
    Ok(entry_data_end(entry)?.next_multiple_of(align))
}

struct FileEntrySpec<'a> {
    name: &'a str,
    ty: CbfsType,
    contents: &'a [u8],
    compression: Compression,
    decompressed_size: usize,
    extra_attributes: &'a [u8],
}

fn write_file_entry(region: &mut [u8], offset: usize, spec: FileEntrySpec<'_>) -> Result<()> {
    let name_header_len = file_header_len(spec.name, Compression::None, 0);
    let compression_len = if spec.compression == Compression::None {
        0
    } else {
        core::mem::size_of::<CbfsCompressionAttribute>()
    };
    let header_len = name_header_len + spec.extra_attributes.len() + compression_len;
    let data_start = offset + header_len;
    let data_end = data_start + spec.contents.len();
    let file_bytes = region
        .get_mut(offset..data_end)
        .ok_or(Error::InvalidOffset {
            what: "CBFS entry",
            offset,
        })?;
    file_bytes.fill(0xff);
    let attributes_offset =
        if spec.compression == Compression::None && spec.extra_attributes.is_empty() {
            0
        } else {
            checked_u32(name_header_len, "CBFS attributes offset")?
        };
    let header = CbfsFileHeader::new(
        checked_u32(spec.contents.len(), "CBFS entry length")?,
        spec.ty.raw(),
        attributes_offset,
        checked_u32(header_len, "CBFS data offset")?,
    );
    file_bytes[..CBFS_FILE_HEADER_LEN].copy_from_slice(header.as_bytes());
    let name_start = CBFS_FILE_HEADER_LEN;
    let name_end = name_start + spec.name.len();
    file_bytes[name_start..name_end].copy_from_slice(spec.name.as_bytes());
    file_bytes[name_end] = 0;
    file_bytes[name_header_len..name_header_len + spec.extra_attributes.len()]
        .copy_from_slice(spec.extra_attributes);
    if spec.compression != Compression::None {
        let attr_start = name_header_len + spec.extra_attributes.len();
        let attr = CbfsCompressionAttribute::new(
            spec.compression.raw(),
            checked_u32(spec.decompressed_size, "CBFS decompressed size")?,
        );
        file_bytes[attr_start..header_len].copy_from_slice(attr.as_bytes());
    }
    file_bytes[header_len..].copy_from_slice(spec.contents);
    Ok(())
}

fn write_empty_entry(region: &mut [u8], offset: usize, end: usize) -> Result<()> {
    let header_len = file_header_len("", Compression::None, 0);
    if end < offset + header_len {
        return Err(Error::InvalidOffset {
            what: "CBFS empty entry",
            offset,
        });
    }
    let len = end - offset - header_len;
    let entry = region.get_mut(offset..end).ok_or(Error::InvalidOffset {
        what: "CBFS empty entry",
        offset,
    })?;
    entry.fill(0xff);
    let header = CbfsFileHeader::new(
        checked_u32(len, "CBFS empty entry length")?,
        CbfsType::NULL.raw(),
        0,
        checked_u32(header_len, "CBFS data offset")?,
    );
    entry[..CBFS_FILE_HEADER_LEN].copy_from_slice(header.as_bytes());
    entry[CBFS_FILE_HEADER_LEN] = 0;
    Ok(())
}

fn file_header_len(name: &str, compression: Compression, extra_attributes_len: usize) -> usize {
    let name_header_len =
        CBFS_FILE_HEADER_LEN + (name.len() + 1).next_multiple_of(CBFS_ATTRIBUTE_ALIGN);
    name_header_len
        + extra_attributes_len
        + if compression == Compression::None {
            0
        } else {
            core::mem::size_of::<CbfsCompressionAttribute>()
        }
}

fn checked_u32(value: usize, what: &'static str) -> Result<u32> {
    u32::try_from(value).map_err(|_| Error::InvalidValue {
        what,
        value: value.to_string(),
    })
}

#[cfg(feature = "compression")]
fn compress_entry(compression: Compression, contents: &[u8]) -> Result<(Compression, Vec<u8>)> {
    crate::cbfs::compression::compress(compression, contents)
}

#[cfg(not(feature = "compression"))]
fn compress_entry(compression: Compression, contents: &[u8]) -> Result<(Compression, Vec<u8>)> {
    if compression != Compression::None {
        return Err(Error::UnsupportedCompression(compression.raw()));
    }
    Ok((Compression::None, contents.to_vec()))
}
