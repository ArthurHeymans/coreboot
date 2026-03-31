// SPDX-License-Identifier: GPL-2.0-only

use std::fmt;
use std::str::FromStr;

use crate::cbfs::attributes::Attributes;
use crate::error::{Error, Result};
use crate::format::cbfs::{
    CBFS_FILE_ATTR_TAG_COMPRESSION, CBFS_FILE_ATTR_TAG_UNUSED, CBFS_FILE_ATTR_TAG_UNUSED2,
    CBFS_FILE_HEADER_LEN, CBFS_FILE_MAGIC, CbfsAttributeHeader, CbfsCompressionAttribute,
    CbfsFileHeader,
};
use crate::util::nul_terminated_string;

use zerocopy::FromBytes;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Compression {
    None,
    Lzma,
    Lz4,
    Zstd,
    Unknown(u32),
}

impl Compression {
    pub fn from_raw(raw: u32) -> Self {
        match raw {
            0 => Self::None,
            1 => Self::Lzma,
            2 => Self::Lz4,
            3 => Self::Zstd,
            other => Self::Unknown(other),
        }
    }

    pub fn raw(self) -> u32 {
        match self {
            Self::None => 0,
            Self::Lzma => 1,
            Self::Lz4 => 2,
            Self::Zstd => 3,
            Self::Unknown(value) => value,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Lzma => "LZMA",
            Self::Lz4 => "LZ4",
            Self::Zstd => "ZSTD",
            Self::Unknown(_) => "unknown",
        }
    }
}

impl FromStr for Compression {
    type Err = Error;

    fn from_str(s: &str) -> Result<Self> {
        let compression = match s.to_ascii_lowercase().as_str() {
            "none" => Self::None,
            "lzma" => Self::Lzma,
            "lz4" => Self::Lz4,
            "zstd" => Self::Zstd,
            _ => {
                let value = parse_int(s).ok_or_else(|| Error::InvalidValue {
                    what: "compression",
                    value: s.to_owned(),
                })?;
                Self::Unknown(value)
            }
        };
        Ok(compression)
    }
}

impl fmt::Display for Compression {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Self::Unknown(value) => write!(f, "unknown({value:#x})"),
            other => f.write_str(other.name()),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CbfsType(u32);

impl CbfsType {
    pub const DELETED: Self = Self(0x0000_0000);
    pub const NULL: Self = Self(0xffff_ffff);
    pub const BOOTBLOCK: Self = Self(0x01);
    pub const CBFSHEADER: Self = Self(0x02);
    pub const INTEL_FIT: Self = Self(0x54);
    pub const RAW: Self = Self(0x50);
    pub const AMDFW: Self = Self(0x80);

    pub fn new(raw: u32) -> Self {
        Self(raw)
    }

    pub fn raw(self) -> u32 {
        self.0
    }

    pub fn name(self) -> &'static str {
        match self.0 {
            0x0000_0000 => "deleted",
            0xffff_ffff => "null",
            0x01 => "bootblock",
            0x02 => "cbfs header",
            0x10 => "legacy stage",
            0x11 => "stage",
            0x20 => "simple elf",
            0x21 => "fit_payload",
            0x30 => "optionrom",
            0x40 => "bootsplash",
            0x50 => "raw",
            0x51 => "vsa",
            0x52 => "mbi",
            0x53 => "microcode",
            0x54 => "intel_fit",
            0x60 => "fsp",
            0x61 => "mrc",
            0x62 => "mma",
            0x63 => "efi",
            0x70 => "struct",
            0x80 => "amdfw",
            0xaa => "cmos_default",
            0xab => "spd",
            0xac => "mrc_cache",
            0x01aa => "cmos_layout",
            _ => "(unknown)",
        }
    }
}

impl FromStr for CbfsType {
    type Err = Error;

    fn from_str(s: &str) -> Result<Self> {
        let normalized = s.to_ascii_lowercase().replace('_', " ");
        let ty = match normalized.as_str() {
            "deleted" => Self::DELETED,
            "null" => Self::NULL,
            "bootblock" => Self(0x01),
            "cbfs header" => Self(0x02),
            "legacy stage" => Self(0x10),
            "stage" => Self(0x11),
            "simple elf" => Self(0x20),
            "fit payload" => Self(0x21),
            "optionrom" => Self(0x30),
            "bootsplash" => Self(0x40),
            "raw" => Self::RAW,
            "vsa" => Self(0x51),
            "mbi" => Self(0x52),
            "microcode" => Self(0x53),
            "intel fit" => Self(0x54),
            "fsp" => Self(0x60),
            "mrc" => Self(0x61),
            "mma" => Self(0x62),
            "efi" => Self(0x63),
            "struct" => Self(0x70),
            "amdfw" => Self(0x80),
            "cmos default" => Self(0xaa),
            "spd" => Self(0xab),
            "mrc cache" => Self(0xac),
            "cmos layout" => Self(0x01aa),
            _ => {
                let value = parse_int(s).ok_or_else(|| Error::InvalidValue {
                    what: "CBFS type",
                    value: s.to_owned(),
                })?;
                Self(value)
            }
        };
        Ok(ty)
    }
}

#[derive(Debug, Clone)]
pub struct CbfsEntry<'a> {
    region: &'a [u8],
    offset: usize,
    len: usize,
    ty: CbfsType,
    attributes_offset: usize,
    data_offset: usize,
    name: String,
    compression: Compression,
    decompressed_size: usize,
}

impl<'a> CbfsEntry<'a> {
    pub(crate) fn parse(region: &'a [u8], offset: usize, _align: usize) -> Result<Self> {
        let header_bytes = region
            .get(offset..offset + CBFS_FILE_HEADER_LEN)
            .ok_or_else(|| {
                Error::truncated(
                    "CBFS file",
                    offset,
                    CBFS_FILE_HEADER_LEN,
                    region.len().saturating_sub(offset),
                )
            })?;
        let header =
            CbfsFileHeader::ref_from_bytes(header_bytes).map_err(|_| Error::InvalidOffset {
                what: "CBFS file",
                offset,
            })?;

        if header.magic != *CBFS_FILE_MAGIC {
            return Err(Error::InvalidMagic {
                what: "CBFS file",
                offset,
            });
        }

        let len = header.len.get() as usize;
        let ty = CbfsType(header.ty.get());
        let attributes_offset = header.attributes_offset.get() as usize;
        let data_offset = header.data_offset.get() as usize;

        if data_offset < CBFS_FILE_HEADER_LEN {
            return Err(Error::InvalidOffset {
                what: "CBFS data",
                offset: offset + data_offset,
            });
        }
        let data_end = offset
            .checked_add(data_offset)
            .and_then(|start| start.checked_add(len))
            .ok_or(Error::InvalidOffset {
                what: "CBFS data",
                offset,
            })?;
        if data_end > region.len() {
            return Err(Error::truncated(
                "CBFS data",
                offset + data_offset,
                len,
                region.len().saturating_sub(offset + data_offset),
            ));
        }

        let name_end =
            if attributes_offset >= CBFS_FILE_HEADER_LEN && attributes_offset < data_offset {
                attributes_offset
            } else {
                data_offset
            };
        let name_bytes = region
            .get(offset + CBFS_FILE_HEADER_LEN..offset + name_end)
            .ok_or(Error::InvalidOffset {
                what: "CBFS name",
                offset: offset + CBFS_FILE_HEADER_LEN,
            })?;
        let name = nul_terminated_string(name_bytes);

        let mut compression = Compression::None;
        let mut decompressed_size = len;
        if attributes_offset >= CBFS_FILE_HEADER_LEN && attributes_offset < data_offset {
            let attrs_start = offset + attributes_offset;
            let attrs_end = offset + data_offset;
            let mut cursor = attrs_start;
            while cursor + 8 <= attrs_end {
                let attr_header_bytes = region.get(cursor..cursor + 8).ok_or_else(|| {
                    Error::truncated(
                        "CBFS attribute",
                        cursor,
                        8,
                        region.len().saturating_sub(cursor),
                    )
                })?;
                let attr_header =
                    CbfsAttributeHeader::ref_from_bytes(attr_header_bytes).map_err(|_| {
                        Error::InvalidOffset {
                            what: "CBFS attribute",
                            offset: cursor,
                        }
                    })?;
                let tag = attr_header.tag.get();
                if tag == CBFS_FILE_ATTR_TAG_UNUSED || tag == CBFS_FILE_ATTR_TAG_UNUSED2 {
                    break;
                }
                let attr_len = attr_header.len.get() as usize;
                if attr_len < 8 || cursor + attr_len > attrs_end {
                    return Err(Error::InvalidOffset {
                        what: "CBFS attribute",
                        offset: cursor,
                    });
                }
                if tag == CBFS_FILE_ATTR_TAG_COMPRESSION {
                    let compression_attr_len = core::mem::size_of::<CbfsCompressionAttribute>();
                    if attr_len < compression_attr_len {
                        return Err(Error::InvalidOffset {
                            what: "CBFS compression attribute",
                            offset: cursor,
                        });
                    }
                    let compression_bytes = region
                        .get(cursor..cursor + compression_attr_len)
                        .ok_or_else(|| {
                            Error::truncated(
                                "CBFS compression attribute",
                                cursor,
                                compression_attr_len,
                                region.len().saturating_sub(cursor),
                            )
                        })?;
                    let compression_attr = CbfsCompressionAttribute::ref_from_bytes(
                        compression_bytes,
                    )
                    .map_err(|_| Error::InvalidOffset {
                        what: "CBFS compression attribute",
                        offset: cursor,
                    })?;
                    compression = Compression::from_raw(compression_attr.compression.get());
                    decompressed_size = compression_attr.decompressed_size.get() as usize;
                }
                cursor += attr_len;
            }
        }

        Ok(Self {
            region,
            offset,
            len,
            ty,
            attributes_offset,
            data_offset,
            name,
            compression,
            decompressed_size,
        })
    }

    pub fn offset(&self) -> usize {
        self.offset
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn metadata_len(&self) -> usize {
        self.data_offset
    }

    pub fn data_offset(&self) -> usize {
        self.data_offset
    }

    pub fn attributes_offset(&self) -> Option<usize> {
        (self.attributes_offset != 0).then_some(self.attributes_offset)
    }

    /// Returns the serialized attribute bytes for this entry.
    pub fn attributes_bytes(&self) -> Option<&'a [u8]> {
        let attributes_offset = self.attributes_offset()?;
        self.region
            .get(self.offset + attributes_offset..self.offset + self.data_offset)
    }

    /// Iterates over this entry's serialized CBFS attributes.
    pub fn attributes(&self) -> Attributes<'a> {
        Attributes::new(self.attributes_bytes().unwrap_or_default())
    }

    pub fn total_len(&self) -> usize {
        self.data_offset + self.len
    }

    pub fn entry_type(&self) -> CbfsType {
        self.ty
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn display_name(&self) -> &str {
        if self.name.is_empty() {
            "(empty)"
        } else {
            &self.name
        }
    }

    pub fn compression(&self) -> Compression {
        self.compression
    }

    pub fn decompressed_size(&self) -> usize {
        self.decompressed_size
    }

    pub fn data(&self) -> &'a [u8] {
        let start = self.offset + self.data_offset;
        let end = start + self.len;
        &self.region[start..end]
    }
}

fn parse_int(s: &str) -> Option<u32> {
    if let Some(hex) = s.strip_prefix("0x") {
        u32::from_str_radix(hex, 16).ok()
    } else {
        s.parse::<u32>().ok()
    }
}
