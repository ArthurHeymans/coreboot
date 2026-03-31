// SPDX-License-Identifier: GPL-2.0-only

use zerocopy::byteorder::big_endian::U32;
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned};

pub(crate) const CBFS_HEADER_MAGIC: u32 = 0x4f52_4243; // "ORBC"
pub(crate) const CBFS_ALIGNMENT: usize = 64;
pub(crate) const CBFS_FILE_MAGIC: &[u8; 8] = b"LARCHIVE";
pub(crate) const CBFS_FILE_HEADER_LEN: usize = core::mem::size_of::<CbfsFileHeader>();

pub(crate) const CBFS_ATTRIBUTE_ALIGN: usize = 4;
pub(crate) const CBFS_FILE_ATTR_TAG_COMPRESSION: u32 = 0x4243_5a4c; // "BCZL"
pub(crate) const CBFS_FILE_ATTR_TAG_HASH: u32 = 0x6873_6148; // "hsaH"
pub(crate) const CBFS_FILE_ATTR_TAG_POSITION: u32 = 0x4243_5350; // "BCSP"
pub(crate) const CBFS_FILE_ATTR_TAG_ALIGNMENT: u32 = 0x4243_4c41; // "BCLA"
pub(crate) const CBFS_FILE_ATTR_TAG_IBB: u32 = 0x3249_4242; // "2IBB"
pub(crate) const CBFS_FILE_ATTR_TAG_PADDING: u32 = 0x4744_4150; // "GDAP"
pub(crate) const CBFS_FILE_ATTR_TAG_STAGEHEADER: u32 = 0x5374_6748; // "StgH"
pub(crate) const CBFS_FILE_ATTR_TAG_UNUSED: u32 = 0;
pub(crate) const CBFS_FILE_ATTR_TAG_UNUSED2: u32 = 0xffff_ffff;

const _: () = assert!(core::mem::size_of::<CbfsHeader>() == 32);
const _: () = assert!(CBFS_FILE_HEADER_LEN == 24);
const _: () = assert!(core::mem::size_of::<CbfsAttributeHeader>() == 8);
const _: () = assert!(core::mem::size_of::<CbfsCompressionAttribute>() == 16);

#[derive(Debug, Clone, Copy, FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned)]
#[repr(C)]
pub(crate) struct CbfsHeader {
    pub(crate) magic: U32,
    pub(crate) version: U32,
    pub(crate) romsize: U32,
    pub(crate) bootblocksize: U32,
    pub(crate) align: U32,
    pub(crate) entries_offset: U32,
    pub(crate) architecture: U32,
    pub(crate) pad: U32,
}

impl CbfsHeader {
    pub(crate) fn from_fields(fields: [u32; 8]) -> Self {
        let [
            magic,
            version,
            romsize,
            bootblocksize,
            align,
            entries_offset,
            architecture,
            pad,
        ] = fields;
        Self {
            magic: U32::new(magic),
            version: U32::new(version),
            romsize: U32::new(romsize),
            bootblocksize: U32::new(bootblocksize),
            align: U32::new(align),
            entries_offset: U32::new(entries_offset),
            architecture: U32::new(architecture),
            pad: U32::new(pad),
        }
    }
}

#[derive(Debug, Clone, Copy, FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned)]
#[repr(C)]
pub(crate) struct CbfsFileHeader {
    pub(crate) magic: [u8; 8],
    pub(crate) len: U32,
    pub(crate) ty: U32,
    pub(crate) attributes_offset: U32,
    pub(crate) data_offset: U32,
}

impl CbfsFileHeader {
    pub(crate) fn new(len: u32, ty: u32, attributes_offset: u32, data_offset: u32) -> Self {
        Self {
            magic: *CBFS_FILE_MAGIC,
            len: U32::new(len),
            ty: U32::new(ty),
            attributes_offset: U32::new(attributes_offset),
            data_offset: U32::new(data_offset),
        }
    }
}

#[derive(Debug, Clone, Copy, FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned)]
#[repr(C)]
pub(crate) struct CbfsAttributeHeader {
    pub(crate) tag: U32,
    pub(crate) len: U32,
}

#[derive(Debug, Clone, Copy, FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned)]
#[repr(C)]
pub(crate) struct CbfsCompressionAttribute {
    pub(crate) header: CbfsAttributeHeader,
    pub(crate) compression: U32,
    pub(crate) decompressed_size: U32,
}

impl CbfsCompressionAttribute {
    pub(crate) fn new(compression: u32, decompressed_size: u32) -> Self {
        Self {
            header: CbfsAttributeHeader {
                tag: U32::new(CBFS_FILE_ATTR_TAG_COMPRESSION),
                len: U32::new(core::mem::size_of::<Self>() as u32),
            },
            compression: U32::new(compression),
            decompressed_size: U32::new(decompressed_size),
        }
    }
}
