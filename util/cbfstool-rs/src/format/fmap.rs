// SPDX-License-Identifier: GPL-2.0-only

use zerocopy::byteorder::little_endian::{U16, U32, U64};
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned};

pub(crate) const FMAP_SIGNATURE: &[u8; 8] = b"__FMAP__";
pub(crate) const FMAP_HEADER_LEN: usize = core::mem::size_of::<FmapHeader>();
pub(crate) const FMAP_AREA_LEN: usize = core::mem::size_of::<FmapAreaHeader>();
pub(crate) const FMAP_STRLEN: usize = 32;

pub(crate) const FMAP_AREA_STATIC: u16 = 1 << 0;
pub(crate) const FMAP_AREA_COMPRESSED: u16 = 1 << 1;
pub(crate) const FMAP_AREA_RO: u16 = 1 << 2;
pub(crate) const FMAP_AREA_PRESERVE: u16 = 1 << 3;

const _: () = assert!(FMAP_HEADER_LEN == 56);
const _: () = assert!(FMAP_AREA_LEN == 42);

#[derive(Debug, Clone, Copy, FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned)]
#[repr(C)]
pub(crate) struct FmapHeader {
    pub(crate) signature: [u8; 8],
    pub(crate) ver_major: u8,
    pub(crate) ver_minor: u8,
    pub(crate) base: U64,
    pub(crate) size: U32,
    pub(crate) name: [u8; FMAP_STRLEN],
    pub(crate) nareas: U16,
}

#[derive(Debug, Clone, Copy, FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned)]
#[repr(C)]
pub(crate) struct FmapAreaHeader {
    pub(crate) offset: U32,
    pub(crate) size: U32,
    pub(crate) name: [u8; FMAP_STRLEN],
    pub(crate) flags: U16,
}
