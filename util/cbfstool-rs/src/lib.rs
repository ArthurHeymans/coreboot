// SPDX-License-Identifier: GPL-2.0-only

//! Experimental Rust library for working with coreboot CBFS/ROM images.
//!
//! The crate intentionally keeps the library API separate from the CLI so other
//! Rust tools can parse firmware images without spawning `cbfstool`.

pub mod cbfs;
#[cfg(feature = "elf")]
pub mod elf;
pub mod error;
pub mod fmap;
pub mod fmd;
mod format;
pub mod image;
mod util;
pub mod vboot;

pub use cbfs::attributes::{Attribute, AttributeBuilder, AttributeTag, Attributes};
pub use cbfs::{Cbfs, CbfsEntry, CbfsType, Compression};
#[cfg(feature = "elf")]
pub use elf::{ElfInfo, ElfSegment};
pub use error::{Error, Result};
pub use fmap::{Fmap, FmapArea, FmapFlags};
pub use fmd::FlashmapDescriptor;
pub use image::{AddEntryOptions, FirmwareImage, OwnedFirmwareImage, Region, RegionWriteMode};
pub use vboot::hash::{HashAlgorithm, Vb2Hash};
