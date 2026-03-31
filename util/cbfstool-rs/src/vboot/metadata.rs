// SPDX-License-Identifier: GPL-2.0-only

//! CBFS verification metadata-anchor scanning and patching.

use crate::cbfs::{Cbfs, CbfsType};
use crate::error::{Error, Result};
use crate::fmap::Fmap;
use crate::vboot::hash::{self, HashAlgorithm};

/// Magic bytes embedded in `struct metadata_hash_anchor`.
pub const METADATA_HASH_ANCHOR_MAGIC: &[u8; 8] = b"\xadMdtHsh\x15";

const VB2_HASH_HEADER_LEN: usize = 4;
const ANCHOR_HASH_OFFSET: usize = METADATA_HASH_ANCHOR_MAGIC.len() + VB2_HASH_HEADER_LEN;
const ANCHOR_ALGORITHM_OFFSET: usize = METADATA_HASH_ANCHOR_MAGIC.len() + 3;

/// Describes a metadata-hash anchor found in a byte slice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MetadataHashAnchor {
    offset: usize,
    algorithm: HashAlgorithm,
    digest_size: usize,
}

impl MetadataHashAnchor {
    /// Returns the anchor offset within the scanned byte slice.
    pub fn offset(self) -> usize {
        self.offset
    }

    /// Returns the hash algorithm encoded in the anchor.
    pub fn algorithm(self) -> HashAlgorithm {
        self.algorithm
    }

    /// Returns the byte range containing the CBFS metadata digest.
    pub fn cbfs_hash_range(self) -> core::ops::Range<usize> {
        let start = self.offset + ANCHOR_HASH_OFFSET;
        start..start + self.digest_size
    }

    /// Returns the byte range containing the FMAP digest.
    pub fn fmap_hash_range(self) -> core::ops::Range<usize> {
        let start = self.offset + ANCHOR_HASH_OFFSET + self.digest_size;
        start..start + self.digest_size
    }

    /// Returns the minimum byte range occupied by active anchor fields.
    pub fn active_range(self) -> core::ops::Range<usize> {
        self.offset..self.fmap_hash_range().end
    }
}

/// Finds all valid metadata-hash anchors in `bytes`.
pub fn find_anchors(bytes: &[u8]) -> Result<Vec<MetadataHashAnchor>> {
    let mut anchors = Vec::new();
    for offset in bytes
        .windows(METADATA_HASH_ANCHOR_MAGIC.len())
        .enumerate()
        .filter_map(|(offset, window)| (window == METADATA_HASH_ANCHOR_MAGIC).then_some(offset))
    {
        anchors.push(parse_anchor_at(bytes, offset)?);
    }
    Ok(anchors)
}

/// Finds exactly one metadata-hash anchor in `bytes`.
pub fn find_unique_anchor(bytes: &[u8]) -> Result<Option<MetadataHashAnchor>> {
    let anchors = find_anchors(bytes)?;
    match anchors.as_slice() {
        [] => Ok(None),
        [anchor] => Ok(Some(*anchor)),
        _ => Err(Error::InvalidValue {
            what: "metadata hash anchor",
            value: format!("found {} anchors, expected at most one", anchors.len()),
        }),
    }
}

/// Patches the CBFS metadata and FMAP digests in an anchor-containing byte slice.
pub fn patch_unique_anchor(
    bytes: &mut [u8],
    metadata_digest: &[u8],
    fmap_digest: Option<&[u8]>,
) -> Result<MetadataHashAnchor> {
    let anchor = find_unique_anchor(bytes)?.ok_or(Error::InvalidValue {
        what: "metadata hash anchor",
        value: "not found".to_owned(),
    })?;
    patch_anchor(bytes, anchor, metadata_digest, fmap_digest)?;
    Ok(anchor)
}

/// Patches the CBFS metadata and optional FMAP digests for a known anchor.
pub fn patch_anchor(
    bytes: &mut [u8],
    anchor: MetadataHashAnchor,
    metadata_digest: &[u8],
    fmap_digest: Option<&[u8]>,
) -> Result<()> {
    check_digest_len(anchor, metadata_digest, "CBFS metadata hash")?;
    let cbfs_range = anchor.cbfs_hash_range();
    bytes
        .get_mut(cbfs_range.clone())
        .ok_or(Error::InvalidOffset {
            what: "metadata hash anchor",
            offset: cbfs_range.start,
        })?
        .copy_from_slice(metadata_digest);

    if let Some(fmap_digest) = fmap_digest {
        check_digest_len(anchor, fmap_digest, "FMAP hash")?;
        let fmap_range = anchor.fmap_hash_range();
        bytes
            .get_mut(fmap_range.clone())
            .ok_or(Error::InvalidOffset {
                what: "metadata hash anchor",
                offset: fmap_range.start,
            })?
            .copy_from_slice(fmap_digest);
    }

    Ok(())
}

/// Computes the C-compatible CBFS metadata digest for a CBFS region.
pub fn cbfs_metadata_digest(cbfs: &Cbfs<'_>, algorithm: HashAlgorithm) -> Result<Vec<u8>> {
    let mut metadata = Vec::new();
    for entry in cbfs.entries() {
        let entry = entry?;
        if matches!(entry.entry_type(), CbfsType::NULL | CbfsType::DELETED) {
            continue;
        }
        let start = entry.offset();
        let end = start
            .checked_add(entry.metadata_len())
            .ok_or(Error::InvalidOffset {
                what: "CBFS metadata",
                offset: start,
            })?;
        let bytes = cbfs.data().get(start..end).ok_or(Error::InvalidOffset {
            what: "CBFS metadata",
            offset: start,
        })?;
        metadata.extend_from_slice(bytes);
    }
    hash::digest(algorithm, &metadata)
}

/// Computes the FMAP digest over the serialized FMAP structure.
pub fn fmap_digest(image: &[u8], fmap: &Fmap, algorithm: HashAlgorithm) -> Result<Vec<u8>> {
    let size = fmap_serialized_size(fmap)?;
    let bytes = image
        .get(fmap.offset()..fmap.offset() + size)
        .ok_or(Error::InvalidOffset {
            what: "FMAP",
            offset: fmap.offset(),
        })?;
    hash::digest(algorithm, bytes)
}

fn parse_anchor_at(bytes: &[u8], offset: usize) -> Result<MetadataHashAnchor> {
    let algorithm = bytes
        .get(offset + ANCHOR_ALGORITHM_OFFSET)
        .copied()
        .ok_or(Error::InvalidOffset {
            what: "metadata hash anchor algorithm",
            offset,
        })
        .and_then(HashAlgorithm::from_raw)?;
    let digest_size = algorithm.digest_size();
    let active_end = offset
        .checked_add(ANCHOR_HASH_OFFSET)
        .and_then(|end| end.checked_add(digest_size * 2))
        .ok_or(Error::InvalidOffset {
            what: "metadata hash anchor",
            offset,
        })?;
    if active_end > bytes.len() {
        return Err(Error::truncated(
            "metadata hash anchor",
            offset,
            active_end - offset,
            bytes.len().saturating_sub(offset),
        ));
    }
    Ok(MetadataHashAnchor {
        offset,
        algorithm,
        digest_size,
    })
}

fn check_digest_len(anchor: MetadataHashAnchor, digest: &[u8], what: &'static str) -> Result<()> {
    if digest.len() != anchor.digest_size {
        return Err(Error::InvalidValue {
            what,
            value: format!(
                "{} digest has {} bytes, expected {}",
                anchor.algorithm,
                digest.len(),
                anchor.digest_size
            ),
        });
    }
    Ok(())
}

fn fmap_serialized_size(fmap: &Fmap) -> Result<usize> {
    crate::format::fmap::FMAP_HEADER_LEN
        .checked_add(
            fmap.areas()
                .len()
                .checked_mul(crate::format::fmap::FMAP_AREA_LEN)
                .ok_or(Error::InvalidValue {
                    what: "FMAP size",
                    value: "area count overflow".to_owned(),
                })?,
        )
        .ok_or(Error::InvalidValue {
            what: "FMAP size",
            value: "serialized size overflow".to_owned(),
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "vboot")]
    use crate::cbfs::writer;

    #[test]
    fn finds_and_patches_anchor() -> Result<()> {
        let mut bytes = vec![0xff; 160];
        bytes[7..15].copy_from_slice(METADATA_HASH_ANCHOR_MAGIC);
        bytes[18] = HashAlgorithm::Sha256.raw();

        let anchor = find_unique_anchor(&bytes)?.ok_or(Error::InvalidValue {
            what: "test anchor",
            value: "missing".to_owned(),
        })?;
        assert_eq!(anchor.offset(), 7);
        assert_eq!(anchor.algorithm(), HashAlgorithm::Sha256);

        patch_anchor(&mut bytes, anchor, &[0x11; 32], Some(&[0x22; 32]))?;
        assert_eq!(&bytes[anchor.cbfs_hash_range()], &[0x11; 32]);
        assert_eq!(&bytes[anchor.fmap_hash_range()], &[0x22; 32]);
        Ok(())
    }

    #[cfg(feature = "vboot")]
    #[test]
    fn hashes_cbfs_metadata_only_for_non_empty_entries() -> Result<()> {
        let mut region = vec![0xff; 256];
        writer::create_empty_region(&mut region)?;
        writer::add_raw_entry(&mut region, "a", CbfsType::RAW, b"first")?;
        writer::add_raw_entry(&mut region, "b", CbfsType::RAW, b"second")?;

        let cbfs = Cbfs::parse(&region)?;
        let digest = cbfs_metadata_digest(&cbfs, HashAlgorithm::Sha256)?;
        assert_eq!(digest.len(), 32);
        Ok(())
    }
}
