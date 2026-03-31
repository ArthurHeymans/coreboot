// SPDX-License-Identifier: GPL-2.0-only

//! Builders and parsers for CBFS file attributes.
//!
//! The functions in this module only serialize/deserialise attribute metadata.
//! They do not perform file-system I/O, so they can be reused by front-ends that
//! want to construct CBFS entries without shelling out to `cbfstool-rs`.

use crate::cbfs::Compression;
use crate::error::{Error, Result};
use crate::format::cbfs::{
    CBFS_ATTRIBUTE_ALIGN, CBFS_FILE_ATTR_TAG_ALIGNMENT, CBFS_FILE_ATTR_TAG_COMPRESSION,
    CBFS_FILE_ATTR_TAG_HASH, CBFS_FILE_ATTR_TAG_IBB, CBFS_FILE_ATTR_TAG_PADDING,
    CBFS_FILE_ATTR_TAG_POSITION, CBFS_FILE_ATTR_TAG_STAGEHEADER, CBFS_FILE_ATTR_TAG_UNUSED,
    CBFS_FILE_ATTR_TAG_UNUSED2,
};
use crate::vboot::hash::{HashAlgorithm, Vb2Hash};

/// Identifies a CBFS attribute tag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AttributeTag(u32);

impl AttributeTag {
    /// Marks whole-file compression metadata.
    pub const COMPRESSION: Self = Self(CBFS_FILE_ATTR_TAG_COMPRESSION);
    /// Marks a verified-boot hash attribute.
    pub const HASH: Self = Self(CBFS_FILE_ATTR_TAG_HASH);
    /// Marks a fixed CBFS-region-relative placement.
    pub const POSITION: Self = Self(CBFS_FILE_ATTR_TAG_POSITION);
    /// Marks a requested placement alignment.
    pub const ALIGNMENT: Self = Self(CBFS_FILE_ATTR_TAG_ALIGNMENT);
    /// Marks an initial boot block file.
    pub const IBB: Self = Self(CBFS_FILE_ATTR_TAG_IBB);
    /// Marks metadata padding reserved for generated attributes.
    pub const PADDING: Self = Self(CBFS_FILE_ATTR_TAG_PADDING);
    /// Marks stage loader metadata.
    pub const STAGEHEADER: Self = Self(CBFS_FILE_ATTR_TAG_STAGEHEADER);

    /// Creates a tag from its raw big-endian-on-flash value.
    pub fn new(raw: u32) -> Self {
        Self(raw)
    }

    /// Returns the raw tag value used on flash.
    pub fn raw(self) -> u32 {
        self.0
    }

    /// Returns whether this tag terminates an attribute list.
    pub fn is_unused(self) -> bool {
        self.0 == CBFS_FILE_ATTR_TAG_UNUSED || self.0 == CBFS_FILE_ATTR_TAG_UNUSED2
    }
}

/// A borrowed CBFS attribute.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Attribute<'a> {
    tag: AttributeTag,
    bytes: &'a [u8],
}

impl<'a> Attribute<'a> {
    /// Returns the attribute tag.
    pub fn tag(self) -> AttributeTag {
        self.tag
    }

    /// Returns the full serialized attribute, including tag and length.
    pub fn bytes(self) -> &'a [u8] {
        self.bytes
    }

    /// Returns the payload bytes after the common tag and length header.
    pub fn payload(self) -> &'a [u8] {
        &self.bytes[8..]
    }
}

/// Iterates over serialized CBFS attributes.
#[derive(Debug, Clone)]
pub struct Attributes<'a> {
    bytes: &'a [u8],
    cursor: usize,
    done: bool,
}

impl<'a> Attributes<'a> {
    /// Creates an iterator over an attribute byte range.
    pub fn new(bytes: &'a [u8]) -> Self {
        Self {
            bytes,
            cursor: 0,
            done: false,
        }
    }
}

impl<'a> Iterator for Attributes<'a> {
    type Item = Result<Attribute<'a>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done || self.cursor + 8 > self.bytes.len() {
            return None;
        }

        let header = self.bytes.get(self.cursor..self.cursor + 8)?;
        let tag = u32::from_be_bytes(header[0..4].try_into().ok()?);
        if AttributeTag::new(tag).is_unused() {
            self.done = true;
            return None;
        }
        let len = u32::from_be_bytes(header[4..8].try_into().ok()?) as usize;
        if len < 8 || !len.is_multiple_of(CBFS_ATTRIBUTE_ALIGN) {
            self.done = true;
            return Some(Err(Error::InvalidValue {
                what: "CBFS attribute length",
                value: len.to_string(),
            }));
        }
        let end = match self.cursor.checked_add(len) {
            Some(end) if end <= self.bytes.len() => end,
            _ => {
                self.done = true;
                return Some(Err(Error::InvalidOffset {
                    what: "CBFS attribute",
                    offset: self.cursor,
                }));
            }
        };
        let bytes = &self.bytes[self.cursor..end];
        self.cursor = end;
        Some(Ok(Attribute {
            tag: AttributeTag::new(tag),
            bytes,
        }))
    }
}

/// Builds a serialized CBFS attribute list.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct AttributeBuilder {
    bytes: Vec<u8>,
}

impl AttributeBuilder {
    /// Creates an empty attribute builder.
    pub fn new() -> Self {
        Self::default()
    }

    /// Appends an already-serialized attribute.
    pub fn push_serialized(&mut self, bytes: &[u8]) -> Result<&mut Self> {
        if !bytes.len().is_multiple_of(CBFS_ATTRIBUTE_ALIGN) {
            return Err(Error::InvalidValue {
                what: "CBFS attribute",
                value: format!("length {} is not 4-byte aligned", bytes.len()),
            });
        }
        let mut attrs = Attributes::new(bytes);
        for attr in attrs.by_ref() {
            attr?;
        }
        if attrs.cursor != bytes.len() {
            return Err(Error::InvalidOffset {
                what: "CBFS attribute",
                offset: attrs.cursor,
            });
        }
        self.bytes.extend_from_slice(bytes);
        Ok(self)
    }

    /// Appends an arbitrary attribute payload.
    pub fn push_raw(&mut self, tag: AttributeTag, payload: &[u8]) -> Result<&mut Self> {
        let len = 8_usize
            .checked_add(payload.len())
            .ok_or(Error::InvalidValue {
                what: "CBFS attribute length",
                value: "overflow".to_owned(),
            })?
            .next_multiple_of(CBFS_ATTRIBUTE_ALIGN);
        push_be_u32(&mut self.bytes, tag.raw());
        push_be_u32(&mut self.bytes, checked_u32(len, "CBFS attribute length")?);
        self.bytes.extend_from_slice(payload);
        self.bytes
            .resize(self.bytes.len() + (len - 8 - payload.len()), 0xff);
        Ok(self)
    }

    /// Appends a 32-bit attribute payload.
    pub fn push_u32(&mut self, tag: AttributeTag, value: u32) -> Result<&mut Self> {
        self.push_raw(tag, &value.to_be_bytes())
    }

    /// Appends a compression attribute.
    pub fn compression(
        &mut self,
        compression: Compression,
        decompressed_size: usize,
    ) -> Result<&mut Self> {
        let mut payload = Vec::with_capacity(8);
        push_be_u32(&mut payload, compression.raw());
        push_be_u32(
            &mut payload,
            checked_u32(decompressed_size, "CBFS decompressed size")?,
        );
        self.push_raw(AttributeTag::COMPRESSION, &payload)
    }

    /// Appends a fixed-position attribute.
    pub fn position(&mut self, position: u32) -> Result<&mut Self> {
        self.push_u32(AttributeTag::POSITION, position)
    }

    /// Appends an alignment attribute.
    pub fn alignment(&mut self, alignment: u32) -> Result<&mut Self> {
        self.push_u32(AttributeTag::ALIGNMENT, alignment)
    }

    /// Appends an initial-boot-block marker attribute.
    pub fn ibb(&mut self) -> Result<&mut Self> {
        self.push_raw(AttributeTag::IBB, &[])
    }

    /// Appends padding bytes that later generators may replace with real attributes.
    pub fn padding(&mut self, size: usize) -> Result<&mut Self> {
        let size = size.next_multiple_of(CBFS_ATTRIBUTE_ALIGN);
        if size < 8 {
            return Err(Error::InvalidValue {
                what: "CBFS padding attribute",
                value: "must be at least 8 bytes".to_owned(),
            });
        }
        push_be_u32(&mut self.bytes, AttributeTag::PADDING.raw());
        push_be_u32(
            &mut self.bytes,
            checked_u32(size, "CBFS padding attribute")?,
        );
        self.bytes.resize(self.bytes.len() + size - 8, 0xff);
        Ok(self)
    }

    /// Appends a stage-header attribute.
    pub fn stage_header(
        &mut self,
        load_address: u64,
        entry_offset: u32,
        memory_length: u32,
    ) -> Result<&mut Self> {
        let mut payload = Vec::with_capacity(16);
        payload.extend_from_slice(&load_address.to_be_bytes());
        push_be_u32(&mut payload, entry_offset);
        push_be_u32(&mut payload, memory_length);
        self.push_raw(AttributeTag::STAGEHEADER, &payload)
    }

    /// Appends a vboot hash attribute.
    pub fn hash(&mut self, algorithm: HashAlgorithm, digest: &[u8]) -> Result<&mut Self> {
        let hash = Vb2Hash::new(algorithm, digest)?;
        self.push_raw(AttributeTag::HASH, &hash.to_bytes())
    }

    /// Returns the serialized bytes collected so far.
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// Consumes the builder and returns serialized bytes.
    pub fn into_bytes(self) -> Vec<u8> {
        self.bytes
    }
}

/// Creates a serialized stage-header attribute.
pub fn stage_header(load_address: u64, entry_offset: u32, memory_length: u32) -> Result<Vec<u8>> {
    let mut builder = AttributeBuilder::new();
    builder.stage_header(load_address, entry_offset, memory_length)?;
    Ok(builder.into_bytes())
}

fn push_be_u32(output: &mut Vec<u8>, value: u32) {
    output.extend_from_slice(&value.to_be_bytes());
}

fn checked_u32(value: usize, what: &'static str) -> Result<u32> {
    u32::try_from(value).map_err(|_| Error::InvalidValue {
        what,
        value: value.to_string(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builds_stage_header_attribute() -> Result<()> {
        let bytes = stage_header(0x1122_3344_5566_7788, 0x99aa_bbcc, 0xddee_ff00)?;
        let attrs = Attributes::new(&bytes).collect::<Result<Vec<_>>>()?;
        assert_eq!(attrs.len(), 1);
        assert_eq!(attrs[0].tag(), AttributeTag::STAGEHEADER);
        assert_eq!(attrs[0].bytes().len(), 24);
        assert_eq!(
            &attrs[0].payload()[..8],
            &0x1122_3344_5566_7788_u64.to_be_bytes()
        );
        Ok(())
    }

    #[test]
    fn reserves_padding_attribute() -> Result<()> {
        let mut builder = AttributeBuilder::new();
        builder.padding(9)?;
        let bytes = builder.into_bytes();
        assert_eq!(bytes.len(), 12);
        let attrs = Attributes::new(&bytes).collect::<Result<Vec<_>>>()?;
        assert_eq!(attrs[0].tag(), AttributeTag::PADDING);
        Ok(())
    }

    #[test]
    fn builds_vboot_hash_attribute() -> Result<()> {
        let mut builder = AttributeBuilder::new();
        builder.hash(HashAlgorithm::Sha256, &[0x5a; 32])?;
        let bytes = builder.into_bytes();
        let attrs = Attributes::new(&bytes).collect::<Result<Vec<_>>>()?;
        assert_eq!(attrs.len(), 1);
        assert_eq!(attrs[0].tag(), AttributeTag::HASH);
        assert_eq!(&attrs[0].payload()[..4], &[0, 0, 0, 2]);
        assert_eq!(&attrs[0].payload()[4..36], &[0x5a; 32]);
        Ok(())
    }
}
