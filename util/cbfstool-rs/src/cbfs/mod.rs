// SPDX-License-Identifier: GPL-2.0-only

pub mod attributes;
#[cfg(feature = "compression")]
pub mod compression;
mod entry;
pub mod writer;

use crate::error::{Error, Result};
use crate::format::cbfs::{
    CBFS_ALIGNMENT, CBFS_FILE_HEADER_LEN, CBFS_FILE_MAGIC, CBFS_HEADER_MAGIC, CbfsHeader,
};
use zerocopy::FromBytes;

pub use entry::{CbfsEntry, CbfsType, Compression};

#[derive(Debug, Clone)]
pub struct Cbfs<'a> {
    data: &'a [u8],
    entries_offset: usize,
    align: usize,
}

impl<'a> Cbfs<'a> {
    pub fn parse(data: &'a [u8]) -> Result<Self> {
        Self::parse_with_header_offset(data, None)
    }

    pub fn parse_with_header_offset(data: &'a [u8], header_offset: Option<usize>) -> Result<Self> {
        if let Some(offset) = header_offset {
            let header = Header::parse(data, offset)?;
            return Ok(Self {
                data,
                entries_offset: header.entries_offset,
                align: header.align,
            });
        }

        if data.get(..CBFS_FILE_MAGIC.len()) == Some(CBFS_FILE_MAGIC.as_slice()) {
            return Ok(Self {
                data,
                entries_offset: 0,
                align: CBFS_ALIGNMENT,
            });
        }

        Err(Error::InvalidMagic {
            what: "CBFS",
            offset: 0,
        })
    }

    pub fn entries(&self) -> Entries<'a> {
        Entries {
            data: self.data,
            next_offset: self.entries_offset,
            align: self.align,
            done: false,
        }
    }

    pub fn find_entry(&self, name: &str) -> Result<Option<CbfsEntry<'a>>> {
        self.entries()
            .find_map(|entry| match entry {
                Ok(entry) if entry.name() == name => Some(Ok(entry)),
                Ok(_) => None,
                Err(err) => Some(Err(err)),
            })
            .transpose()
    }

    pub fn data(&self) -> &'a [u8] {
        self.data
    }

    pub fn alignment(&self) -> usize {
        self.align
    }
}

#[derive(Debug, Clone, Copy)]
struct Header {
    entries_offset: usize,
    align: usize,
}

impl Header {
    fn parse(data: &[u8], offset: usize) -> Result<Self> {
        let header_bytes = data
            .get(offset..offset + core::mem::size_of::<CbfsHeader>())
            .ok_or_else(|| {
                Error::truncated(
                    "CBFS header",
                    offset,
                    core::mem::size_of::<CbfsHeader>(),
                    data.len().saturating_sub(offset),
                )
            })?;
        let header =
            CbfsHeader::ref_from_bytes(header_bytes).map_err(|_| Error::InvalidOffset {
                what: "CBFS header",
                offset,
            })?;
        if header.magic.get() != CBFS_HEADER_MAGIC {
            return Err(Error::InvalidMagic {
                what: "CBFS header",
                offset,
            });
        }
        let align = header.align.get() as usize;
        let entries_offset = header.entries_offset.get() as usize;
        if align < CBFS_FILE_HEADER_LEN {
            return Err(Error::InvalidValue {
                what: "CBFS alignment",
                value: align.to_string(),
            });
        }
        if entries_offset >= data.len() {
            return Err(Error::InvalidOffset {
                what: "CBFS entries",
                offset: entries_offset,
            });
        }
        Ok(Self {
            entries_offset,
            align,
        })
    }
}

#[derive(Debug, Clone)]
pub struct Entries<'a> {
    data: &'a [u8],
    next_offset: usize,
    align: usize,
    done: bool,
}

impl<'a> Iterator for Entries<'a> {
    type Item = Result<CbfsEntry<'a>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done || self.next_offset >= self.data.len() {
            return None;
        }

        let offset = self.next_offset;
        if self.data.get(offset..offset + CBFS_FILE_MAGIC.len()) != Some(CBFS_FILE_MAGIC.as_slice())
        {
            self.done = true;
            return None;
        }

        match CbfsEntry::parse(self.data, offset, self.align) {
            Ok(entry) => {
                let next =
                    (offset + entry.data_offset() + entry.len()).next_multiple_of(self.align);
                if next <= offset || next > self.data.len() {
                    self.done = true;
                } else {
                    self.next_offset = next;
                }
                Some(Ok(entry))
            }
            Err(err) => {
                self.done = true;
                Some(Err(err))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_empty_region_entry() -> Result<()> {
        let mut data = vec![0xff; 128];
        data[..8].copy_from_slice(CBFS_FILE_MAGIC);
        data[8..12].copy_from_slice(&36_u32.to_be_bytes());
        data[12..16].copy_from_slice(&0xffff_ffff_u32.to_be_bytes());
        data[20..24].copy_from_slice(&28_u32.to_be_bytes());
        data[24] = 0;

        let cbfs = Cbfs::parse(&data)?;
        let entries = cbfs.entries().collect::<Result<Vec<_>>>()?;
        assert_eq!(entries.len(), 1);
        assert_eq!(entries[0].name(), "");
        assert_eq!(entries[0].len(), 36);
        Ok(())
    }

    #[test]
    fn compacts_removed_entries_to_trailing_space() -> Result<()> {
        let mut data = empty_cbfs_region(256);
        writer::add_raw_entry(&mut data, "first", CbfsType::RAW, &[1, 2])?;
        writer::add_raw_entry(&mut data, "second", CbfsType::RAW, &[3, 4])?;
        writer::remove_entry(&mut data, "first")?;

        writer::compact_region(&mut data)?;

        let cbfs = Cbfs::parse(&data)?;
        assert!(cbfs.find_entry("first")?.is_none());
        let second = cbfs
            .find_entry("second")?
            .ok_or_else(|| Error::EntryNotFound("second".to_owned()))?;
        assert_eq!(second.offset(), 0);
        assert_eq!(second.data(), &[3, 4]);
        Ok(())
    }

    #[test]
    fn copies_only_payload_entries_and_preserves_final_pointer_space() -> Result<()> {
        let mut source = empty_cbfs_region(256);
        writer::add_raw_entry(&mut source, "header", CbfsType::CBFSHEADER, &[0xaa])?;
        writer::add_raw_entry(&mut source, "payload", CbfsType::RAW, &[0xbb, 0xcc])?;
        let mut destination = empty_cbfs_region(256);
        destination[252..256].copy_from_slice(&[0xde, 0xad, 0xbe, 0xef]);

        writer::copy_region(&source, &mut destination)?;

        assert_eq!(&destination[252..256], &[0xde, 0xad, 0xbe, 0xef]);
        let cbfs = Cbfs::parse(&destination)?;
        assert!(cbfs.find_entry("header")?.is_none());
        let entries = cbfs.entries().collect::<Result<Vec<_>>>()?;
        let trailing_empty = entries
            .iter()
            .find(|entry| entry.entry_type() == CbfsType::NULL)
            .ok_or_else(|| Error::EntryNotFound("trailing empty entry".to_owned()))?;
        assert_eq!(trailing_empty.offset() + trailing_empty.total_len(), 252);
        let payload = cbfs
            .find_entry("payload")?
            .ok_or_else(|| Error::EntryNotFound("payload".to_owned()))?;
        assert_eq!(payload.data(), &[0xbb, 0xcc]);
        Ok(())
    }

    #[test]
    fn copies_entry_ending_at_reserved_pointer_space() -> Result<()> {
        let mut source = empty_cbfs_region(320);
        writer::add_raw_entry(&mut source, "payload", CbfsType::RAW, &[0x5a; 220])?;
        let mut destination = empty_cbfs_region(256);
        destination[252..256].copy_from_slice(&[0xde, 0xad, 0xbe, 0xef]);

        writer::copy_region(&source, &mut destination)?;

        assert_eq!(&destination[252..256], &[0xde, 0xad, 0xbe, 0xef]);
        let cbfs = Cbfs::parse(&destination)?;
        let entries = cbfs.entries().collect::<Result<Vec<_>>>()?;
        assert_eq!(entries.len(), 1);
        let payload = cbfs
            .find_entry("payload")?
            .ok_or_else(|| Error::EntryNotFound("payload".to_owned()))?;
        assert_eq!(payload.offset() + payload.total_len(), 252);
        assert_eq!(payload.data(), &[0x5a; 220]);
        Ok(())
    }

    #[test]
    fn copies_final_entry_in_unaligned_region() -> Result<()> {
        let mut source = empty_cbfs_region(320);
        writer::add_raw_entry(&mut source, "payload", CbfsType::RAW, &[0x5a; 218])?;
        let mut destination = empty_cbfs_region(254);
        destination[250..254].copy_from_slice(&[0xde, 0xad, 0xbe, 0xef]);

        writer::copy_region(&source, &mut destination)?;

        assert_eq!(&destination[250..254], &[0xde, 0xad, 0xbe, 0xef]);
        let cbfs = Cbfs::parse(&destination)?;
        let entries = cbfs.entries().collect::<Result<Vec<_>>>()?;
        assert_eq!(entries.len(), 1);
        let payload = cbfs
            .find_entry("payload")?
            .ok_or_else(|| Error::EntryNotFound("payload".to_owned()))?;
        assert_eq!(payload.offset() + payload.total_len(), 250);
        assert_eq!(payload.data(), &[0x5a; 218]);
        Ok(())
    }

    #[test]
    fn truncates_and_expands_trailing_space() -> Result<()> {
        let mut data = empty_cbfs_region(256);
        writer::add_raw_entry(&mut data, "payload", CbfsType::RAW, &[1, 2, 3, 4])?;

        let size = writer::truncate_region(&mut data)?;
        assert_eq!(size, 64);
        assert!(data[size..].iter().all(|byte| *byte == 0xff));
        let entries = Cbfs::parse(&data)?.entries().collect::<Result<Vec<_>>>()?;
        assert_eq!(entries.len(), 1);

        writer::expand_region(&mut data)?;
        let entries = Cbfs::parse(&data)?.entries().collect::<Result<Vec<_>>>()?;
        assert_eq!(entries.len(), 2);
        assert_eq!(entries[1].entry_type(), CbfsType::NULL);
        Ok(())
    }

    #[cfg(not(feature = "compression"))]
    #[test]
    fn rejects_compressed_entry_without_compression_feature() -> Result<()> {
        let mut data = empty_cbfs_region(256);

        let result = writer::add_entry(&mut data, "data", CbfsType::RAW, b"data", Compression::Lz4);

        assert!(matches!(
            result,
            Err(Error::UnsupportedCompression(kind)) if kind == Compression::Lz4.raw()
        ));
        Ok(())
    }

    #[cfg(feature = "compression")]
    #[test]
    fn adds_compressed_entry() -> Result<()> {
        let mut data = vec![0xff; 4096];
        data[..8].copy_from_slice(CBFS_FILE_MAGIC);
        data[8..12].copy_from_slice(&4068_u32.to_be_bytes());
        data[12..16].copy_from_slice(&0xffff_ffff_u32.to_be_bytes());
        data[20..24].copy_from_slice(&28_u32.to_be_bytes());
        data[24] = 0;

        let contents = b"compressible cbfs entry".repeat(64);
        writer::add_entry(
            &mut data,
            "data",
            CbfsType::RAW,
            &contents,
            Compression::Lz4,
        )?;

        let cbfs = Cbfs::parse(&data)?;
        let entry = cbfs
            .find_entry("data")?
            .ok_or_else(|| Error::EntryNotFound("data".to_owned()))?;
        assert_eq!(entry.compression(), Compression::Lz4);
        let decompressed =
            compression::decompress(entry.compression(), entry.data(), entry.decompressed_size())?;
        assert_eq!(decompressed, contents);
        Ok(())
    }

    #[test]
    fn adds_entry_at_fixed_offset() -> Result<()> {
        let mut data = empty_cbfs_region(512);

        writer::add_entry_with_attributes_at(
            &mut data,
            "placed",
            CbfsType::RAW,
            &[0xaa, 0xbb],
            Compression::None,
            &[],
            writer::EntryPlacement::Offset(128),
        )?;

        let cbfs = Cbfs::parse(&data)?;
        let entry = cbfs
            .find_entry("placed")?
            .ok_or_else(|| Error::EntryNotFound("placed".to_owned()))?;
        assert_eq!(entry.offset(), 128);
        assert_eq!(entry.data(), &[0xaa, 0xbb]);
        Ok(())
    }

    #[test]
    fn adds_entry_with_alignment() -> Result<()> {
        let mut data = empty_cbfs_region(512);

        writer::add_entry_with_attributes_at(
            &mut data,
            "aligned",
            CbfsType::RAW,
            &[0xcc],
            Compression::None,
            &[],
            writer::EntryPlacement::Alignment(128),
        )?;

        let cbfs = Cbfs::parse(&data)?;
        let entry = cbfs
            .find_entry("aligned")?
            .ok_or_else(|| Error::EntryNotFound("aligned".to_owned()))?;
        assert_eq!(entry.offset() % 128, 0);
        Ok(())
    }

    #[cfg(feature = "vboot")]
    #[test]
    fn adds_vboot_hash_attribute_for_stored_data() -> Result<()> {
        use crate::cbfs::attributes::AttributeTag;
        use crate::vboot::hash::{HashAlgorithm, digest};

        let mut data = empty_cbfs_region(512);
        writer::add_entry_with_attributes_and_hash_at(
            &mut data,
            "hashed",
            CbfsType::RAW,
            b"payload",
            Compression::None,
            &[],
            Some(HashAlgorithm::Sha256),
            writer::EntryPlacement::Any,
        )?;

        let cbfs = Cbfs::parse(&data)?;
        let entry = cbfs
            .find_entry("hashed")?
            .ok_or_else(|| Error::EntryNotFound("hashed".to_owned()))?;
        let attrs = entry.attributes().collect::<Result<Vec<_>>>()?;
        assert_eq!(attrs.len(), 1);
        assert_eq!(attrs[0].tag(), AttributeTag::HASH);
        assert_eq!(&attrs[0].payload()[..4], &[0, 0, 0, 2]);
        assert_eq!(
            &attrs[0].payload()[4..],
            digest(HashAlgorithm::Sha256, b"payload")?
        );
        Ok(())
    }

    fn empty_cbfs_region(size: usize) -> Vec<u8> {
        let mut data = vec![0xff; size];
        data[..8].copy_from_slice(CBFS_FILE_MAGIC);
        let payload_len = (size - 28) as u32;
        data[8..12].copy_from_slice(&payload_len.to_be_bytes());
        data[12..16].copy_from_slice(&CbfsType::NULL.raw().to_be_bytes());
        data[20..24].copy_from_slice(&28_u32.to_be_bytes());
        data[24] = 0;
        data
    }
}
