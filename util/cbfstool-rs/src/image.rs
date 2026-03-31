// SPDX-License-Identifier: GPL-2.0-only

use crate::cbfs::writer::{
    EntryPlacement, add_entry, add_entry_with_attributes_and_hash_at, add_raw_entry,
    compact_region, copy_region, create_empty_region, expand_region, remove_entry, truncate_region,
};
use crate::cbfs::{Cbfs, CbfsType, Compression};
use crate::error::{Error, Result};
use crate::fmap::{Fmap, FmapArea};
use crate::vboot::hash::HashAlgorithm;

#[derive(Debug, Clone)]
pub struct FirmwareImage<'a> {
    data: &'a [u8],
    fmap: Option<Fmap>,
}

impl<'a> FirmwareImage<'a> {
    pub fn parse(data: &'a [u8]) -> Result<Self> {
        let fmap = Fmap::find(data)?;
        Ok(Self { data, fmap })
    }

    pub fn data(&self) -> &'a [u8] {
        self.data
    }

    pub fn fmap(&self) -> Option<&Fmap> {
        self.fmap.as_ref()
    }

    pub fn regions(&self, include_readonly: bool) -> Vec<Region> {
        match &self.fmap {
            Some(fmap) => fmap
                .areas()
                .iter()
                .filter_map(|area| self.region_from_area(area).ok())
                .filter(|region| include_readonly || !region.is_read_only())
                .collect(),
            None => vec![Region {
                name: "COREBOOT".to_owned(),
                offset: 0,
                size: self.data.len(),
                is_cbfs: Cbfs::parse(self.data).is_ok(),
                read_only: false,
            }],
        }
    }

    pub fn region(&self, name: Option<&str>) -> Result<Region> {
        match (&self.fmap, name) {
            (Some(fmap), Some(name)) => {
                let area = fmap
                    .find_area(name)
                    .ok_or_else(|| Error::RegionNotFound(name.to_owned()))?;
                self.region_from_area(area)
            }
            (Some(fmap), None) => {
                let area = fmap.find_area("COREBOOT").ok_or(Error::NoCbfsRegion)?;
                self.region_from_area(area)
            }
            (None, Some(name)) if name != "COREBOOT" => Err(Error::RegionNotFound(name.to_owned())),
            (None, _) => Ok(Region {
                name: "COREBOOT".to_owned(),
                offset: 0,
                size: self.data.len(),
                is_cbfs: Cbfs::parse(self.data).is_ok(),
                read_only: false,
            }),
        }
    }

    pub fn region_bytes(&self, region: &Region) -> Result<&'a [u8]> {
        self.data
            .get(region.offset..region.offset + region.size)
            .ok_or(Error::InvalidOffset {
                what: "region",
                offset: region.offset,
            })
    }

    pub fn cbfs(&self, name: Option<&str>) -> Result<Cbfs<'a>> {
        let region = self.region(name)?;
        if !region.is_cbfs {
            return Err(Error::NoCbfsRegion);
        }
        Cbfs::parse(self.region_bytes(&region)?)
    }

    fn region_from_area(&self, area: &FmapArea) -> Result<Region> {
        let offset = area.offset() as usize;
        let size = area.size() as usize;
        let bytes = self
            .data
            .get(offset..offset + size)
            .ok_or(Error::InvalidOffset {
                what: "FMAP area",
                offset,
            })?;
        Ok(Region {
            name: area.name().to_owned(),
            offset,
            size,
            is_cbfs: Cbfs::parse(bytes).is_ok(),
            read_only: self.area_is_read_only(area),
        })
    }

    fn area_is_read_only(&self, area: &FmapArea) -> bool {
        if area.flags().is_read_only() || area.name() == "FMAP" {
            return true;
        }
        let Some(fmap) = &self.fmap else {
            return false;
        };
        let start = area.offset();
        let Some(end) = area.offset().checked_add(area.size()) else {
            return true;
        };
        fmap.areas().iter().any(|other| {
            if other.index() == area.index() {
                return false;
            }
            let other_start = other.offset();
            let Some(other_end) = other.offset().checked_add(other.size()) else {
                return false;
            };
            start <= other_start && other_end <= end
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Region {
    name: String,
    offset: usize,
    size: usize,
    is_cbfs: bool,
    read_only: bool,
}

#[derive(Debug, Clone)]
pub struct OwnedFirmwareImage {
    data: Vec<u8>,
}

#[derive(Debug, Clone, Copy)]
pub struct AddEntryOptions<'a> {
    pub region_name: Option<&'a str>,
    pub name: &'a str,
    pub ty: CbfsType,
    pub contents: &'a [u8],
    pub compression: Compression,
    pub extra_attributes: &'a [u8],
    pub hash_algorithm: Option<HashAlgorithm>,
    pub placement: EntryPlacement,
}

impl<'a> AddEntryOptions<'a> {
    pub fn new(name: &'a str, ty: CbfsType, contents: &'a [u8]) -> Self {
        Self {
            region_name: None,
            name,
            ty,
            contents,
            compression: Compression::None,
            extra_attributes: &[],
            hash_algorithm: None,
            placement: EntryPlacement::Any,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegionWriteMode {
    Exact,
    FillUpward { fill: Option<u8> },
    FillDownward { fill: Option<u8> },
}

impl OwnedFirmwareImage {
    pub fn from_vec(data: Vec<u8>) -> Self {
        Self { data }
    }

    /// Creates an image containing a single empty, headerless CBFS region.
    pub fn create_empty_cbfs(size: usize) -> Result<Self> {
        let mut data = vec![0xff; size];
        create_empty_region(&mut data)?;
        Ok(Self { data })
    }

    /// Creates an FMAP-partitioned firmware image from serialized FMAP bytes.
    pub fn create_partitioned(fmap_bytes: &[u8], cbfs_regions: &[&str]) -> Result<Self> {
        let fmap = Fmap::parse_at(fmap_bytes, 0)?;
        let mut data = vec![0xff; fmap.size() as usize];

        for region_name in cbfs_regions {
            let area = fmap
                .find_area(region_name)
                .ok_or_else(|| Error::RegionNotFound((*region_name).to_owned()))?;
            let offset = area.offset() as usize;
            let size = area.size() as usize;
            let end = offset.checked_add(size).ok_or(Error::InvalidOffset {
                what: "CBFS region",
                offset,
            })?;
            let region = data.get_mut(offset..end).ok_or(Error::InvalidOffset {
                what: "CBFS region",
                offset,
            })?;
            create_empty_region(region)?;
        }

        let fmap_area = fmap.find_area("FMAP").ok_or_else(|| Error::InvalidValue {
            what: "FMAP",
            value: "flashmap does not contain an FMAP area".to_owned(),
        })?;
        let fmap_offset = fmap_area.offset() as usize;
        let fmap_area_size = fmap_area.size() as usize;
        if fmap_bytes.len() > fmap_area_size {
            return Err(Error::InvalidValue {
                what: "FMAP size",
                value: format!(
                    "{} bytes does not fit in FMAP area of {fmap_area_size} bytes",
                    fmap_bytes.len()
                ),
            });
        }
        let fmap_end = fmap_offset
            .checked_add(fmap_bytes.len())
            .ok_or(Error::InvalidOffset {
                what: "FMAP area",
                offset: fmap_offset,
            })?;
        let fmap_destination = data
            .get_mut(fmap_offset..fmap_end)
            .ok_or(Error::InvalidOffset {
                what: "FMAP area",
                offset: fmap_offset,
            })?;
        fmap_destination.copy_from_slice(fmap_bytes);

        Ok(Self { data })
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.data
    }

    pub fn as_mut_bytes(&mut self) -> &mut [u8] {
        &mut self.data
    }

    pub fn into_vec(self) -> Vec<u8> {
        self.data
    }

    pub fn add_raw_entry(
        &mut self,
        region_name: Option<&str>,
        name: &str,
        ty: CbfsType,
        contents: &[u8],
    ) -> Result<()> {
        let image = FirmwareImage::parse(&self.data)?;
        let region = image.region(region_name)?;
        let bytes = self
            .data
            .get_mut(region.offset..region.offset + region.size)
            .ok_or(Error::InvalidOffset {
                what: "region",
                offset: region.offset,
            })?;
        add_raw_entry(bytes, name, ty, contents)
    }

    pub fn add_entry(
        &mut self,
        region_name: Option<&str>,
        name: &str,
        ty: CbfsType,
        contents: &[u8],
        compression: Compression,
    ) -> Result<()> {
        let image = FirmwareImage::parse(&self.data)?;
        let region = image.region(region_name)?;
        let bytes = self
            .data
            .get_mut(region.offset..region.offset + region.size)
            .ok_or(Error::InvalidOffset {
                what: "region",
                offset: region.offset,
            })?;
        add_entry(bytes, name, ty, contents, compression)
    }

    pub fn add_entry_with_attributes(
        &mut self,
        region_name: Option<&str>,
        name: &str,
        ty: CbfsType,
        contents: &[u8],
        compression: Compression,
        extra_attributes: &[u8],
    ) -> Result<()> {
        let mut options = AddEntryOptions::new(name, ty, contents);
        options.region_name = region_name;
        options.compression = compression;
        options.extra_attributes = extra_attributes;
        self.add_entry_with_options(options)
    }

    pub fn add_entry_with_options(&mut self, options: AddEntryOptions<'_>) -> Result<()> {
        let image = FirmwareImage::parse(&self.data)?;
        let region = image.region(options.region_name)?;
        let bytes = self
            .data
            .get_mut(region.offset..region.offset + region.size)
            .ok_or(Error::InvalidOffset {
                what: "region",
                offset: region.offset,
            })?;
        add_entry_with_attributes_and_hash_at(
            bytes,
            options.name,
            options.ty,
            options.contents,
            options.compression,
            options.extra_attributes,
            options.hash_algorithm,
            options.placement,
        )
    }

    pub fn remove_entry(&mut self, region_name: Option<&str>, name: &str) -> Result<()> {
        let image = FirmwareImage::parse(&self.data)?;
        let region = image.region(region_name)?;
        let bytes = self
            .data
            .get_mut(region.offset..region.offset + region.size)
            .ok_or(Error::InvalidOffset {
                what: "region",
                offset: region.offset,
            })?;
        remove_entry(bytes, name)
    }

    pub fn compact_region(&mut self, region_name: Option<&str>) -> Result<()> {
        let image = FirmwareImage::parse(&self.data)?;
        let region = image.region(region_name)?;
        let bytes = self
            .data
            .get_mut(region.offset..region.offset + region.size)
            .ok_or(Error::InvalidOffset {
                what: "region",
                offset: region.offset,
            })?;
        compact_region(bytes)
    }

    pub fn copy_region(
        &mut self,
        source_region_name: &str,
        destination_region_name: Option<&str>,
    ) -> Result<()> {
        let image = FirmwareImage::parse(&self.data)?;
        let source_region = image.region(Some(source_region_name))?;
        let destination_region = image.region(destination_region_name)?;
        reject_overlapping_regions(&source_region, &destination_region)?;
        let source = image.region_bytes(&source_region)?.to_vec();
        let destination = self
            .data
            .get_mut(destination_region.offset..destination_region.offset + destination_region.size)
            .ok_or(Error::InvalidOffset {
                what: "region",
                offset: destination_region.offset,
            })?;
        copy_region(&source, destination)
    }

    pub fn expand_region(&mut self, region_name: Option<&str>) -> Result<()> {
        let image = FirmwareImage::parse(&self.data)?;
        let region = image.region(region_name)?;
        let bytes = self
            .data
            .get_mut(region.offset..region.offset + region.size)
            .ok_or(Error::InvalidOffset {
                what: "region",
                offset: region.offset,
            })?;
        expand_region(bytes)
    }

    pub fn truncate_region(&mut self, region_name: Option<&str>) -> Result<usize> {
        let image = FirmwareImage::parse(&self.data)?;
        let region = image.region(region_name)?;
        let bytes = self
            .data
            .get_mut(region.offset..region.offset + region.size)
            .ok_or(Error::InvalidOffset {
                what: "region",
                offset: region.offset,
            })?;
        truncate_region(bytes)
    }

    pub fn write_region(
        &mut self,
        region_name: Option<&str>,
        contents: &[u8],
        mode: RegionWriteMode,
    ) -> Result<()> {
        let image = FirmwareImage::parse(&self.data)?;
        let region = image.region(region_name)?;
        let offset = match mode {
            RegionWriteMode::Exact => {
                if contents.len() != region.size {
                    return Err(Error::InvalidValue {
                        what: "region write size",
                        value: format!(
                            "input has {} bytes but target region has {} bytes",
                            contents.len(),
                            region.size
                        ),
                    });
                }
                0
            }
            RegionWriteMode::FillUpward { .. } => {
                if contents.len() > region.size {
                    return Err(Error::InvalidValue {
                        what: "region write size",
                        value: format!(
                            "input has {} bytes but target region has {} bytes",
                            contents.len(),
                            region.size
                        ),
                    });
                }
                0
            }
            RegionWriteMode::FillDownward { .. } => {
                if contents.len() > region.size {
                    return Err(Error::InvalidValue {
                        what: "region write size",
                        value: format!(
                            "input has {} bytes but target region has {} bytes",
                            contents.len(),
                            region.size
                        ),
                    });
                }
                region.size - contents.len()
            }
        };

        let bytes = self
            .data
            .get_mut(region.offset..region.offset + region.size)
            .ok_or(Error::InvalidOffset {
                what: "region",
                offset: region.offset,
            })?;
        match mode {
            RegionWriteMode::Exact => {}
            RegionWriteMode::FillUpward { fill } | RegionWriteMode::FillDownward { fill } => {
                if let Some(fill) = fill {
                    bytes.fill(fill);
                }
            }
        }
        bytes[offset..offset + contents.len()].copy_from_slice(contents);
        Ok(())
    }
}

fn reject_overlapping_regions(source: &Region, destination: &Region) -> Result<()> {
    let source_end = source
        .offset
        .checked_add(source.size)
        .ok_or(Error::InvalidOffset {
            what: "source region",
            offset: source.offset,
        })?;
    let destination_end =
        destination
            .offset
            .checked_add(destination.size)
            .ok_or(Error::InvalidOffset {
                what: "destination region",
                offset: destination.offset,
            })?;

    if source.offset < destination_end && destination.offset < source_end {
        return Err(Error::InvalidValue {
            what: "copy regions",
            value: format!(
                "source region '{}' overlaps destination region '{}'",
                source.name, destination.name
            ),
        });
    }

    Ok(())
}

impl Region {
    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn offset(&self) -> usize {
        self.offset
    }

    pub fn size(&self) -> usize {
        self.size
    }

    pub fn is_cbfs(&self) -> bool {
        self.is_cbfs
    }

    pub fn is_read_only(&self) -> bool {
        self.read_only
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::format::fmap::{FMAP_AREA_LEN, FMAP_HEADER_LEN, FMAP_SIGNATURE};

    #[test]
    fn creates_empty_cbfs_image() -> Result<()> {
        let image = OwnedFirmwareImage::create_empty_cbfs(128)?;
        let parsed = FirmwareImage::parse(image.as_bytes())?;
        let cbfs = parsed.cbfs(None)?;
        let entries = cbfs.entries().collect::<Result<Vec<_>>>()?;
        assert_eq!(entries.len(), 1);
        assert_eq!(entries[0].entry_type(), CbfsType::NULL);
        Ok(())
    }

    #[test]
    fn writes_whole_region_exactly() -> Result<()> {
        let mut image = OwnedFirmwareImage::from_vec(vec![0xff; 4]);
        image.write_region(None, &[1, 2, 3, 4], RegionWriteMode::Exact)?;
        assert_eq!(image.as_bytes(), &[1, 2, 3, 4]);
        Ok(())
    }

    #[test]
    fn writes_partial_region_downward_with_fill() -> Result<()> {
        let mut image = OwnedFirmwareImage::from_vec(vec![0xff; 6]);
        image.write_region(
            None,
            &[1, 2],
            RegionWriteMode::FillDownward { fill: Some(0) },
        )?;
        assert_eq!(image.as_bytes(), &[0, 0, 0, 0, 1, 2]);
        Ok(())
    }

    #[test]
    fn rejects_overlapping_copy_regions_without_modifying_image() -> Result<()> {
        let data = fmap_image_with_overlapping_cbfs_regions();
        let original = data.clone();
        let mut image = OwnedFirmwareImage::from_vec(data);

        let result = image.copy_region("SOURCE", Some("DEST"));

        assert!(matches!(
            result,
            Err(Error::InvalidValue {
                what: "copy regions",
                ..
            })
        ));
        assert_eq!(image.as_bytes(), original.as_slice());
        Ok(())
    }

    fn fmap_image_with_overlapping_cbfs_regions() -> Vec<u8> {
        const FMAP_OFFSET: usize = 384;
        const IMAGE_LEN: usize = FMAP_OFFSET + FMAP_HEADER_LEN + 2 * FMAP_AREA_LEN;

        let mut data = vec![0xff; IMAGE_LEN];
        data[..8].copy_from_slice(b"LARCHIVE");
        data[8..12].copy_from_slice(&228_u32.to_be_bytes());
        data[12..16].copy_from_slice(&CbfsType::NULL.raw().to_be_bytes());
        data[20..24].copy_from_slice(&28_u32.to_be_bytes());
        data[24] = 0;

        let fmap = FMAP_OFFSET;
        data[fmap..fmap + FMAP_SIGNATURE.len()].copy_from_slice(FMAP_SIGNATURE);
        data[fmap + 8] = 1;
        data[fmap + 10..fmap + 18].copy_from_slice(&0_u64.to_le_bytes());
        data[fmap + 18..fmap + 22].copy_from_slice(&(IMAGE_LEN as u32).to_le_bytes());
        write_fmap_name(&mut data[fmap + 22..fmap + 54], "FLASH");
        data[fmap + 54..fmap + 56].copy_from_slice(&2_u16.to_le_bytes());

        let first_area = fmap + FMAP_HEADER_LEN;
        write_fmap_area(
            &mut data[first_area..first_area + FMAP_AREA_LEN],
            0,
            256,
            "SOURCE",
        );
        let second_area = first_area + FMAP_AREA_LEN;
        write_fmap_area(
            &mut data[second_area..second_area + FMAP_AREA_LEN],
            128,
            256,
            "DEST",
        );
        data
    }

    fn write_fmap_area(area: &mut [u8], offset: u32, size: u32, name: &str) {
        area[0..4].copy_from_slice(&offset.to_le_bytes());
        area[4..8].copy_from_slice(&size.to_le_bytes());
        write_fmap_name(&mut area[8..40], name);
        area[40..42].copy_from_slice(&0_u16.to_le_bytes());
    }

    fn write_fmap_name(destination: &mut [u8], name: &str) {
        destination.fill(0);
        let name = name.as_bytes();
        destination[..name.len()].copy_from_slice(name);
    }
}
