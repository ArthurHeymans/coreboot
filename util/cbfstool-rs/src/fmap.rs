// SPDX-License-Identifier: GPL-2.0-only

use crate::error::{Error, Result};
use crate::format::fmap::{
    FMAP_AREA_COMPRESSED, FMAP_AREA_LEN, FMAP_AREA_PRESERVE, FMAP_AREA_RO, FMAP_AREA_STATIC,
    FMAP_HEADER_LEN, FMAP_SIGNATURE, FmapAreaHeader, FmapHeader,
};
use crate::util::nul_terminated_string;

use zerocopy::FromBytes;

#[derive(Debug, Clone)]
pub struct Fmap {
    offset: usize,
    base: u64,
    size: u32,
    name: String,
    areas: Vec<FmapArea>,
}

impl Fmap {
    pub fn find(data: &[u8]) -> Result<Option<Self>> {
        let Some(offset) = data
            .windows(FMAP_SIGNATURE.len())
            .position(|w| w == FMAP_SIGNATURE)
        else {
            return Ok(None);
        };
        Self::parse_at(data, offset).map(Some)
    }

    pub fn parse_at(data: &[u8], offset: usize) -> Result<Self> {
        if data.get(offset..offset + FMAP_SIGNATURE.len()) != Some(FMAP_SIGNATURE.as_slice()) {
            return Err(Error::InvalidMagic {
                what: "FMAP",
                offset,
            });
        }
        let header = data.get(offset..offset + FMAP_HEADER_LEN).ok_or_else(|| {
            Error::truncated(
                "FMAP header",
                offset,
                FMAP_HEADER_LEN,
                data.len().saturating_sub(offset),
            )
        })?;
        let header = FmapHeader::ref_from_bytes(header).map_err(|_| Error::InvalidOffset {
            what: "FMAP header",
            offset,
        })?;
        let base = header.base.get();
        let size = header.size.get();
        let name = nul_terminated_string(&header.name);
        let nareas = header.nareas.get() as usize;
        let areas_len = nareas
            .checked_mul(FMAP_AREA_LEN)
            .ok_or(Error::InvalidOffset {
                what: "FMAP areas",
                offset,
            })?;
        let areas_start = offset + FMAP_HEADER_LEN;
        let areas_end = areas_start + areas_len;
        let areas_bytes = data.get(areas_start..areas_end).ok_or_else(|| {
            Error::truncated(
                "FMAP areas",
                areas_start,
                areas_len,
                data.len().saturating_sub(areas_start),
            )
        })?;

        let areas = areas_bytes
            .chunks_exact(FMAP_AREA_LEN)
            .enumerate()
            .map(|(idx, area)| {
                let area =
                    FmapAreaHeader::ref_from_bytes(area).map_err(|_| Error::InvalidOffset {
                        what: "FMAP area",
                        offset: areas_start + idx * FMAP_AREA_LEN,
                    })?;
                Ok(FmapArea {
                    offset: area.offset.get(),
                    size: area.size.get(),
                    name: nul_terminated_string(&area.name),
                    flags: FmapFlags(area.flags.get()),
                    index: idx,
                })
            })
            .collect::<Result<Vec<_>>>()?;

        Ok(Self {
            offset,
            base,
            size,
            name,
            areas,
        })
    }

    pub fn offset(&self) -> usize {
        self.offset
    }

    pub fn base(&self) -> u64 {
        self.base
    }

    pub fn size(&self) -> u32 {
        self.size
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn areas(&self) -> &[FmapArea] {
        &self.areas
    }

    pub fn find_area(&self, name: &str) -> Option<&FmapArea> {
        self.areas.iter().find(|area| area.name == name)
    }
}

#[derive(Debug, Clone)]
pub struct FmapArea {
    offset: u32,
    size: u32,
    name: String,
    flags: FmapFlags,
    index: usize,
}

impl FmapArea {
    pub fn offset(&self) -> u32 {
        self.offset
    }

    pub fn size(&self) -> u32 {
        self.size
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn flags(&self) -> FmapFlags {
        self.flags
    }

    pub fn index(&self) -> usize {
        self.index
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FmapFlags(u16);

impl FmapFlags {
    pub fn raw(self) -> u16 {
        self.0
    }

    pub fn is_static(self) -> bool {
        self.0 & FMAP_AREA_STATIC != 0
    }

    pub fn is_compressed(self) -> bool {
        self.0 & FMAP_AREA_COMPRESSED != 0
    }

    pub fn is_read_only(self) -> bool {
        self.0 & FMAP_AREA_RO != 0
    }

    pub fn is_preserve(self) -> bool {
        self.0 & FMAP_AREA_PRESERVE != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_fmap_is_none() -> Result<()> {
        assert!(Fmap::find(b"not a flash map")?.is_none());
        Ok(())
    }
}
