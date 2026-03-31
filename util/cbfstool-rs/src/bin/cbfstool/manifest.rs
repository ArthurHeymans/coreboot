// SPDX-License-Identifier: GPL-2.0-only

use std::fs;
use std::path::{Path, PathBuf};
use std::str::FromStr;

use cbfstool_rs::cbfs::attributes;
use cbfstool_rs::cbfs::writer::{EntryPlacement, hash_attributes_allowed};
use cbfstool_rs::vboot::metadata;
use cbfstool_rs::{
    AddEntryOptions, CbfsType, Compression, Error, FirmwareImage, HashAlgorithm,
    OwnedFirmwareImage, RegionWriteMode, Result,
};
use serde::Deserialize;

use crate::cli::BuildImageArgs;
use crate::commands::{elf_payload, elf_stage, flat_binary_payload};
use crate::fileio::{rewrite_firmware_image, write_new_firmware_image};

pub(crate) fn build_image(args: BuildImageArgs) -> Result<()> {
    let manifest = Manifest::read(&args.manifest)?;
    if manifest.version != 1 {
        return Err(Error::InvalidValue {
            what: "manifest version",
            value: manifest.version.to_string(),
        });
    }

    let mut image = if let Some(fmap) = manifest.fmap.as_ref() {
        let fmap_bytes = fs::read(&fmap.binary)?;
        let regions = fmap
            .cbfs_regions
            .iter()
            .map(String::as_str)
            .collect::<Vec<_>>();
        OwnedFirmwareImage::create_partitioned(&fmap_bytes, &regions)?
    } else {
        OwnedFirmwareImage::create_empty_cbfs(manifest.rom_size.ok_or(Error::InvalidValue {
            what: "manifest rom_size",
            value: "required when fmap is omitted".to_owned(),
        })?)?
    };

    let cbfs_verification = manifest
        .vboot
        .as_ref()
        .and_then(|vboot| vboot.cbfs_verification.as_ref())
        .filter(|verification| verification.enabled);

    let deferred_bootblock = manifest
        .bootblock
        .as_ref()
        .filter(|bootblock| should_defer_bootblock(bootblock, cbfs_verification));

    if let Some(bootblock) = manifest.bootblock.as_ref() {
        if deferred_bootblock.is_none() {
            apply_cbfs_entry(&mut image, bootblock, cbfs_verification)?;
        }
    }
    for entry in &manifest.cbfs {
        apply_cbfs_entry(&mut image, entry, cbfs_verification)?;
    }
    for entry in &manifest.ints {
        apply_int_entry(&mut image, entry)?;
    }
    for write in &manifest.raw_regions {
        apply_raw_region_write(&mut image, write)?;
    }
    if let Some(vboot) = manifest.vboot.as_ref() {
        apply_vboot_writes(&mut image, vboot)?;
    }
    if let Some(bootblock) = deferred_bootblock {
        apply_deferred_bootblock_entry(&mut image, bootblock, cbfs_verification)?;
    }
    if let Some(verification) = cbfs_verification {
        finalize_metadata_anchors(&mut image, verification)?;
    }

    let output = args
        .output
        .as_deref()
        .or(manifest.output.as_deref())
        .ok_or(Error::InvalidValue {
            what: "manifest output",
            value: "missing output path; pass --output or set manifest.output".to_owned(),
        })?;
    write_image(output, image.into_vec())
}

fn write_image(path: &Path, contents: Vec<u8>) -> Result<()> {
    if path.exists() {
        rewrite_firmware_image(path, contents)
    } else {
        write_new_firmware_image(path, contents)
    }
}

fn apply_cbfs_entry(
    image: &mut OwnedFirmwareImage,
    entry: &ManifestCbfsEntry,
    cbfs_verification: Option<&ManifestVbootCbfsVerification>,
) -> Result<()> {
    let source = fs::read(&entry.source)?;
    let regions = entry.regions();
    let placement = entry.placement.entry_placement()?;

    for region in regions {
        match entry.kind()? {
            EntryKind::Raw(ty) => image.add_entry_with_options(AddEntryOptions {
                region_name: Some(region),
                name: &entry.name,
                ty,
                contents: &source,
                compression: entry.compression,
                extra_attributes: &entry.extra_attributes()?,
                hash_algorithm: entry.hash_algorithm(region, ty, cbfs_verification)?,
                placement,
            })?,
            EntryKind::Payload => {
                let payload = elf_payload(&source, entry.compression)?;
                image.add_entry_with_options(AddEntryOptions {
                    region_name: Some(region),
                    name: &entry.name,
                    ty: CbfsType::new(0x20),
                    contents: &payload,
                    compression: Compression::None,
                    extra_attributes: &entry.extra_attributes()?,
                    hash_algorithm: entry.hash_algorithm(
                        region,
                        CbfsType::new(0x20),
                        cbfs_verification,
                    )?,
                    placement,
                })?;
            }
            EntryKind::Stage => {
                let stage = elf_stage(&source)?;
                let mut attrs = attributes::AttributeBuilder::new();
                attrs.stage_header(stage.load_address, stage.entry_offset, stage.mem_len)?;
                attrs.push_serialized(&entry.extra_attributes()?)?;
                let attrs = attrs.into_bytes();
                image.add_entry_with_options(AddEntryOptions {
                    region_name: Some(region),
                    name: &entry.name,
                    ty: CbfsType::new(0x11),
                    contents: &stage.data,
                    compression: entry.compression,
                    extra_attributes: &attrs,
                    hash_algorithm: entry.hash_algorithm(
                        region,
                        CbfsType::new(0x11),
                        cbfs_verification,
                    )?,
                    placement,
                })?;
            }
            EntryKind::FlatBinary => {
                let load_address = entry.load_address.ok_or(Error::InvalidValue {
                    what: "flat-binary load_address",
                    value: format!("missing for {}", entry.name),
                })?;
                let entry_point = entry.entry_point.ok_or(Error::InvalidValue {
                    what: "flat-binary entry_point",
                    value: format!("missing for {}", entry.name),
                })?;
                let payload =
                    flat_binary_payload(&source, load_address, entry_point, entry.compression)?;
                image.add_entry_with_options(AddEntryOptions {
                    region_name: Some(region),
                    name: &entry.name,
                    ty: CbfsType::new(0x20),
                    contents: &payload,
                    compression: Compression::None,
                    extra_attributes: &entry.extra_attributes()?,
                    hash_algorithm: entry.hash_algorithm(
                        region,
                        CbfsType::new(0x20),
                        cbfs_verification,
                    )?,
                    placement,
                })?;
            }
        }
    }
    Ok(())
}

fn should_defer_bootblock(
    bootblock: &ManifestCbfsEntry,
    cbfs_verification: Option<&ManifestVbootCbfsVerification>,
) -> bool {
    cbfs_verification.is_some()
        && matches!(bootblock.kind(), Ok(EntryKind::Raw(_)))
        && fs::read(&bootblock.source)
            .ok()
            .and_then(|bytes| metadata::find_unique_anchor(&bytes).ok().flatten())
            .is_some()
}

fn apply_deferred_bootblock_entry(
    image: &mut OwnedFirmwareImage,
    entry: &ManifestCbfsEntry,
    cbfs_verification: Option<&ManifestVbootCbfsVerification>,
) -> Result<()> {
    let source = fs::read(&entry.source)?;
    let EntryKind::Raw(ty) = entry.kind()? else {
        return Err(Error::InvalidValue {
            what: "deferred bootblock",
            value: "only raw bootblock entries can be finalized before CBFS insertion".to_owned(),
        });
    };
    let source_anchor = metadata::find_unique_anchor(&source)?.ok_or(Error::InvalidValue {
        what: "metadata hash anchor",
        value: format!("not found in {}", entry.source.display()),
    })?;
    if let Some(configured) = cbfs_verification.and_then(|verification| verification.hash_algorithm)
        && configured != source_anchor.algorithm()
    {
        return Err(Error::InvalidValue {
            what: "vboot.cbfs_verification.hash_algorithm",
            value: format!(
                "manifest uses {configured}, but bootblock anchor uses {}",
                source_anchor.algorithm()
            ),
        });
    }

    for region in entry.regions() {
        let mut provisional = image.clone();
        add_cbfs_entry_from_contents(&mut provisional, entry, region, ty, &source, None)?;
        let metadata_digest =
            metadata_digest_for_region(&provisional, region, source_anchor.algorithm())?;
        let fmap_digest = fmap_digest_for_image(&provisional, source_anchor.algorithm())?;

        let mut patched_source = source.clone();
        metadata::patch_unique_anchor(
            &mut patched_source,
            &metadata_digest,
            fmap_digest.as_deref(),
        )?;

        let mut patched_image = image.clone();
        add_cbfs_entry_from_contents(&mut patched_image, entry, region, ty, &patched_source, None)?;
        let final_metadata_digest =
            metadata_digest_for_region(&patched_image, region, source_anchor.algorithm())?;
        if final_metadata_digest != metadata_digest {
            return Err(Error::InvalidValue {
                what: "deferred bootblock metadata hash",
                value: format!(
                    "CBFS metadata changed after patching {}; use an uncompressed/stable-size bootblock container or reserve a fixed placement",
                    entry.name
                ),
            });
        }
        *image = patched_image;
    }

    Ok(())
}

fn add_cbfs_entry_from_contents(
    image: &mut OwnedFirmwareImage,
    entry: &ManifestCbfsEntry,
    region: &str,
    ty: CbfsType,
    contents: &[u8],
    hash_algorithm: Option<HashAlgorithm>,
) -> Result<()> {
    image.add_entry_with_options(AddEntryOptions {
        region_name: Some(region),
        name: &entry.name,
        ty,
        contents,
        compression: entry.compression,
        extra_attributes: &entry.extra_attributes()?,
        hash_algorithm,
        placement: entry.placement.entry_placement()?,
    })
}

fn metadata_digest_for_region(
    image: &OwnedFirmwareImage,
    region: &str,
    algorithm: HashAlgorithm,
) -> Result<Vec<u8>> {
    let parsed = FirmwareImage::parse(image.as_bytes())?;
    let cbfs = parsed.cbfs(Some(region))?;
    metadata::cbfs_metadata_digest(&cbfs, algorithm)
}

fn fmap_digest_for_image(
    image: &OwnedFirmwareImage,
    algorithm: HashAlgorithm,
) -> Result<Option<Vec<u8>>> {
    let parsed = FirmwareImage::parse(image.as_bytes())?;
    parsed
        .fmap()
        .map(|fmap| metadata::fmap_digest(image.as_bytes(), fmap, algorithm))
        .transpose()
}

fn finalize_metadata_anchors(
    image: &mut OwnedFirmwareImage,
    verification: &ManifestVbootCbfsVerification,
) -> Result<()> {
    let containers = anchor_containers_for_image(image, verification)?;
    for container in containers {
        patch_anchor_container(image, &container, verification.hash_algorithm)?;
    }
    Ok(())
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum ResolvedAnchorContainer {
    BootblockRegion {
        cbfs_region: String,
        region: String,
    },
    CbfsEntry {
        cbfs_region: String,
        region: String,
        name: String,
    },
}

fn anchor_containers_for_image(
    image: &OwnedFirmwareImage,
    verification: &ManifestVbootCbfsVerification,
) -> Result<Vec<ResolvedAnchorContainer>> {
    let mut containers = Vec::new();
    for container in &verification.anchor_containers {
        containers.push(container.resolve()?);
    }

    for cbfs_region in &verification.cbfs_regions {
        if containers
            .iter()
            .any(|container| container.cbfs_region() == cbfs_region)
        {
            continue;
        }
        if let Some(container) = default_anchor_container(image, cbfs_region)? {
            containers.push(container);
        }
    }
    Ok(containers)
}

fn default_anchor_container(
    image: &OwnedFirmwareImage,
    cbfs_region: &str,
) -> Result<Option<ResolvedAnchorContainer>> {
    let parsed = FirmwareImage::parse(image.as_bytes())?;
    if cbfs_region == "COREBOOT_B" && parsed.region(Some("BOOTBLOCK_B")).is_ok() {
        return Ok(Some(ResolvedAnchorContainer::BootblockRegion {
            cbfs_region: cbfs_region.to_owned(),
            region: "BOOTBLOCK_B".to_owned(),
        }));
    }

    if cbfs_region == "COREBOOT" && parsed.region(Some("BOOTBLOCK")).is_ok() {
        return Ok(Some(ResolvedAnchorContainer::BootblockRegion {
            cbfs_region: cbfs_region.to_owned(),
            region: "BOOTBLOCK".to_owned(),
        }));
    }

    let Ok(cbfs) = parsed.cbfs(Some(cbfs_region)) else {
        return Ok(None);
    };
    for (name, ty) in [
        ("bootblock", CbfsType::BOOTBLOCK),
        ("apu/amdfw", CbfsType::AMDFW),
    ] {
        if cbfs
            .find_entry(name)?
            .filter(|entry| entry.entry_type() == ty)
            .is_some()
        {
            return Ok(Some(ResolvedAnchorContainer::CbfsEntry {
                cbfs_region: cbfs_region.to_owned(),
                region: cbfs_region.to_owned(),
                name: name.to_owned(),
            }));
        }
    }

    Ok(None)
}

fn patch_anchor_container(
    image: &mut OwnedFirmwareImage,
    container: &ResolvedAnchorContainer,
    configured_algorithm: Option<HashAlgorithm>,
) -> Result<()> {
    match container {
        ResolvedAnchorContainer::BootblockRegion {
            cbfs_region,
            region,
        } => patch_region_anchor(image, cbfs_region, region, configured_algorithm),
        ResolvedAnchorContainer::CbfsEntry {
            cbfs_region,
            region,
            name,
        } => patch_cbfs_entry_anchor(image, cbfs_region, region, name, configured_algorithm),
    }
}

fn patch_region_anchor(
    image: &mut OwnedFirmwareImage,
    cbfs_region: &str,
    region_name: &str,
    configured_algorithm: Option<HashAlgorithm>,
) -> Result<()> {
    let parsed = FirmwareImage::parse(image.as_bytes())?;
    let region = parsed.region(Some(region_name))?;
    let region_start = region.offset();
    let region_end = region_start
        .checked_add(region.size())
        .ok_or(Error::InvalidOffset {
            what: "metadata hash anchor region",
            offset: region_start,
        })?;
    let region_bytes = parsed.region_bytes(&region)?;
    let Some(anchor) = metadata::find_unique_anchor(region_bytes)? else {
        return Ok(());
    };
    let algorithm = anchor.algorithm();
    validate_anchor_algorithm(configured_algorithm, algorithm)?;
    let metadata_digest = metadata_digest_for_region(image, cbfs_region, algorithm)?;
    let fmap_digest = fmap_digest_for_image(image, algorithm)?;
    drop(parsed);

    let bytes = image
        .as_mut_bytes()
        .get_mut(region_start..region_end)
        .ok_or(Error::InvalidOffset {
            what: "metadata hash anchor region",
            offset: region_start,
        })?;
    metadata::patch_anchor(bytes, anchor, &metadata_digest, fmap_digest.as_deref())
}

fn patch_cbfs_entry_anchor(
    image: &mut OwnedFirmwareImage,
    cbfs_region: &str,
    region_name: &str,
    entry_name: &str,
    configured_algorithm: Option<HashAlgorithm>,
) -> Result<()> {
    let parsed = FirmwareImage::parse(image.as_bytes())?;
    let region = parsed.region(Some(region_name))?;
    let cbfs = parsed.cbfs(Some(region_name))?;
    let entry = cbfs
        .find_entry(entry_name)?
        .ok_or_else(|| Error::EntryNotFound(entry_name.to_owned()))?;
    let entry_data_start = region
        .offset()
        .checked_add(entry.offset())
        .and_then(|offset| offset.checked_add(entry.data_offset()))
        .ok_or(Error::InvalidOffset {
            what: "metadata hash anchor CBFS entry",
            offset: region.offset(),
        })?;
    let entry_data_end = entry_data_start
        .checked_add(entry.len())
        .ok_or(Error::InvalidOffset {
            what: "metadata hash anchor CBFS entry",
            offset: entry_data_start,
        })?;
    let Some(anchor) = metadata::find_unique_anchor(entry.data())? else {
        return Ok(());
    };
    let algorithm = anchor.algorithm();
    validate_anchor_algorithm(configured_algorithm, algorithm)?;
    let metadata_digest = metadata_digest_for_region(image, cbfs_region, algorithm)?;
    let fmap_digest = fmap_digest_for_image(image, algorithm)?;
    drop(parsed);

    let bytes = image
        .as_mut_bytes()
        .get_mut(entry_data_start..entry_data_end)
        .ok_or(Error::InvalidOffset {
            what: "metadata hash anchor CBFS entry",
            offset: entry_data_start,
        })?;
    metadata::patch_anchor(bytes, anchor, &metadata_digest, fmap_digest.as_deref())
}

impl ResolvedAnchorContainer {
    fn cbfs_region(&self) -> &str {
        match self {
            Self::BootblockRegion { cbfs_region, .. } | Self::CbfsEntry { cbfs_region, .. } => {
                cbfs_region
            }
        }
    }
}

fn infer_cbfs_region_from_bootblock_region(region: &str) -> String {
    if region == "BOOTBLOCK_B" {
        "COREBOOT_B".to_owned()
    } else {
        "COREBOOT".to_owned()
    }
}

fn validate_anchor_algorithm(
    configured: Option<HashAlgorithm>,
    discovered: HashAlgorithm,
) -> Result<()> {
    if let Some(configured) = configured
        && configured != discovered
    {
        return Err(Error::InvalidValue {
            what: "vboot.cbfs_verification.hash_algorithm",
            value: format!("manifest uses {configured}, but anchor uses {discovered}"),
        });
    }
    Ok(())
}

fn apply_int_entry(image: &mut OwnedFirmwareImage, entry: &ManifestIntEntry) -> Result<()> {
    let contents = entry.value.to_le_bytes();
    for region in entry.regions() {
        image.add_entry_with_options(AddEntryOptions {
            region_name: Some(region),
            name: &entry.name,
            ty: CbfsType::RAW,
            contents: &contents,
            compression: Compression::None,
            extra_attributes: &[],
            hash_algorithm: None,
            placement: entry.placement.entry_placement()?,
        })?;
    }
    Ok(())
}

fn apply_raw_region_write(
    image: &mut OwnedFirmwareImage,
    write: &ManifestRawRegionWrite,
) -> Result<()> {
    let contents = fs::read(&write.source)?;
    let mode = match write.mode {
        RawWriteMode::Exact => RegionWriteMode::Exact,
        RawWriteMode::FillUpward => RegionWriteMode::FillUpward { fill: write.fill },
        RawWriteMode::FillDownward => RegionWriteMode::FillDownward { fill: write.fill },
    };
    image.write_region(Some(&write.region), &contents, mode)
}

fn apply_vboot_writes(image: &mut OwnedFirmwareImage, vboot: &ManifestVboot) -> Result<()> {
    validate_vboot_partitions(vboot)?;

    if let Some(gbb) = vboot.gbb.as_ref() {
        let contents = fs::read(&gbb.source)?;
        image.write_region(
            Some(&gbb.region),
            &contents,
            RegionWriteMode::FillUpward { fill: Some(0) },
        )?;
    }

    if let Some(fwid) = vboot.fwid.as_ref() {
        let mut contents = Vec::with_capacity(fwid.model.len() + fwid.version.len() + 1);
        contents.extend_from_slice(fwid.model.as_bytes());
        contents.extend_from_slice(fwid.version.as_bytes());
        contents.push(0);
        for region in &fwid.regions {
            image.write_region(
                Some(region),
                &contents,
                RegionWriteMode::FillUpward { fill: Some(0) },
            )?;
        }
    }

    if let Some(shared_data) = vboot
        .shared_data
        .as_ref()
        .filter(|shared_data| shared_data.enabled)
    {
        image.write_region(
            Some(&shared_data.region),
            &[0],
            RegionWriteMode::FillUpward { fill: Some(0) },
        )?;
    }

    Ok(())
}

fn validate_vboot_partitions(vboot: &ManifestVboot) -> Result<()> {
    let Some(partitions) = vboot.partitions.as_ref() else {
        return Ok(());
    };
    if partitions.ro.is_empty() || partitions.rw.iter().any(|region| region.is_empty()) {
        return Err(Error::InvalidValue {
            what: "vboot partitions",
            value: "partition region names must not be empty".to_owned(),
        });
    }
    Ok(())
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Manifest {
    version: u32,
    #[serde(default)]
    output: Option<PathBuf>,
    #[serde(default)]
    rom_size: Option<usize>,
    #[serde(default)]
    fmap: Option<ManifestFmap>,
    #[serde(default)]
    cbfs: Vec<ManifestCbfsEntry>,
    #[serde(default)]
    raw_regions: Vec<ManifestRawRegionWrite>,
    #[serde(default)]
    ints: Vec<ManifestIntEntry>,
    #[serde(default)]
    bootblock: Option<ManifestCbfsEntry>,
    #[serde(default)]
    vboot: Option<ManifestVboot>,
}

impl Manifest {
    fn read(path: &Path) -> Result<Self> {
        let text = fs::read_to_string(path)?;
        serde_json::from_str(&text).map_err(|err| Error::InvalidValue {
            what: "manifest JSON",
            value: err.to_string(),
        })
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ManifestFmap {
    binary: PathBuf,
    cbfs_regions: Vec<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ManifestCbfsEntry {
    name: String,
    source: PathBuf,
    #[serde(default = "default_regions")]
    regions: Vec<String>,
    #[serde(rename = "type", deserialize_with = "deserialize_entry_kind")]
    kind: EntryKind,
    #[serde(
        default = "default_compression",
        deserialize_with = "deserialize_optional_compression"
    )]
    compression: Compression,
    #[serde(default)]
    placement: ManifestPlacement,
    #[serde(default)]
    ibb: bool,
    #[serde(default)]
    padding: Option<usize>,
    #[serde(default)]
    load_address: Option<u64>,
    #[serde(default)]
    entry_point: Option<u64>,
    #[serde(default, deserialize_with = "deserialize_optional_hash_algorithm")]
    hash: Option<HashAlgorithm>,
}

impl ManifestCbfsEntry {
    fn regions(&self) -> impl Iterator<Item = &str> {
        self.regions.iter().map(String::as_str)
    }

    fn kind(&self) -> Result<EntryKind> {
        Ok(self.kind)
    }

    fn extra_attributes(&self) -> Result<Vec<u8>> {
        let mut builder = attributes::AttributeBuilder::new();
        if self.ibb {
            builder.ibb()?;
        }
        if let Some(size) = self.padding {
            builder.padding(size)?;
        }
        Ok(builder.into_bytes())
    }

    fn hash_algorithm(
        &self,
        region: &str,
        ty: CbfsType,
        cbfs_verification: Option<&ManifestVbootCbfsVerification>,
    ) -> Result<Option<HashAlgorithm>> {
        if let Some(hash) = self.hash {
            return Ok(Some(hash));
        }

        let Some(verification) = cbfs_verification else {
            return Ok(None);
        };
        if !verification
            .cbfs_regions
            .iter()
            .any(|verified| verified == region)
        {
            return Ok(None);
        }
        if !hash_attributes_allowed(ty) {
            return Ok(None);
        }
        verification
            .hash_algorithm
            .ok_or(Error::InvalidValue {
                what: "vboot.cbfs_verification.hash_algorithm",
                value: "required until metadata anchor algorithm discovery is implemented"
                    .to_owned(),
            })
            .map(Some)
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ManifestIntEntry {
    name: String,
    value: u64,
    #[serde(default = "default_regions")]
    regions: Vec<String>,
    #[serde(default)]
    placement: ManifestPlacement,
}

impl ManifestIntEntry {
    fn regions(&self) -> impl Iterator<Item = &str> {
        self.regions.iter().map(String::as_str)
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ManifestVboot {
    #[serde(default)]
    partitions: Option<ManifestVbootPartitions>,
    #[serde(default)]
    fwid: Option<ManifestVbootFwid>,
    #[serde(default)]
    gbb: Option<ManifestVbootGbb>,
    #[serde(default)]
    shared_data: Option<ManifestVbootSharedData>,
    #[serde(default)]
    cbfs_verification: Option<ManifestVbootCbfsVerification>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ManifestVbootPartitions {
    ro: String,
    #[serde(default)]
    rw: Vec<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ManifestVbootFwid {
    model: String,
    version: String,
    regions: Vec<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ManifestVbootGbb {
    #[serde(default = "default_gbb_region")]
    region: String,
    source: PathBuf,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ManifestVbootSharedData {
    #[serde(default)]
    enabled: bool,
    #[serde(default = "default_shared_data_region")]
    region: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ManifestVbootCbfsVerification {
    #[serde(default)]
    enabled: bool,
    #[serde(default)]
    cbfs_regions: Vec<String>,
    #[serde(default, deserialize_with = "deserialize_optional_hash_algorithm")]
    hash_algorithm: Option<HashAlgorithm>,
    #[serde(default)]
    anchor_containers: Vec<ManifestAnchorContainer>,
}

#[derive(Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case", deny_unknown_fields)]
enum ManifestAnchorContainer {
    BootblockRegion {
        region: String,
        #[serde(default)]
        cbfs_region: Option<String>,
    },
    CbfsEntry {
        region: String,
        name: String,
        #[serde(default)]
        cbfs_region: Option<String>,
    },
}

impl ManifestAnchorContainer {
    fn resolve(&self) -> Result<ResolvedAnchorContainer> {
        match self {
            Self::BootblockRegion {
                region,
                cbfs_region,
            } => Ok(ResolvedAnchorContainer::BootblockRegion {
                cbfs_region: cbfs_region
                    .clone()
                    .unwrap_or_else(|| infer_cbfs_region_from_bootblock_region(region)),
                region: region.clone(),
            }),
            Self::CbfsEntry {
                region,
                name,
                cbfs_region,
            } => Ok(ResolvedAnchorContainer::CbfsEntry {
                cbfs_region: cbfs_region.clone().unwrap_or_else(|| region.clone()),
                region: region.clone(),
                name: name.clone(),
            }),
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ManifestRawRegionWrite {
    region: String,
    source: PathBuf,
    #[serde(default)]
    mode: RawWriteMode,
    #[serde(default)]
    fill: Option<u8>,
}

#[derive(Debug, Default, Clone, Copy, Deserialize)]
#[serde(rename_all = "kebab-case")]
enum RawWriteMode {
    #[default]
    Exact,
    FillUpward,
    FillDownward,
}

#[derive(Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
struct ManifestPlacement {
    #[serde(default)]
    base: Option<usize>,
    #[serde(default)]
    align: Option<usize>,
}

impl ManifestPlacement {
    fn entry_placement(&self) -> Result<EntryPlacement> {
        match (self.base, self.align) {
            (Some(_), Some(_)) => Err(Error::InvalidValue {
                what: "manifest placement",
                value: "base and align are mutually exclusive".to_owned(),
            }),
            (Some(base), None) => Ok(EntryPlacement::Offset(base)),
            (None, Some(align)) => Ok(EntryPlacement::Alignment(align)),
            (None, None) => Ok(EntryPlacement::Any),
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum EntryKind {
    Raw(CbfsType),
    Stage,
    Payload,
    FlatBinary,
}

fn deserialize_entry_kind<'de, D>(deserializer: D) -> std::result::Result<EntryKind, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let value = String::deserialize(deserializer)?;
    match value.as_str() {
        "stage" => Ok(EntryKind::Stage),
        "payload" | "simple elf" | "simple_elf" => Ok(EntryKind::Payload),
        "flat-binary" | "flat_binary" => Ok(EntryKind::FlatBinary),
        _ => CbfsType::from_str(&value)
            .map(EntryKind::Raw)
            .map_err(|err| serde::de::Error::custom(err.to_string())),
    }
}

fn deserialize_optional_compression<'de, D>(
    deserializer: D,
) -> std::result::Result<Compression, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let value = Option::<String>::deserialize(deserializer)?;
    match value {
        Some(value) => {
            Compression::from_str(&value).map_err(|err| serde::de::Error::custom(err.to_string()))
        }
        None => Ok(Compression::None),
    }
}

fn deserialize_optional_hash_algorithm<'de, D>(
    deserializer: D,
) -> std::result::Result<Option<HashAlgorithm>, D::Error>
where
    D: serde::Deserializer<'de>,
{
    Option::<String>::deserialize(deserializer)?
        .map(|value| HashAlgorithm::from_str(&value))
        .transpose()
        .map_err(|err| serde::de::Error::custom(err.to_string()))
}

fn default_regions() -> Vec<String> {
    vec!["COREBOOT".to_owned()]
}

fn default_compression() -> Compression {
    Compression::None
}

fn default_gbb_region() -> String {
    "GBB".to_owned()
}

fn default_shared_data_region() -> String {
    "SHARED_DATA".to_owned()
}
