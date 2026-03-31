// SPDX-License-Identifier: GPL-2.0-only

use std::fs;
use std::path::Path;

use cbfstool_rs::cbfs::attributes;
use cbfstool_rs::cbfs::writer::{
    EntryPlacement, master_header_bytes, relative_master_header_pointer,
};
use cbfstool_rs::{
    AddEntryOptions, Cbfs, CbfsType, Compression, Error, FirmwareImage, OwnedFirmwareImage,
    RegionWriteMode, Result,
};

use crate::cli::{
    AddArgs, AddFlatBinaryArgs, AddIntArgs, AddMasterHeaderArgs, AddPayloadArgs, AddStageArgs,
    CopyArgs, CreateArgs, EntryPlacementArgs, ExtractArgs, LayoutArgs, PrintArgs, ReadArgs,
    RegionCommandArgs, RemoveArgs, WriteArgs,
};
use crate::fileio::{rewrite_firmware_image, write_new_firmware_image};

pub(crate) fn layout(image: &FirmwareImage<'_>, args: &LayoutArgs) -> Result<()> {
    if args.include_readonly {
        println!(
            "This image contains the following sections that can be accessed with this tool:\n"
        );
    } else {
        println!(
            "This image contains the following sections not marked read-only (filtering is advisory):\n"
        );
    }
    image
        .regions(args.include_readonly)
        .into_iter()
        .for_each(|region| {
            let access = if region.is_read_only() {
                "read-only, "
            } else {
                ""
            };
            let cbfs = if region.is_cbfs() { "CBFS, " } else { "" };
            println!(
                "'{}' ({}{}size {}, offset {})",
                region.name(),
                access,
                cbfs,
                region.size(),
                region.offset()
            );
        });
    Ok(())
}

pub(crate) fn print_cbfs(image: &FirmwareImage<'_>, args: &PrintArgs) -> Result<()> {
    if args.machine_parseable {
        println!("Name\tOffset\tType\tMetadata Size\tData Size\tTotal Size");
    }
    for region_name in selected_regions(args.region.region.as_deref()) {
        let cbfs = image.cbfs(region_name)?;
        if !args.machine_parseable {
            if let Some(region) = region_name
                .or(Some("COREBOOT"))
                .filter(|_| image.fmap().is_some())
            {
                println!("FMAP REGION: {region}");
            }
            println!(
                "{:<30} {:>10} {:<14} {:>8} Comp",
                "Name", "Offset", "Type", "Size"
            );
        }
        cbfs.entries().try_for_each(|entry| {
            let entry = entry?;
            if args.machine_parseable {
                println!(
                    "{}\t{:#x}\t{}\t{:#x}\t{:#x}\t{:#x}",
                    entry.display_name(),
                    entry.offset(),
                    entry.entry_type().name(),
                    entry.metadata_len(),
                    entry.len(),
                    entry.total_len()
                );
            } else {
                let comp = compression_summary(entry.compression(), entry.decompressed_size());
                println!(
                    "{:<30} {:#10x} {:<14} {:>8} {}",
                    entry.display_name(),
                    entry.offset(),
                    entry.entry_type().name(),
                    entry.len(),
                    comp
                );
            }
            Ok::<(), Error>(())
        })?;
    }
    Ok(())
}

pub(crate) fn read_region(image: &FirmwareImage<'_>, args: ReadArgs) -> Result<()> {
    let region = image.region(args.region.region.as_deref())?;
    fs::write(args.file, image.region_bytes(&region)?)?;
    Ok(())
}

pub(crate) fn extract(image: &FirmwareImage<'_>, args: ExtractArgs) -> Result<()> {
    let cbfs: Cbfs<'_> = image.cbfs(args.region.region.as_deref())?;
    let entry = cbfs
        .find_entry(&args.name)?
        .ok_or(Error::EntryNotFound(args.name))?;
    let data = if args.unprocessed || entry.compression() == Compression::None {
        entry.data().to_vec()
    } else {
        cbfstool_rs::cbfs::compression::decompress(
            entry.compression(),
            entry.data(),
            entry.decompressed_size(),
        )?
    };
    fs::write(args.file, data)?;
    Ok(())
}

pub(crate) fn add(image_path: &Path, bytes: Vec<u8>, args: AddArgs) -> Result<()> {
    let contents = fs::read(args.file)?;
    let mut image = OwnedFirmwareImage::from_vec(bytes);
    let placement = placement(&args.placement)?;
    for region_name in selected_regions(args.region.region.as_deref()) {
        image.add_entry_with_options(AddEntryOptions {
            region_name,
            name: &args.name,
            ty: args.file_type,
            contents: &contents,
            compression: args.compression,
            extra_attributes: &[],
            hash_algorithm: args.hash_algorithm,
            placement,
        })?;
    }
    rewrite_firmware_image(image_path, image.into_vec())?;
    Ok(())
}

pub(crate) fn add_int(image_path: &Path, bytes: Vec<u8>, args: AddIntArgs) -> Result<()> {
    let contents = args.value.to_le_bytes();
    let mut image = OwnedFirmwareImage::from_vec(bytes);
    let placement = placement(&args.placement)?;
    for region_name in selected_regions(args.region.region.as_deref()) {
        image.add_entry_with_options(AddEntryOptions {
            region_name,
            name: &args.name,
            ty: CbfsType::RAW,
            contents: &contents,
            compression: Compression::None,
            extra_attributes: &[],
            hash_algorithm: None,
            placement,
        })?;
    }
    rewrite_firmware_image(image_path, image.into_vec())?;
    Ok(())
}

pub(crate) fn add_flat_binary(
    image_path: &Path,
    bytes: Vec<u8>,
    args: AddFlatBinaryArgs,
) -> Result<()> {
    if args.load_address == 0 || args.entry_point == 0 {
        return Err(Error::InvalidValue {
            what: "flat binary payload address",
            value: "load address and entry point must both be non-zero".to_owned(),
        });
    }
    let input = fs::read(args.file)?;
    let payload = flat_binary_payload(
        &input,
        args.load_address,
        args.entry_point,
        args.compression,
    )?;
    let mut image = OwnedFirmwareImage::from_vec(bytes);
    let placement = placement(&args.placement)?;
    for region_name in selected_regions(args.region.region.as_deref()) {
        image.add_entry_with_options(AddEntryOptions {
            region_name,
            name: &args.name,
            ty: CbfsType::new(0x20),
            contents: &payload,
            compression: Compression::None,
            extra_attributes: &[],
            hash_algorithm: args.hash_algorithm,
            placement,
        })?;
    }
    rewrite_firmware_image(image_path, image.into_vec())?;
    Ok(())
}

pub(crate) fn add_payload(image_path: &Path, bytes: Vec<u8>, args: AddPayloadArgs) -> Result<()> {
    let input = fs::read(args.file)?;
    let payload = elf_payload(&input, args.compression)?;
    let mut image = OwnedFirmwareImage::from_vec(bytes);
    let placement = placement(&args.placement)?;
    for region_name in selected_regions(args.region.region.as_deref()) {
        image.add_entry_with_options(AddEntryOptions {
            region_name,
            name: &args.name,
            ty: CbfsType::new(0x20),
            contents: &payload,
            compression: Compression::None,
            extra_attributes: &[],
            hash_algorithm: args.hash_algorithm,
            placement,
        })?;
    }
    rewrite_firmware_image(image_path, image.into_vec())?;
    Ok(())
}

pub(crate) fn add_stage(image_path: &Path, bytes: Vec<u8>, args: AddStageArgs) -> Result<()> {
    if let Some(ignored) = args.ignore_sections.as_deref() {
        eprintln!(
            "cbfstool-rs: warning: add-stage --ignore-sec={ignored} is accepted but not applied yet"
        );
    }
    let input = fs::read(args.file)?;
    let stage = elf_stage(&input)?;
    let stage_header =
        attributes::stage_header(stage.load_address, stage.entry_offset, stage.mem_len)?;
    let mut image = OwnedFirmwareImage::from_vec(bytes);
    let placement = placement(&args.placement)?;
    for region_name in selected_regions(args.region.region.as_deref()) {
        image.add_entry_with_options(AddEntryOptions {
            region_name,
            name: &args.name,
            ty: CbfsType::new(0x11),
            contents: &stage.data,
            compression: args.compression,
            extra_attributes: &stage_header,
            hash_algorithm: args.hash_algorithm,
            placement,
        })?;
    }
    rewrite_firmware_image(image_path, image.into_vec())?;
    Ok(())
}

pub(crate) fn add_master_header(
    image_path: &Path,
    bytes: Vec<u8>,
    args: AddMasterHeaderArgs,
) -> Result<()> {
    if let Some(topswap_size) = args.topswap_size {
        eprintln!(
            "cbfstool-rs: warning: add-master-header --topswap-size={topswap_size:#x} is accepted but secondary top-swap pointer updates are not implemented yet"
        );
    }
    let region_name = args.region.region.as_deref();
    let region = FirmwareImage::parse(&bytes)?.region(region_name)?;
    let header = master_header_bytes(region.offset(), region.size())?;
    let mut image = OwnedFirmwareImage::from_vec(bytes);
    image.add_raw_entry(
        region_name,
        "cbfs master header",
        CbfsType::CBFSHEADER,
        &header,
    )?;

    let mut data = image.into_vec();
    let parsed = FirmwareImage::parse(&data)?;
    let region = parsed.region(region_name)?;
    let cbfs = parsed.cbfs(region_name)?;
    let entry = cbfs
        .find_entry("cbfs master header")?
        .ok_or(Error::EntryNotFound("cbfs master header".to_owned()))?;
    let header_offset =
        entry
            .offset()
            .checked_add(entry.data_offset())
            .ok_or(Error::InvalidOffset {
                what: "CBFS master header",
                offset: entry.offset(),
            })?;
    let relative = relative_master_header_pointer(region.size(), header_offset)?;
    let pointer_offset = region
        .offset()
        .checked_add(region.size().saturating_sub(4))
        .ok_or(Error::InvalidOffset {
            what: "CBFS master header pointer",
            offset: region.offset(),
        })?;
    drop(parsed);
    data[pointer_offset..pointer_offset + 4].copy_from_slice(&relative.to_le_bytes());
    rewrite_firmware_image(image_path, data)?;
    Ok(())
}

pub(crate) fn create(image_path: &Path, args: CreateArgs) -> Result<()> {
    if let Some(flashmap) = args.flashmap.as_deref() {
        return create_partitioned_image(image_path, flashmap, args.fmap_regions.as_deref());
    }

    let size = args.size.ok_or(Error::InvalidValue {
        what: "image size",
        value: "-s/--size is required without -M/--flashmap".to_owned(),
    })?;
    let image = OwnedFirmwareImage::create_empty_cbfs(size)?;
    write_new_firmware_image(image_path, image.into_vec())?;
    Ok(())
}

fn create_partitioned_image(
    image_path: &Path,
    flashmap_path: &Path,
    fmap_regions: Option<&str>,
) -> Result<()> {
    let fmap_bytes = fs::read(flashmap_path)?;
    let cbfs_regions = fmap_regions.unwrap_or("COREBOOT");
    let region_names = cbfs_regions
        .split(',')
        .filter(|name| !name.is_empty())
        .collect::<Vec<_>>();
    let image = OwnedFirmwareImage::create_partitioned(&fmap_bytes, &region_names)?;
    write_new_firmware_image(image_path, image.into_vec())?;
    Ok(())
}

pub(crate) fn write_region(image_path: &Path, bytes: Vec<u8>, args: WriteArgs) -> Result<()> {
    let image = FirmwareImage::parse(&bytes)?;
    for region_name in selected_regions(args.region.region.as_deref()) {
        let region = image.region(region_name)?;
        if region.is_cbfs() && !args.force {
            return Err(Error::InvalidValue {
                what: "write target",
                value: format!(
                    "region '{}' is a CBFS; use add/remove or pass --force",
                    region.name()
                ),
            });
        }
    }
    drop(image);

    let contents = fs::read(args.file)?;
    if contents.starts_with(b"__FMAP__") {
        return Err(Error::InvalidValue {
            what: "write input",
            value: "input appears to be an FMAP".to_owned(),
        });
    }
    if contents.starts_with(b"LARCHIVE") && !args.force {
        return Err(Error::InvalidValue {
            what: "write input",
            value: "input appears to be a CBFS; pass --force to write it raw".to_owned(),
        });
    }

    let mode = if args.fill_upward {
        RegionWriteMode::FillUpward { fill: args.fill }
    } else if args.fill_downward {
        RegionWriteMode::FillDownward { fill: args.fill }
    } else {
        if args.fill.is_some() {
            return Err(Error::InvalidValue {
                what: "write fill",
                value: "-i/--int requires -u or -d".to_owned(),
            });
        }
        RegionWriteMode::Exact
    };

    let mut image = OwnedFirmwareImage::from_vec(bytes);
    for region_name in selected_regions(args.region.region.as_deref()) {
        image.write_region(region_name, &contents, mode)?;
    }
    rewrite_firmware_image(image_path, image.into_vec())?;
    Ok(())
}

pub(crate) fn remove(image_path: &Path, bytes: Vec<u8>, args: RemoveArgs) -> Result<()> {
    let mut image = OwnedFirmwareImage::from_vec(bytes);
    for region_name in selected_regions(args.region.region.as_deref()) {
        image.remove_entry(region_name, &args.name)?;
    }
    rewrite_firmware_image(image_path, image.into_vec())?;
    Ok(())
}

pub(crate) fn compact(image_path: &Path, bytes: Vec<u8>, args: RegionCommandArgs) -> Result<()> {
    eprintln!("cbfstool-rs: warning: compacting a CBFS does not honor fixed addresses");
    let mut image = OwnedFirmwareImage::from_vec(bytes);
    for region_name in selected_regions(args.region.region.as_deref()) {
        image.compact_region(region_name)?;
    }
    rewrite_firmware_image(image_path, image.into_vec())?;
    Ok(())
}

pub(crate) fn copy(image_path: &Path, bytes: Vec<u8>, args: CopyArgs) -> Result<()> {
    let mut image = OwnedFirmwareImage::from_vec(bytes);
    for region_name in selected_regions(args.region.region.as_deref()) {
        image.copy_region(&args.source_region, region_name)?;
    }
    rewrite_firmware_image(image_path, image.into_vec())?;
    Ok(())
}

pub(crate) fn expand(image_path: &Path, bytes: Vec<u8>, args: RegionCommandArgs) -> Result<()> {
    let mut image = OwnedFirmwareImage::from_vec(bytes);
    for region_name in selected_regions(args.region.region.as_deref()) {
        image.expand_region(region_name)?;
    }
    rewrite_firmware_image(image_path, image.into_vec())?;
    Ok(())
}

pub(crate) fn truncate(image_path: &Path, bytes: Vec<u8>, args: RegionCommandArgs) -> Result<()> {
    let mut image = OwnedFirmwareImage::from_vec(bytes);
    let mut sizes = Vec::new();
    for region_name in selected_regions(args.region.region.as_deref()) {
        sizes.push(image.truncate_region(region_name)?);
    }
    rewrite_firmware_image(image_path, image.into_vec())?;
    for size in sizes {
        println!("{size:#x}");
    }
    Ok(())
}

pub(crate) fn flat_binary_payload(
    input: &[u8],
    load_address: u64,
    entry_point: u64,
    compression: Compression,
) -> Result<Vec<u8>> {
    const SEGMENT_HEADER_LEN: usize = 28;
    const PAYLOAD_SEGMENT_CODE: u32 = 0x434f_4445;
    const PAYLOAD_SEGMENT_ENTRY: u32 = 0x454e_5452;

    let (stored_compression, stored_input) = payload_compress(compression, input)?;
    let data_offset = checked_usize_to_u32(SEGMENT_HEADER_LEN * 2, "payload data offset")?;
    let input_len = checked_usize_to_u32(input.len(), "payload memory length")?;
    let stored_len = checked_usize_to_u32(stored_input.len(), "payload data length")?;
    let mut output = Vec::with_capacity(SEGMENT_HEADER_LEN * 2 + stored_input.len());
    push_payload_segment(
        &mut output,
        PAYLOAD_SEGMENT_CODE,
        stored_compression,
        data_offset,
        load_address,
        stored_len,
        input_len,
    );
    push_payload_segment(
        &mut output,
        PAYLOAD_SEGMENT_ENTRY,
        Compression::None,
        0,
        entry_point,
        0,
        0,
    );
    output.extend_from_slice(&stored_input);
    Ok(output)
}

#[cfg(feature = "elf")]
pub(crate) fn elf_payload(input: &[u8], compression: Compression) -> Result<Vec<u8>> {
    use object::{Object, ObjectSegment};

    const SEGMENT_HEADER_LEN: usize = 28;
    const PAYLOAD_SEGMENT_CODE: u32 = 0x434f_4445;
    const PAYLOAD_SEGMENT_ENTRY: u32 = 0x454e_5452;

    let file = object::File::parse(input).map_err(|err| Error::InvalidValue {
        what: "ELF payload",
        value: err.to_string(),
    })?;
    let segments = file
        .segments()
        .filter(|segment| segment.size() != 0)
        .collect::<Vec<_>>();
    let mut payload_segments = Vec::with_capacity(segments.len() + 1);
    let mut payload_data = Vec::new();
    let mut data_offset = checked_usize_to_u32(
        SEGMENT_HEADER_LEN * (segments.len() + 1),
        "payload data offset",
    )?;
    for segment in segments {
        let data = segment.data().map_err(|err| Error::InvalidValue {
            what: "ELF payload segment",
            value: err.to_string(),
        })?;
        let (stored_compression, stored_data) = payload_compress(compression, data)?;
        let mem_len = checked_u64_to_u32(segment.size(), "ELF segment memory length")?;
        let stored_len = checked_usize_to_u32(stored_data.len(), "ELF segment data length")?;
        payload_segments.push((
            PAYLOAD_SEGMENT_CODE,
            stored_compression,
            data_offset,
            segment.address(),
            stored_len,
            mem_len,
        ));
        data_offset = data_offset
            .checked_add(stored_len)
            .ok_or(Error::InvalidValue {
                what: "payload data offset",
                value: "offset overflow".to_owned(),
            })?;
        payload_data.extend_from_slice(&stored_data);
    }
    payload_segments.push((
        PAYLOAD_SEGMENT_ENTRY,
        Compression::None,
        0,
        file.entry(),
        0,
        0,
    ));

    let mut output =
        Vec::with_capacity(SEGMENT_HEADER_LEN * payload_segments.len() + payload_data.len());
    for (ty, stored_compression, offset, load_addr, len, mem_len) in payload_segments {
        push_payload_segment(
            &mut output,
            ty,
            stored_compression,
            offset,
            load_addr,
            len,
            mem_len,
        );
    }
    output.extend_from_slice(&payload_data);
    Ok(output)
}

#[cfg(not(feature = "elf"))]
pub(crate) fn elf_payload(_input: &[u8], _compression: Compression) -> Result<Vec<u8>> {
    Err(Error::InvalidValue {
        what: "ELF payload",
        value: "add-payload requires the elf feature".to_owned(),
    })
}

#[derive(Debug)]
pub(crate) struct StageImage {
    pub(crate) data: Vec<u8>,
    pub(crate) load_address: u64,
    pub(crate) entry_offset: u32,
    pub(crate) mem_len: u32,
}

#[cfg(feature = "elf")]
pub(crate) fn elf_stage(input: &[u8]) -> Result<StageImage> {
    use object::{Object, ObjectSegment};

    let file = object::File::parse(input).map_err(|err| Error::InvalidValue {
        what: "ELF stage",
        value: err.to_string(),
    })?;
    let mut segments = Vec::new();
    for segment in file.segments().filter(|segment| segment.size() != 0) {
        let start = segment.address();
        let mem_end = start
            .checked_add(segment.size())
            .ok_or(Error::InvalidValue {
                what: "ELF stage segment",
                value: "address overflow".to_owned(),
            })?;
        let data = segment.data().map_err(|err| Error::InvalidValue {
            what: "ELF stage segment",
            value: err.to_string(),
        })?;
        let data_len = u64::try_from(data.len()).map_err(|_| Error::InvalidValue {
            what: "ELF stage segment length",
            value: data.len().to_string(),
        })?;
        let data_end = start.checked_add(data_len).ok_or(Error::InvalidValue {
            what: "ELF stage segment",
            value: "address overflow".to_owned(),
        })?;
        segments.push((start, data_end, mem_end, data.to_vec()));
    }
    if segments.is_empty() {
        return Err(Error::InvalidValue {
            what: "ELF stage",
            value: "no loadable segments".to_owned(),
        });
    }

    let load_address =
        segments
            .iter()
            .map(|segment| segment.0)
            .min()
            .ok_or(Error::InvalidValue {
                what: "ELF stage",
                value: "no loadable segments".to_owned(),
            })?;
    let data_end = segments
        .iter()
        .map(|segment| segment.1)
        .max()
        .ok_or(Error::InvalidValue {
            what: "ELF stage",
            value: "no loadable data".to_owned(),
        })?;
    let mem_end = segments
        .iter()
        .map(|segment| segment.2)
        .max()
        .ok_or(Error::InvalidValue {
            what: "ELF stage",
            value: "no loadable memory".to_owned(),
        })?;
    if file.entry() < load_address || file.entry() >= mem_end {
        return Err(Error::InvalidValue {
            what: "ELF stage entry",
            value: format!(
                "{:#x} is outside {:#x}..{:#x}",
                file.entry(),
                load_address,
                mem_end
            ),
        });
    }

    let data_len = checked_u64_to_usize(data_end - load_address, "ELF stage data length")?;
    let mut data = vec![0; data_len];
    for (start, _, _, bytes) in segments {
        let offset = checked_u64_to_usize(start - load_address, "ELF stage segment offset")?;
        let end = offset.checked_add(bytes.len()).ok_or(Error::InvalidValue {
            what: "ELF stage segment",
            value: "offset overflow".to_owned(),
        })?;
        data[offset..end].copy_from_slice(&bytes);
    }
    Ok(StageImage {
        data,
        load_address,
        entry_offset: checked_u64_to_u32(file.entry() - load_address, "ELF stage entry offset")?,
        mem_len: checked_u64_to_u32(mem_end - load_address, "ELF stage memory length")?,
    })
}

#[cfg(not(feature = "elf"))]
pub(crate) fn elf_stage(_input: &[u8]) -> Result<StageImage> {
    Err(Error::InvalidValue {
        what: "ELF stage",
        value: "add-stage requires the elf feature".to_owned(),
    })
}

fn push_payload_segment(
    out: &mut Vec<u8>,
    ty: u32,
    compression: Compression,
    offset: u32,
    load_addr: u64,
    len: u32,
    mem_len: u32,
) {
    push_be_u32(out, ty);
    push_be_u32(out, compression.raw());
    push_be_u32(out, offset);
    out.extend_from_slice(&load_addr.to_be_bytes());
    push_be_u32(out, len);
    push_be_u32(out, mem_len);
}

#[cfg(feature = "compression")]
fn payload_compress(compression: Compression, data: &[u8]) -> Result<(Compression, Vec<u8>)> {
    cbfstool_rs::cbfs::compression::compress(compression, data)
}

#[cfg(not(feature = "compression"))]
fn payload_compress(compression: Compression, data: &[u8]) -> Result<(Compression, Vec<u8>)> {
    if compression != Compression::None {
        return Err(Error::UnsupportedCompression(compression.raw()));
    }
    Ok((Compression::None, data.to_vec()))
}

fn push_be_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn checked_usize_to_u32(value: usize, what: &'static str) -> Result<u32> {
    u32::try_from(value).map_err(|_| Error::InvalidValue {
        what,
        value: value.to_string(),
    })
}

#[cfg(feature = "elf")]
fn checked_u64_to_u32(value: u64, what: &'static str) -> Result<u32> {
    u32::try_from(value).map_err(|_| Error::InvalidValue {
        what,
        value: value.to_string(),
    })
}

#[cfg(feature = "elf")]
fn checked_u64_to_usize(value: u64, what: &'static str) -> Result<usize> {
    usize::try_from(value).map_err(|_| Error::InvalidValue {
        what,
        value: value.to_string(),
    })
}

pub(crate) fn placement(args: &EntryPlacementArgs) -> Result<EntryPlacement> {
    if let Some(offset) = args.base_address {
        return Ok(EntryPlacement::Offset(offset));
    }
    if let Some(alignment) = args.alignment {
        if alignment == 0 {
            return Err(Error::InvalidValue {
                what: "CBFS entry alignment",
                value: "alignment must be non-zero".to_owned(),
            });
        }
        return Ok(EntryPlacement::Alignment(alignment));
    }
    Ok(EntryPlacement::Any)
}

fn selected_regions(region: Option<&str>) -> Vec<Option<&str>> {
    match region {
        Some(region) => region
            .split(',')
            .filter(|name| !name.is_empty())
            .map(Some)
            .collect(),
        None => vec![None],
    }
}

fn compression_summary(compression: Compression, decompressed_size: usize) -> String {
    if compression == Compression::None {
        "none".to_owned()
    } else {
        format!("{compression} ({decompressed_size} decompressed)")
    }
}
