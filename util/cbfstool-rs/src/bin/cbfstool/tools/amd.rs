// SPDX-License-Identifier: GPL-2.0-only

use std::fs;
use std::io::{Read, Write};

use cbfstool_rs::{Error, Result};

use crate::cli::AmdCompressArgs;

pub(crate) fn amd_compress_tool(args: AmdCompressArgs) -> Result<()> {
    const AMD_HEADER_SIZE: usize = 256;
    const AMD_SIZE_OFFSET: usize = 0x14;
    const AMD_UNCOMPRESSED_MAX: usize = 0x30_0000;

    if args.uncompress {
        let input = fs::read(&args.input)?;
        let compressed_size_bytes =
            input
                .get(AMD_SIZE_OFFSET..AMD_SIZE_OFFSET + 4)
                .ok_or_else(|| Error::InvalidValue {
                    what: "AMD compression header",
                    value: format!("input has {} bytes", input.len()),
                })?;
        let compressed_size = u32::from_le_bytes([
            compressed_size_bytes[0],
            compressed_size_bytes[1],
            compressed_size_bytes[2],
            compressed_size_bytes[3],
        ]) as usize;
        let compressed = input
            .get(AMD_HEADER_SIZE..AMD_HEADER_SIZE + compressed_size)
            .ok_or_else(|| Error::InvalidValue {
                what: "AMD compressed payload",
                value: format!(
                    "header says {compressed_size} bytes, input has {} payload bytes",
                    input.len().saturating_sub(AMD_HEADER_SIZE)
                ),
            })?;
        let mut decoder = flate2::read::ZlibDecoder::new(compressed);
        let mut output = Vec::new();
        decoder.read_to_end(&mut output)?;
        let max_size = args.max_size.unwrap_or(AMD_UNCOMPRESSED_MAX);
        if output.len() > max_size {
            return Err(Error::InvalidValue {
                what: "AMD uncompressed size",
                value: format!("{} exceeds maximum {max_size}", output.len()),
            });
        }
        fs::write(args.output, output)?;
        return Ok(());
    }

    let input = amd_input_payload(&fs::read(&args.input)?)?;
    if let Some(max_size) = args.max_size
        && input.len() > max_size
    {
        return Err(Error::InvalidValue {
            what: "AMD input size",
            value: format!("{} exceeds maximum {max_size}", input.len()),
        });
    }
    if args.elfcopy {
        fs::write(args.output, input)?;
        return Ok(());
    }

    let mut encoder = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
    encoder.write_all(&input)?;
    let compressed = encoder.finish()?;
    let compressed_size = u32::try_from(compressed.len()).map_err(|_| Error::InvalidValue {
        what: "AMD compressed size",
        value: format!("{} exceeds 32-bit header range", compressed.len()),
    })?;
    let mut output = vec![0; AMD_HEADER_SIZE];
    output[AMD_SIZE_OFFSET..AMD_SIZE_OFFSET + 4].copy_from_slice(&compressed_size.to_le_bytes());
    output.extend_from_slice(&compressed);
    fs::write(args.output, output)?;
    Ok(())
}

fn amd_input_payload(input: &[u8]) -> Result<Vec<u8>> {
    if input.get(..4) != Some(b"\x7fELF") {
        return Ok(input.to_vec());
    }
    amd_elf_payload(input)
}

#[cfg(feature = "elf")]
fn amd_elf_payload(input: &[u8]) -> Result<Vec<u8>> {
    use object::{Object, ObjectSegment};

    let file = object::File::parse(input).map_err(|err| Error::InvalidValue {
        what: "ELF",
        value: err.to_string(),
    })?;
    let segments = file
        .segments()
        .filter(|segment| segment.size() != 0)
        .collect::<Vec<_>>();
    if segments.len() != 1 {
        return Err(Error::InvalidValue {
            what: "ELF load segments",
            value: format!(
                "expected exactly one non-empty load segment, found {}",
                segments.len()
            ),
        });
    }
    let (offset, size) = segments[0].file_range();
    let start = usize::try_from(offset).map_err(|_| Error::InvalidValue {
        what: "ELF segment offset",
        value: offset.to_string(),
    })?;
    let len = usize::try_from(size).map_err(|_| Error::InvalidValue {
        what: "ELF segment size",
        value: size.to_string(),
    })?;
    let end = start.checked_add(len).ok_or(Error::InvalidOffset {
        what: "ELF segment",
        offset: start,
    })?;
    input
        .get(start..end)
        .map(|segment| segment.to_vec())
        .ok_or(Error::InvalidOffset {
            what: "ELF segment",
            offset: start,
        })
}

#[cfg(not(feature = "elf"))]
fn amd_elf_payload(_input: &[u8]) -> Result<Vec<u8>> {
    Err(Error::InvalidValue {
        what: "ELF input",
        value: "amd-compress ELF input requires the elf feature".to_owned(),
    })
}
