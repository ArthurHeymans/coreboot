// SPDX-License-Identifier: GPL-2.0-only

use std::io::{Cursor, Read, Write};

use crate::cbfs::Compression;
use crate::error::{Error, Result};

/// Decompresses CBFS entry data with the algorithm recorded in its compression attribute.
pub fn decompress(algorithm: Compression, data: &[u8], expected_size: usize) -> Result<Vec<u8>> {
    let output = match algorithm {
        Compression::None => data.to_vec(),
        Compression::Lzma => decompress_lzma(data)?,
        Compression::Lz4 => decompress_lz4(data)?,
        Compression::Zstd => decompress_zstd(data)?,
        Compression::Unknown(raw) => return Err(Error::UnsupportedCompression(raw)),
    };

    if output.len() != expected_size {
        return Err(Error::InvalidValue {
            what: "decompressed size",
            value: format!("expected {expected_size} bytes, got {}", output.len()),
        });
    }
    Ok(output)
}

/// Compresses CBFS entry data.
///
/// If compression does not make the data smaller, the original data is returned
/// with [`Compression::None`], matching cbfstool's effective behavior while
/// keeping the library result explicit.
pub fn compress(algorithm: Compression, data: &[u8]) -> Result<(Compression, Vec<u8>)> {
    let compressed = match algorithm {
        Compression::None => return Ok((Compression::None, data.to_vec())),
        Compression::Lzma => compress_lzma(data)?,
        Compression::Lz4 => compress_lz4(data)?,
        Compression::Zstd => compress_zstd(data),
        Compression::Unknown(raw) => return Err(Error::UnsupportedCompression(raw)),
    };

    if compressed.len() >= data.len() {
        Ok((Compression::None, data.to_vec()))
    } else {
        Ok((algorithm, compressed))
    }
}

fn decompress_lzma(data: &[u8]) -> Result<Vec<u8>> {
    let mut input = Cursor::new(data);
    let mut output = Vec::new();
    lzma_rs::lzma_decompress(&mut input, &mut output).map_err(|err| Error::Compression {
        algorithm: "LZMA",
        message: err.to_string(),
    })?;
    Ok(output)
}

fn compress_lzma(data: &[u8]) -> Result<Vec<u8>> {
    let mut input = Cursor::new(data);
    let mut output = Vec::new();
    lzma_rs::lzma_compress(&mut input, &mut output).map_err(|err| Error::Compression {
        algorithm: "LZMA",
        message: err.to_string(),
    })?;
    Ok(output)
}

fn decompress_lz4(data: &[u8]) -> Result<Vec<u8>> {
    let mut decoder = lz4_flex::frame::FrameDecoder::new(Cursor::new(data));
    let mut output = Vec::new();
    decoder
        .read_to_end(&mut output)
        .map_err(|err| Error::Compression {
            algorithm: "LZ4",
            message: err.to_string(),
        })?;
    Ok(output)
}

fn compress_lz4(data: &[u8]) -> Result<Vec<u8>> {
    let mut encoder = lz4_flex::frame::FrameEncoder::new(Vec::new());
    encoder.write_all(data).map_err(|err| Error::Compression {
        algorithm: "LZ4",
        message: err.to_string(),
    })?;
    encoder.finish().map_err(|err| Error::Compression {
        algorithm: "LZ4",
        message: err.to_string(),
    })
}

fn decompress_zstd(data: &[u8]) -> Result<Vec<u8>> {
    rust_zstd::decompress(data).map_err(|message| Error::Compression {
        algorithm: "ZSTD",
        message,
    })
}

fn compress_zstd(data: &[u8]) -> Vec<u8> {
    rust_zstd::compress_to_vec(data)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_supported_algorithms() -> Result<()> {
        let data = b"coreboot cbfs compression test data".repeat(128);
        [Compression::Lzma, Compression::Lz4, Compression::Zstd]
            .into_iter()
            .try_for_each(|algorithm| {
                let (stored_algorithm, compressed) = compress(algorithm, &data)?;
                let decompressed = decompress(stored_algorithm, &compressed, data.len())?;
                assert_eq!(decompressed, data);
                Ok(())
            })
    }
}
