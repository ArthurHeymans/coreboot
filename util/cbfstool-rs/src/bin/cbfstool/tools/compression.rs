// SPDX-License-Identifier: GPL-2.0-only

use std::fs;
use std::path::Path;
use std::time::Instant;

use cbfstool_rs::{Compression, Error, Result};

use crate::cli::{CbfsCompressionArgs, CbfsCompressionCommand};

pub(crate) fn cbfs_compression_tool(args: CbfsCompressionArgs) -> Result<()> {
    match args.command {
        CbfsCompressionCommand::Benchmark => benchmark_compression(),
        CbfsCompressionCommand::Compress {
            input,
            output,
            algorithm,
        } => compress_file(&input, &output, algorithm, true),
        CbfsCompressionCommand::Rawcompress {
            input,
            output,
            algorithm,
        } => compress_file(&input, &output, algorithm, false),
    }
}

fn benchmark_compression() -> Result<()> {
    const BENCHMARK_SIZE: usize = 10 * 1024 * 1024;
    const BENCHMARK_TEXT: &[u8] = b"cbfs-compression-tool benchmark data\n";

    let data = BENCHMARK_TEXT
        .iter()
        .copied()
        .cycle()
        .take(BENCHMARK_SIZE)
        .collect::<Vec<_>>();

    [
        Compression::None,
        Compression::Lzma,
        Compression::Lz4,
        Compression::Zstd,
    ]
    .into_iter()
    .try_for_each(|algorithm| {
        println!("measuring '{}'", algorithm.name());
        let start = Instant::now();
        let (stored_algorithm, compressed) =
            cbfstool_rs::cbfs::compression::compress(algorithm, &data)?;
        println!(
            "compressing {} bytes to {} with {} took {:.3} seconds",
            data.len(),
            compressed.len(),
            stored_algorithm.name(),
            start.elapsed().as_secs_f64()
        );
        Ok(())
    })
}

fn compress_file(
    input: &Path,
    output: &Path,
    algorithm: Compression,
    write_header: bool,
) -> Result<()> {
    let data = fs::read(input)?;
    let (stored_algorithm, compressed) =
        cbfstool_rs::cbfs::compression::compress(algorithm, &data)?;
    let mut output_data = Vec::with_capacity(compressed.len() + usize::from(write_header) * 8);
    if write_header {
        let original_len = u32::try_from(data.len()).map_err(|_| Error::InvalidValue {
            what: "input size",
            value: format!("{} exceeds cbfs-compression-tool header range", data.len()),
        })?;
        output_data.extend_from_slice(&stored_algorithm.raw().to_le_bytes());
        output_data.extend_from_slice(&original_len.to_le_bytes());
    }
    output_data.extend_from_slice(&compressed);
    fs::write(output, output_data)?;
    Ok(())
}
