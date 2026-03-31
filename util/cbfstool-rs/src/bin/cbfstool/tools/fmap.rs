// SPDX-License-Identifier: GPL-2.0-only

use std::fs;
use std::io::Read;
use std::path::Path;

use cbfstool_rs::{Error, FlashmapDescriptor, Result};

use crate::cli::FmapToolArgs;

pub(crate) fn fmap_tool(args: FmapToolArgs) -> Result<()> {
    let input = if args.input == Path::new("-") {
        let mut input = String::new();
        std::io::stdin().read_to_string(&mut input)?;
        input
    } else {
        fs::read_to_string(&args.input)?
    };
    let descriptor = FlashmapDescriptor::parse(&input)?;
    let fmap = descriptor.to_fmap_bytes()?;
    fs::write(&args.output, &fmap)?;

    if let Some(header) = args.header.as_deref() {
        write_fmap_header(header, &descriptor, fmap.len())?;
    }

    let cbfs_regions = descriptor.cbfs_section_names().join(",");
    eprintln!(
        "SUCCESS: Wrote {} bytes to file '{}'{}",
        fmap.len(),
        args.output.display(),
        if args.header.is_some() {
            " (and generated header)"
        } else {
            ""
        }
    );
    eprintln!("The sections containing CBFSes are: ");
    println!("{cbfs_regions}");
    if let Some(regions) = args.regions.as_deref() {
        fs::write(regions, format!("{cbfs_regions}\n"))?;
    }
    Ok(())
}

fn write_fmap_header(path: &Path, descriptor: &FlashmapDescriptor, fmap_size: usize) -> Result<()> {
    let fmap_offset = descriptor
        .absolute_offset("FMAP")
        .ok_or(Error::InvalidValue {
            what: "FMD descriptor",
            value: "missing FMAP section".to_owned(),
        })?;
    let mut header = String::new();
    header.push_str("#ifndef FMAPTOOL_GENERATED_HEADER_H_\n");
    header.push_str("#define FMAPTOOL_GENERATED_HEADER_H_\n\n");
    header.push_str(&format!("#define FMAP_OFFSET {fmap_offset:#x}\n"));
    header.push_str(&format!("#define FMAP_SIZE {fmap_size:#x}\n\n"));
    header.push_str("#define FMAP_TERMINAL_SECTIONS \"");
    header.push_str(&descriptor.terminal_section_names().join(" "));
    header.push_str(" \"\n\n");
    for (name, offset, size) in descriptor.section_defines() {
        header.push_str(&format!("#define FMAP_SECTION_{name}_START {offset:#x}\n"));
        header.push_str(&format!("#define FMAP_SECTION_{name}_SIZE {size:#x}\n"));
    }
    header.push_str("\n#endif\n");
    fs::write(path, header)?;
    Ok(())
}
