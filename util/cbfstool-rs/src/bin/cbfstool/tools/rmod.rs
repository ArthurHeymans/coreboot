// SPDX-License-Identifier: GPL-2.0-only

use std::fs;

use cbfstool_rs::{Error, Result};

use crate::cli::RmodArgs;
use crate::common::{push_le_u16, push_le_u32};

pub(crate) fn rmod_tool(args: RmodArgs) -> Result<()> {
    use object::{Object, ObjectSegment};

    let input = fs::read(&args.input)?;
    let file = object::File::parse(input.as_slice()).map_err(|err| Error::InvalidValue {
        what: "rmod input ELF",
        value: err.to_string(),
    })?;
    if file.kind() != object::ObjectKind::Executable {
        return Err(Error::InvalidValue {
            what: "rmod input ELF",
            value: "input is not an executable ELF".to_owned(),
        });
    }
    let mut load_segments = file.segments().filter(|segment| segment.address() != 0);
    let segment = load_segments.next().ok_or(Error::InvalidValue {
        what: "rmod input ELF",
        value: "no loadable program segment found".to_owned(),
    })?;
    if load_segments.next().is_some() {
        return Err(Error::InvalidValue {
            what: "rmod input ELF",
            value: "multiple loadable program segments are not supported yet".to_owned(),
        });
    }
    let payload = segment.data().map_err(|err| Error::InvalidValue {
        what: "rmod input ELF segment",
        value: err.to_string(),
    })?;
    let link_start = u32::try_from(segment.address()).map_err(|_| Error::InvalidValue {
        what: "rmod link address",
        value: format!("{:#x}", segment.address()),
    })?;
    let mem_size = u32::try_from(segment.size()).map_err(|_| Error::InvalidValue {
        what: "rmod program size",
        value: segment.size().to_string(),
    })?;
    let entry = u32::try_from(file.entry()).map_err(|_| Error::InvalidValue {
        what: "rmod entry point",
        value: format!("{:#x}", file.entry()),
    })?;

    let payload_begin = 56_u32;
    let payload_end = payload_begin
        .checked_add(
            u32::try_from(payload.len()).map_err(|_| Error::InvalidValue {
                what: "rmod payload size",
                value: payload.len().to_string(),
            })?,
        )
        .ok_or(Error::InvalidValue {
            what: "rmod payload size",
            value: payload.len().to_string(),
        })?;
    let mut output = Vec::with_capacity(payload_end as usize);
    push_le_u16(&mut output, 0xf8fe);
    output.push(1);
    output.push(0);
    push_le_u32(&mut output, payload_begin);
    push_le_u32(&mut output, payload_end);
    push_le_u32(&mut output, payload_end);
    push_le_u32(&mut output, payload_end);
    push_le_u32(&mut output, link_start);
    push_le_u32(&mut output, mem_size);
    push_le_u32(&mut output, entry);
    (0..7).for_each(|_| push_le_u32(&mut output, 0));
    output.extend_from_slice(payload);
    fs::write(args.output, output)?;
    Ok(())
}

#[cfg(not(feature = "elf"))]
pub(crate) fn rmod_tool(_args: RmodArgs) -> Result<()> {
    Err(Error::InvalidValue {
        what: "rmod",
        value: "rmod requires the elf feature".to_owned(),
    })
}
