// SPDX-License-Identifier: GPL-2.0-only

use cbfstool_rs::{Error, Result};

pub(crate) fn read_u8(data: &[u8], offset: usize, what: &'static str) -> Result<u8> {
    data.get(offset)
        .copied()
        .ok_or(Error::InvalidOffset { what, offset })
}

pub(crate) fn read_u16(data: &[u8], offset: usize, what: &'static str) -> Result<u16> {
    let bytes = data
        .get(offset..offset + 2)
        .ok_or(Error::InvalidOffset { what, offset })?;
    Ok(u16::from_le_bytes([bytes[0], bytes[1]]))
}

pub(crate) fn read_u32(data: &[u8], offset: usize, what: &'static str) -> Result<u32> {
    let bytes = data
        .get(offset..offset + 4)
        .ok_or(Error::InvalidOffset { what, offset })?;
    Ok(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
}

pub(crate) fn read_u64(data: &[u8], offset: usize, what: &'static str) -> Result<u64> {
    let bytes = data
        .get(offset..offset + 8)
        .ok_or(Error::InvalidOffset { what, offset })?;
    Ok(u64::from_le_bytes([
        bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
    ]))
}

pub(crate) fn push_le_u16(output: &mut Vec<u8>, value: u16) {
    output.extend_from_slice(&value.to_le_bytes());
}

pub(crate) fn push_le_u32(output: &mut Vec<u8>, value: u32) {
    output.extend_from_slice(&value.to_le_bytes());
}

pub(crate) fn write_le_u32(
    data: &mut [u8],
    offset: usize,
    value: u32,
    what: &'static str,
) -> Result<()> {
    let bytes = data
        .get_mut(offset..offset + 4)
        .ok_or(Error::InvalidOffset { what, offset })?;
    bytes.copy_from_slice(&value.to_le_bytes());
    Ok(())
}

pub(crate) fn fixed_string(bytes: &[u8]) -> String {
    bytes
        .iter()
        .copied()
        .take_while(|byte| *byte != 0)
        .map(char::from)
        .collect()
}

pub(crate) fn hex_string(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect::<Vec<_>>()
        .join("")
}

pub(crate) fn parse_hex_bytes(value: &str) -> Result<Vec<u8>> {
    let hex = value.strip_prefix("0x").unwrap_or(value);
    if !hex.len().is_multiple_of(2) {
        return Err(Error::InvalidValue {
            what: "hex bytes",
            value: value.to_owned(),
        });
    }
    (0..hex.len())
        .step_by(2)
        .map(|index| {
            u8::from_str_radix(&hex[index..index + 2], 16).map_err(|err| Error::InvalidValue {
                what: "hex bytes",
                value: err.to_string(),
            })
        })
        .collect()
}

pub(crate) fn usize_to_u32(value: usize, what: &'static str) -> Result<u32> {
    u32::try_from(value).map_err(|_| Error::InvalidValue {
        what,
        value: value.to_string(),
    })
}
