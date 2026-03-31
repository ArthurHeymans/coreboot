// SPDX-License-Identifier: GPL-2.0-only

use std::fs;

use cbfstool_rs::{Error, Result};

use crate::cli::{ElogArgs, ElogCommand};
use crate::common::{hex_string, parse_hex_bytes, usize_to_u32};

pub(crate) fn elog_tool(args: ElogArgs) -> Result<()> {
    let file = args.file.ok_or(Error::InvalidValue {
        what: "ELOG file",
        value: "-f/--file is required; flashrom RW_ELOG access is not supported".to_owned(),
    })?;
    let mut data = fs::read(&file)?;
    verify_elog(&data)?;
    match args.command {
        ElogCommand::List => list_elog(&data),
        ElogCommand::Clear => {
            let used = elog_used_size(&data)?;
            data[8..].fill(0xff);
            let used = usize_to_u32(used, "ELOG used size")?;
            init_elog_event(&mut data, 8, 0x16, &used.to_le_bytes())?;
            fs::write(&file, data)?;
            Ok(())
        }
        ElogCommand::Add {
            event_type,
            event_data,
        } => {
            let bytes = event_data
                .as_deref()
                .map(parse_hex_bytes)
                .transpose()?
                .unwrap_or_default();
            let offset = 8 + elog_used_size(&data)?;
            init_elog_event(&mut data, offset, event_type, &bytes)?;
            fs::write(&file, data)?;
            Ok(())
        }
    }
}

fn verify_elog(data: &[u8]) -> Result<()> {
    if data.len() < 8 || data.get(..4) != Some(b"ELOG".as_slice()) || data[4] != 1 || data[5] != 8 {
        return Err(Error::InvalidMagic {
            what: "ELOG",
            offset: 0,
        });
    }
    Ok(())
}

fn elog_used_size(data: &[u8]) -> Result<usize> {
    let mut offset = 8;
    while offset + 8 <= data.len() {
        let event_type = data[offset];
        let length = data[offset + 1] as usize;
        if event_type == 0xff || length == 0 {
            break;
        }
        if length <= 8 || length > 255 || offset + length > data.len() {
            return Err(Error::InvalidValue {
                what: "ELOG event length",
                value: length.to_string(),
            });
        }
        offset += length;
    }
    Ok(offset - 8)
}

fn list_elog(data: &[u8]) -> Result<()> {
    let mut offset = 8;
    let mut count = 0;
    while offset + 8 <= data.len() {
        let event_type = data[offset];
        let length = data[offset + 1] as usize;
        if event_type == 0xff || length == 0 || length <= 8 || offset + length > data.len() {
            break;
        }
        let payload = &data[offset + 8..offset + length - 1];
        println!(
            "{} | {} | {}{}",
            count,
            elog_timestamp(&data[offset..offset + 8]),
            elog_event_name(event_type),
            elog_event_data(event_type, payload)
        );
        count += 1;
        offset += length;
    }
    Ok(())
}

fn init_elog_event(data: &mut [u8], offset: usize, event_type: u8, payload: &[u8]) -> Result<()> {
    let length = 8_usize
        .checked_add(payload.len())
        .and_then(|value| value.checked_add(1))
        .ok_or(Error::InvalidValue {
            what: "ELOG event length",
            value: payload.len().to_string(),
        })?;
    if length > 255 || offset + length > data.len() {
        return Err(Error::InvalidValue {
            what: "ELOG buffer space",
            value: format!("need {length} bytes at offset {offset:#x}"),
        });
    }
    let (year, month, day, hour, minute, second) = utc_now_components();
    data[offset] = event_type;
    data[offset + 1] = length as u8;
    data[offset + 2] = bin_to_bcd((year % 100) as u8);
    data[offset + 3] = bin_to_bcd(month);
    data[offset + 4] = bin_to_bcd(day);
    data[offset + 5] = bin_to_bcd(hour);
    data[offset + 6] = bin_to_bcd(minute);
    data[offset + 7] = bin_to_bcd(second);
    data[offset + 8..offset + 8 + payload.len()].copy_from_slice(payload);
    data[offset + length - 1] = 0;
    let checksum = data[offset..offset + length]
        .iter()
        .fold(0_u8, |sum, byte| sum.wrapping_add(*byte));
    data[offset + length - 1] = checksum.wrapping_neg();
    Ok(())
}

fn elog_timestamp(header: &[u8]) -> String {
    format!(
        "20{:02x}-{:02x}-{:02x} {:02x}:{:02x}:{:02x}",
        header[2], header[3], header[4], header[5], header[6], header[7]
    )
}

fn elog_event_name(kind: u8) -> &'static str {
    match kind {
        0x16 => "Log area cleared",
        0x17 => "System boot",
        0x81 => "Kernel Event",
        0x90 => "OS Boot",
        0x91 => "EC Event",
        0x96 => "Power On",
        0x97 => "Power Button",
        0x9a => "System Reset",
        0x9d => "ACPI Enter",
        0x9e => "ACPI Wake",
        0x9f => "Wake Source",
        0xa2 => "Management Engine",
        0xff => "End of log",
        _ => "Unknown",
    }
}

fn elog_event_data(kind: u8, payload: &[u8]) -> String {
    if payload.is_empty() {
        return String::new();
    }
    if matches!(kind, 0x16 | 0x17) && payload.len() == 4 {
        return format!(
            " | {}",
            u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]])
        );
    }
    format!(" | {}", hex_string(payload))
}

fn bin_to_bcd(value: u8) -> u8 {
    ((value / 10) << 4) | (value % 10)
}

fn utc_now_components() -> (i32, u8, u8, u8, u8, u8) {
    let seconds = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |duration| duration.as_secs());
    let days = (seconds / 86_400) as i64;
    let secs_of_day = seconds % 86_400;
    let (year, month, day) = civil_from_days(days);
    (
        year,
        month,
        day,
        (secs_of_day / 3600) as u8,
        ((secs_of_day % 3600) / 60) as u8,
        (secs_of_day % 60) as u8,
    )
}

fn civil_from_days(days: i64) -> (i32, u8, u8) {
    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = mp + if mp < 10 { 3 } else { -9 };
    ((y + i64::from(m <= 2)) as i32, m as u8, d as u8)
}
