// SPDX-License-Identifier: GPL-2.0-only

use std::ffi::CStr;

pub(crate) fn nul_terminated_string(bytes: &[u8]) -> String {
    let bytes = CStr::from_bytes_until_nul(bytes)
        .map(CStr::to_bytes)
        .unwrap_or(bytes);
    String::from_utf8_lossy(bytes).into_owned()
}
