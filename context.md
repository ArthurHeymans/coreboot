# Code Context

## Files Retrieved

1. `src/drivers/amd/atombios/atombios_driver.c` (lines 79-184, 293-314, 1366-1435, 2975-3400, 4125-4245) - table indices, path state, GPIO lookup, connector discovery/fallback, and mode selection.
2. `src/drivers/amd/atombios/atombios.h` (lines 3200-3312, 3898-3945, 8819-8849) - IntegratedSystemInfo V2, LVDS native timing, and SupportedDevicesInfo layouts already available locally.
3. `/home/arthur/src/linux/drivers/gpu/drm/radeon/radeon_atombios.c` (lines 47-162, 520-838, 855-1000, 1611-1768) - Linux GPIO quirks, Object_Header discovery including IGP PCIE_CONNECTOR resolution, legacy-table fallback, and LVDS native-mode parsing.
4. `/home/arthur/src/linux/drivers/gpu/drm/radeon/radeon_connectors.c` (references found at lines 808, 1441, 1715) - Linux connector-specific mode validation entry points; outside the requested comparison file but relevant to the mode-validation gap.
5. `/home/arthur/src/linux/drivers/gpu/drm/radeon/atombios_dp.c` (reference found at line 469) - DP bandwidth mode validation helper.

## Key Code

### Recommended smallest next slice: remove the index-based GPIO fallback

**Severity: high (unsafe MMIO / wrong DDC bus).** In `atombios_discover_display_paths()`, coreboot assigns `GPIO_I2C_Info.asGPIO_Info[i]` to display path `i` when connector records and SupportedDevicesInfo did not supply DDC (`atombios_driver.c:3354-3373`). There is no AtomBIOS relationship between display-path ordinal and GPIO-table ordinal. This can make `atombios_read_edid()` bit-bang unrelated GPIO registers.

```c
/* current unsafe assumption, atombios_driver.c:3362-3371 */
if (i < ATOM_MAX_SUPPORTED_DEVICE) {
        paths[i].i2c_line = i2c_info->asGPIO_Info[i].sucI2cId.ucAccess;
        paths[i].i2c_valid = 1;
}
```

Linux never does this. It resolves the exact I2C ID from an `ATOM_I2C_RECORD` and then calls `radeon_lookup_i2c_gpio()` (`radeon_atombios.c:748-783`), or uses the exact `sucI2cId` from SupportedDevicesInfo (`radeon_atombios.c:930-971`). Its only generic missing-DDC fallback is a narrowly scoped board/ASIC quirk to ID `0x93` (`radeon_atombios.c:427-437`).

**Minimal implementation:** delete `atombios_driver.c:3354-3373`; do not invent another fallback. Existing `i2c_valid` checks at `atombios_driver.c:4145-4151` then safely skip probing unresolved paths. This is a root-cause fix, buildable without hardware, and should get one parser-level/self-check proving an object path without an I2C record remains invalid. No new type or dependency is needed.

### Next practical feature slice after safety: native LVDS timing

**Severity: high functional gap (internal LVDS panels cannot be selected).** Coreboot explicitly skips LVDS during EDID probing (`atombios_driver.c:4145-4151`) and also excludes LVDS/internal panels from default path selection (`atombios_driver.c:4200-4227`). Consequently a laptop with only LVDS reaches “no usable display path.” The header already defines `ATOM_LVDS_INFO`, `ATOM_LVDS_INFO_V12`, and `LCDPANEL_CAP_READ_EDID` (`atombios.h:3898-3945`), but the driver has no LVDS_Info data-table index or parser.

Linux reference `radeon_atombios_get_lvds_info()` (`radeon_atombios.c:1616-1768`) converts `sLCDTiming` into the native mode:

- `usPixClk * 10` kHz
- active: `usHActive`, `usVActive`
- blanking: `usHBlanking_Time`, `usVBlanking_Time`
- sync offsets/widths: `usHSyncOffset`, `usHSyncWidth`, `usVSyncOffset`, `usVSyncWidth`
- polarity/interlace flags from `susModeMiscInfo.usAccess`.

Small implementation boundary: add the V1 `LVDS_Info` data index, a pure `atombios_get_lvds_native_mode(ctx, struct edid_mode *)`, and use it only when selecting an LVDS path after external EDID probes fail. Validate nonzero clock/active dimensions and sane blanking/sync ranges before modeset. Fake-EDID patch records, DRR, panel physical dimensions, and runtime power management can wait.

### SupportedDevicesInfo is enrichment, not a true fallback

**Severity: medium-high compatibility gap.** `atombios_apply_supported_device_info()` (`atombios_driver.c:2985-3068`) correctly avoids overriding Object_Header connector types and fills missing I2C, but it only iterates paths already created by Object_Header. If Object_Header is absent/old, `atombios_discover_display_paths()` returns zero at lines 3189-3203 and SupportedDevicesInfo is never able to create paths. Linux instead has a separate `radeon_get_atom_connector_info_from_supported_devices_table()` (`radeon_atombios.c:855-1000+`) used as the legacy fallback.

The eventual fix should be a separate discovery function that creates/merges connector entries by device index from `ATOM_SUPPORTED_DEVICES_INFO`/`_2d1` (`atombios.h:8819-8849`), preserving Linux’s `frev > 1` max-device rule and DDC IDs. Do not stretch `atombios_apply_supported_device_info()` into both creation and enrichment; first make discovery call object parsing, then SupportedDevices parsing only if object parsing yields no usable paths. This is larger than the GPIO deletion and LVDS parser and should not be bundled with the safety fix.

### IGP PCIE_CONNECTOR is unresolved

**Severity: medium-high on older IGP/APU VBIOSes.** Coreboot maps unknown connector object IDs to type 0 (`atombios_driver.c:3236-3272`); it has no `CONNECTOR_OBJECT_ID_PCIE_CONNECTOR` handling. Linux detects IGP + PCIE_CONNECTOR, parses `IntegratedSystemInfo`, selects `ulDDISlot1Config` or `ulDDISlot2Config` by connector enum, then derives real connector ID from bits 23:16 and lane info from bits 15:0 (`radeon_atombios.c:582-625`). `ATOM_INTEGRATED_SYSTEM_INFO_V2` already exists locally (`atombios.h:3272-3312`), but coreboot currently lacks an IntegratedSystemInfo index and a reliable IGP-family predicate in this path. Resolve this as a later isolated parser slice; confirm which table revisions/device IDs are in scope before coding.

### Mode validation is too narrow

**Severity: medium.** Coreboot only rejects >165 MHz single-link TMDS (`atombios_driver.c:4155-4163`) and checks DP bandwidth during link preparation (`atombios_driver.c:2046-2087, 2390-2411`). It does not validate zero/overflowing EDID timing fields, CRTC field widths, LVDS native bounds, DAC clock limits, or framebuffer/resource arithmetic before programming `SetCRTC_UsingDTDTiming` and scanout.

Linux distributes validation by connector: LVDS `radeon_connectors.c:808`, DVI `:1441`, DP `:1715`, with DP bandwidth in `atombios_dp.c:469`. For firmware, do not port DRM status machinery. Add one small pure validator over `struct edid_mode` at the trust boundary after EDID/LVDS decoding and before `do_modeset`: nonzero clock/active, sync intervals within blanking, totals without 16-bit overflow, and connector transport clock/bandwidth checks. This naturally unit-tests without hardware.

### GPIO parsing also lacks bounds hardening

**Severity: medium.** `atombios_lookup_i2c_gpio()` computes `(size - header) / entry` without first checking `size >= sizeof(header)` (`atombios_driver.c:1404-1435`), and `atombios_i2c_bus_from_gpio()` shifts `1u << firmware_shift` without checking shift `< 32` (`atombios_driver.c:1377-1402`). Malformed ROM data can underflow the count or invoke undefined shifts. Linux’s old implementation has similar assumptions (`radeon_atombios.c:85-162`), so this is safety hardening rather than parity. A pure validation guard is appropriate before constructing a bus.

## Architecture

`atombios_init()` initializes V1/V2 master-table indices, runs ASIC init, calls `atombios_discover_display_paths()`, probes DDC/AUX, decodes one EDID into `struct edid_mode`, then programs CRTC/PLL/encoder/transmitter. Object_Header is the primary topology source; connector records provide exact DDC and HPD IDs; SupportedDevicesInfo currently only enriches existing paths. GPIO IDs are resolved later by `atombios_lookup_i2c_gpio()` into MMIO bus records.

The ordering should remain: exact object record -> exact SupportedDevices entry -> unresolved. Never infer GPIO by array position. Native LVDS timing is a mode source parallel to EDID, not topology discovery. IGP PCIE_CONNECTOR resolution belongs inside object-path parsing before connector-type filtering.

## Start Here

Open `src/drivers/amd/atombios/atombios_driver.c` at lines 3354-3373 first. Removing this unsupported path-to-GPIO ordinal mapping is the smallest safe, Linux-aligned slice and prevents wrong-register access without requiring hardware. Then implement LVDS native timing as the next practical feature slice.

```acceptance-report
{
  "criteriaSatisfied": [
    {
      "id": "criterion-1",
      "status": "satisfied",
      "evidence": "Concrete severity-ranked findings cite coreboot and Linux paths and exact functions/line ranges for GPIO fallback, LVDS_Info, SupportedDevicesInfo, IGP PCIE_CONNECTOR, and mode validation."
    }
  ],
  "changedFiles": [
    "context.md"
  ],
  "testsAddedOrUpdated": [],
  "commandsRun": [
    {
      "command": "targeted grep/read inspection of the three requested files and Linux mode-validation references",
      "result": "passed",
      "summary": "Located exact parser, fallback, LVDS, IGP, GPIO, and mode-selection code."
    },
    {
      "command": "jj root && jj status --no-pager",
      "result": "passed",
      "summary": "Confirmed Jujutsu repository; pre-existing working-copy modifications are present in Kconfig and atombios_driver.c."
    }
  ],
  "validationOutput": [
    "No source files edited; findings written to /home/arthur/src/coreboot/context.md.",
    "Linux has exact-record GPIO resolution and no ordinal path-to-GPIO fallback."
  ],
  "residualRisks": [
    "No hardware validation was performed.",
    "Working copy already contains unrelated modifications to src/drivers/amd/atombios/Kconfig and atombios_driver.c; preserve them during implementation.",
    "Exact IntegratedSystemInfo table index/revision support must be confirmed before implementing IGP PCIE_CONNECTOR resolution."
  ],
  "noStagedFiles": true,
  "diffSummary": "Added scouting report only; no requested source files changed.",
  "reviewFindings": [
    "high: src/drivers/amd/atombios/atombios_driver.c:3354-3373 - display-path ordinal is incorrectly treated as GPIO_I2C_Info ordinal, risking wrong MMIO access.",
    "high: src/drivers/amd/atombios/atombios_driver.c:4145-4151,4200-4227 - LVDS is excluded from both probing and fallback selection despite native timing tables being available.",
    "medium-high: src/drivers/amd/atombios/atombios_driver.c:2985-3068,3189-3203 - SupportedDevicesInfo cannot act as fallback when Object_Header discovery fails.",
    "medium-high: src/drivers/amd/atombios/atombios_driver.c:3236-3272 - IGP PCIE_CONNECTOR remains unknown instead of being resolved through IntegratedSystemInfo."
  ],
  "manualNotes": "Recommended first patch is deletion of unsafe GPIO ordinal fallback; recommended next feature patch is a pure LVDS native-mode parser plus selection."
}
```
