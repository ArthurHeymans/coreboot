# Code Context

## Files Retrieved

1. `src/drivers/amd/atombios/atombios_driver.c` (lines 60-216) — v1/v2 command indices and unavailable v2 commands.
2. `src/drivers/amd/atombios/atombios_driver.c` (lines 360-620) — GPU profile model; no DCN scanout/profile exists.
3. `src/drivers/amd/atombios/atombios_driver.c` (lines 3740-4170) — DIG, transmitter, CRTC-source, and timing helpers with supported parameter revisions.
4. `src/drivers/amd/atombios/atombios_driver.c` (lines 4260-4601) — AtomFirmware classification, diagnostics, and unconditional return.
5. `src/drivers/amd/atombios/atombios_driver.c` (lines 4603-5090) — complete legacy discovery/modesetting flow following diagnostics.
6. `src/drivers/amd/atombios/atombios_driver.c` (lines 5070-5190) — PCI registration of `0x15d8` in the v2 driver.
7. `src/drivers/amd/atombios/atombios_v2.c` (lines 1-330) — bounded v2 ROM, table, DisplayObjectInfo, FirmwareInfo, and DCE_Info parsers.
8. `src/drivers/amd/atombios/atomfirmware.h` (lines 1-220) — v2 master-table layout and object-record ABI.
9. `src/drivers/amd/atombios/atombios.h` (lines 723-748, 1134-1140, 1559-1575, 2015-2033) — legacy parameter layouts currently reused by command helpers.

## Key Code

### Immediate blocker

**Blocker:** `src/drivers/amd/atombios/atombios_driver.c:4588-4600`

For `1002:15d8`, `expected_v2` is true because `atomfirmware_display_ip()` maps it to `ATOMFIRMWARE_DCN10`. Initialization creates a minimal `v2_ctx`, runs `atomfirmware_diagnostics()`, then unconditionally returns:

```c
atombios_init_cmd_indices_v2(&cmd_idx);
atomfirmware_diagnostics(&v2_ctx, vbios_size, dev->device);
return;
```

Thus no ASIC initialization, display discovery, EDID, command execution, modeset, or framebuffer registration can occur.

### Smallest concrete change to proceed beyond diagnostics

The smallest safe change is **not** merely deleting `return`. Add a v2 continuation immediately after diagnostics which:

1. Allocates and attaches `v2_ctx.scratch` and `scratch_size_bytes`.
2. Executes v2 `ASIC_Init` using its AtomFirmware parameter ABI.
3. Converts `atom_v2_parse_display_object_info()` results into `struct atombios_display_path`.
4. Continues through a **v2/DCN-specific path**, rather than falling into legacy object-table casts and Evergreen register programming.

At minimum this touches:

- `src/drivers/amd/atombios/atombios_driver.c`
- `src/drivers/amd/atombios/atomfirmware.h` to add the missing v2 command parameter structures, unless kept locally.

`atombios_v2.c` already provides sufficient bounded parsers; it need not change merely to get past diagnostics.

### Why deleting `return` is unsafe

The legacy continuation expects `ctx` from `atom_parse()`, but the v2 branch only initializes `v2_ctx.card`, `bios`, `cmd_table`, and `data_table`. After the return, the code uses the unrelated/uninitialized `ctx` variable.

Even if redirected to `ctx = &v2_ctx`, legacy flow still assumes:

- v1 object-table layouts in `atombios_discover_display_paths()`;
- v1 `ASIC_Init` parameters in `atom_asic_init()`;
- AVIVO/Evergreen scanout registers in `atombios_program_framebuffer()`;
- Evergreen color/unblank/postmode registers;
- commands deliberately marked absent in `atombios_init_cmd_indices_v2()`.

### Existing helpers that can be reused

- `atom_v2_parse_rom_header()`, `atom_v2_get_table()`, and all three v2 data parsers in `atombios_v2.c`.
- `atom_execute_table()` and the ATOM bytecode interpreter, provided a complete `atom_context`, scratch space, and correct parameter allocation are supplied.
- `atombios_dig_encoder_setup()` already has content-revision 5 handling through `DIG_ENCODER_CONTROL_PARAMETERS_V5`.
- `atombios_transmitter_control()` already has content-revision 6 handling through `DIG_TRANSMITTER_CONTROL_PARAMETERS_V1_6`.
- `atombios_set_pixel_clock()` has content-revision 7 handling through `PIXEL_CLOCK_PARAMETERS_V7`.
- `atombios_select_crtc_source()` can structurally handle revision 3 and later via its default case, but DCN firmware revision/semantics must be verified before reuse.
- Generic EDID decoding and DP link-status/training algorithms may be reusable after v2 I2C/AUX command wrappers use the correct parameter ABI.
- `atombios_get_bar()`, framebuffer registration, and scratch allocation logic are generation-independent.

### Helpers that cannot be reused unchanged

1. **`atom_asic_init()`**  
   It uses legacy `ASIC_INIT_PS_ALLOCATION` / `ASIC_INIT_PS_ALLOCATION_V1_2`. AtomFirmware `ASIC_Init` is format/content **2.1**, not the legacy **1.x** allocation. A dedicated v2 wrapper and parameter structure are required.

2. **`atombios_discover_display_paths()`**  
   It casts v1 `ATOM_OBJECT_HEADER` and supported-device tables. Use `atom_v2_parse_display_object_info()` and translate its paths instead.

3. **`atombios_program_framebuffer()` and both implementations beneath it**  
   They support only `ATOMBIOS_SCANOUT_AVIVO` and `ATOMBIOS_SCANOUT_EVERGREEN`. DCN1 uses hubp/dpp/opp/mpc/otg-style registers and addressing, not Evergreen DCP/CRTC scanout registers.

4. **`atombios_program_crtc_postmode()`**  
   It calls `atombios_yuv_setup()`, `atombios_set_crtc_overscan()`, and `atombios_disable_scaler()`. Their indices are explicitly `-1` for v2 at lines 169-171. It therefore cannot remain a mandatory chained operation.

5. **`atombios_evergreen_unblank_scanout()` and Evergreen color setup**  
   Wrong register generation for DCN1.

6. **Legacy GPIO/DDC assumptions**  
   The v2 index map points both `data_gpio_i2c` and `data_gpio_pin` to `gpio_pin_lut`; this does not make the legacy `GPIO_I2C_Info` parser compatible. Prefer v2 command-table I2C/AUX transactions and parsed DisplayObjectInfo records.

### Command parameter revision mismatches

For the DCN1 AtomFirmware interface, the important command revisions are:

| Command | AtomFirmware revision | Current compatibility |
|---|---:|---|
| `ASIC_Init` | **2.1** | Mismatch: `atom_asic_init()` supplies legacy 1.x allocations |
| `DIGxEncoderControl` | **1.5** | Reusable: `atombios_dig_encoder_setup()` has v5 layout |
| `DIG1TransmitterControl` | **1.6** | Reusable: `atombios_transmitter_control()` has v1.6 layout |
| `SetPixelClock` | **1.7** | Reusable structurally: `atombios_set_pixel_clock()` has v7 layout; units differ from older revisions (`ulPixelClock` is 100 Hz) |
| `SetCRTC_UsingDTDTiming` | typically **1.3** | Existing helper uses one legacy structure without checking revision; must verify ABI before reuse |
| `SelectCRTC_Source` | typically **1.4** | Existing helper only names v1-v3 and treats later revisions as v3; unsafe without confirming 1.4 compatibility |
| `BlankCRTC` / `EnableCRTC` | **1.1** | Small parameter commands are likely reusable |
| `EnableDispPowerGating` | generation-dependent AtomFirmware ABI | Must validate action/controller fields before reuse |
| `ProcessI2cChannelTransaction` | v2 firmware command ABI | Legacy wrapper must not be assumed compatible solely because the table index exists |
| `ProcessAuxChannelTransaction` | v2 firmware command ABI | Same constraint; DP algorithm is reusable, command marshalling requires validation |

The exact revisions present in a particular board VBIOS are emitted by `atomfirmware_log_command_revision()`; the code currently logs them but does not validate them before invoking legacy wrappers.

## Architecture

`amd_atombios_init()` splits solely on PCI-ID-derived `expected_v2`. Legacy GPUs go through `atom_parse()`, v1 command indices, ASIC init, object discovery, EDID, and a DCE/AVIVO modeset. AtomFirmware GPUs get bounded v2 parsing and diagnostics only.

For Picasso, the parsers and several late-generation command marshallers already exist. The missing bridge is:

```text
v2 ROM/context
  → v2 ASIC_Init allocation
  → DisplayObjectInfo-to-atombios_display_path conversion
  → v2 I2C/AUX probing
  → compatible AtomFirmware command sequence
  → DCN1 scanout programming
```

The first three are relatively small. A functioning framebuffer is not a one-line opt-in because the existing native framebuffer and unblank code targets pre-DCN register blocks.

## Review Findings

- **Blocker:** `src/drivers/amd/atombios/atombios_driver.c:4598-4600` — unconditional return guarantees `1002:15d8` stops after diagnostics.
- **Blocker:** `src/drivers/amd/atombios/atombios_driver.c:1050-1173` — only AVIVO/Evergreen scanout exists; using it on DCN1 writes the wrong registers.
- **High:** `src/drivers/amd/atombios/atombios_driver.c:4634-4639` — `atom_asic_init()` uses v1 allocations, incompatible with AtomFirmware `ASIC_Init` 2.1.
- **High:** `src/drivers/amd/atombios/atombios_driver.c:3395-3490` — legacy display discovery must not parse DisplayObjectInfo using v1 casts.
- **High:** `src/drivers/amd/atombios/atombios_driver.c:4260-4275` — mandatory postmode helpers correspond to commands marked absent for v2.
- **Medium:** DIG 1.5, transmitter 1.6, and pixel-clock 1.7 cases exist and should be reused rather than reimplemented, but their default revision handling is too permissive for unknown later revisions.

## Residual Risks

- Actual command revisions and table sizes vary by Picasso/Raven2 VBIOS; diagnostics output from target hardware is needed to attest each command ABI.
- DCN1 register programming is the substantive missing component for visible output; merely running `ASIC_Init` only satisfies “proceed beyond diagnostics,” not successful modesetting.
- Firmware/SMU may already initialize portions of DCN on some platforms, but relying on preserved state would be board-specific and fragile.
- No runtime hardware validation was possible in this read-only review.

## Start Here

Open `src/drivers/amd/atombios/atombios_driver.c:4588-4600` first. Replace the diagnostics-only terminal branch with a deliberately separate v2 continuation; do not route the minimal `v2_ctx` directly into the legacy block.