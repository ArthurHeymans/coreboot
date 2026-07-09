# Code Context

## Files Retrieved

1. `/tmp/linux-6.12/drivers/gpu/drm/amd/display/dc/hubp/dcn10/dcn10_hubp.c` (lines 41-60, 155-200, 258-288, 349-412, 535-552, 829-839) — HUBP blanking, pitch encoding, format, address-latch order, surface configuration, and viewport.
2. `/tmp/linux-6.12/drivers/gpu/drm/amd/display/dc/hubp/dcn10/dcn10_hubp.h` (lines 117-340) — DCN1 register-list composition and HUBP field definitions.
3. `/tmp/linux-6.12/drivers/gpu/drm/amd/display/dc/dpp/dcn10/dcn10_dpp.c` (lines 301-382, 507-516) — DPP fixed-format setup, XRGB/ARGB8888 encoding, alpha disable, and clock enable.
4. `/tmp/linux-6.12/drivers/gpu/drm/amd/display/dc/dpp/dcn10/dcn10_dpp_cm.c` (lines 773-783) — minimal ARGB8888 CNVC defaults.
5. `/tmp/linux-6.12/drivers/gpu/drm/amd/display/dc/mpc/dcn10/dcn10_mpc.c` (lines 195-240, 298-306) — single-plane MPCC and MPC output routing.
6. `/tmp/linux-6.12/drivers/gpu/drm/amd/display/dc/opp/dcn10/dcn10_opp.c` (lines 367-373) — OPP pipe-clock enable.
7. `/tmp/linux-6.12/drivers/gpu/drm/amd/display/dc/resource/dcn10/dcn10_resource.c` (lines 312-320, 414-421) — Raven/Picasso DCN10 register-list instantiation.
8. `/tmp/linux-6.12/drivers/gpu/drm/amd/include/asic_reg/dcn/dcn_1_0_offset.h` (lines 2316-2371, 3460-3479, 5361-5388, 5509-5510) — register offsets and DCN segment index.
9. `/tmp/linux-6.12/drivers/gpu/drm/amd/include/asic_reg/dcn/dcn_1_0_sh_mask.h` (lines 9130-9178, 9235-9249, 9306-9317, 12662-12740, 18798-18818, 19292-19293) — shifts and masks.

## Key Code

### Address composition

All cited registers have `BASE_IDX 2`. For Raven/Picasso:

```text
register_index = adev->reg_offset[DCN_HWIP][0][2] + mmREGISTER
MMIO byte offset = register_index * 4
```

The generated `mm...` constants are DWORD offsets, not absolute byte addresses. The runtime DCN segment-2 base must be obtained from the initialized ASIC register-offset table; adding a hard-coded PCI BAR address directly to `mmREGISTER` is incorrect.

### Smallest defensible post-timing sequence

This sequence assumes the normal DCN initialization has already programmed HUBBUB/VM or physical aperture, HUBP requestor/DLG/TTU values, DPP scaler bypass for 1:1, formatter, clocks/power domains, and OTG0→OPP0. Those are not implied merely by “OTG timing and DP link enabled.”

Use read-modify-write for fields unless the register is explicitly written as a complete value.

1. **Blank HUBP0 while changing the surface**

   `HUBP0_DCHUBP_CNTL`, offset `segment2 + 0x0566`:

   | Field | Shift | Mask | Value |
   |---|---:|---:|---:|
   | `HUBP_BLANK_EN` | 0 | `0x00000001` | 1 |
   | `HUBP_TTU_DISABLE` | 12 | `0x00001000` | 1 |
   | `HUBP_VTG_SEL` | 4 | `0x000000f0` | 0 (OTG0) |

   Evidence: `dcn10_hubp.c:41-48`; masks at `dcn_1_0_sh_mask.h:9235-9249`.

2. **Enable processing clocks if not already enabled**

   - `DPP_TOP0_DPP_CONTROL`, offset `segment2 + 0x0c3d`:
     `DPP_CLOCK_ENABLE`, shift 4, mask `0x10`, value 1.
   - `OPP0_OPP_PIPE_CONTROL.OPP_PIPE_CLOCK_EN = 1`.

   Evidence: `dcn10_dpp.c:507-516`; `dcn10_opp.c:367-373`; DPP offset at `dcn_1_0_offset.h:3460-3461`.

3. **Configure HUBP0 for linear 32-bit graphics**

   - `HUBP0_DCSURF_TILING_CONFIG.SW_MODE = 0` (linear); clear metadata/pipe-alignment fields for a non-DCC linear surface.
   - `HUBP0_DCSURF_SURFACE_CONFIG`, offset `segment2 + 0x0559`:
     `SURFACE_PIXEL_FORMAT = 8` (`ARGB8888` hardware encoding used for XRGB8888); rotation 0 and mirror 0.
   - Disable DCC in `HUBPREQ0_DCSURF_SURFACE_CONTROL.PRIMARY_SURFACE_DCC_EN = 0`.

   Evidence: tiling/config call chain `dcn10_hubp.c:535-552`; format mapping `dcn10_hubp.c:258-272`.

4. **Program pitch**

   `HUBPREQ0_DCSURF_SURFACE_PITCH`, offset `segment2 + 0x057b`:

   | Field | Shift | Mask | Value |
   |---|---:|---:|---:|
   | `PITCH` | 0 | `0x00003fff` | `1366 - 1 = 1365 = 0x555` |
   | `META_PITCH` | 16 | `0x3fff0000` | 0 |

   Thus the complete relevant value is `0x00000555`.

   **Important:** DCN1 pitch is expressed in pixels/elements for this format, not bytes. Writing `5464-1` would be wrong. Evidence: `pitch = plane_size->surface_pitch - 1` in `dcn10_hubp.c:168-196`; masks at `dcn_1_0_sh_mask.h:9306-9309`.

5. **Program viewport**

   - `HUBP0_DCSURF_PRI_VIEWPORT_START`, offset `segment2 + 0x055c`: X=0, Y=0 → `0`.
   - `HUBP0_DCSURF_PRI_VIEWPORT_DIMENSION`, offset `segment2 + 0x055d`:
     width=1366 (`0x556`), height=768 (`0x300`).

   With the DCN1 width/height 14-bit fields at bits 0 and 16, the value is:

   ```text
   (768 << 16) | 1366 = 0x03000556
   ```

   Evidence: `dcn10_hubp.c:829-839`; offsets at `dcn_1_0_offset.h:2322-2325`; width mask at `dcn_1_0_sh_mask.h:9166-9168`.

6. **Configure DPP0 input conversion**

   - `CNVC_CFG0_CNVC_SURFACE_PIXEL_FORMAT`, offset `segment2 + 0x0c47`:
     field shift 0, mask `0x7f`, value 8.
   - `CNVC_CFG0_FORMAT_CONTROL`, offset `segment2 + 0x0c48`:
     `CNVC_BYPASS=0`, `ALPHA_EN=0`, `FORMAT_EXPANSION_MODE=0`, `OUTPUT_FP=0`.

   `ALPHA_EN=0` is what turns the ARGB8888 memory encoding into XRGB semantics.

   Evidence: `dcn10_dpp.c:301-307,331-334,380-382`; masks at `dcn_1_0_sh_mask.h:12729-12740`; minimal initialization at `dcn10_dpp_cm.c:773-783`.

7. **Route DPP0 through MPCC0 to OPP0**

   - `MPCC0_MPCC_BOT_SEL`, `segment2 + 0x1631`: field mask `0xf`, value `0xf` (no bottom layer).
   - `MPCC0_MPCC_CONTROL`, `segment2 + 0x1633`:
     `MPCC_MODE = MPCC_BLEND_MODE_TOP_LAYER_ONLY`; alpha blending disabled/default.
   - `MPCC0_MPCC_TOP_SEL`, `segment2 + 0x1630`: field mask `0xf`, value 0 (DPP0).
   - `MPCC0_MPCC_OPP_ID`, `segment2 + 0x1632`: field mask `0xf`, value 0 (OPP0).
   - `MPCC0_MPCC_UPDATE_LOCK_SEL`, `segment2 + 0x1635`: value 0.
   - `MPC_OUT0_MUX`, `segment2 + 0x172f`: shift 0, mask `0xf`, value 0 (MPCC0).

   Program BOT/mode first, then TOP/OPP/lock, then output mux. Evidence: `dcn10_mpc.c:226-240`; output mux behavior at `dcn10_mpc.c:298-306`; offsets at `dcn_1_0_offset.h:5361-5372,5509-5510`; masks at `dcn_1_0_sh_mask.h:18798-18818,19292-19293`.

8. **Program framebuffer address last, high before low**

   - `HUBPREQ0_DCSURF_PRIMARY_SURFACE_ADDRESS_HIGH`,
     `segment2 + 0x057e`: value `0`.
   - `HUBPREQ0_DCSURF_PRIMARY_SURFACE_ADDRESS`,
     `segment2 + 0x057d`: value `0xe0000000`.

   The low-address write latches the surface state, so ordering is mandatory. Evidence: `dcn10_hubp.c:374-377,405-411`; offsets at `dcn_1_0_offset.h:2364-2367`; low mask `0xffffffff` at `dcn_1_0_sh_mask.h:9316-9317`.

9. **Unblank**

   RMW `HUBP0_DCHUBP_CNTL`:

   ```text
   HUBP_VTG_SEL    = 0
   HUBP_BLANK_EN   = 0
   HUBP_TTU_DISABLE = 0
   ```

   Evidence: `dcn10_hubp.c:41-48`.

## Architecture

```text
0xe0000000 linear XRGB8888
  → HUBP0 fetch/viewport
  → DPP0 CNVC, alpha ignored, 1:1 scaler bypass
  → MPCC0 top-only
  → MPC_OUT0_MUX
  → OPP0
  → already-running OTG0/DP stream
```

## Review Findings

- **blocker:** An exact cold-state display sequence cannot be derived from the stated precondition. `dcn10_hubp.c:535-729` shows that normal plane enable additionally depends on requestor, DLG, and TTU programming. OTG and DP link enablement does not establish those values.
- **high:** Physical address `0xe0000000` only works if HUBBUB VM/system-aperture configuration makes it directly accessible. Otherwise this must be a GPU virtual address with valid page tables.
- **high:** A 1366-pixel linear pitch is accepted by the 14-bit register, but actual Raven scanout allocation/alignment requirements are external to these register definitions. A firmware-created framebuffer may have a larger aligned pitch.
- **medium:** OPP formatter/color-depth programming is assumed to have been completed with the stream. OPP0 routing itself is MPC-driven; OPP only needs its pipe clock enabled here.

## Start Here

Open `/tmp/linux-6.12/drivers/gpu/drm/amd/display/dc/hubp/dcn10/dcn10_hubp.c` first. It contains the authoritative fetch configuration and explicitly documents the address-latch ordering and pixel-based `pitch - 1` encoding.