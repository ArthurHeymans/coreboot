# Complete DCN1 one-pipe implementation plan

## Inherited decisions

- Vilboz/Picasso (`1002:15d8`) must initialize DCN1 natively. No option ROM execution, payload/GOP dependency, restored register dump, or register-replay fallback is acceptable.
- AtomFirmware command execution, panel power, AUX, EDID, transmitter control, and one-lane HBR link training already work. Do not rewrite them except where their current ordering conflicts with Linux DCN1.
- DCN1 remains a separate backend in `dcn10.c`; DCE12 and later DCN generations stay separate.
- Success requires visible output and stable Linux takeover. `fb_add_framebuffer_info()` alone is not success.
- Current hardware evidence is consistent: OTG is enabled, unblanked, clocked, and counting; HUBP0 reads `0x0000000a` (`NO_OUTSTANDING_REQ=1`, `IN_BLANK=1`) and `EARLIEST_INUSE=0`. The frontend has never accepted/fetched the surface.

## Diagnosis

The current `dcn10.c` is not a small Linux port. It is a prototype that combines selected Linux calls with guessed constants and omits whole commit phases. Continuing to add one register at a time is structurally wrong.

The most important trajectory error is outside `dcn10.c`: `atomfirmware_enable_edp_link()` currently invokes Atom `SetCRTC_UsingDTDTiming`, `EnableCRTC`, and `BlankCRTC` before the direct DCN backend rewrites OTG state. Linux DCN1 does not use those legacy Atom CRTC tables for the DCN timing-generator commit. It uses Atom for the pixel/PHY clock and transmitter services, while `dcn10_optc.c` directly owns OTG/VTG timing and blanking. The current path therefore has two independent timing owners and enables the link/backlight before the frontend commit exists.

Other blocker-level omissions or approximations are:

1. No complete cold pipe reset/clean state. Linux blanks/locks active OTGs, disconnects and resets all MPCC/OPP muxes, resets DPP state, initializes OTG double buffering/underflow state, and power-gates unused frontends before constructing a new state.
2. No atomic OTG update lock around timing, HUBP/DPP/MPC, surface, and address programming.
3. Incomplete HUBP requestor and deadline programming. Only one luma request-size register and a fraction of DLG/TTU fields are written. Chroma/cursor registers and all remaining VBLANK/NOMINAL fields are left at unknown cold-state values.
4. Hand-derived DLG/TTU values are not the Linux DCN1 calculation. `vstartup=13`, `DST_Y_PREFETCH=4`, `MIN_DST_Y_NEXT_START=vblank_start*4`, request delivery, and TTU values are independently guessed rather than produced as one internally consistent DML result.
5. No explicit memory model. `v2_fw.mc_base` is treated as a usable surface address without proving whether it is local MC physical, system physical, or GPU virtual. Linux either mirrors MMHUB VM state into HUBP or uses a valid local-framebuffer aperture.
6. `DCSURF_ADDR_CONFIG` is never programmed. Linux always derives it from GFX9 address configuration even for a linear surface.
7. DPP setup is incomplete: no `DSCL_AUTOCAL`, line-buffer format/memory control, degamma/input-CSC bypass, bias/scale defaults, or cursor disable. Writing `DSCL_SCL_MODE=6` and dimensions alone is not `dpp1_dscl_set_scaler_manual_scale()`.
8. MPC is not initialized from a known disconnected state and blending fields are not programmed. Only route fields are written.
9. OPP formatter/clamping/bit-depth state and OTG blank color are not explicitly initialized.
10. Hubbub watermarks/arbitration and local framebuffer mode are assumed to remain valid after ASIC/golden init. Linux programs urgent, stutter, pstate, saturation, and outstanding-request state as part of the bandwidth commit.
11. No staged validation gates. The code registers a framebuffer even when HUBP is still blank and has fetched address zero.

## Drift / contradiction check

- The bounded AtomFirmware parser and no-fallback decisions remain valid.
- The recent incremental DLG/global-sync additions should not be treated as accepted implementation. They are prototypes and should be deleted, not accumulated.
- Reading MMHUB/GMC/DCHUBBUB state is not GOP snapshot replay. Linux itself reads MMHUB aperture/page-table and DCHUBBUB FB location established by ASIC/GMC initialization. The prohibited path is copying a known-visible GOP register image or restoring dumped values.
- A fixed one-pipe implementation is consistent with the requested scope. A DRM resource graph, generic multi-plane compositor, cursor, audio, runtime modesets, DCC, scaling, and GPU page-table manager are not required.

## Recommendation

Replace the prototype with one complete, staged, Linux-derived DCN1 cold-state commit. Do not request another hardware flash until the full boundary below is implemented, host-tested, built, and reviewed.

# File-level implementation boundary

## 1. `src/drivers/amd/atombios/dcn10_regs.h` — new generated subset

Add one private header containing only the Raven/Picasso DCN1 registers and masks used by this backend. Copy names, offsets, base-index selection, shifts, and masks from Linux 6.12:

- `vega10_ip_offset.h`
- `dcn/dcn_1_0_offset.h`
- `dcn/dcn_1_0_sh_mask.h`
- the MMHUB/GMC offset/mask headers used by `dcn10_hwseq.c`

Use named field-pack/update helpers. Remove anonymous masks such as `0x101`, `0x0000077f`, `0x301`, `0x3fff8000`, and complete-register magic values from the backend.

Register families required:

- Global/power/clock: `DC_IP_REQUEST_CNTL`, `DOMAIN0..7_PG_CONFIG/STATUS`, `DCCG_GATE_DISABLE_CNTL{,2}`, `DCFCLK_CNTL`.
- Hubbub/FB: `DCHUBBUB_SDPIF_FB_BASE/TOP/OFFSET`, `SDPIF_AGP_BASE/BOT/TOP`, all A-D urgency/PTE/SR/pstate watermarks, `DCHUBBUB_ARB_SAT_LEVEL`, `DCHUBBUB_ARB_DF_REQ_OUTSTAND`, `DCHUBBUB_ARB_DRAM_STATE_CNTL`, watermark-change control, fault/debug status used for validation.
- HUBP0: `DCHUBP_CNTL`, clock, request-size L/C, `HUBPRET_CONTROL`, `DCN_EXPANSION_MODE`, full DLG/VBLANK/NOM/PER_LINE set, full luma/chroma/cursor TTU set, VM/aperture set, `DCSURF_ADDR_CONFIG`, tiling/config/control/pitch/viewports/flip/address/earliest-in-use.
- DPP0: clock, `DSCL_AUTOCAL`, `DSCL_CONTROL`, line-buffer registers, recout/MPC size/scaler mode, CNVC format/control, degamma/input CSC, bias/scale, cursor disable.
- MPC/MPCC0 and OPP0: all disconnect/connect/status/blending fields, output mux, OPP clock, FMT bit-depth/clamp/control and blank-color registers.
- OTG0/VTG0: exact timing/global-sync/double-buffer/input-clock/control/data-source/blank/color/status/frame-count/underflow/update-lock registers.
- Pixel-clock readback: the DCN1 clock-source `PIXEL_RATE_CNTL`, phase, modulo, and PHYPLL source registers used by the selected ComboPHY PLL.
- VGA controls retained from the exact Linux `dcn10_disable_vga()` sequence.

A small script or documented comparison command must prove every copied offset/mask equals Linux 6.12; do not transcribe silently.

## 2. `src/drivers/amd/atombios/dcn10.h` — narrow data contract

Replace `struct dcn10_timing` plus six scalar arguments with one explicit state contract. Do not build DRM-style objects or callback tables.

Suggested structures:

```c
struct dcn10_mode {
    uint16_t h_active, h_total, h_front_porch, h_sync_width;
    uint16_t v_active, v_total, v_front_porch, v_sync_width;
    uint32_t pixel_clock_khz;
    bool hsync_positive, vsync_positive;
};

struct dcn10_surface {
    void *cpu_address;
    uint64_t mc_address;
    size_t aperture_size;
    uint32_t width, height, pitch_pixels;
    enum dcn10_address_mode address_mode; /* initially LOCAL_MC_PHYSICAL only */
};

struct dcn10_clocks {
    uint32_t dchub_ref_khz;
    uint32_t dispclk_khz;
    uint32_t dppclk_khz; /* DCN1 tied to DISPCLK; no divide for first commit */
    uint32_t dprefclk_khz;
};

struct dcn10_memory {
    uint64_t fb_base, fb_top, fb_offset;
    uint32_t gb_addr_config;
};

struct dcn10_commit {
    struct dcn10_mode mode;
    struct dcn10_surface surface;
    struct dcn10_clocks clocks;
    struct dcn10_memory memory;
    struct dcn10_rq_regs rq;
    struct dcn10_dlg_regs dlg;
    struct dcn10_ttu_regs ttu;
    struct dcn10_watermarks wm;
    uint8_t pipe_count;
};
```

Expose staged functions rather than one opaque 400-line call:

```c
int dcn10_cold_init(void *mmio, uint8_t pipe_count);
int dcn10_prepare_commit(void *mmio, struct dcn10_commit *commit);
int dcn10_program_stream(void *mmio, const struct dcn10_commit *commit);
int dcn10_commit_plane(void *mmio, const struct dcn10_commit *commit);
int dcn10_unblank_and_validate(void *mmio, const struct dcn10_commit *commit,
                               uint32_t *pitch_bytes);
```

The split is sequencing, not abstraction. No interfaces/factories or support for other generations.

## 3. `src/drivers/amd/atombios/dcn10_calc.c` — new pure one-plane calculator

Port the complete calculation *for this supported state only*:

- one pipe / one plane
- linear GFX9 layout
- ARGB8888/XRGB use
- horizontal scan
- 1:1 scaling
- no DCC, metadata, chroma, cursor, writeback, split, ODM, or GPU VM/PTE fetch
- 1366x768 is the first fixture, but dimensions and timing remain inputs

Source the formulas and defaults from Linux 6.12:

- `dml/calcs/dcn_calcs.c`: `dcn10_soc_defaults`, `dcn10_ip_defaults`, one-plane clock/global-sync/watermark calculation.
- `dml/dml1_display_rq_dlg_calc.c`: `dml1_rq_dlg_get_rq_params()` and `dml1_rq_dlg_get_dlg_params()`.
- `hubp/dcn10/dcn10_hubp.c`: every field consumed by `hubp1_program_requestor()`, `hubp1_program_deadline()`, and `hubp1_setup_interdependent()`.

Do not copy the whole floating-point DML library. Implement the fixed case with integer/fixed-point helpers and retain the same round-up/floor points. Populate complete structures, including fields that are zero in this case:

- RQ: detector plane base, all four expansion modes, complete luma request fields, complete chroma request fields set to valid disabled/zero values.
- DLG: `refcyc_h_blank_end`, `dlg_vblank_end`, `min_dst_y_next_start`, `refcyc_per_htotal`, scaler X/Y delay, ref/pixel ratio, prefetch destination/ratio, VM/row vblank values, all luma/chroma PTE/meta vblank and nominal values, pre/active line delivery.
- TTU: high/low QoS, active/prefetch request delivery for luma, valid zero chroma/cursor entries, fixed QoS/ramp flags, minimum TTU vblank, flip QoS.
- OTG global sync: `vstartup_start`, `vupdate_offset`, `vupdate_width`, `vready_offset`, and pstate keepout generated by the same calculation—not a separate hand formula.
- Watermarks: at least one internally consistent boot state using DCN1 defaults (`urgent=4us`, SR exit `17us`, SR enter+exit `19us`, DRAM change `17us`, 21-bit clamping). Program A-D identically for firmware. Firmware may deliberately disable self-refresh/pstate switching rather than implement dynamic DPM, but urgent watermarks, saturation (`60 * refclk_mhz`), and minimum outstanding requests (`68`) are still required.

Fail preparation on overflow, unsupported address mode, insufficient aperture, invalid mode, insufficient link bandwidth, or a clock below calculated demand. Calculations happen before the first MMIO write.

## 4. `src/drivers/amd/atombios/dcn10.c` — replace, do not extend

Delete these prototype pieces:

- `div_round_up_u64()` and the hand-built global-sync calculation.
- `dcn10_program_dlg_ttu()` in its entirety.
- hard-coded `DCN_EXPANSION_MODE=0x56` and `DCHUBP_REQ_SIZE_CONFIG=0x055d1b70`.
- hand-packed DPP/CNVC/DSCL/MPC/OTG complete-register values and magic masks.
- the assumption that programming only luma TTU/request registers is enough.
- the single giant `dcn10_program_scanout()` sequencing function.

Retain the test-pattern fill as temporary bring-up output, but move it before the commit and add a readback/fence check. Remove it only after visible scanout and Linux takeover are validated.

Implement the following Linux-derived phases.

### Phase A — cold global initialization

Called once after Atom `ASIC_Init` and `EnableDispPowerGating(ASIC_PIPE_INIT / all PIPE_DISABLE)`, before panel/link programming:

1. Execute exact `dcn10_disable_vga()`; conditional silence when VGA is already disabled is expected.
2. Initialize DCN clock-gating state corresponding to `dcn10_init_hw()` (`DCCG_GATE_DISABLE_CNTL{,2}=0`, clear `DCFCLK_GATE_DIS`) after confirming the registers exist.
3. For every firmware-reported pipe (bounded to DCN1 maximum four): blank/lock a running OTG, initialize OTG blank/timing double buffering, clear OTG underflow, disconnect MPCC/OPP routing, set MPCC TOP/BOT/OPP/LOCK and each OPP mux to `0xf`, reset DPP state, blank/disable HUBP, and leave unused domains gated.
4. Preserve Linux's global Atom golden initialization ordering. Do not duplicate or replace `ASIC_Init`.
5. Validate all MPCC instances idle/disconnected before later connecting MPCC0.

### Phase B — memory and hubbub initialization

1. Read and validate `DCHUBBUB_SDPIF_FB_BASE/TOP/OFFSET` established by ASIC/GMC initialization.
2. Support only `LOCAL_MC_PHYSICAL` initially. Prove `[surface.mc_address, surface.mc_address + size)` lies in the local FB interval; otherwise fail. Do not silently treat an address as GPU virtual.
3. Program Linux `FRAME_BUFFER_MODE_LOCAL_ONLY`: invalidate AGP aperture as in `hubbub1_update_dchub()` and preserve the VBIOS-created local FB location.
4. Read `GB_ADDR_CONFIG` and decode the six GFX9 fields required by `DCSURF_ADDR_CONFIG`; program them explicitly.
5. Because this first implementation does not create GPU page tables, do not enable DCN L1 translation merely because Linux supports it. PTE/meta request and DLG fields must consistently be disabled/zero. A later GPU-VM implementation would have to port `mmhub_read_vm_system_aperture_settings()`, `mmhub_read_vm_context0_settings()`, address rebasing by FB base/offset, and all DCN VM registers as one unit.
6. Program the calculated A-D watermarks, saturation, minimum outstanding requests, and a deliberate firmware self-refresh/pstate policy before enabling requests.

### Phase C — stream timing while blank

Keep the Atom `SetPixelClock`/ComboPHY PLL operation, then perform direct DCN timing exactly as Linux `dcn10_enable_stream_timing()` and `optc1_program_timing()`:

1. Enable OPTC input clock and OTG clock and poll `*_CLOCK_ON`.
2. Verify pixel-clock source/phase/modulo/PLL readback corresponds to the requested mode.
3. Take `OTG_MASTER_UPDATE_LOCK` and poll lock status.
4. Run `optc1_tg_init()`: blank-data double buffer, range timing double buffer, clear underflow.
5. Program H/V totals, blank/sync/polarity, min/max total, interlace off, DP start point, global sync, VTG FP2/VCOUNT, input data format, and timing divide mode with named fields.
6. Program black blank color and keep OTG blanked.
7. Set OPP0 as OPTC0 source, enable VTG, enable OTG master, poll current-master state and verify the frame counter moves.

### Phase D — power and frontend commit under the same lock

1. Set `DC_IP_REQUEST_CNTL.IP_REQUEST_EN=1`.
2. Power on DPP0 and HUBP0 by clearing each POWER_GATE and polling `PGFSM_POWER_ON`; then clear `IP_REQUEST_EN`. Do not combine FORCEON and POWER_GATE into an undocumented write.
3. Enable HUBP clock, DPP clock (DCN1 DPPCLK tied to DISPCLK, no divide for this commit), and OPP pipe clock.
4. Select HUBP0 VTG0 while HUBP remains blank and TTU disabled.
5. Program complete calculated requestor, deadline, interdependent DLG, and TTU sets. Clear unused chroma/cursor register sets explicitly.
6. Program DPP fixed RGB path: CNVC ARGB8888, expansion-zero, alpha behavior matching the single opaque plane, degamma fixed-format/bypass, input CSC sRGB bypass, identity bias/scale, cursor disabled.
7. Program DSCL exact 1:1 path: AUTOCAL off, boundary mode zero, recout, MPC dimensions, `SCALING_444_BYPASS`, 36-bpp line-buffer format with alpha disabled for opaque scanout, and valid LB memory control. No filter coefficients are needed in bypass.
8. Reset/verify MPCC0 idle, then program TOP=DPP0, BOT=`0xf`, MODE=top-only, OPP=0, update-lock selection=0, complete global alpha/gain/blending fields, and `MPC_OUT0_MUX=0`.
9. Program OPP0 fixed 8-bpc RGB formatter: no spatial/temporal dither, RGB clamping/full range, pixel encoding RGB, stereo off, pipe clock enabled.
10. Program HUBP surface config in Linux order: DCC/TMZ/meta disabled; linear tiling; GFX9 address config; ARGB8888 format/crossbar; rotation/mirror zero; pitch; primary and secondary graphics viewports; unused chroma viewports zero; flip type immediate=false.
11. Write the surface address HIGH first and LOW last. The low write is the latch point.
12. Clear HUBP `BLANK_EN`, `TTU_DISABLE`, and `DISABLE` only after every dependency above is committed.
13. Release OTG update lock at the end of the same transaction.

### Phase E — link unblank/backlight and validation

Refactor the current Atom/link helper so ordering is:

1. Panel power and AUX/EDID.
2. DIG stream setup, panel mode, CRTC source selection, Atom pixel-clock/ComboPHY programming.
3. Direct DCN Phase C timing, still blank.
4. Transmitter enable and DP link training.
5. Direct DCN Phase D plane commit.
6. Unblank OTG and DP video stream.
7. Enable panel backlight last.

For DCN1, remove Atom `SetCRTC_UsingDTDTiming`, `EnableCRTC`, and `BlankCRTC` from `atomfirmware_enable_edp_link()`. They conflict with direct OPTC ownership. Split that helper into prepare/train/finalize functions rather than keeping one boolean expression with premature backlight enable.

Do not register the framebuffer until all hardware gates pass:

- power domains report ON;
- OPTC/OTG clocks report ON and OTG current-master state is set;
- OTG frame count changes;
- update lock was acquired/released;
- MPCC0 is connected to DPP0/OPP0 and is not idle;
- HUBP `BLANK_EN=0`, `DISABLE=0`, `TTU_DISABLE=0`, and `IN_BLANK=0` within a bounded number of frames;
- no HUBP or OPTC underflow;
- address readback equals the requested address;
- `DCSURF_EARLIEST_INUSE` equals the requested address;
- no DCHUBBUB/MMHUB protection fault or request-stall status is asserted.

Failure returns directly with no fallback and keeps the panel blank/backlight off.

## 5. `src/drivers/amd/atombios/atombios_driver.c` — orchestration only

Keep parsing, Atom calls, AUX/EDID, and link training here. Move DCN register knowledge out.

Changes required:

- Build `dcn10_commit` from bounded VBIOS data, EDID mode, framebuffer resource, and validated MMIO memory/address configuration.
- Call `dcn10_cold_init()` once at the Linux-equivalent point after ASIC/golden init.
- Split `atomfirmware_enable_edp_link()` as described above and make DCN1 the sole timing/blanking owner.
- Pass explicit clocks: DCHUB reference, DISPCLK/DPPCLK, DP reference, and pixel clock. Do not equate DCE refclk, DCHUB refclk, DISPCLK, and DPPCLK by name or accident.
- Keep `v2_fw.mc_base` only after the backend proves it is inside local framebuffer range.
- Call `fb_add_framebuffer_info()` only after `dcn10_unblank_and_validate()` succeeds.
- Keep DCN1 device gating (`0x15d8`) and no-fallback behavior unchanged.

## 6. Tests

Add focused host tests; hardware-only logic without these checks is unfinished.

### `tests/drivers/dcn10-calc-test.c`

- A Vilboz fixture: 1366x768, 72.7 MHz, 1526x793 totals, one linear 32-bpp plane, 1408-pixel pitch, DCN1 default SoC/IP constants.
- Expected RQ/DLG/TTU/global-sync/watermark fields generated independently from Linux 6.12 DML (a temporary Linux-side fixture generator is acceptable; captured GOP registers are not).
- Boundary tests: too-small aperture, arithmetic overflow, unsupported address mode, inadequate clock, invalid timing, and field-width overflow.

### `tests/drivers/dcn10-sequence-test.c`

Use a fake MMIO array plus a write log. One test should prove the required order:

`cold reset -> memory/watermarks -> lock -> clocks/timing blank -> IP request -> power -> requestor/DLG/TTU -> DPP -> MPC/OPP -> surface config -> address high -> address low -> HUBP unblank -> unlock -> OTG unblank`.

Model status bits changing for PGFSM, clocks, update lock, OTG frame count, MPCC status, HUBP blank, and earliest-in-use. Negative tests must fail on timeout/fault and must not report framebuffer success.

### Register parity check

Add a small maintainer script or documented command that compares every offset/shift/mask in `dcn10_regs.h` with Linux 6.12 generated headers. This prevents the existing class of naked-offset mistakes.

## Validation gates before the next flash

1. `dcn10-calc-test` passes all positive and negative fixtures.
2. `dcn10-sequence-test` passes and confirms no framebuffer registration on any failed gate.
3. Existing six bounded AtomFirmware parser tests pass.
4. Vilboz build succeeds with native init enabled and with the Atom driver disabled where applicable.
5. `checkpatch` and whitespace checks pass for the scoped diff.
6. A manual source audit maps each backend function to the cited Linux function and confirms all fields written by the fixed-case Linux calls are either programmed or intentionally zero/disabled.
7. Only then flash once and collect a single structured state report. Do not resume one-register experiments.

## Hardware acceptance

The first hardware run is one acceptance run, not another exploratory increment:

- visible color bars before payload/Linux;
- HUBP exits blank and earliest-in-use matches `0x000000f400000000`;
- no OTG/HUBP underflow or memory fault;
- framebuffer contents can be modified visibly;
- backlight remains off on commit failure;
- repeated warm and cold boots work;
- Linux amdgpu takes over without display loss, VM fault, hang, or forced reset.

Only after those pass should color bars be replaced with normal framebuffer contents and support be described as complete.

## Risks

- The largest unresolved fact is address semantics: `v2_fw.mc_base` must be proven local MC physical. If it is GPU virtual, a page-table manager is required and the local-only scope must be revised explicitly.
- Boot DISPCLK from DCE_Info is not automatically the current DPP/DCHUB reference. Read/derive clocks using the same Linux clock-manager rules before feeding DML.
- Linux DCN1 DML uses floating-point calculations in the driver. The firmware fixed-point port must match every rounding point, or its independent fixture will expose the mismatch.
- Watermark/pstate policy affects takeover and memory safety. A deliberate static firmware policy is acceptable; leaving command-table defaults implicit is not.
- The direct backend must own CRTC timing exclusively. Leaving legacy Atom CRTC calls in the DCN1 path can invalidate an otherwise correct commit.

## Need from main agent

No product decision is required to begin. The implementation should default to the narrow `LOCAL_MC_PHYSICAL`, one-pipe, no-DCC/no-VM/no-scaling path and fail closed if hardware/VBIOS inputs do not satisfy it. Revise that inherited assumption only if validated MMIO shows `v2_fw.mc_base` is not a local MC physical address.

## Suggested execution prompt

Implementation handoff is warranted:

> Replace the current DCN1 prototype with the complete one-pipe Linux 6.12-derived commit in `/tmp/dcn10-complete-plan.md`. Preserve Atom parser/link-training work and all dirty changes. First add the private register subset and pure RQ/DLG/TTU/watermark calculator plus host tests; then implement cold reset, local-memory/hubbub setup, exact OPTC lock/timing, complete HUBP/DPP/DSCL/MPC/OPP commit, address latch, unblank, and validation. Refactor the DCN1 Atom link helper so direct DCN owns CRTC timing/blanking and backlight is last. Do not add GOP snapshots, fallback, generic DRM abstractions, GPU VM, DCC, scaling, or multi-plane support. Do not request another flash until all host tests/build/lint/review gates pass.

```acceptance-report
{
  "criteriaSatisfied": [
    {
      "id": "criterion-1",
      "status": "satisfied",
      "evidence": "The plan is limited to replacing the DCN1 prototype for a single Raven/Picasso eDP pipe and explicitly excludes GOP replay, fallback, GPU VM, DCC, scaling, multi-plane, other display generations, and generic DRM infrastructure."
    },
    {
      "id": "criterion-2",
      "status": "satisfied",
      "evidence": "The plan maps implementation phases to Linux 6.12 functions, identifies current conflicting/guessed code to delete, defines required structures and inputs, specifies full register families and sequencing, and provides host/build/lint/hardware acceptance gates."
    }
  ],
  "changedFiles": [],
  "testsAddedOrUpdated": [],
  "commandsRun": [
    {
      "command": "read current dcn10.c/dcn10.h and targeted atombios_driver.c call sites",
      "result": "passed",
      "summary": "Confirmed the prototype's hand calculations, partial block programming, framebuffer registration despite HUBP blank, and conflicting Atom CRTC sequencing."
    },
    {
      "command": "inspect Linux 6.12 dcn10_hwseq.c, dcn10_optc.c, dcn10_hubp.c, dcn10_dpp.c, dcn10_dpp_dscl.c, dcn10_mpc.c, dcn10_opp.c, dcn10_hubbub.c, dcn_calcs.c, dml1_display_rq_dlg_calc.c, dce_clock_source.c, and dcn10_resource.c",
      "result": "passed",
      "summary": "Traced cold initialization, memory/hubbub, timing lock, frontend power, complete plane programming, address latch, watermark, and validation dependencies."
    }
  ],
  "validationOutput": [
    "Review-only task: no source file was edited.",
    "Findings written to /tmp/dcn10-complete-plan.md.",
    "Current `HUBP cntl=0x0000000a` decodes as NO_OUTSTANDING_REQ plus IN_BLANK, consistent with an incomplete frontend commit rather than failed DP training."
  ],
  "residualRisks": [
    "Surface address semantics must be proven local MC physical before implementation accepts v2_fw.mc_base.",
    "The fixed-point DML subset requires an independent Linux-generated fixture to validate rounding.",
    "Visible output and Linux takeover remain hardware acceptance requirements."
  ],
  "noStagedFiles": true,
  "diffSummary": "No source diff; produced the requested complete implementation boundary and acceptance plan.",
  "reviewFindings": [
    "blocker: src/drivers/amd/atombios/atombios_driver.c - DCN1 currently uses both legacy Atom CRTC timing/enable/blank commands and direct OPTC programming, creating two timing owners.",
    "blocker: src/drivers/amd/atombios/dcn10.c - requestor/DLG/TTU, memory/address config, DPP/DSCL, MPC/OPP, locking, watermarks, and validation are incomplete or guessed.",
    "high: src/drivers/amd/atombios/dcn10.c - framebuffer registration proceeds while HUBP remains IN_BLANK and earliest-in-use is zero."
  ],
  "manualNotes": "Do not flash the current incremental build. Implement and review the complete boundary before the next hardware run."
}
```
