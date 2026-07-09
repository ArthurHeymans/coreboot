## Inherited decisions

- Review only; no files edited.
- ASIC initialization, eDP power, EDID/mode selection, HBR link training, OTG/CRTC, and backlight already work.
- The remaining blocker is native DCN1 memory-to-pixel scanout.
- Final support must be native; GOP/option-ROM fallback or register replay is not the target.

## Diagnosis

### Review findings

1. **Blocker — HUBP/DPP/MPC alone is not a credible minimal implementation**
   - `/home/arthur/src/linux/drivers/gpu/drm/amd/display/dc/hubp/dcn10/dcn10_hubp.c` programs much more than surface address and pitch:
     - tiling/DCC/format/rotation
     - viewport and size
     - VM/system aperture
     - request sizes
     - DLG deadlines and TTU QoS
     - high-address-before-low-address latching
   - `/home/arthur/src/linux/drivers/gpu/drm/amd/display/dc/hwss/dcn10/dcn10_hwseq.c` additionally sequences power, clocks, HUBP blanking, soft reset, and recovery.
   - A few HUBP, DPP, and MPC writes can still produce permanent black, underflow, or memory faults.

2. **High — the critical routing is independently programmable**
   - `/home/arthur/src/linux/drivers/gpu/drm/amd/display/dc/mpc/dcn10/dcn10_mpc.c:211-230` shows a single-plane path requires:
     - `MPCC0_MPCC_BOT_SEL.MPCC_BOT_SEL = 0xf`
     - `MPCC0_MPCC_CONTROL.MPCC_MODE = TOP_LAYER_ONLY`
     - `MPCC0_MPCC_TOP_SEL.MPCC_TOP_SEL = DPP0`
     - `MPCC0_MPCC_OPP_ID.MPCC_OPP_ID = OPP0`
     - `MPC_OUT0_MUX.MPC_OUT_MUX = MPCC0`
   - Missing any link leaves the enabled OTG transmitting blank pixels.

3. **High — address and aperture semantics are dangerous**
   - `/home/arthur/src/linux/drivers/gpu/drm/amd/display/dc/hubp/dcn10/dcn10_hubp.c:373-411` explicitly writes `DCSURF_PRIMARY_SURFACE_ADDRESS_HIGH` before `DCSURF_PRIMARY_SURFACE_ADDRESS`; the low write latches the address.
   - The address is a GPU-visible address, not necessarily the CPU framebuffer BAR address.
   - `DCN_VM_SYSTEM_APERTURE_*` and `DCN_VM_MX_L1_TLB_CNTL` must agree with whether scanout uses VRAM or system memory.

4. **High — exact MMIO addresses must be resolved, not guessed**
   - Register lists are assembled in `/home/arthur/src/linux/drivers/gpu/drm/amd/display/dc/resource/dcn10/dcn10_resource.c`.
   - Raw definitions and masks live in:
     - `/home/arthur/src/linux/drivers/gpu/drm/amd/include/asic_reg/dcn/dcn_1_0_offset.h`
     - `/home/arthur/src/linux/drivers/gpu/drm/amd/include/asic_reg/dcn/dcn_1_0_sh_mask.h`
   - Linux’s `SRI/SRII` macros add segment bases. Copying a naked generated register index or forgetting DWORD-to-byte conversion can target the wrong block.

5. **Medium — local version mismatch**
   - The inspected checkout reports `6.18.0`, despite the requested Linux 6.12 baseline. A local `v6.12` tag exists, but this review did not re-run inspection through `git show v6.12:...`.
   - The DCN1 concepts are stable, but exact 6.12 paths/definitions should be verified before copying offsets.

### Exact register allowlist for the comparative dump

Dump named registers and decoded fields, not an indiscriminate MMIO range:

- **Power/clocks**
  - `DOMAIN0_PG_CONFIG`, `DOMAIN0_PG_STATUS`
  - `HUBP0_HUBP_CLK_CNTL`
  - `DPP_TOP0_DPP_CONTROL`
- **HUBP attachment**
  - `HUBP0_DCHUBP_CNTL`: `HUBP_VTG_SEL`, `HUBP_DISABLE`, `HUBP_BLANK_EN`
- **Surface**
  - `HUBP0_DCSURF_SURFACE_CONFIG`
  - `HUBP0_DCSURF_TILING_CONFIG`
  - `HUBP0_DCSURF_SURFACE_CONTROL`
  - `HUBP0_DCSURF_SURFACE_PITCH`
  - `HUBP0_DCSURF_SURFACE_SIZE`
  - `HUBPREQ0_DCSURF_PRIMARY_SURFACE_ADDRESS_HIGH`
  - `HUBPREQ0_DCSURF_PRIMARY_SURFACE_ADDRESS`
  - `HUBP0_DCSURF_PRI_VIEWPORT_START`
  - `HUBP0_DCSURF_PRI_VIEWPORT_DIMENSION`
- **Requestor/DLG/TTU**
  - `HUBP0_HUBPRET_CONTROL`
  - `HUBP0_DCN_EXPANSION_MODE`
  - `HUBP0_DCHUBP_REQ_SIZE_CONFIG`
  - `HUBP0_BLANK_OFFSET_0`, `BLANK_OFFSET_1`
  - `HUBP0_DST_DIMENSIONS`, `DST_AFTER_SCALER`
  - `HUBP0_REF_FREQ_TO_PIX_FREQ`
  - `HUBP0_PREFETCH_SETTINS`
  - `HUBP0_VBLANK_PARAMETERS_0..7`
  - `HUBP0_PER_LINE_DELIVERY`, `PER_LINE_DELIVERY_PRE`
  - `HUBP0_DCN_TTU_QOS_WM`
  - `HUBP0_DCN_SURF0_TTU_CNTL0`, `DCN_SURF0_TTU_CNTL1`
  - `HUBP0_DCN_GLOBAL_TTU_CNTL`
- **VM/aperture**
  - `DCN_VM_SYSTEM_APERTURE_DEFAULT_ADDR_MSB/LSB`
  - `DCN_VM_SYSTEM_APERTURE_LOW_ADDR_MSB/LSB`
  - `DCN_VM_SYSTEM_APERTURE_HIGH_ADDR_MSB/LSB`
  - `DCN_VM_MX_L1_TLB_CNTL`
- **DPP/IPP**
  - `DPP_TOP0_DPP_CONTROL`
  - `CNVC_CFG0_CNVC_SURFACE_PIXEL_FORMAT`
  - `CNVC_CFG0_FORMAT_CONTROL`
  - `DSCL0_SCL_MODE`, `DSCL0_DSCL_CONTROL`
  - `DSCL0_RECOUT_START`, `DSCL0_RECOUT_SIZE`, `DSCL0_MPC_SIZE`
- **MPC**
  - `MPCC0_MPCC_TOP_SEL`
  - `MPCC0_MPCC_BOT_SEL`
  - `MPCC0_MPCC_CONTROL`
  - `MPCC0_MPCC_STATUS`
  - `MPCC0_MPCC_OPP_ID`
  - `MPCC0_MPCC_UPDATE_LOCK_SEL`
  - `MPC_OUT0_MUX`
- **OPP/OTG verification**
  - OPP0 `FMT_CONTROL`, `FMT_BIT_DEPTH_CONTROL`, and clamp registers
  - OTG0 control/status, blank state, master-enable, and underflow status
- **Hubbub**
  - active urgency, pstate-change, self-refresh watermarks, arbitration, and outstanding-request registers from `dcn10_hubbub.c`

## Drift / contradiction check

Recommending a dump first does **not** revise the earlier conclusion that GOP register replay is not a credible backend. The dump is measurement only: it identifies the required power, aperture, DLG/TTU, and routing state before implementing native logic.

Proceeding directly with “minimal HUBP/DPP/MPC” would quietly assume that ASIC_Init already established every surrounding clock, VM, watermark, and OPP prerequisite. Linux does not make that assumption.

## Recommendation

**First add one narrow GOP/native comparative register dump.**

Capture the allowlist:

1. Immediately after a known-visible GOP initialization, before Linux changes display state.
2. At the equivalent point in the current native path after CRTC/link setup but before final unblank/backlight.

Then implement only the observed single-pipe delta in `src/drivers/amd/atombios/atombios_v2.c` or its existing DCN1 call path:

- linear, non-DCC XRGB8888
- one framebuffer
- HUBP0/DPP0/MPCC0/OPP0/OTG0
- no scaling
- native panel mode
- decoded readback and underflow/fault checks

Do not replay the GOP dump wholesale. Use Linux field definitions and sequencing, with captured values only to determine which ASIC_Init defaults are safe to retain.

If GOP cannot produce a known-visible Vilboz baseline, use a known-good Linux amdgpu snapshot instead; do not compare against another black state.

## Risks

- Wrong GPU address or aperture can cause memory faults rather than merely black output.
- Incorrect pitch units, alignment, format, or DCC state can produce corruption or black.
- Missing DLG/TTU/watermark programming can underflow intermittently even if a test pattern appears.
- Power-gated register writes may be discarded or hang polling.
- Incorrect update-lock order can leave mixed old/new state.
- Color-conversion or OPP format state may turn valid pixels black or tinted.
- Exact offsets still require verification against the Linux 6.12 tag.

## Need from main agent

None. Default to the comparative dump; fall back to a known-good Linux snapshot if GOP is not visibly functional.

## Suggested execution prompt

> Add a read-only DCN1 scanout-state dump to the existing AtomBIOS v2 diagnostics. Use a fixed named allowlist covering HUBP0 surface/address/requestor/DLG/TTU, DCN VM aperture, DPP0/IPP0, MPCC0/MPC_OUT0, OPP0, OTG0 status, hubbub watermarks, and domain clocks. Resolve byte offsets and masks from Linux v6.12 `dcn_1_0_offset.h`, `dcn_1_0_sh_mask.h`, and `dcn10_resource.c`; do not guess or replay values. Emit identical output for a known-visible GOP/Linux baseline and the native black-screen point. Do not add scanout writes in this patch.