## Review
- Correct: `/tmp/linux-6.12/drivers/gpu/drm/amd/display/dc/hubp/dcn10/dcn10_hubp.c:535` shows `hubp1_program_surface_config()` is only one slice of HUBP setup. Linux separately programs requestor (`:552`), deadline/TTU (`:586`), VM aperture/context (`:762`), and clock/VTG state (`:1283`/`:1286`).
- Correct: `/tmp/linux-6.12/drivers/gpu/drm/amd/display/dc/mpc/dcn10/dcn10_mpc.c:195` shows MPC insertion is not one register: it programs `MPCC_BOT_SEL`, `MPCC_CONTROL`, `MPCC_TOP_SEL`, `MPCC_OPP_ID`, `MPCC_UPDATE_LOCK_SEL`, and `MUX[opp].MPC_OUT_MUX`.
- Correct: `/tmp/linux-6.12/drivers/gpu/drm/amd/display/dc/dpp/dcn10/dcn10_dpp.c:501` shows DPP has its own `DPP_CONTROL.DPP_CLOCK_ENABLE`; Atom OTG/link setup does not prove the DPP datapath is clocked.
- Correct: `/tmp/linux-6.12/drivers/gpu/drm/amd/display/dc/hubbub/dcn10/dcn10_hubbub.c:583` shows watermarks are programmed separately, including arbiter saturation/outstanding request settings.
- Blocker: A “minimal DCN1 scanout” that writes only HUBP address/pitch/format plus MPC connection is not attested safe. It only works if firmware already left every hidden prerequisite valid. Linux evidence says those prerequisites are distinct programmed steps, not implied by surface address or MPC mux.
- Note: `/home/arthur/src/coreboot/src/drivers/amd/atombios/atombios_driver.c:361` only models legacy `AVIVO`/`EVERGREEN` scanout layouts; I found no current DCN scanout path in the inspected file. `plan.md` and `progress.md` were absent.

Mandatory hidden prerequisites:
- DPP/HUBP clocks: `DPP_CONTROL.DPP_CLOCK_ENABLE`, `HUBP_CLK_CNTL.HUBP_CLOCK_ENABLE`, likely `HUBP_VTG_SEL`.
- HUBP requestor/deadline/TTU: `HUBPRET_CONTROL`, `DCN_EXPANSION_MODE`, `DCHUBP_REQ_SIZE_CONFIG(_C)`, `BLANK_OFFSET_*`, `DST_DIMENSIONS`, `DST_AFTER_SCALER`, `REF_FREQ_TO_PIX_FREQ`, `VBLANK_PARAMETERS_*`, `NOM_PARAMETERS_*`, `PER_LINE_DELIVERY*`, `DCN_TTU_QOS_WM`, `DCN_SURF*_TTU_CNTL*`, `DCN_GLOBAL_TTU_CNTL`.
- VM/aperture: `DCN_VM_SYSTEM_APERTURE_*`, `DCN_VM_CONTEXT0_PAGE_TABLE_*`, fault default regs, `DCN_VM_MX_L1_TLB_CNTL.ENABLE_L1_TLB/SYSTEM_ACCESS_MODE`.
- DCHUBBUB/FB/watermarks: `DCHUBBUB_SDPIF_FB_BASE/TOP`, `AGP_*`, urgent/stutter/pstate watermarks, `DCHUBBUB_ARB_SAT_LEVEL`, `DCHUBBUB_ARB_DF_REQ_OUTSTAND`.
- Format path: DCC disabled or valid, linear tiling/address config, viewport/size/rotation, DPP scaler/color bypass sane, OPP/OTG already connected.

Least dangerous staged patch:
1. Add a board-gated/read-only DCN1 register dump in `atombios_driver.c` after Atom OTG/link setup. No writes.
2. Dump HUBP0/DPP0/MPC0/HUBBUB/OTG/OPP registers above and compare against Linux for the same mode.
3. Only after a known-good dump, add an experimental opt-in replay/bring-up path: first clocks + VM/aperture + watermarks/requestor/deadline from the known-good dump, then surface config/address, then MPCC mux. Do not invent DLG/watermark values in coreboot first pass.