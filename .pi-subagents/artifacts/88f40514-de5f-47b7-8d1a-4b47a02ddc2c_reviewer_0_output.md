# DCN1 Raven/Picasso Linux 6.12 parity matrix

Sources checked: coreboot `src/drivers/amd/atombios/dcn10.c`, `src/drivers/amd/atombios/atombios_driver.c`; Linux 6.12 `drivers/gpu/drm/amd/display/dc/hwss/dcn10/dcn10_hwseq.c`, `resource/dcn10/dcn10_resource.c`, plus referenced DCN10 component files. `plan.md` and `progress.md` were requested but absent under `/home/arthur/src/coreboot`.

Scope classification: one cold-init visible eDP plane, pipe0, linear physical UMA framebuffer, non-DCC, ARGB8888, no cursor, no scaling, no GPUVM, no MPO, no DSC/audio/PSR/ABM/gamma beyond bypass.

## Review
- Correct: coreboot v2 path runs AtomFirmware ASIC_Init before DCN programming (`atomfirmware_asic_init()` calls `asic_init`, `atombios_driver.c:4509-4524`; main calls it at `atombios_driver.c:4774-4778`).
- Correct: coreboot has pipe0 HUBP/DPP/MPC/OTG direct programming for a linear ARGB8888 surface (`dcn10_program_scanout()`, `dcn10.c:290-418`) and v2 eDP link/timing setup (`atomfirmware_enable_edp_link()`, `atombios_driver.c:4577-4621`).
- Note: Linux's ordered cold path is split: resource creation (`dcn10_resource_construct()`, `dcn10_resource.c:1314-1662`), init (`dcn10_init_hw()`, `dcn10_hwseq.c:1542-1680`), stream timing (`dcn10_enable_stream_timing()`, `dcn10_hwseq.c:961-1045`), and plane programming (`dcn10_program_pipe()`, `dcn10_hwseq.c:2989-3036`).

## Ordered parity checklist

| # | Ordered Linux step and registers | Required for this pipe? | Coreboot parity / implementation action | Severity |
|---|---|---|---|---|
| 1 | Post-ASIC init: Linux enters DC after BIOS/ASIC services; core DCN init starts at `dcn10_init_hw()` (`dcn10_hwseq.c:1542`). Resource model creates HUBP/DPP/OPP/TG/MPC/HUBBUB (`dcn10_resource.c:1563-1662`). | Required | OK: v2 `atomfirmware_asic_init()` passes boot SCLK/MCLK to `asic_init` (`atombios_driver.c:4509-4524`) before DCN actions (`atombios_driver.c:4774-4778`). | none |
| 2 | Clocks/DCCG: Linux `clk_mgr->init_clocks()` (`dcn10_hwseq.c:1554-1555`), optional `dccg_init()` (`DCCG` object may be NULL on DCN1, `dcn10_hwseq.c:1563-1565`), ref clocks via `get_dccg_ref_freq()`/`get_dchub_ref_freq()` (`dcn10_hwseq.c:1574-1593`). Pixel clock through `clock_source->program_pix_clk()` in `dcn10_enable_stream_timing()` (`dcn10_hwseq.c:982-991`). Registers later touched: `DCCG_GATE_DISABLE_CNTL`, `DCCG_GATE_DISABLE_CNTL2`, `DCFCLK_CNTL.DCFCLK_GATE_DIS` (`dcn10_hwseq.c:1669-1676`). | Required: boot DISPCLK/DPPCLK and pixel clock. DCCG object-specific DTO is irrelevant when Linux has no DCN1 DCCG object. | Partial: coreboot uses VBIOS `SetPixelClock` (`atombios_driver.c:4606-4608`, function body `atombios_driver.c:2725-2830`) and direct `OPTC_INPUT_CLOCK_CONTROL`/`OTG_CLOCK_CONTROL` (`dcn10.c:367-368`). It does not mirror Linux clock gating writes or `clk_mgr` watermarks notification (`dcn10_hwseq.c:1678-1679`). Checklist: keep VBIOS pixel clock; no new DCCG object needed for DCN1; optionally clear DCN clock-gate registers if cold silicon needs it. | medium |
| 3 | BIOS golden init: Linux `dcn10_bios_golden_init()` calls `enable_disp_power_gating(... ASIC_PIPE_INIT)` then `ASIC_PIPE_DISABLE` for every pipe (`dcn10_hwseq.c:866-904`), registers through VBIOS command table. | Required | OK: `atomfirmware_init_display_pipes()` sends `ATOM_INIT` then `ATOM_DISABLE` per pipe through `enabledisppowergating` (`atombios_driver.c:4555-4574`) and main calls it after VGA disable (`atombios_driver.c:4779-4785`). | none |
| 4 | Pipe reset/init: Linux `init_pipes()` called during `dcn10_init_hw()` when not seamless (`dcn10_hwseq.c:1629-1635`); old-context reset uses `dcn10_reset_hw_ctx_wrap()` and `dcn10_reset_back_end_for_pipe()` (`dcn10_hwseq.c:1729-1759`). MPC init clears `MPCC_TOP_SEL`, `MPCC_BOT_SEL`, `MPCC_OPP_ID`, `MPCC_UPDATE_LOCK_SEL`, `MPC_OUT_MUX` (`dcn10_mpc.c:377-389`). | Required only to clear stale mux/pipe state; old-context reset irrelevant for first cold coreboot mode. | Partial: coreboot does BIOS pipe disable and then programs pipe0, but does not globally clear all MPCCs/MPC muxes. Checklist: for cold-only path at least clear MPCC0/MUX0 before insert; for robustness clear all four MPCCs like `mpc1_mpc_init()`. | medium |
| 5 | VGA handoff: Linux `dcn10_disable_vga()` reads `D1VGA_CONTROL`..`D4VGA_CONTROL`, writes them 0, sets `VGA_TEST_CONTROL.VGA_TEST_ENABLE` and `.VGA_TEST_RENDER_START` (`dcn10_hwseq.c:635-666`). | Required if VGA mode active | OK: coreboot `dcn10_disable_vga()` mirrors the same registers and writes `VGA_TEST_CONTROL | 0x101` (`dcn10.c:122-142`), called for Raven/Picasso v2 (`atombios_driver.c:4779-4780`). | none |
| 6 | Link encoder/eDP physical link: Linux initializes link encoders via `link_enc->hw_init()` (`dcn10_hwseq.c:1597-1606`), blanks DP displays (`dcn10_hwseq.c:1617-1618`), later unblanks DP and backlight in `dcn10_unblank_stream()` (`dcn10_hwseq.c:3871-3893`). | Required | OK via ATOM path, not DCN HWSS: panel power `ATOM_TRANSMITTER_ACTION_POWER_ON` (`atombios_driver.c:4530-4552`), `DIGxEncoderControl` stream setup/panel mode and `DIG1TransmitterControl` init/enable/LCD_BLON (`atombios_driver.c:4597-4616`), DP training sequence (`atombios_driver.c:2138-2427`). | none |
| 7 | Power domains: Linux ungates DPP/HUBP through `power_on_plane_resources()`: optional root clock, `DC_IP_REQUEST_CNTL.IP_REQUEST_EN`, `dpp_pg_control()`, `hubp_pg_control()` (`dcn10_hwseq.c:790-814`); `dcn10_dpp_pg_control()` uses `DOMAIN1_PG_CONFIG/STATUS` for DPP0 (`dcn10_hwseq.c:677-728`), `dcn10_hubp_pg_control()` uses `DOMAIN0_PG_CONFIG/STATUS` for HUBP0 (`dcn10_hwseq.c:730-788`). | Required | Partial: coreboot `dcn10_power_on_domain()` updates `DOMAIN0_PG_CONFIG` and `DOMAIN1_PG_CONFIG`, waits `DOMAIN*_PG_STATUS` (`dcn10.c:145-156`, called `dcn10.c:313-318`). It omits `DC_IP_REQUEST_CNTL.IP_REQUEST_EN` wrapping. Checklist: if intermittent PG failures occur, add the Linux `DC_IP_REQUEST_CNTL` gate around domain writes. | medium |
| 8 | HUBBUB global: Linux constructs hubbub (`dcn10_resource.c:697-710`, `1650-1655`), controls self-refresh in golden/watermark paths (`dcn10_hubbub.c:91-102`), and should not touch FB location for local mode (`DCHUBBUB_SDPIF_FB_*`, `dcn10_hubbub.c:625-672`). | Partly required: local aperture is set by ASIC_Init; self-refresh can affect stability but not basic format. | Gap: coreboot relies on ASIC_Init for aperture and has no `DCHUBBUB_ARB_DRAM_STATE_CNTL` or `DCHUBBUB_SDPIF_*` code. For one physical UMA pipe, do not program VM/aperture unless diagnostics show the ASIC_Init aperture is wrong. | medium |
| 9 | VM/aperture: Linux only programs HUBP VM if `gpu_vm_support`: `dcn10_program_pte_vm()` reads MMHUB aperture/context0 then calls `hubp_set_vm_system_aperture_settings()`/`hubp_set_vm_context0_settings()` (`dcn10_hwseq.c:2504-2515`, called at `2541-2542`). | Irrelevant for physical UMA non-GPUVM scanout | OK to skip. Coreboot uses physical GPU address `v2_fw.mc_base` with CPU BAR pointer (`atombios_driver.c:4852-4854`). Checklist: confirm `v2_fw.mc_base` equals the framebuffer MC base for target boards; otherwise pass `mc_base + fb_offset`, not CPU BAR. | medium |
| 10 | HUBP blank/clock/VTG: Linux blanks via `hubp1_set_blank()` using `DCHUBP_CNTL.HUBP_BLANK_EN` and `HUBP_TTU_DISABLE`, waits `HUBP_NO_OUTSTANDING_REQ` (`dcn10_hubp.c:41-62`); enables `hubp_clk_cntl()` (`dcn10_hwseq.c:2533-2535`); selects VTG in `hubp_vtg_sel()` from `dcn10_update_dchubp_dpp()` (`dcn10_hwseq.c:2833-2839`). | Required | Mostly OK: coreboot blanks and disables TTU (`dcn10.c:320-323`), enables `HUBP_CLK_CNTL.HUBP_CLOCK_ENABLE` (`dcn10.c:326`), clears `HUBP_VTG_SEL_MASK` to select VTG0 (`dcn10.c:320-323`, `396-399`). Missing `HUBP_NO_OUTSTANDING_REQ` wait. | low |
| 11 | HUBP request/DLG/TTU: Linux `hubp_setup()` calls `hubp1_program_requestor()` (`DCN_EXPANSION_MODE`, `DCHUBP_REQ_SIZE_CONFIG`) and `hubp1_program_deadline()` (`BLANK_OFFSET_0/1`, `DST_*`, `REF_FREQ_TO_PIX_FREQ`, `PREFETCH_SETTINS`, `PER_LINE_DELIVERY*`, `DCN_TTU_QOS_WM`, `DCN_SURF0_TTU_CNTL0/1`, `DCN_GLOBAL_TTU_CNTL`) (`dcn10_hubp.c:552-729`); invoked from `dcn10_update_dchubp_dpp()` (`dcn10_hwseq.c:2840-2849`). | Required | Partial: coreboot programs a one-plane integer subset of the same registers (`dcn10.c:237-287`, `332-333`). Checklist: keep non-DCC/linear assumptions; if underflow appears, replace constants/formulas with Linux DML `dml1_rq_dlg_get_dlg_params()` outputs for the exact mode. | high |
| 12 | HUBP surface config/address: Linux `hubp1_program_surface_config()` sets tiling, pitch, rotation, DCC, pixel format (`DCSURF_TILING_CONFIG`, `DCSURF_SURFACE_PITCH`, `DCSURF_SURFACE_CONFIG`, `DCSURF_SURFACE_CONTROL`, `HUBPRET_CONTROL`) (`dcn10_hubp.c:156-195`, `219-272`, `528-549`). Address latch via `hubp1_program_surface_flip_and_addr()` writes `DCSURF_FLIP_CONTROL`, high then low `DCSURF_PRIMARY_SURFACE_ADDRESS_HIGH/ADDRESS` (`dcn10_hubp.c:349-411`). | Required | OK for linear ARGB8888 non-DCC: coreboot writes `DCSURF_SURFACE_CONTROL`, `DCSURF_TILING_CONFIG`, `DCSURF_SURFACE_CONFIG=8`, pitch, viewport, flip, `HUBPRET_CONTROL`, high then low address (`dcn10.c:337-346`, `392-393`). DCC/meta/video/stereo paths irrelevant. | none |
| 13 | DPP/CNVC: Linux `dpp1_cnv_setup()` programs `FORMAT_CONTROL`/`CNVC_SURFACE_PIXEL_FORMAT` and alpha (`dcn10_dpp.c:274-382`); clock via `dpp1_dppclk_control()` on `DPP_CONTROL.DPP_CLOCK_ENABLE` (`dcn10_dpp.c:501-516`). | Required | OK for ARGB8888: coreboot sets `DPP_CONTROL.DPP_CLOCK_ENABLE`, `CNVC_SURFACE_PIXEL_FORMAT=8`, `CNVC_FORMAT_CONTROL=0x100` (`dcn10.c:327`, `349-350`). Input CSC, cursor, gamma, HDR multiplier irrelevant for firmware test pattern. | low |
| 14 | DSCL/scaler: Linux `dpp1_dscl_set_scaler_manual_scale()` sets `DSCL_CONTROL.SCL_BOUNDARY_MODE`, `MPC_SIZE`, `SCL_MODE.DSCL_MODE`; 1:1 RGB returns `DSCL_MODE_SCALING_444_BYPASS` (`dcn10_dpp_dscl.c:613-669`). Recout uses `RECOUT_START`, `RECOUT_SIZE` (`dcn10_dpp_dscl.c:577-600`). | Required in bypass/1:1 form | OK: coreboot writes `DSCL_CONTROL=0`, `DSCL_RECOUT_START=0`, `DSCL_RECOUT_SIZE`, `DSCL_MPC_SIZE`, `DSCL_SCL_MODE` to bypass value 6 (`dcn10.c:351-355`). Scaling taps, filters, LB irrelevant. | none |
| 15 | MPC composition: Linux `mpc1_insert_plane()` programs `MPCC_BOT_SEL`, `MPCC_CONTROL.MPCC_MODE`, `MPCC_TOP_SEL`, `MPCC_OPP_ID`, `MPCC_UPDATE_LOCK_SEL`, `MPC_OUT_MUX` (`dcn10_mpc.c:195-246`). | Required | OK for DPP0→MPCC0→OPP0: coreboot writes same mux/register set (`dcn10.c:358-363`). Alpha/MPO paths irrelevant. | none |
| 16 | OPP/FMT: Linux DCN10 HWSS has OPP FMT programming left in disabled/TODO block in `dcn10_enable_stream_timing()` (`dcn10_hwseq.c:1011-1055`); OPP pipe clock enabled in `dcn10_enable_plane()` through `opp_pipe_clock_control()` (`dcn10_hwseq.c:2536-2539`). | Required: OPP pipe clock. FMT programming irrelevant for 8bpc RGB in Linux DCN10 path. | OK for clock: coreboot sets `OPP_PIPE_CONTROL.OPP_PIPE_CLOCK_ENABLE` (`dcn10.c:328-329`). No FMT action needed unless adding non-8bpc/YUV/HDMI. | none |
| 17 | OTG timing/global sync: Linux `dcn10_enable_stream_timing()` enables OPTC clock, programs pixel clock, then `program_timing()` (`dcn10_hwseq.c:982-1010`). `optc1_program_timing()` writes `OTG_H_TOTAL`, `OTG_H_SYNC_A`, `OTG_H_BLANK_START_END`, `OTG_H_SYNC_A_CNTL`, `OTG_V_TOTAL`, `OTG_V_TOTAL_MIN/MAX`, `OTG_V_SYNC_A`, `OTG_V_BLANK_START_END`, `OTG_V_SYNC_A_CNTL`, `OTG_INTERLACE_CONTROL`, `OTG_CONTROL`, `OTG_H_TIMING_CNTL` (`dcn10_optc.c:156-330`). `optc1_program_global_sync()` writes `OTG_VSTARTUP_PARAM`, `OTG_VUPDATE_PARAM`, `OTG_VREADY_PARAM` (`dcn10_optc.c:63-93`). | Required | OK/partial: coreboot writes equivalent timing/global-sync registers (`dcn10.c:181-234`) and enables `OPTC_INPUT_CLOCK_CONTROL`, `OTG_CLOCK_CONTROL`, `OPTC_DATA_SOURCE_SELECT`, `VTG_CONTROL`, `OTG_CONTROL`, `OTG_BLANK_CONTROL` (`dcn10.c:367-373`). Checklist: verify polarity/porch packing against Linux `dc_crtc_timing` for target EDID. | medium |
| 18 | OTG enable/blank: Linux `optc1_enable_optc_clock()` sets `OPTC_INPUT_CLOCK_CONTROL.OPTC_INPUT_CLK_EN/GATE_DIS` and `OTG_CLOCK_CONTROL.OTG_CLOCK_EN/GATE_DIS` with waits (`dcn10_optc.c:483-517`); `optc1_enable_crtc()` sets `OPTC_DATA_SOURCE_SELECT.OPTC_SRC_SEL`, `OTG_CONTROL.OTG_DISABLE_POINT_CNTL/OTG_MASTER_EN` (`dcn10_optc.c:524-549`); blanking uses `OTG_BLANK_CONTROL.OTG_BLANK_DATA_EN/OTG_BLANK_DE_MODE` (`dcn10_optc.c:430-458`). | Required | Partial: coreboot sets the same control bits but does not wait for `OPTC_INPUT_CLK_ON`/`OTG_CLOCK_ON` and uses fixed `mdelay(1)` (`dcn10.c:367-373`). Checklist: add register waits if bring-up is flaky. | low |
| 19 | Locking/latching: Linux locks top pipe TG through `dcn10_pipe_control_lock()` (`dcn10_hwseq.c:1930-1953`); OPTC lock registers are `OTG_GLOBAL_CONTROL0.OTG_MASTER_UPDATE_LOCK_SEL` and `OTG_MASTER_UPDATE_LOCK.UPDATE_LOCK_STATUS` (`dcn10_optc.c:674-693`). Pending wait uses VBLANK→VACTIVE (`dcn10_hwseq.c:3038-3068`). HUBP address latches on low address write (`dcn10_hubp.c:374-411`). | Mostly irrelevant before first enable; address latch required. | OK for address latch: high then low (`dcn10.c:392-393`). No TG update lock. Checklist: skip lock during cold blanked programming; add lock/pending only for live mode changes. | none |
| 20 | Watermarks: Linux bandwidth paths call `clk_mgr->update_clocks()` and `hubbub->program_watermarks()` (`dcn10_hwseq.c:3119-3141`, `3157-3179`). `hubbub1_program_watermarks()` writes `DCHUBBUB_ARB_DATA_URGENCY_WATERMARK_*`, `DCHUBBUB_ARB_PTE_META_URGENCY_WATERMARK_*`, `DCHUBBUB_ARB_ALLOW_SR_*`, `DCHUBBUB_ARB_ALLOW_DRAM_CLK_CHANGE_WATERMARK_*`, `DCHUBBUB_ARB_SAT_LEVEL`, `DCHUBBUB_ARB_DF_REQ_OUTSTAND` (`dcn10_hubbub.c:257-609`). | Required for robust scanout/pstate, though visible pipe may work with BIOS defaults. | Gap: coreboot only programs per-HUBP TTU/QoS (`dcn10.c:273-287`) and does not program DCHUBBUB global watermarks. Checklist: either preserve/verify BIOS watermarks or add a conservative single-pipe watermark set; this is the main Linux parity gap. | high |
| 21 | Validation: Linux resource funcs validate bandwidth/plane/global (`dcn10_validate_bandwidth()`→`dcn_validate_bandwidth`, `dcn10_resource.c:1141-1153`; `dcn10_validate_plane()`, `1155-1163`; `dcn10_validate_global()`, `1165-1219`; registered `1278-1287`). | Required as static constraints, not runtime DML for firmware | Partial: coreboot validates null/clock/vblank/aperture size (`dcn10.c:302-312`) but not mode bandwidth. Checklist: for this scope require ARGB8888 only, pitch aligned to 64 pixels, aperture >= pitch*height, nonzero ref/disp/pixel clocks, h/v timing sane; no DCC/video/scaling. | medium |

## Compact implementation checklist

1. Keep v2 sequence: `asic_init` → `dcn10_disable_vga()` → `ATOM_INIT/ATOM_DISABLE` display-pipe golden init → eDP power → AUX/EDID → link/timing → scanout.
2. Before programming plane0, clear MPCC0/MUX0 (or all MPCCs) to Linux reset values: `MPCC_TOP_SEL=0xf`, `MPCC_BOT_SEL=0xf`, `MPCC_OPP_ID=0xf`, `MPCC_UPDATE_LOCK_SEL=0xf`, `MPC_OUT_MUX=0xf`.
3. Power pipe0 domains: `DOMAIN0_PG_CONFIG/STATUS`, `DOMAIN1_PG_CONFIG/STATUS`; add `DC_IP_REQUEST_CNTL.IP_REQUEST_EN` wrap only if needed.
4. Program clocks minimally: VBIOS `SetPixelClock`; direct enable `HUBP_CLK_CNTL`, `DPP_CONTROL.DPP_CLOCK_ENABLE`, `OPP_PIPE_CONTROL.OPP_PIPE_CLOCK_ENABLE`, `OPTC_INPUT_CLOCK_CONTROL`, `OTG_CLOCK_CONTROL`; add clock-on waits if unstable.
5. Program HUBP for linear non-DCC ARGB8888 only: blank, requestor/DLG/TTU, `DCSURF_*`, `HUBPRET_CONTROL`, high-then-low primary address.
6. Program DPP/DSCL bypass: `CNVC_SURFACE_PIXEL_FORMAT=8`, `CNVC_FORMAT_CONTROL`, `DSCL_*` 1:1 bypass.
7. Program MPC one-plane path: DPP0→MPCC0→OPP0.
8. Program OTG timing/global sync/blank: `OTG_H_*`, `OTG_V_*`, `OTG_VSTARTUP_PARAM`, `OTG_VUPDATE_PARAM`, `OTG_VREADY_PARAM`, `OPTC_DATA_SOURCE_SELECT`, `VTG_CONTROL`, `OTG_CONTROL`, `OTG_BLANK_CONTROL`.
9. Use no update lock for cold blanked programming; use high-then-low HUBP address latch.
10. Close parity gap: either prove BIOS `DCHUBBUB_ARB_*` watermarks survive golden init for one pipe, or add conservative `hubbub1_program_watermarks()` equivalent.
11. Keep validation bounded to one linear physical UMA ARGB8888 pipe; reject DCC, GPUVM, tiled, video formats, scaling, multiple planes, and insufficient aperture.

## Findings

- high: `src/drivers/amd/atombios/dcn10.c:273-287` programs per-HUBP TTU/QoS but coreboot has no equivalent for Linux `hubbub1_program_watermarks()` global `DCHUBBUB_ARB_*` watermarks (`/tmp/linux-6.12/.../dcn10_hubbub.c:257-609`). Required for robust Linux parity.
- high: `src/drivers/amd/atombios/dcn10.c:237-287` implements a reduced integer DLG/TTU subset instead of Linux DML-derived request/DLG/TTU register generation (`/tmp/linux-6.12/.../dcn10_hwseq.c:2765-2850`, `/tmp/linux-6.12/.../dcn10_hubp.c:552-729`). Acceptable only for the stated non-DCC linear ARGB8888 pipe until underflow is observed.
- medium: `src/drivers/amd/atombios/dcn10.c:358-363` inserts MPCC0 but no all-MPCC reset equivalent to Linux `mpc1_mpc_init()` (`/tmp/linux-6.12/.../dcn10_mpc.c:377-389`). Cold ASIC-init may hide it; S3/resume/stale state would not.
- medium: `src/drivers/amd/atombios/dcn10.c:145-156` ungates domains without Linux `DC_IP_REQUEST_CNTL.IP_REQUEST_EN` wrapper (`/tmp/linux-6.12/.../dcn10_hwseq.c:790-814`).
- medium: `src/drivers/amd/atombios/atombios_driver.c:4852-4854` uses `v2_fw.mc_base` as the GPU scanout address while CPU writes through `fb_bar`; target board must confirm those refer to the same UMA aperture base.

## Residual risks

- No hardware run was performed; this is source parity only.
- Missing `/home/arthur/src/coreboot/plan.md` and `progress.md` means no local plan/progress assumptions were available.
- Linux DCN1 relies on DML/SMU/clk-manager context not fully present in firmware; the checklist intentionally scopes to one boot pipe.

```acceptance-report
{
  "criteriaSatisfied": [
    {
      "id": "criterion-1",
      "status": "satisfied",
      "evidence": "Findings cite coreboot and Linux file paths/line ranges with severity; residual risks are listed."
    }
  ],
  "changedFiles": [],
  "testsAddedOrUpdated": [],
  "commandsRun": [
    {
      "command": "read /home/arthur/src/coreboot/plan.md and progress.md",
      "result": "passed",
      "summary": "Both files were requested but absent (ENOENT/no find matches)."
    },
    {
      "command": "read/grep/nl coreboot dcn10.c and atombios_driver.c",
      "result": "passed",
      "summary": "Inspected v2 call path, DCN1 register programming, and line-number evidence."
    },
    {
      "command": "read/grep/nl Linux 6.12 dcn10_hwseq.c, dcn10_resource.c, dcn10_hubp.c, dcn10_dpp.c, dcn10_dpp_dscl.c, dcn10_optc.c, dcn10_mpc.c, dcn10_hubbub.c",
      "result": "passed",
      "summary": "Inspected Linux ordered init/programming path and component register functions."
    }
  ],
  "validationOutput": [
    "Read-only parity matrix produced at /tmp/dcn10-hwseq-parity.md."
  ],
  "residualRisks": [
    "No hardware execution; source parity only.",
    "Requested plan.md/progress.md were absent.",
    "DML/SMU/clk-manager behavior is reduced to one linear boot pipe."
  ],
  "noStagedFiles": true,
  "diffSummary": "No repository edits; wrote required report artifact only.",
  "reviewFindings": [
    "high: src/drivers/amd/atombios/dcn10.c:273-287 - no Linux-equivalent DCHUBBUB_ARB_* global watermark programming.",
    "high: src/drivers/amd/atombios/dcn10.c:237-287 - reduced DLG/TTU model versus Linux DML-derived HUBP programming.",
    "medium: src/drivers/amd/atombios/dcn10.c:358-363 - no all-MPCC reset equivalent before insert.",
    "medium: src/drivers/amd/atombios/dcn10.c:145-156 - power-domain ungate lacks Linux DC_IP_REQUEST_CNTL wrapper.",
    "medium: src/drivers/amd/atombios/atombios_driver.c:4852-4854 - GPU address v2_fw.mc_base must match CPU framebuffer aperture."
  ],
  "manualNotes": "No code edits were made."
}
```