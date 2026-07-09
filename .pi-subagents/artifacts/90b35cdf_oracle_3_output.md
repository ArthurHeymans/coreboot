Turn budget wrap-up was requested after 8 assistant turns (soft limit 8, grace 2). Process-mode live steering is unavailable, so the child was warned at launch to wrap up by this budget. Output may be partial.

## Inherited decisions

- Native DCN1 only; no GOP snapshots, register replay, option-ROM fallback, or payload fallback.
- Link training, panel power, OTG timing, and OTG master enable already work.
- `DCHUBP_CNTL=0xA` means HUBP remains internally blank with no requests.
- Visible output and Linux takeover remain the acceptance criteria.

## Diagnosis

Linux does not make `atom.c` perform scanout. Atom command tables initialize the ASIC, clocks, PHY, and link; Linux Display Core then performs a full DCN state commit.

Current `dcn10.c` implements fragments of that commit using approximated values. It is not yet a complete Linux-equivalent cold-init path.

### Concrete findings

1. **blocker — `src/drivers/amd/atombios/dcn10.c`: missing `DCSURF_ADDR_CONFIG`**
   - Linux `hubp1_program_tiling()` always programs memory topology, even for linear surfaces.
   - Current code only clears `DCSURF_TILING_CONFIG`.
   - Required fields include pipe count, banks, interleave, shader engines, RBs/SE, and compressed fragments.

2. **blocker — `dcn10.c`: incomplete and guessed RQ/DLG/TTU programming**
   - Linux programs the full requestor, deadline, TTU, and interdependent register sets.
   - Current omissions include:
     - `DCHUBP_REQ_SIZE_CONFIG_C`
     - `VBLANK_PARAMETERS_1..4`
     - `NOM_PARAMETERS_0..7`
     - chroma/cursor TTU controls
     - `HUBPREQ_DEBUG_DB` VREADY workaround
   - Current assumptions are not Linux-equivalent:
     - `DCHUBP_REQ_SIZE_CONFIG = 0x055d1b70`
     - `BLANK_OFFSET_1 = v_blank_start * 4`
     - fixed four-line prefetch
     - request count derived from pitch rather than DML’s swath geometry
     - fixed 4 µs urgency
     - boot DISPCLK treated as active DISPCLK/DPPCLK
     - fixed 8 MHz deep-sleep DCFCLK
   - These values must come from a complete single-plane DCN1 DML calculation.

3. **blocker — `dcn10.c`: no atomic update-lock sequence**
   - Linux programs global sync, timing, HUBP, DPP, MPC, surface address, and blank state while the OTG/MPCC update domain is locked.
   - It then unlocks and waits for VBLANK followed by VACTIVE to ensure VUPDATE occurs.
   - Current code writes live registers, waits a fixed 20 ms, and assumes they latched.

4. **blocker — `dcn10.c`: VM/aperture state is assumed, not established**
   - Linux validates or programs DCN VM system aperture, context-0 settings, L1 TLB mode, and DCHUBBUB framebuffer/AGP ranges.
   - Current code assumes `v2_fw.mc_base` is immediately fetchable.
   - The GPU address must be checked against `DCHUBBUB_SDPIF_FB_BASE/TOP` and the relevant DCN VM aperture.

5. **high — `dcn10.c`: incomplete scaler initialization**
   - Linux explicitly disables DSCL AutoCal, clears boundary mode, programs recout/MPC size, and then selects bypass/scaling mode.
   - Current code does not explicitly program `DSCL_AUTOCAL`; its raw `SCL_MODE=6` assumes prior state is compatible.

6. **high — `dcn10.c`: incomplete DPP color pipeline**
   - Linux initializes CNVC, input CSC bypass, degamma mode, bias/scale, gamut bypass, output CSC, and HDR multiplier.
   - Current code only writes pixel format and `FORMAT_CONTROL`.
   - These omissions may not prevent memory requests, but can produce black output after fetch starts.

7. **high — `dcn10.c`: OPP formatter is not initialized**
   - Linux programs bit-depth reduction, RGB pixel encoding, clamping, and dynamic expansion after source selection.
   - Current code only enables the OPP pipe clock.

8. **high — `dcn10.c`: MPC insertion is incomplete for cold state**
   - Linux removes stale MPCC state or asserts idle, inserts MPCC0, programs blending/global gain/alpha, and verifies routing.
   - Current code assumes MPCC0 is idle and only programs routing fields.

9. **high — `dcn10.c`: HUBBUB bandwidth state is omitted**
   - Linux programs urgent, PTE/meta, stutter, and pstate watermarks, plus:
     - arbitration saturation
     - minimum outstanding DF requests
     - self-refresh policy
   - Firmware golden init may leave usable defaults, but a complete cold-init backend cannot assume that.

10. **medium — `atom.c` is not the current blocker**
    - Vilboz’s used command tables rely on supported direct-MMIO and framebuffer opcodes.
    - ASIC init, AUX, transmitter control, and link training already prove the interpreter executes the required paths.
    - Porting more interpreter details alone cannot make HUBP request pixels.

11. **informational — missing VGA log is expected**
    - Linux skips the VGA transition workaround when all VGA mode-enable bits are already clear.

## Drift / contradiction check

The current approach has drifted from “implement Linux DCN1” into incrementally approximating individual DML and hardware-sequencer values. That conflicts with the user’s requirement for a comprehensive Linux-derived implementation.

The prior assumption that a small hand-derived DLG subset would be sufficient should be revised. Linux treats RQ, DLG, TTU, global sync, clocks, VM, watermarks, and update locking as one validated state commit.

## Recommendation

Replace the incremental register additions with these implementation phases:

1. **DCN1 hardware state model**
   - Define named registers and masks from Linux 6.12.
   - Read actual DISPCLK/DPPCLK/DCFCLK and memory topology.
   - Establish local-only framebuffer and DCN VM/aperture state.

2. **Cold pipe reset**
   - Golden init.
   - Disable/reset all pipes.
   - Remove stale MPCC routing.
   - Power HUBP0/DPP0 and verify PGFSM/clock-on state.
   - Disable cursor, DCC, VM translation if physical local addressing is selected.

3. **Complete single-plane DML calculator**
   - Port the narrow Linux DCN1 path for one linear, non-DCC, non-VM, ARGB8888 plane.
   - Produce complete RQ, DLG, TTU, global-sync, and watermark structures.
   - Do not retain fixed register constants.

4. **Atomic stream commit**
   - Lock OTG/MPCC updates.
   - Program pixel clock source, OTG timing/global sync, OPP RGB888 formatter, clamping, and bit-depth policy.
   - Keep OTG blanked.

5. **Atomic plane commit**
   - Program memory topology and tiling.
   - Program full RQ/DLG/TTU and VREADY workaround.
   - Initialize DPP CNVC/CSC/degamma/scaler.
   - Insert and configure MPCC0.
   - Write surface high address then low address.
   - Clear HUBP blank/TTU disable.

6. **Latch and validate**
   - Unlock.
   - Wait for VBLANK then VACTIVE, not a fixed delay.
   - Require:
     - update-pending clear
     - `HUBP_IN_BLANK=0`
     - earliest-in-use equals requested address
     - no HUBP/OTG underflow
     - no VM protection fault
     - MPCC0 routed and active
   - Only then enable backlight and register the framebuffer.

## Risks

- FirmwareInfo `mc_base` may describe UMA base but still require DCHUBBUB translation.
- Exact DML output depends on actual clocks and Raven topology, not only EDID and boot clocks.
- Watermarks must be conservative enough for cold boot and later Linux takeover.
- Direct register commits without update locking can remain nondeterministic even with correct values.

## Need from main agent

No product decision is required. The existing hand-derived DLG path should be replaced rather than extended.

## Suggested execution prompt

> Replace the current incremental `dcn10_program_scanout()` implementation with a complete Linux 6.12-derived one-pipe DCN1 state commit. Port the narrow linear, non-DCC, physical-UMA ARGB8888 path: cold pipe reset, memory topology/aperture, full RQ/DLG/TTU and watermarks, DPP/DSCL/CSC, MPC/OPP, OTG update locking, surface latch, VBLANK/VACTIVE wait, and fault/underflow validation. Use named fields and calculated structures; remove guessed constants and fixed-delay latching.