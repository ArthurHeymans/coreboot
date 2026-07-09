Implemented the Linux 6.12-derived fixed one-pipe DCN1 commit for Vilboz/Picasso.

Key changes:
- Removed Atom CRTC timing/enable/blank ownership from the DCN1 path. Atom now provides ASIC/PLL/DIG/PHY/link services; direct DCN programs OTG timing and blanking. The trained link parameters are preserved across prepare/train/backlight phases, and backlight is enabled only after plane validation.
- Replaced the prototype timing/DLG math with `dcn10_calc.c`, a pure fixed-case calculator for one linear, non-DCC, non-VM ARGB8888 plane. It produces the complete enabled luma RQ/DLG/TTU/global-sync/watermark state and explicit zero state for disabled chroma/cursor/VM/meta paths.
- Added cold DCCG/MPC/HUBP initialization, DC_IP_REQUEST-wrapped HUBP0/DPP0 power-on, clock/status waits, OTG frame-count validation, update locking, all-MPCC reset, local-FB aperture validation, GFX9-to-DCSURF address topology translation, local-only AGP invalidation, four watermark sets, conservative static self-refresh/pstate policy, full fixed RGB DPP/DSCL/MPC/OPP/surface setup, high-before-low address latch, and bounded HUBP blank/address/underflow validation.
- Plane failures keep the backlight off, reblank after a failed live commit, and never register the framebuffer.
- Added calculator, aperture, fake-MMIO successful-sequence, and fail-closed host tests.

Changed files:
- `src/drivers/amd/atombios/Makefile.mk`
- `src/drivers/amd/atombios/atombios_driver.c`
- `src/drivers/amd/atombios/dcn10.c`
- `src/drivers/amd/atombios/dcn10.h`
- `src/drivers/amd/atombios/dcn10_calc.c` (new)
- `tests/drivers/Makefile.mk`
- `tests/drivers/dcn10-test.c` (new)

Validation:
- `make build-tests/drivers/dcn10-test && make tests/drivers/dcn10-test`: 4/4 passed.
- `make build-tests/drivers/atombios-v2-test && make tests/drivers/atombios-v2-test`: 6/6 passed.
- `make -j2`: Vilboz firmware build passed with no compiler warnings/errors in the build log.
- Scoped `jj diff --git | util/lint/checkpatch.pl --no-tree -`: 0 errors, 0 warnings.
- The source-index diagnostics still report missing coreboot headers such as `console/console.h`/`cbfs.h`; the supervisor approved treating these as stale compile-database/include-path false positives because both host and firmware compilers resolve them cleanly. No source include workaround or unrelated clang configuration was added.

Residual hardware-only risks:
- The first boot must confirm ASIC_Init exposes `v2_fw.mc_base` inside `DCHUBBUB_SDPIF_FB_BASE/TOP` and that Picasso's live `GB_ADDR_CONFIG` uses a DCN1-supported pipe-interleave encoding. The backend now fails closed with precise diagnostics otherwise.
- Visible output, repeated cold/warm boots, framebuffer modification, and Linux amdgpu takeover still require hardware validation.
- The static firmware watermark policy deliberately prevents self-refresh and memory-pstate changes during coreboot; Linux is expected to replace that state during takeover.

```acceptance-report
{
  "criteriaSatisfied": [
    {
      "id": "criterion-1",
      "status": "satisfied",
      "evidence": "Implemented only the requested Picasso 0x15d8 one-pipe linear physical-UMA ARGB8888 path; no GOP replay, fallback, GPU VM, DCC, scaling, cursor, audio, multi-plane, or generic DRM framework was added."
    },
    {
      "id": "criterion-2",
      "status": "satisfied",
      "evidence": "Added four host tests, retained six parser tests, completed a warning-free Vilboz build, and passed scoped checkpatch with 0 errors/warnings."
    }
  ],
  "changedFiles": [
    "src/drivers/amd/atombios/Makefile.mk",
    "src/drivers/amd/atombios/atombios_driver.c",
    "src/drivers/amd/atombios/dcn10.c",
    "src/drivers/amd/atombios/dcn10.h",
    "src/drivers/amd/atombios/dcn10_calc.c",
    "tests/drivers/Makefile.mk",
    "tests/drivers/dcn10-test.c"
  ],
  "testsAddedOrUpdated": [
    "tests/drivers/dcn10-test.c",
    "tests/drivers/Makefile.mk"
  ],
  "commandsRun": [
    {
      "command": "make build-tests/drivers/dcn10-test && make tests/drivers/dcn10-test",
      "result": "passed",
      "summary": "4/4 calculator, bounds, fake-MMIO sequence, and fail-closed tests passed."
    },
    {
      "command": "make build-tests/drivers/atombios-v2-test && make tests/drivers/atombios-v2-test",
      "result": "passed",
      "summary": "Existing bounded AtomFirmware parser tests passed 6/6."
    },
    {
      "command": "make -j2",
      "result": "passed",
      "summary": "Built google/zork (Vilboz); no compiler warning/error lines in the build log."
    },
    {
      "command": "jj diff --git -- <scoped paths> | util/lint/checkpatch.pl --no-tree -",
      "result": "passed",
      "summary": "0 errors, 0 warnings."
    }
  ],
  "validationOutput": [
    "dcn10-test: 4 tests run, 4 passed",
    "atombios-v2-test: 6 tests run, 6 passed",
    "Built google/zork (Vilboz)",
    "checkpatch: total 0 errors, 0 warnings"
  ],
  "residualRisks": [
    "Hardware must validate the ASIC_Init local-FB aperture and live GFX9 address topology.",
    "Visible scanout and Linux takeover remain hardware-only acceptance gates.",
    "Source-index missing-header diagnostics are stale compile-database false positives; real builds pass."
  ],
  "noStagedFiles": true,
  "diffSummary": "Replaced the incremental DCN1 prototype with a staged fixed one-pipe cold-init/stream/plane commit, pure calculator, fail-closed orchestration, and host tests.",
  "reviewFindings": [
    "no source/build/test/checkpatch blockers",
    "hardware acceptance remains required before claiming complete native support"
  ],
  "manualNotes": "The Jujutsu working copy already contained extensive unrelated dirty changes and .pi-subagents artifacts; they were preserved and not modified or deleted."
}
```
