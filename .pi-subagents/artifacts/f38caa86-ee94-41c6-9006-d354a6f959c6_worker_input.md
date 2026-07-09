# Task for worker

You are a delegated subagent running from a fork of the parent session. Treat the inherited conversation as reference-only context, not a live thread to continue. Do not continue or answer prior messages as if they are waiting for a reply. Your sole job is to execute the task below and return a focused result for that task using your tools.

Task:
Implement the complete Linux 6.12-derived one-pipe Raven/Picasso DCN1 native display commit for Vilboz in the current working copy. This is a writing task: you are the sole writer. Preserve every existing dirty change and do not modify/delete .pi-subagents artifacts or unrelated files.

Read first:
- /tmp/dcn10-complete-plan.md (authoritative implementation boundary)
- /tmp/dcn10-hwseq-parity.md
- src/drivers/amd/atombios/dcn10.c, dcn10.h, atombios_driver.c, atom.c, atomfirmware.h
- relevant Linux 6.12 files under /tmp/linux-6.12/drivers/gpu/drm/amd/display/dc and generated register headers.

User requirement: stop incremental one-register experiments. Replace the current dcn10.c prototype wholesale with a coherent, complete narrow backend for exactly one pipe/plane: Picasso 0x15d8, eDP, linear non-DCC physical UMA, ARGB8888/XRGB, 1:1 scaling, no GPU VM, no cursor/audio/multi-plane. No GOP snapshots, restored register values, option-ROM/payload fallback, or generic DRM framework.

Required work:
1. Eliminate dual timing ownership on DCN1: Atom may provide ASIC/PLL/PHY/transmitter/link services, but direct DCN owns OTG timing and blanking. Split/reorder atomfirmware_enable_edp_link so backlight is last and legacy Atom SetCRTC timing/EnableCRTC/BlankCRTC are not used for DCN1.
2. Replace guessed DLG/global-sync/constants with one internally consistent Linux-derived fixed-case calculator and complete RQ/DLG/TTU structures. Program every field/register consumed by Linux hubp1_program_requestor(), hubp1_program_deadline(), hubp1_setup_interdependent(), explicitly zeroing disabled chroma/cursor/VM/meta paths.
3. Implement cold reset/known state, DC_IP_REQUEST_CNTL-wrapped HUBP0/DPP0 power-on, clock-on waits, all-MPCC/MPC reset, complete DCSURF_ADDR_CONFIG from validated GFX9 address topology, local framebuffer/hubbub aperture validation, conservative Linux-derived A-D watermarks/arbitration/static firmware pstate/self-refresh policy.
4. Implement named-register/mask helpers (private header if useful), exact OTG lock/timing/global-sync/enable sequence, complete DPP CNVC/CSC/degamma/bias/scaler-bypass setup, MPCC0/OPP0/FMT setup, surface high-then-low latch, bounded VBLANK/VACTIVE/update/blank/address/underflow/fault validation.
5. Fail closed and keep backlight off; do not register framebuffer until all gates pass.
6. Add smallest practical host tests for pure calculator and fake-MMIO sequence/order/failure behavior, plus Makefile wiring. Do not over-generalize beyond the fixed supported path.

Where exact current hardware inputs are required, read/validate the Linux-equivalent MMIO state established by ASIC_Init; do not substitute captured GOP values. If an input cannot be obtained safely, fail closed with a precise diagnostic rather than retaining a guess.

Verification required before finishing:
- existing atombios-v2 parser tests pass
- new DCN10 tests pass
- Vilboz firmware build succeeds
- scoped checkpatch has 0 errors/warnings
- lens/build diagnostics show no new blocking errors
- report changed paths, commands/results, and any residual hardware-only risks.

Use jj rather than git for repository inspection/diff. Do not commit.

---
**Output:**
Write your findings to exactly this path: /tmp/dcn10-worker-result.md
This path is authoritative for this run.
Ignore any other output filename or output path mentioned elsewhere, including output destinations in the base agent prompt, system prompt, or task instructions.

## Acceptance Contract
Acceptance level: reviewed
Completion is not accepted from prose alone. End with a structured acceptance report.

Criteria:
- criterion-1: Implement the requested change without widening scope
- criterion-2: Return evidence sufficient for an independent acceptance review

Required evidence: changed-files, tests-added, commands-run, validation-output, residual-risks, no-staged-files

Review gate: optional by reviewer.

Finish with a fenced JSON block tagged `acceptance-report` in this shape:
Use empty arrays when no items apply; array fields contain strings unless object entries are shown.
```acceptance-report
{
  "criteriaSatisfied": [
    {
      "id": "criterion-1",
      "status": "satisfied",
      "evidence": "specific proof"
    }
  ],
  "changedFiles": [
    "src/file.ts"
  ],
  "testsAddedOrUpdated": [
    "test/file.test.ts"
  ],
  "commandsRun": [
    {
      "command": "command",
      "result": "passed",
      "summary": "short result"
    }
  ],
  "validationOutput": [
    "validation output or concise summary"
  ],
  "residualRisks": [
    "none"
  ],
  "noStagedFiles": true,
  "diffSummary": "short description of the diff",
  "reviewFindings": [
    "blocker: file.ts:12 - issue found, or no blockers"
  ],
  "manualNotes": "anything else the parent should know"
}
```