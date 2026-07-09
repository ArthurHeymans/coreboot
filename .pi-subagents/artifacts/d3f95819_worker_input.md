# Task for worker

You are reviving a previous subagent conversation.

Original run: f38caa86-ee94-41c6-9006-d354a6f959c6
Original agent: worker
Original session file: /home/arthur/.pi/agent/sessions/--home-arthur-src-coreboot--/2026-07-10T09-35-27-720Z_019f4b61-c9e8-70d9-a9db-536139a31e61.jsonl

Use the stored session context as background. Answer the orchestrator's follow-up below. Do not assume the original child process is still alive.

Follow-up:
Your implementation exists and builds; the run failed only acceptance bookkeeping. Before finalizing, address these parent-review gaps and re-run verification:

1. The fixed DPP path is not complete versus Linux dcn10_update_dpp(): program identity CM_BNS_VALUES_R/G/B (scale 0x2000, bias 0) and any required fixed RGB degamma/CSC bypass fields with exact named masks/offsets.
2. The 1:1 DSCL_MODE_SCALING_444_BYPASS path still runs dpp1_dscl_set_lb(); add the exact LB_DATA_FORMAT and LB_MEMORY_CTRL state for fixed ARGB8888, 36-bpp line buffer/opaque plane as Linux computes. Confirm DSCL mode value and offsets from v6.12.
3. min_set_viewport() writes secondary graphics viewports too; explicitly program them or provide source-backed proof they are irrelevant on non-stereo cold state.
4. Re-check the fixed no-GPUVM/no-DCC DLG path. Linux dml1 still computes t_vm/t_r0 and VBLANK/NOM fields; do not call the implementation 'complete' while blindly zeroing every VBLANK/NOM register. Either derive/program the exact required fixed-case values, or prove with Linux field usage that each zeroed field is ignored when VM/DCC are disabled. Update structures/tests accordingly.
5. The current fake-MMIO test does not actually record/assert write order despite the report claiming it does. Add a practical write-order assertion/hook, or correct the report and add the strongest feasible sequence invariant test.
6. Verify every newly copied offset/mask via a script/command against Linux generated headers; include result.

Do not broaden scope or touch unrelated files. Preserve current code. Re-run DCN tests, parser tests, Vilboz build, scoped checkpatch. Finish with concise blockers/residual risks and a valid structured acceptance report.

## Acceptance Contract
Acceptance level: reviewed
Completion is not accepted from prose alone. End with a structured acceptance report.

Criteria:
- criterion-1: Implement the requested change without widening scope
- criterion-2: Return evidence sufficient for an independent acceptance review

Required evidence: changed-files, tests-added, commands-run, validation-output, residual-risks, no-staged-files

Review gate: required by reviewer.

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