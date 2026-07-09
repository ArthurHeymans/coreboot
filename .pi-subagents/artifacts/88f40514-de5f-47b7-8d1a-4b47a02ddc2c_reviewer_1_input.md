# Task for reviewer

[Read from: /home/arthur/src/coreboot/plan.md, /home/arthur/src/coreboot/progress.md]

Produce a complete read-only register/formula matrix for the narrow Linux 6.12 DCN1 one-plane path. Compare coreboot dcn10.c with Linux dcn10_hubp.c/h, dcn10_dpp.c and dpp_dscl.c, dcn10_mpc.c, dcn10_opp.c, dcn10_optc.c, dcn10_hubbub.c, dce_clock_source.c, and DML1. For 1366x768 linear non-DCC ARGB8888 physical UMA, list every register actually written, fields/masks, ordering, and exact source-derived calculation. Include DCSURF_ADDR_CONFIG/topology, full RQ/DLG/TTU, VM and hubbub aperture, watermarks, FMT/color bypass, update locks, and waits/status. Mark any values that cannot be derived from current inputs and identify Linux source of the missing input. No edits.

---
**Output:**
Write your findings to exactly this path: /tmp/dcn10-register-parity.md
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