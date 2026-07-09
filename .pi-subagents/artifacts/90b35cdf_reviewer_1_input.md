# Task for reviewer

[Read from: /home/arthur/src/coreboot/plan.md, /home/arthur/src/coreboot/progress.md]

Thoroughly compare /home/arthur/src/coreboot/src/drivers/amd/atombios/dcn10.c plane programming against Linux 6.12 DCN1 HUBP/DPP/DSCL/MPC/OPP implementation under /tmp/linux-6.12/drivers/gpu/drm/amd/display/dc/{hubp,dpp,mpc,opp}/dcn10. Identify every required register and sequence for one linear non-DCC ARGB8888 1366x768 plane, including requestor, DLG/TTU, VM/aperture, address, clocks, blank/unblank, and status checks. Do not edit. Return exact offsets/masks/formulas and source evidence.

## Acceptance Contract
Acceptance level: reviewed
Completion is not accepted from prose alone. End with a structured acceptance report.

Criteria:
- criterion-1: Return concrete findings with file paths and severity when applicable

Required evidence: changed-files, tests-added, commands-run, validation-output, residual-risks, no-staged-files

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