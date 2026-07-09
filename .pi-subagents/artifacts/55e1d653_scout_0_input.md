# Task for scout

Read-only. In /tmp/linux-6.12 AMD DCN1 sources, derive the smallest exact register sequence to display a linear XRGB8888 framebuffer at physical 0xe0000000, mode 1366x768, pitch 1366*4, using pipe0 on Raven/Picasso DCN1 after OTG timing and DP link are already enabled. Focus HUBP0, DPP0, MPC/MPCC0, OPP0 routing and blank/unblank. Provide register names, offset composition (DCN segment bases), fields/masks/values, and source file/line evidence. Do not edit.

## Acceptance Contract
Acceptance level: attested
Completion is not accepted from prose alone. End with a structured acceptance report.

Criteria:
- criterion-1: Return concrete findings with file paths and severity when applicable

Required evidence: review-findings, residual-risks

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