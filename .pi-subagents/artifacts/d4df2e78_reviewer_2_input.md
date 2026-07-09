# Task for reviewer

[Read from: /home/arthur/src/coreboot/plan.md, /home/arthur/src/coreboot/progress.md]

Audit the current full native AtomFirmware v2/DCN1 orchestration in /home/arthur/src/coreboot/src/drivers/amd/atombios/atombios_driver.c plus dcn10.c. Check ownership boundaries (Atom ASIC/PLL/PHY/link vs direct DCN timing/plane), sequencing of panel power/link/stream/backlight/framebuffer registration, all failure rollback paths, warm/cold behavior, and Linux takeover hazards. Compare to Linux 6.12 where useful. Do not edit. Report only actionable software implementation gaps; do not use pending hardware validation as a blocker.

---
**Output:**
Write your findings to exactly this path: /tmp/dcn10-review-orchestration.md
This path is authoritative for this run.
Ignore any other output filename or output path mentioned elsewhere, including output destinations in the base agent prompt, system prompt, or task instructions.

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