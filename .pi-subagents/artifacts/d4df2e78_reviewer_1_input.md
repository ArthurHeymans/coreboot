# Task for reviewer

[Read from: /home/arthur/src/coreboot/plan.md, /home/arthur/src/coreboot/progress.md]

Audit /home/arthur/src/coreboot/src/drivers/amd/atombios/dcn10_calc.c and deadline/watermark programming in dcn10.c against Linux 6.12 DCN1 DML/RQ/DLG/TTU/hubbub code under /tmp/linux-6.12. Target exactly one Picasso eDP ARGB8888 linear physical-UMA plane, 1:1, no GPUVM/DCC. Find concrete wrong formulas, field packing, missing required registers, unsafe clocks/units, or invented constants. Do not edit. Provide source-backed minimal fixes and rank severity. Ignore lack of hardware validation.

---
**Output:**
Write your findings to exactly this path: /tmp/dcn10-review-dml.md
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