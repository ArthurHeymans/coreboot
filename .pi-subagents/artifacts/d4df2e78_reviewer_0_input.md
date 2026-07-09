# Task for reviewer

[Read from: /home/arthur/src/coreboot/plan.md, /home/arthur/src/coreboot/progress.md]

Read current DCN1 implementation in /home/arthur/src/coreboot/src/drivers/amd/atombios/{dcn10.c,dcn10.h,dcn10_calc.c,atombios_driver.c} and compare the fixed Picasso one-pipe cold-state commit against Linux 6.12 sources under /tmp/linux-6.12/drivers/gpu/drm/amd/display/dc. Focus only concrete implementation omissions or wrong sequencing in cold reset, DCCG/clocks, power gating, OTG locking/timing, HUBP/DPP/DSCL/MPC/OPP/FMT and final validation. Do not edit. Rank flash-blocking findings with exact current lines and Linux source evidence. Ignore hardware validation status as a reason not to implement.

---
**Output:**
Write your findings to exactly this path: /tmp/dcn10-review-hwseq.md
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