# Task for reviewer

Review the newly implemented DCN1 files for hardware-blocking correctness before a Vilboz flash. Read src/drivers/amd/atombios/dcn10.c, dcn10.h, dcn10_calc.c, the DCN1 orchestration in atombios_driver.c, and tests/drivers/dcn10-test.c. Compare exact fields/offsets/formulas against Linux 6.12 under /tmp/linux-6.12. Focus only on concrete blocker/high-severity mistakes, especially: incorrect register base/offset/mask; invalid GB_ADDR_CONFIG translation; framebuffer units; OTG lock/clock/timing; power gating; MPCC state; DPP/DSCL format; HUBP requestor/DLG/TTU completeness; watermark layout; whether zeroing VBLANK/NOM fields is valid without GPU VM/DCC; address latch and validation. Note that build/tests pass but tests may encode invented values. Do not edit. Return a concise ranked list with exact source evidence and state whether this is safe enough for one hardware validation.

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