# Task for reviewer

[Read from: /home/arthur/src/coreboot/plan.md, /home/arthur/src/coreboot/progress.md]

Produce a complete read-only Linux 6.12 parity matrix for native Raven/Picasso DCN1 cold init. Compare /home/arthur/src/coreboot/src/drivers/amd/atombios/dcn10.c and v2 call path in atombios_driver.c against /tmp/linux-6.12/drivers/gpu/drm/amd/display/dc/hwss/dcn10/dcn10_hwseq.c plus dcn10_resource.c. Cover every ordered step from post-ASIC-init to one visible eDP plane: clocks/DCCG, BIOS golden init, pipe reset, power domains, VGA, link encoder, hubbub, VM/aperture, HUBP, DPP, DSCL, MPC, OPP/FMT, OTG, locking/latching, watermarks, validation. Cite exact Linux functions and register names. Explicitly classify required vs irrelevant for one linear non-DCC physical-UMA ARGB8888 pipe. No edits. Return a compact implementation checklist, not exploration prose.

---
**Output:**
Write your findings to exactly this path: /tmp/dcn10-hwseq-parity.md
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