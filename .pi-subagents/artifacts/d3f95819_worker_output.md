Implemented the six DCN1 review corrections and updated `/tmp/dcn10-worker-result.md`.

Changed files: `dcn10.c`, `dcn10.h`, `dcn10_calc.c`, `tests/drivers/Makefile.mk`, `tests/drivers/dcn10-test.c`.

Validation: DCN 4/4, parser 6/6, Linux register checks 32/32, Vilboz build passed, checkpatch 0/0.

Open risks/questions: Hardware-visible output and Linux takeover remain unvalidated; payload warning remains.

Recommended next step: Independent review, then a fail-safe Vilboz hardware run.