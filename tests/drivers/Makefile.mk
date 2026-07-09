# SPDX-License-Identifier: GPL-2.0-only

tests-y += efivars-test
tests-y += atombios-v2-test
tests-y += dcn10-test

dcn10-test-srcs += tests/drivers/dcn10-test.c
dcn10-test-srcs += src/drivers/amd/atombios/dcn10.c
dcn10-test-srcs += src/drivers/amd/atombios/dcn10_calc.c
dcn10-test-srcs += tests/stubs/console.c
dcn10-test-cflags += -DDCN10_TEST_TRACE

atombios-v2-test-srcs += tests/drivers/atombios-v2-test.c
atombios-v2-test-srcs += src/drivers/amd/atombios/atombios_v2.c

efivars-test-srcs += tests/drivers/efivars.c
efivars-test-srcs += src/drivers/efi/efivars.c
efivars-test-srcs += tests/stubs/console.c
efivars-test-srcs += src/commonlib/region.c

efivars-test-cflags += -I src/vendorcode/intel/edk2/UDK2017/MdePkg/Include/
efivars-test-cflags += -I src/vendorcode/intel/edk2/UDK2017/MdePkg/Include/Ia32/
efivars-test-cflags += -I src/vendorcode/intel/edk2/UDK2017/MdePkg/Include/Pi/
efivars-test-cflags += -I src/vendorcode/intel/edk2/UDK2017/MdeModulePkg/Include/
