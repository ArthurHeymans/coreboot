# AMD SP3: native Rome/S8030 initialization

The selectable Tyan S8030 port boots Linux on the S8030GM2NE-NFX bench with
an EPYC 7502P. It runs native x86 initialization after the vendor PSP/ABL
trains memory and releases the BSP. The existing Zeppelin workspace and SoC
remain separate.

## Supported scope

* Rome, one socket, up to 128 threads and four NBIO root bridges.
* S8030 GPIO, BMC UART/KCS and configurable PCIe slot bifurcation. The tested
  configuration is `11411`: PCIE#3 uses an x4x4x4x4 NVMe carrier.
* Live DF3 topology and resource windows, NBIO quadrant programming, SMU V11,
  two-pass DXIO training, Huashan SATA/USB, APOB-driven AP release, CCX setup,
  SMM relocation, PCI enumeration and interrupt routing.
* MADT, IVRS, FADT and ACPI CPU power-state tables. ACPI S5 poweroff is tested.

The CPU-identification and pure topology helpers also describe Naples, Milan
and two-socket systems. **Their firmware initialization is not implemented by
this port.** Runtime initialization rejects unsupported generations and 2P
systems rather than reusing Rome's sequence. Milan, Naples, other S8030 board
variants, warm reboot, suspend/resume and other CPU OPNs remain unvalidated.
No synthetic per-NBIO NUMA domains are emitted; SRAT/CRAT support is pending.

## Firmware image and building

Supply the board-owned, complete 32 MiB donor through `SP3_VENDOR_ROM`:

```text
bca9c7c6c7484ceb8b32fc3b43bd66158fa8255a456c217ba5bdb8e85b0af93d
```

The build pins this hash and preserves flash `0..0x50afff`, including both
PSP/BIOS directory levels and the vendor PSP/ABL/SMU/PMU/APCB firmware. BIOS
directory entry `0x62` loads the replacement reset image from flash `0xdb0000`
to DRAM `0x75db0000`, length `0x250000`, ending at `0x76000000`.

| Flash area | Offset | Size |
| --- | --- | --- |
| AMDFW | `0` | `0x50b000` |
| FMAP | `0x50b000` | `0x1000` |
| COREBOOT/CBFS | `0x50c000` | `0x8a4000` |
| BOOTBLOCK | `0xdb0000` | `0x250000` |
| VENDOR_UPPER, preserved verbatim | `0x1000000` | `0x1000000` |

The output is a full 32 MiB image. The AMD boot-device mapping exposes the
lower 16 MiB through ROM2; CBFS stays entirely in that window. On the bench,
**flash only the lower 16 MiB**, with the host off, and verify its SHA-256
readback. Never overwrite the only donor backup or erase the upper half.

Start with `configs/config.tyan_s8030`, set the donor path and payload through
menuconfig, then build normally. This configuration defaults to serial-only
graphics and no payload; select the Linux payload and provide a kernel for
boot testing. The console is 115200 baud. On NixOS, the tested build uses:

```sh
nix-shell -p zlib openssl ncurses pkg-config --run \
  'make UPDATED_SUBMODULES=1 XGCCPATH=/path/to/xgcc/bin/ -j16'
```

The trailing slash in `XGCCPATH` matters. `UPDATED_SUBMODULES=1` assumes the
required submodules are already available.

The donor's B0 microcode is old. `SP3_UCODE_8310_FILE` optionally replaces the
x86 CBFS patch with a newer raw, signed AMD patch for equivalence ID `0x8310`;
it does not modify the preserved vendor firmware. The bench uses `0x0830107c`
from the locally installed linux-firmware AMD container. Its extracted patch
SHA-256 is `2164defb9f689647705ce28f18526aaca4d22ee0de6866c3eb1ed33fbc511d38`.

## Hardware validation

The read-only Linux hardware-test payload completed and powered the host off
through ACPI. It enumerated all 64 threads, 128 GiB of ECC RAM, six WD NVMe
drives, both Intel i210s, the Mellanox adapter and the SAS HBA/storage. EDAC
reported zero corrected and uncorrected errors. No bench data disks were
mounted or written.

Early boots exposed a server APOB larger than the inherited 128 KiB bound
(actual size `0x21958`), tagged DXIO response kinds, missing internal-bridge
ACPI objects, inappropriate root-IRQ lookups and an incorrect flash-size
configuration. These were corrected rather than hidden. The subsequent boot
has no coreboot error-level messages or ACPI namespace errors. Expected
optional-file warnings remain for absent Windows SLIC and ASPEED VGA ROMs;
the board uses the BMC UART, not an x86 VGA option ROM.

Images, lower-half readback hashes and complete serial logs are local to the
`sp3` analysis/bench checkout and this workspace's `.local/` directory. A Rome
1P boot does not validate the unimplemented generations or two-socket paths.

## Evidence for identification and topology

Local analysis: `sp3/reports/SOC_DF_SP3.md` sections 2.2, 2.3 and 3.1, with
`sp3/reports/soc-df-contracts.json` and `sp3/reports/module-catalog.json`.
The recovered CPU gates, topology-register operands and DF3 primary-IOMS
selection are **instruction-checked**. Extra fail-closed validation is port
policy, not a claim about donor behavior. Integration references also include
`PSP_APOB_SP3`, `NBIO_SMU_SP3`, `PCIE_DXIO_SP3`, `FCH_SP3` and `CCX_SP3` in
that checkout's `reports/` directory and the working fstart SP3 implementation.

All addresses below are PEI XIP VAs. Donors and extracted executables remain
local; executable hashes are recorded for reproducibility.

| Donor | Module / GUID | Entry VA |
| --- | --- | --- |
| `naples-s8026-v202` | AmdSocSp3ZpPei / `5127f36a-955e-4b7d-89bc-7ec0266841e6` | `0xffe0741c` |
| `rome-s8030-v404` | AmdSocSp3RmPei / `3e905f4d-4ab9-f1ce-9359-56a247b3eb79` | `0xffe1c44c` |
| `milan-s8030-v404` | AmdSocSp3GnPei / `d2a639e3-4ea3-9cd9-7280-379f2bd3f959` | `0xffd39d40` |

Executable SHA-256, in table order:

```text
72678c4c50167d1e7add0418bce9e639bd30ac6923b9a72c0c8c587676d3fb10
41f1463f802c4648c81ef367f08fc237d83b1d0a549fac4beb6e1714d5045de2
25b809924088d0cb803ad75f0ec776f03086efe00f7fcebb778cf73c7387a8e2
```

Rome SoC dispatch is at `0xffe1c45d`; primary-IOMS selection and physical/logical
ordering are at `0xffe21d74` and `0xffe20886`. Milan's fuse-filtered construction
starts at `0xffd39d40`. Naples SoC dispatch is at `0xffe0742d`.
