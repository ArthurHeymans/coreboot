## Inherited decisions

- Review only; no files edited.
- Vilboz is Raven2/Picasso-class DCN 1.x using AtomFirmware v2.
- The target is native visible display, not an option-ROM or payload fallback.
- The existing v2 implementation is intentionally diagnostics-only.

## Diagnosis

### Findings

1. **High — VBIOS lookup bypasses the platform mapping**
   - `src/drivers/amd/atombios/atombios_driver.c:1327-1329` constructs the CBFS name directly from `dev->vendor` and `dev->device`, selecting `pci1002,15d8.rom`.
   - `src/soc/amd/picasso/graphics.c:9-23` maps Picasso ID `1002:15d8` to Raven2 VBIOS ID `1002:15dd` when `soc_is_raven2()`.
   - `src/soc/amd/picasso/include/soc/cpu.h:12-15` confirms the Raven2 image header uses `1002:15dd`.
   - **Immediate fix:** make the custom loader consume `map_oprom_vendev()`—preferably through the existing PCI ROM lookup path—or construct its filename from the mapped 32-bit ID. Do not change the SoC mapping to `15d8`.

2. **Blocker — correcting the ROM name cannot produce display**
   - `src/drivers/amd/atombios/atombios_driver.c:4586-4600` parses v2 tables, emits diagnostics, and unconditionally returns.
   - `src/drivers/amd/atombios/Kconfig:22-29` explicitly states that v2 provides no DCE12/DCN modesetting.
   - `src/drivers/amd/atombios/atombios_driver.c:361-364` defines only AVIVO and Evergreen scanout layouts.
   - `src/drivers/amd/atombios/atombios_driver.c:1170-1173` dispatches only between those two legacy backends.
   - Thus the wrong ROM is a real mapping bug, but the root cause of no visible display is the complete absence of a DCN1 scanout backend.

3. **High — the diagnostics path is wired as if initialization succeeded**
   - `src/soc/amd/common/block/graphics/graphics.c:213-217` calls `amd_atombios_init()` and returns before `pci_dev_init()`.
   - With `vilboz_defconfig:5-7` enabling AtomBIOS, native VGA init, and no payload, there is deliberately no later display provider.
   - This is acceptable for diagnostics, but it must not be represented as native display support or framebuffer availability.

4. **High — broad v2 device registration exceeds implemented capability**
   - `src/drivers/amd/atombios/atombios_driver.c:5141-5187` registers DCN1 through DCN3 devices while every one receives only diagnostics.
   - Until generation-specific backends exist, support claims should remain explicitly diagnostic and must not silently displace a working initializer on unrelated boards.

## Drift / contradiction check

- The selected `15dd` image and the custom loader’s `15d8` image are not interchangeable lookup policies. The custom loader quietly replaced the established platform mapping with raw PCI identity.
- AtomFirmware parsing has advanced, but parsing topology and table revisions is not scanout implementation.
- Reusing the legacy Evergreen path for DCN1 would contradict the generation-aware split and write unrelated register layouts.

## Recommendation

1. **Land the mapping correction first as an isolated root fix.**
   - Use one mapped vendor/device value for every CBFS lookup.
   - Preserve the physical PCI ID for device classification; mapping is only for selecting the VBIOS.
   - Hardware validation must show both “Using RV2 VBIOS” and loading `pci1002,15dd.rom`.

2. **Keep expectations explicit: that patch only makes diagnostics trustworthy.**
   - It should not claim framebuffer or visible-display success.

3. **Shortest credible visible-display route: a narrow DCN1 Vilboz eDP backend.**
   - Scope it to Raven/Picasso DCN1, one pipe, linear XRGB8888, and the internal eDP path.
   - Reuse the v2 topology parser and port the required DCN1 register definitions/sequencing from Linux’s `dcn10` implementation.
   - Minimum functional sequence includes display power/clocks, AUX/EDID and DPCD, DP link training, OTG timing, HUBP/DPP/MPC/OPP setup, surface address/pitch/format, watermarks/aperture, unblank, panel power, and backlight.
   - Require valid MMIO and framebuffer BARs before entering this backend.
   - Defer HDMI, multiple pipes, arbitrary DCN generations, scaling, color management, and resume until the single eDP path displays reliably.

A captured GOP register replay or forcing v2 tables through the Evergreen backend is shorter code, but not a credible implementation: link training, clocks, framebuffer addresses, and panel variants are runtime-dependent.

## Risks

- Correct VBIOS selection may expose different table revisions or topology than current `15d8` diagnostics.
- DCN1 initialization depends on AGESA/SMU power and clock state; Linux sequencing must be reduced carefully rather than copied as isolated register writes.
- Vilboz may use multiple panel models, so a single hard-coded mode is suitable only as a bring-up checkpoint, not the final backend.
- The current v2 registration can suppress other initialization paths if enabled outside this controlled no-fallback configuration.

## Need from main agent

None. The immediate mapping fix and the separate DCN1 backend requirement are independently established.

## Suggested execution prompt

> Fix only the Raven2 VBIOS lookup mismatch: make `load_vbios()` use the platform-mapped vendor/device identity already provided by `map_oprom_vendev()`, preferably by reusing coreboot’s PCI ROM lookup helper. Keep the physical PCI ID for DCN classification. Add the smallest applicable check and validate that Vilboz logs load `pci1002,15dd.rom`. Do not claim visible display or add DCN scanout in this patch.