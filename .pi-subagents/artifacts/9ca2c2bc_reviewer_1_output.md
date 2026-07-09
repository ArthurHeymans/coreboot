## Review

- Correct:
  - `src/drivers/amd/atombios/atombios_driver.c:162-178` already maps AtomFirmware v2 command-table indices for `asic_init`, `setpixelclock`, `setcrtc_usingdtdtiming`, `enablecrtc`, `selectcrtc_source`, `digxencodercontrol`, and `dig1transmittercontrol`.
  - `atombios_driver.c:4057-4131`, `4140-4169`, and `4173-4178` already have callable paths for `SelectCRTC_Source`, DTD timing, and `EnableCRTC`.
  - Linux reference confirms the Raven/Picasso-relevant parameter writers:
    - `/tmp/atombios-v2-linux/command_table2.c:128-186`: `DIGxEncoderControl` v1.5 uses `dig_encoder_stream_setup_parameters_v1_5`.
    - `/tmp/atombios-v2-linux/command_table2.c:265-303`: `DIG1TransmitterControl` v1.6 uses `dig_transmitter_control_ps_allocation_v1_6`.
    - `/tmp/atombios-v2-linux/command_table2.c:480-553`: `SetPixelClock` v1.7 uses `set_pixel_clock_parameter_v1_7`.
    - `/tmp/atombios-v2-linux/command_table2.c:602-686`: DTD timing fills `set_crtc_using_dtd_timing_parameters`.
    - `/tmp/atombios-v2-linux/command_table2.c:691-732`: `EnableCRTC` is still a small `{ crtc_id, enable }` table call.

- Blocker:
  - `src/drivers/amd/atombios/atombios_driver.c:4024-4035` appears unsafe for AtomFirmware `DIG1TransmitterControl` v1.6: it writes `ucDigEncoderSel = (1 << path->dig_encoder)`. Linux v2 code writes `digfe_sel = cmd->dig_encoder_sel_to_atom(cntl->engine_id)` at `/tmp/atombios-v2-linux/command_table2.c:284`, which is an enum/index value, not a bitmask. For DIG0 this is likely `0`, while current coreboot sends `1`.

- Note:
  - Minimum safe AtomFirmware-v2 eDP sequence for the logged revisions:
    1. `ASIC_Init` v2.1 using `asic_init_ps_allocation_v2_1`.
    2. Parse object path, eDP connector, DIG engine, PHY, HPD, DPCD, panel timing.
    3. `SelectCRTC_Source` v2.3: `{ crtc_id, encoder_id/DIG, encode_mode = DP, dst_bpc }`.
    4. `SetPixelClock` v1.7: `{ pixclk_100hz, crtc_id, pll_id, encoderobjid, encoder_mode = DP, deep_color_ratio, misc flags }`.
    5. `SetCRTC_UsingDTDTiming` v1.1-compatible DTD struct: active, blanking, sync offsets/widths, polarity, borders, CRTC.
    6. `DIGxEncoderControl` v1.5 stream setup: DIG id, stream setup/action, pixel clock in 10 kHz, DP mode, lane count, bpc.
    7. `DIG1TransmitterControl` v1.6: PHY id, action, DP mode or lane-set, lane count, HPD select, DIG front-end select, connector object id, symbol clock.
    8. DP/eDP link training: DPCD link config, training patterns via `DIGxEncoderControl`, voltage/pre-emphasis via transmitter control, then `DP_VIDEO_ON`.
    9. `EnableCRTC` and unblank once DCN scanout path is valid.

- Note:
  - Still requires direct DCN register programming: framebuffer/surface address, pitch, format/swizzle, viewport, HUBP/HUBBUB, DPP, MPC/MPCC, OPP, OTG/TG enable/routing, pipe clocks/watermarks as needed. The Atom command tables cover firmware/link/clock helper calls; they do not replace a DCN display-pipe setup.