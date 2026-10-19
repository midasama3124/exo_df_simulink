# CLAUDE.md: exo_df Simulink project, shoe VN-100 acquisition (vn100_acq)

## Project context
- Third journal paper: phase-gated integral directional forgetting (IDF) adaptive controller on a unilateral (right) 2-DoF hip exoskeleton (sagittal + frontal), CubeMars AK10-9 motors over CAN (PCAN-USB) at 1 kHz.
- Run-time phase estimator: thigh-mounted VectorNav VN-100 on its own COM port, same Simulink model (not in `exo_df.slx` yet).
- Shoe-mounted VN-100 (right shoe, lateral heel counter or instep, x roughly anterior, on COM8 on the lab PC): **offline validation only** (ground-truth gait events / reference phase). Nothing in the controller or phase estimator may read it.
- Platform: Windows desktop, MATLAB/Simulink **R2018a**, **QUARC 2.7** (Quanser). Model: `exo_df.slx`, CAN through QUARC Peak CAN blocks (`quarc_library/Devices/Third-Party/Peak System/CAN Interfaces/Peak CAN`).
- **Run mode: QUARC External mode** (QUARC Windows 64-bit target `quarc_win64.tlc`, fixed-step discrete 0.001 s). QUARC documents Peak CAN as *not supported in normal simulation*, so normal mode cannot drive the motors and is not real time. Normal-mode runs are only a fallback for the IMU path itself (`p.logger = 'tofile'`, pacing on).
- Original spec: `prompt_foot_imu_acquisition.md` (it assumed normal mode). Full documentation of the implementation: `vn100_acq/README.md` (read it before changing the IMU code). QUARC docs: `C:\Program Files\Quanser\QUARC\quarc\doc\*.html`.

## Hard constraints (do not break)
- The 1 kHz step must never block or be delayed by the IMU path: non-blocking reads only, bounded work per call, fixed-size buffers, errors raised only in `mdlStart`, never at run time.
- `IMU_RAW` has no outports. Any monitor added later connects to scopes/displays only, never to the controller or phase estimator.
- R2018a compatibility: no `serialport` (use `serial`), no `matlab -batch`, no reliance on `string` features. No Statistics, Signal Processing or Instrument Control toolbox (use `vn_prctile`, `vn_lp_zerophase`, Java sockets). DSP System Toolbox only for the fallback UDP Receive block.
- C compilers supported by R2018a: MinGW-w64 4.9.2 or MSVC 2015/2017 (MSVC 2017 on the lab PC). Build MEX with `vn_build`; QUARC > Build compiles the `.c` sources found next to the MEX files into the real-time code.
- Code generation: S-function parameters must be numeric (no char parameters in generated code). The MEX `mdlStart` must not touch hardware unless `ssGetSimMode(S) == SS_SIMMODE_NORMAL`, because Simulink also runs it during QUARC builds and on the host side of External mode.
- **Safety:** the model drives motors. Never start `exo_df`, `vn_bench_test` run steps, `qc_start_model`, or anything that opens the COM ports or CAN without the user explicitly doing it. Hardware steps are the user's. Building a **scratch** model with only `IMU_RAW` (`qc_build_model`, never started, removed from the target afterwards) was used for offline testing.
- Do not edit `.slx` files directly. Change the model through scripts (`vn_add_imu_raw`, `set_param`), back up `exo_df.slx` first, and let the user save.

## Architecture (vn100_acq/)
Pipeline: `vn_configure` (once per IMU) → `IMU_RAW` in the model logs raw bytes → `vn_parse` → `vn_timestamps` → optional `vn_gait_events` / `vn_phase_reference`.

| File | Role |
|---|---|
| `vn_params_default.m` | all parameters; `[choice]` entries are implementer choices |
| `vn_configure.m` | baud scan, read regs 1–4, write 6, 76, 77, 75, 5; readback; rate check; `$VNWNV`; `verify_only` mode |
| `vn_check_ftdi_latency.m` | reads FTDI latency timer from the registry (never writes) |
| `vn_raw_rx.c` | Level-2 C S-function, Win32 non-blocking COM reader, Ts_rx = 2 ms, ≤ N_buf = 128 B per call; params `port number, baud, Ts_rx, N_buf`; 9 outputs (port 9 = bytes packed 4 per double) |
| `qpc_clock.c` | S-function, QueryPerformanceCounter [s] at 1 kHz |
| `qpc_now.c` | MEX, QPC for the fallback forwarder |
| `vn_build.m` | compiles the three C files (MEX) |
| `vn_add_imu_raw.m` | inserts `IMU_RAW` (RX + CLK subsystems); `p.logger 'quarc'` = QUARC To Host File (MAT v5), `'tofile'` = Simulink To File (Timeseries); `'replace'` option |
| `vn_set_run.m`, `vn_run_files.m`, `vn_rx_param_str.m`, `vn_model_name.m`, `vn_log_run.m`, `vn_port_number.m`, `vn_is_quarc.m` | log names `<subj>_<run>_*.mat` (QUARC: `%{vn_log_run}` in the To Host File name, no rebuild), block params, RX on/off, `files.rebuild` flag; model name or path accepted |
| `vn_bench_test.m` | M1 acceptance test: runs A/B (QUARC: build, `qc_start_model`, wait, `qc_stop_model`), analysis, power-cycle verify |
| `vn_parse.m`, `vn_scan_packets.m`, `vn_decode_packets.m`, `vn_crc16.m`, `vn_make_layout.m`, `vn_load_tofile.m`, `vn_unpack_bytes.m` | offline decoding |
| `vn_timestamps.m` | lower-envelope fit sensor time → QPC → t_sim |
| `vn_gait_events.m`, `vn_phase_reference.m`, `vn_lp_zerophase.m`, `vn_prctile.m` | optional M3 |
| `vn_selftest.m` | hardware-free end-to-end test of M2/M3 on a synthetic stream, in both log formats |
| `vn_udp_forwarder.m` | fallback path (second MATLAB, serial → UDP 127.0.0.1:25000) |

## Data formats
- Logs per run in `p.log_dir` (default `C:\exo_logs`, local, not OneDrive), complete only after the model stops:
  - QUARC (To Host File, MAT-file v.5, one variable `[t_sim; data]`, one column per sample): `<subj>_<run>_imu_bytes.mat` (var `imu_bytes_p4`, 32 doubles per call = 128 bytes packed little endian `b0 + 256 b1 + 65536 b2 + 16777216 b3`), `<subj>_<run>_imu_diag.mat` (var `imu_diag`, 7 rows), `<subj>_<run>_clk.mat` (var `clk`, QPC at 1 kHz). About 180 kB/s.
  - To File (normal simulation): same names, Timeseries, var `imu_bytes` uint8 × 128. About 116 kB/s.
  - `vn_load_tofile` reads both; `vn_parse` unpacks `*_p4` variables and counts calls missing from the log (`pk.diag.log_dropped_calls`).
- `imu_diag` columns: 1 bytes read, 2 QPC after read [s], 3 bytes still queued, 4 cumulative bytes, 5 cumulative error calls, 6 this call's error mask (CE_* bits; 0x40000000 ClearCommError failed, 0x80000000 ReadFile failed), 7 call duration [µs].
- Packet (`base`, mask 0x0131): `0xFA`, `0x01`, mask LE, TimeStartup uint64 ns @4, quaternion [x y z w] float32 @12 (scalar last), gyro rad/s @28, accel m/s² @40, CRC-16 MSB first @52; 54 B, 400 Hz = 21.6 kB/s. CRC = CRC-16-CCITT poly 0x1021 init 0 (XMODEM); over group byte..CRC gives 0.
- Register 75 written as `$VNWRG,75,<AsyncMode>,<800/400=2>,01,0131`; AsyncMode auto-detected (port 1 or 2) and saved in `vn_unit_info.mat`.

## Commands
- MATLAB from a shell (R2018a has no `-batch`). Use a short output path for `vn_selftest`: `-v7.3` saves fail on very long paths.
  `matlab -nosplash -nodesktop -wait -logfile out.log -r "try, cd('vn100_acq'); vn_selftest; catch e, disp(getReport(e)); end; exit"`
- QUARC from MATLAB: `qc_set_options(mdl)` (QUARC defaults), `qc_build_model`, `qc_start_model`, `qc_stop_model`, `qc_get_simulation_status` (`'external'` while connected), `qc_get_downloaded_models('shmem://quarc-target:1')`, `qc_remove_downloaded_model(uri, name)`.
- Session order (user runs hardware steps): `vn_build` → `vn_configure('COM8')` → QUARC > Set default options on `exo_df` → `vn_add_imu_raw('exo_df', p, 'replace')` + save → `vn_bench_test('exo_df', 'all', p)` → per run `vn_set_run('exo_df', subj, run, p)` (+ `qc_build_model` if `rebuild`) and `qc_start_model` → offline `vn_parse`, `vn_timestamps`, `vn_gait_events`.

## Status (as of 2026-10-19)
- Done by the user: `vn_build`, `vn_configure` (COM8), `vn_add_imu_raw` with the original To File layout, model saved.
- First bench run A (normal mode, To File): the model had pacing off, a variable-step solver, StopTime 10 and target `grt.tlc`, so 600 s of simulation ran in 2.16 s of wall time. IMU path itself fine (0 errors, ~22.6 kB/s). Led to the QUARC change.
- QUARC adaptation implemented and tested offline on the lab PC: `vn_selftest` PASS (first MATLAB run of M2/M3; both log formats; missing-call count); `vn_raw_rx.c` MEX compiles; C packing = `vn_unpack_bytes`; `IMU_RAW` (quarc logger) in a scratch model QUARC-builds and downloads (never started, removed from target). That first build exposed the MEX `mdlStart` opening COM7 during code generation, now guarded.
- Backups before the change: `code/exo_df_simulink_backup_20261019/` (model `exo_df_before_quarc_imu.slx` + all IMU sources).
- Done by the user: `vn_build`, QUARC options on `exo_df`, `IMU_RAW` replaced, **bench run A under QUARC (2026-10-19 ~16:52)**: 599.97 s wall for 600 s, S-function errors 0, queued max 0, call mean 6.2 µs / P99 14.6 µs / max 123 µs (9 calls > 100 µs: fails the strict max criterion), 43.20 B per call, drift −2.65 ppm, rate 400.0011 Hz (slot count), quaternion norm 1, static |gyro| 0.0013 rad/s, static |acc| 9.51 m/s² (check flat on a table: expect ~9.81).
- Run A log holes: `ExtModeTrigDuration` = 10000 → one 2 ms call lost at most 10 s buffer boundaries (53 holes, all sensor gaps/CRC failures come from them). Fix: duration longer than the longest run, then re-run A.
- 1 kHz loop under QUARC (run A): step mean 1.000 ms, P99 1.50 ms, P99.9 1.78 ms, max 2.28 ms, 67 steps > 2 ms, pacing lag max 0.97 ms (no drift). Compare with run B.
- Not yet done (user): re-run A with the longer trigger duration, run B, analyze, power-cycle verify.

## Open verification items
- Binary layout (group-1 bits/sizes, register 75–77 format) was checked only against UM001 rev 1.2.8 (firmware 1.1, which predates binary output) and public drivers. Confirm against the firmware-2 manual/ICD of the unit; fill `p.manual_version`.
- Record the FTDI latency timer (must be 1 ms) and the firmware printed by `vn_configure`.
- QUARC run-time behaviour not yet seen: COM port opened by the real-time code, To Host File writing the logs to `C:\exo_logs` with the substituted run name, no missing calls over 600 s, whether a changed To Host File name needs a rebuild (flag errs on the safe side).
- Spec gaps to resolve with the user: no gait-event method section and no "Sec. 10.1" for `hs_bias`/`to_bias` calibration (biases default 0); section cross-references in the spec are off (see README "Issues found in the prompt").

## Pending changes (agreed, not yet implemented)
1. ~~Log location~~: done (`p.log_dir = 'C:\exo_logs'` default).
2. **Scope the existing diagnostics** (vn_raw_rx ports 2–8) in External mode: bytes per call ≈ 43, cumulative bytes slope ≈ 21.6 kB/s, error count flat at 0, queued < 128. Shows bytes flow, not packet validity. Scopes add External-mode upload traffic: decimate.
3. **Online health monitor `vn_rx_monitor.c`** (Level-2 C S-function, inside `IMU_RAW`; numeric params only, must build under QUARC):
   - inputs: port 1 bytes (uint8[N_buf]) and port 2 count from `vn_raw_rx`, at Ts_rx;
   - fixed ring buffer (e.g. 256 B), header + CRC check, at most ~3 packets per call, bounded work;
   - outputs: latest gyro (3) and accel (3), packet count, packets/s over the last 1 s (expect 400), CRC-failure count, sensor-time gap count (dt > 1.5/400 s);
   - connects only to scopes/displays; scope decimated to ~100 Hz with few channels; plot the gyro axis that runs side to side through the foot (pitch axis): one large spike per swing;
   - the logged raw bytes and offline `vn_parse` remain the record of truth.
4. **Post-trial check script** (e.g. `vn_quick_check(subj, run, p)`): after each run stops, run `vn_parse` + `vn_timestamps` and print one pass/fail line (CRC failures, gaps, duplicates, resets, S-function errors, calls missing from the log, packet rate within 0.1 % of 400 Hz, drift, max queued, max call duration). Use between protocol blocks so bad trials are repeated before the session ends.
5. **Re-run bench run A** with the monitor and an open scope included, to confirm the 1 kHz timing criteria still pass (A vs B within 5 %).

## Conventions
- Parameters live in `vn_params_default.m`; mark new non-specified choices `[choice]` and list them in README "Implementer choices".
- Keep `vn100_acq/README.md` and `vn100_acq/CLAUDE.md` in sync with any change (wiring/index maps, parameter table, file list).
- Test offline changes with `vn_selftest` before anything touches hardware. For S-function changes, also QUARC-build a scratch model containing only `IMU_RAW` (never start it).
