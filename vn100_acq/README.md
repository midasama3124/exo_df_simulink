# Shoe-mounted VN-100: raw acquisition in Simulink R2018a, timestamps and gait events

The shoe IMU is for **offline validation only**. `IMU_RAW` has no outports, and nothing in the controller or the phase estimator reads it. The 1 kHz step never waits on the IMU: one non-blocking `ReadFile` of at most `N_buf` bytes every `Ts_rx`, fixed-size buffers, errors raised only at start.

**Run mode: QUARC External mode.** `exo_df` drives the motors through QUARC Peak CAN blocks, and QUARC documents those as *not supported in normal simulation*. Real time therefore comes from QUARC: the model is built into real-time code for the QUARC Windows 64-bit target and Simulink connects to it in External mode. `vn_raw_rx.c` and `qpc_clock.c` are compiled into that code, and `IMU_RAW` logs through QUARC **To Host File** blocks, which write MAT-files on the host. The original normal-simulation layout (Simulink To File) is still available as `p.logger = 'tofile'`.

## Files

| Milestone | File | Purpose |
|---|---|---|
| M1 | `vn_params_default.m` | every parameter (table below) |
| M1 | `vn_configure.m` | one-time VN-100 setup and `verify_only` readback |
| M1 | `vn_check_ftdi_latency.m` | reads (never writes) the FTDI latency timer from the registry |
| M1 | `vn_raw_rx.c` | Level-2 C S-function, non-blocking COM reader (Win32 API) |
| M1 | `qpc_clock.c` | Level-2 C S-function, QPC seconds at 1 kHz |
| M1 | `vn_build.m` | compiles the two S-functions and `qpc_now` |
| M1 | `vn_add_imu_raw.m` | inserts the `IMU_RAW` subsystem into your model |
| M1 | `vn_set_run.m`, `vn_run_files.m`, `vn_rx_param_str.m`, `vn_model_name.m` | log names `<subj>_<run>_*.mat`, block parameters, RX on/off, rebuild flag |
| M1 | `vn_log_run.m` | current `<subj>_<run>`, read by the To Host File names through `%{vn_log_run}` |
| M1 | `vn_port_number.m`, `vn_is_quarc.m` | `'COM7'` → 7; QUARC target check |
| M1 | `vn_bench_test.m` | acceptance test (runs A and B, analysis, power-cycle check) |
| M2 | `vn_parse.m` | byte stream → CRC-checked packets, receipt times, diagnostics |
| M2 | `vn_timestamps.m` | lower-envelope fit sensor time → QPC → `t_sim` |
| M2 | `vn_selftest.m` | hardware-free test of M2 (and M3) on a synthetic stream |
| M3 (optional) | `vn_gait_events.m`, `vn_phase_reference.m` | HS/TO from the shoe gyro, reference phase |
| shared | `vn_make_layout.m`, `vn_scan_packets.m`, `vn_decode_packets.m`, `vn_crc16.m`, `vn_load_tofile.m`, `vn_unpack_bytes.m`, `vn_prctile.m`, `vn_lp_zerophase.m` | helpers (no toolboxes needed) |
| fallback | `vn_udp_forwarder.m`, `qpc_now.c` | second-MATLAB serial→UDP path |

## Setup checklist

- [ ] **Firmware.** `vn_configure` prints registers 1–4. Binary outputs (registers 75–77) need VN-100 firmware 2.x or later; on older firmware the write of register 75 returns VN error 8 and the script says so.
- [ ] **Manual.** Check the field table below against the VN-100 manual/ICD for the printed firmware and write its revision into `p.manual_version` (it is saved in `vn_unit_info.mat`). See "Verification status" for what was checked and what still needs checking.
- [ ] **USB adapter latency.** If the dev-kit cable is FTDI-based, set Device Manager → Ports → COMx → Properties → Port Settings → Advanced → Latency Timer = **1 ms** (Windows default 16 ms), then unplug/replug. `vn_configure` reads the value from the registry and warns if it is not 1 ms. Value found: ____ ms.
- [ ] **Compiler.** `mex -setup C` with MinGW-w64 4.9.2 or MSVC 2015/2017, then `vn_build`. QUARC builds with its own compiler setting (MSVC on this PC).
- [ ] **COM port.** Note it in Device Manager and set `p.port` (`'COM8'` or `8`). The thigh VN-100 of the phase estimator is on a different port. The port number is compiled into the real-time code: rebuild after changing it.
- [ ] **Path.** Keep this folder (with the compiled `.mexw64` and the `.c` sources) on the MATLAB path; QUARC > Build compiles the `.c` files found next to the MEX files.
- [ ] **QUARC model settings.** QUARC > Set default options (or `qc_set_options('exo_df')`): QUARC Windows 64-bit target, External mode, Fixed-step 0.001 s. Use the discrete fixed-step solver. QUARC sets StopTime to `inf`.
- [ ] **Log folder.** `p.log_dir` = a local, non-synced folder (default `C:\exo_logs`, created by `vn_set_run`). Not OneDrive: sync activity on growing log files can add timing jitter. Copy the files afterwards.
- [ ] **External mode signals.** In the External Mode Control Panel → Signal & Triggering, the three `ToFile_*` To Host File blocks must be selected (they are by default), with source manual and "Arm when connecting" on.
- [ ] **Mounting.** Right shoe, lateral heel counter or instep, x-axis roughly anterior, strapped rigidly.

## How to run

```matlab
% once per PC
vn_build                                   % compiles vn_raw_rx, qpc_clock, qpc_now (MEX)
p = vn_params_default();  p.port = 'COM8';  p.log_dir = 'C:\exo_logs';

% once per IMU (model stopped), and whenever the configuration changes
info = vn_configure(p.port);               % or vn_configure(p.port, struct('fields','base+ypr'))

% once per model (QUARC target set first: QUARC > Set default options)
vn_add_imu_raw('exo_df', p);               % 'replace' as 3rd argument swaps an older IMU_RAW
save_system('exo_df');

% M1 acceptance (required before subjects). Builds and runs the real-time
% code, which drives the CAN bus: make the motors safe first.
R = vn_bench_test('exo_df', 'all', p);     % or steps 'A', 'B', 'analyze', 'powercycle'

% every session run: same <subj>_<run> as the phase-estimator logs
f = vn_set_run('exo_df', 'S03', 'EF_blk2', p);   % run name only: no rebuild
if f.rebuild, qc_build_model('exo_df'); end      % port/baud/Ts/N_buf or RX on/off changed
qc_start_model('exo_df');                        % or QUARC > Start; stop with QUARC > Stop

% offline (M2, M3)
f  = vn_run_files('S03', 'EF_blk2', p);
pk = vn_parse(f.bytes, f.diag);            % layout from vn_layout.mat in p.cfg_dir
ts = vn_timestamps(pk, f.clk);
ev = vn_gait_events(pk, ts);               % optional
phi_ref = vn_phase_reference(ev, t_sim_of_phase_estimator_log);

vn_selftest                                % no hardware: checks M2/M3 end to end
```

## Wiring and index maps

`vn_add_imu_raw` builds this at the model root (positions are tidy enough to edit by hand):

`p.logger = 'quarc'` (default, QUARC External mode):

```
<mdl>/IMU_RAW                              (no inports, no outports)
  RX   vn_raw_rx  Ts_rx ── 1 uint8[N_buf] ──► Terminator
                         ── 2..8 ─► Data Type Conversion (double) ─► Mux(7) ─► ToFile_diag   To Host File  <log_dir>\%{vn_log_run}_imu_diag.mat   variable imu_diag
                         ── 9 double[N_buf/4] (bytes packed 4 per double) ──► ToFile_bytes  To Host File  <log_dir>\%{vn_log_run}_imu_bytes.mat  variable imu_bytes_p4
  CLK  qpc_clock  1 kHz ── 1 double ─────────────────────────────────────────► ToFile_clk    To Host File  <log_dir>\%{vn_log_run}_clk.mat        variable clk
```

To Host File writes MAT-file v.5 on the host when the model stops: one variable `[t_sim; data]`, one column per sample. It stores everything as double, so the bytes come from port 9, packed 4 per double (`b0 + 256 b1 + 65536 b2 + 16777216 b3`, exact) and restored by `vn_unpack_bytes`. QUARC evaluates `%{vn_log_run}` in the host MATLAB each time the model starts. `vn_set_run` sets that name, so changing the run needs no rebuild.

`p.logger = 'tofile'` (normal simulation only): port 1 → To File `imu_bytes` (Timeseries format, since Array format cannot store uint8), ports 2..8 → To File `imu_diag`, `qpc_clock` → To File `clk`, file names written directly; port 9 terminated.

RX is its own subsystem so bench run B can comment it out while `qpc_clock` keeps logging (a rebuild under QUARC).

`vn_raw_rx` parameters: `port number, baud, Ts_rx, N_buf` (numeric, non-tunable: changing one needs a rebuild). Generated code does not support char S-function parameters, so `vn_rx_param_str` passes `vn_port_number(p.port)`. Outputs and the `imu_diag` columns:

| S-fcn port | `imu_diag` column | Type | Content |
|---|---|---|---|
| 1 | (imu_bytes) | uint8 [N_buf] | bytes read this call, tail zero-padded |
| 2 | 1 | uint16 | bytes read this call |
| 3 | 2 | double | QPC after the read [s] (receipt time) |
| 4 | 3 | uint32 | bytes still queued after the read (snapshot − n) |
| 5 | 4 | uint32 | cumulative bytes read |
| 6 | 5 | uint32 | cumulative calls with a nonzero error mask |
| 7 | 6 | uint32 | this call's error mask: CE_RXOVER 0x1, CE_OVERRUN 0x2, CE_RXPARITY 0x4, CE_FRAME 0x8, CE_BREAK 0x10; 0x40000000 ClearCommError failed; 0x80000000 ReadFile failed |
| 8 | 7 | double | call duration t1 − t0 [µs] |
| 9 | (imu_bytes_p4) | double [ceil(N_buf/4)] | port-1 bytes packed 4 per double, little endian |

`mdlOutputs` order: QPC t0 → `ClearCommError` → `ReadFile(min(cbInQue, N_buf))` → QPC t1 → zero-pad → pack port 9 (32 iterations). Port settings: `CreateFileA("\\.\COMx")`, `SetupComm(65536, 4096)`, 8N1, binary, no flow control, DTR/RTS on, `fAbortOnError = FALSE`, `COMMTIMEOUTS {MAXDWORD, 0, 0}`, `PurgeComm(RXCLEAR|RXABORT)`. The MEX file opens the port only in a normal simulation (`ssGetSimMode == SS_SIMMODE_NORMAL`). Simulink also runs the MEX `mdlStart` during code generation (QUARC > Build) and on the host side of External mode, and the port must not be opened there. The real-time code always opens it.

Expected log rate with the defaults: QUARC: imu_bytes 500 × 33 × 8 B, imu_diag 500 × 8 × 8 B, clk 1000 × 2 × 8 B ≈ 180 kB/s, about 270 MB for 25 min. To File: ≈ 116 kB/s. `vn_bench_test` measures it.

## Packet layout (binary output 1, common group)

Packet = `0xFA`, group `0x01`, field mask (uint16 LE), payload in ascending field-bit order (little-endian), CRC-16 (2 bytes, **MSB first**). CRC = CRC-16-CCITT, polynomial 0x1021, initial 0 (the XMODEM variant); run over the group byte through the two CRC bytes it gives 0 for a valid packet.

| Bit | Field | Type | Bytes | Offset (from 0xFA) | Units |
|---|---|---|---|---|---|
| 0 | TimeStartup | uint64 | 8 | 4 | ns since start-up |
| 3 | YawPitchRoll (`+ypr`) | 3 × float32 | 12 | (inserted after TimeStartup when present) | deg |
| 4 | Quaternion | 4 × float32 | 16 | 12 | **[x y z w], scalar last**; `pk.quat_wxyz` reorders |
| 5 | AngularRate (compensated) | 3 × float32 | 12 | 28 | rad/s |
| 8 | Accel (compensated) | 3 × float32 | 12 | 40 | m/s² |
| 10 | MagPres (`+mag`) | 5 × float32 | 20 | (after Accel) | G, G, G, °C, kPa |
| – | CRC | uint16 BE | 2 | 52 | – |

`base`: mask 0x0131, payload 48 B, packet 54 B; 400 Hz → 21.6 kB/s, 23 % of 921600 baud. `+ypr` 0x0139 (66 B), `+ypr+mag` 0x0539 (86 B). `vn_make_layout` computes offsets for any mask; `vn_configure` saves the result as `vn_layout.mat`.

Register writes used: `$VNWRG,06,0`; `$VNWRG,76,0,1,01,0001` and the same for 77 (AsyncMode 0 = off); `$VNWRG,75,<AsyncMode>,<imu_rate/rate>,01,<mask hex>`; `$VNWRG,05,921600`; `$VNWNV`. All commands carry the 8-bit XOR checksum of the characters between `$` and `*`.

### Verification status

Checked against VN-100 User Manual UM001 rev 1.2.8 (firmware v1.1): 8-bit XOR checksum and the CRC routine (the table-driven `vn_crc16` reproduces the manual's bitwise routine on random data; checksums of the manual's example commands reproduce), quaternion order (scalar last), register numbers and formats 1–7, the reply to a baud change being sent at the old baud, `$VNWNV` taking about 0.5 s, the error codes, and the Rugged unit having two UARTs (RS-232 port 1, TTL port 2). That manual predates the binary protocol, so **bits/sizes of the common group and the register 75–77 format still need checking against the firmware-2 manual/ICD of your unit**. Two run-time checks back this up: `vn_configure` reads registers back, and measures the packet rate from TimeStartup (error if it is not 400 Hz ± 1 %, with the implied IMU rate printed).

## Parameters (`vn_params_default.m`)

| Name | Default | Notes |
|---|---|---|
| port | 'COM7' | set per PC (`'COM8'` or `8`); vn_raw_rx gets the number |
| log_dir | 'C:\exo_logs' | local, non-synced; absolute (To Host File) |
| logger | 'quarc' | 'quarc' = To Host File (External mode), 'tofile' = To File (normal simulation) |
| baud | 921600 | written to register 5 |
| rate | 400 Hz | RateDivisor = imu_rate / rate |
| imu_rate | 800 Hz | nominal; checked by the rate measurement |
| fields | 'base' (0x0131) | 'base+ypr' (0x0139), 'base+ypr+mag' (0x0539) |
| async_mode | 'auto' | detect, or 1, 2, 3 |
| Ts_rx | 0.002 s | 0.001–0.01 |
| N_buf | 128 B | rule: > 3 × bytes per Ts_rx at nominal rate (see issue 6) |
| driver buffer | 65536 B | `VNRX_DRIVER_RXBUF` in `vn_raw_rx.c` (SetupComm) |
| Ts_clk | 0.001 s | qpc_clock |
| f_lp_events | 15 Hz | zero-phase |
| ms_thr_frac | 0.5 | × median mid-swing peak |
| to_win, hs_win | 0.4, 0.3 (× T) | search windows |
| acc_ref, acc_ref_win, acc_ref_thr | false, ±0.040 s, 1.5 g | optional HS refinement |
| hs_bias, to_bias | 0, 0 | calibration offsets (see spec issue 4) |
| gap_thr | 1.5 / rate s | |
| seg_gap | 1 s | |
| env_pct, env_bin | 5th percentile, 2 s | lower-envelope fit |
| env_tol | 1 µs | convergence |

Bench-test limits (`bench_*`, including the QUARC waits `bench_start_timeout` 120 s, `bench_stop_margin` 60 s, `bench_file_timeout` 60 s), fallback settings (`udp_*`, `fwd_*`) and the choices below are in the same file.

## Implementer choices

1. **AsyncMode detection** (`async_mode = 'auto'`). The prompt asks to verify which VN serial port the USB link uses. `vn_configure` enables binary output 1 at 10 Hz (divisor 80) on port 1, listens 0.6 s for CRC-valid packets, then tries port 2, and saves the result. Set `async_mode` to 1, 2 or 3 to skip.
2. **Write order when the found baud is too slow.** The VN-100 factory baud is 115200, which cannot carry 21.6 kB/s (VN error 12, "insufficient baud rate"). If the final stream would use more than 80 % of the found baud, register 5 is written before the final register 75 (order 6, 76, 77, 75 at 10 Hz, 5, 75). Otherwise register 5 is last, as specified. `info.write_order` records which.
3. **Registers 76/77 off** are written as `0,1,01,0001`: AsyncMode 0 disables the output; the other fields are just valid values. Only AsyncMode is compared on readback.
4. **Scan retries.** Two `$VNRRG,01` attempts per baud (200 ms each), because the first command after a host baud change is sometimes lost.
5. **Rate check.** After configuring (and in `verify_only`), 3 s of packets are parsed; the median TimeStartup step must give 400 Hz ± 1 % with no CRC failures.
6. **Replies** are matched by command and register number with either checksum type (2-hex XOR or 4-hex CRC16), so interleaved binary packets do not confuse the configuration script.
7. **Port 4 (queued)** is the `cbInQue` snapshot minus the bytes read, not a second `ClearCommError` call (keeps the call short).
8. **Port 7** carries the mask of *this* call (0 if none). It is logged every call, so a "last nonzero mask" can be derived offline. Bits 30/31 flag failed `ClearCommError`/`ReadFile`.
9. **Port 1** is a 1-D signal of width N_buf; `vn_load_tofile` accepts either orientation.
10. **Duplicates** are counted as 0 ≤ Δt_sensor < 0.5/rate.
11. **CRC failures** are counted at header matches only; a packet whose header is damaged shows up as skipped bytes plus a gap.
12. **Segments** with fewer than 100 packets are not fitted (times NaN). Envelope iterations are capped at 50. Packets more than 50 ms outside the qpc_clock range get `t_imu_sim = NaN`.
13. **Bench test** under QUARC sets StopTime = `bench_T`, builds (`qc_build_model`), starts (`qc_start_model`), waits while `qc_get_simulation_status` is `'external'`, calls `qc_stop_model` after `bench_T + bench_stop_margin` or on Ctrl+C, then waits for new, stable To Host File logs. In normal simulation it uses `SimulationCommand 'start'` and refuses to run with pacing off. It beeps at 60 s, excludes the first 2 s from the timing statistics, and offers `bench_timing_abs` (default 0 = strict 5 %) because run-to-run P99.9 on Windows can differ by more than 5 % by chance.
14. **Toolbox-free.** Percentiles (`vn_prctile`, same definition as `prctile`), zero-phase filter (`vn_lp_zerophase`, 2nd-order Butterworth forward-backward), peak finding and UDP (Java sockets) avoid the Statistics, Signal Processing and Instrument Control toolboxes.
15. **Gait events** (M3): sagittal axis = principal axis of the gyro (sign: mid-swing peak positive); mid-swing peaks must also exceed 1 rad/s (rejects standing); minimum spacing 0.6 s before T is known, then 0.5 T; parabolic sub-sample refinement; an event whose minimum sits on the window edge, or whose stride time is outside 0.5–1.5 × median, is marked invalid. Event times are sensor sample times mapped through the segment fit, so they carry the same (uncorrected) minimum transport latency as `t_imu_sim`.
16. **Files** `vn_unit_info.mat` and `vn_layout.mat` are saved next to the code (`p.cfg_dir`). `verify_only` saves `vn_unit_info_verify_<date>.mat` and never overwrites the configuration record.
17. **QUARC logging** (`logger = 'quarc'`): QUARC To Host File, MAT-file v.5, chosen over Simulink To File, which under QUARC writes on the target side and cannot take a run name without a rebuild. The bytes are packed 4 per double on a new port 9 instead of logging uint8 as double (8× larger) or using raw binary files (no time row, so drops could not be seen).
18. **Run names** come from `%{vn_log_run}` in the To Host File names. `vn_log_run` keeps the value in memory and in `vn_current_run.mat` (survives a MATLAB restart). If none is set it returns `unset_<time>`, so an earlier log is never overwritten.
19. **Rebuild flag.** `vn_set_run` returns `rebuild = true` when an S-function parameter, RX on/off or a To Host File name string changed on a QUARC model. Whether a changed To Host File name (e.g. a new `log_dir`) really needs a rebuild was not checked; the flag errs on the safe side.
20. **Missing calls.** `vn_parse` counts calls missing from the log from holes in its time row (`log_dropped_calls`), and `vn_bench_test` requires 0 for RX and for `qpc_clock`, plus a wall-clock run length ≥ 90 % of `bench_T` (catches an unpaced run).
21. **MEX port guard.** The MEX `mdlStart` opens the COM port only when `ssGetSimMode == SS_SIMMODE_NORMAL`. Without the guard a QUARC build tried to open the port.
22. **Default `log_dir`** = `C:\exo_logs` (was the current folder).
23. **Bench packet rate** counts sample slots (`round(Δt_sensor × rate)`), not received packets, so packets lost in the log do not show up as a slow sensor. Run A: 400.0000 Hz sensor clock, 400.0011 Hz PC clock.
24. **External-mode trigger duration.** To Host File data passes through Simulink External mode, which collects buffers of `ExtModeTrigDuration` base-rate steps and restarts at each boundary. With 10000 (10 s), run A lost one 2 ms `vn_raw_rx` sample at most boundaries: 53 holes at t = 10, 20, …, 530 s, 1787 bytes, 51 packets, all 50 sensor gaps and 16 CRC failures. The S-function read every byte (QPC and cumulative bytes are continuous). Set the duration longer than the longest run (External Mode Control Panel → Signal & Triggering → Duration, or `set_param(mdl, 'ExtModeTrigDuration', ...)`).

## Issues found in the prompt

1. Sec. 1 says "milestone order of Sec. 14"; the milestones are in Sec. 11.
2. Sec. 5 says "only if Sec. 5 fails its acceptance test in Sec. 7"; read as "if Sec. 4 fails the acceptance test of Sec. 6".
3. Sec. 6 refers to parsing "(Sec. 8)"; the parser is Sec. 7.
4. The title and parameter table cover gait events (`f_lp`, `ms_thr`, `to_win`, `hs_win`, `acc_ref_*`, biases "from Sec. 10.1"), but there is no gait-event section and no Sec. 10.1. I implemented the method the parameters imply as an optional M3 (choice 15) with zero biases; please supply the calibration procedure for `hs_bias`/`to_bias` if one is planned.
5. "Register 5 written last" conflicts with VN error 12 when the unit starts at 115200 (choice 2).
6. The default N_buf = 128 is 2.96 × the nominal 43.2 B per 2 ms, slightly below the "> 3 ×" rule in the parameter table. The drain margin is still ample (64 kB/s capacity vs 21.6 kB/s); I kept 128. Use 136 or more if you want the rule met literally.

## Fallback path (only if the S-function path fails the bench test)

1. Second MATLAB instance: `vn_udp_forwarder('COM7')` (needs `qpc_now`, built by `vn_build`).
2. In the model, replace the contents of `IMU_RAW/RX` with a non-blocking **UDP Receive** block (DSP System Toolbox or Instrument Control Toolbox): local port 25000, data type uint8, maximum message length 1034, sample time Ts_rx, blocking off. Log its data output to a To File block named `ToFile_bytes` (variable `imu_bytes`) and its length output, converted to double, to `ToFile_diag` (variable `imu_diag`). Keep `IMU_RAW/CLK`. With these names `vn_set_run` still sets the file names.
3. `vn_parse` detects the format (one diag channel) and takes the receipt time from the datagram (`p.source = 'udp'` forces it).

## Tested here vs. on the PC

On the lab PC (2026-10-19, MATLAB R2018a, MSVC 2017, QUARC 2.7.2522), without hardware:
- `vn_selftest` PASS. This was the first MATLAB run of the M2/M3 code. It now also checks the QUARC log format (identical packets and timestamps) and the missing-call count (3/3).
- `vn_raw_rx.c` compiles as MEX. The C packing loop matches `vn_unpack_bytes` on 200 random buffers.
- `vn_add_imu_raw` (quarc logger) builds into a scratch model with the QUARC Windows 64-bit target, diagram update passes, and `qc_build_model` compiles `vn_raw_rx.c`/`qpc_clock.c` into the real-time code and downloads it. The scratch model was removed from the target afterwards and never started.
- `vn_set_run` rebuild flag: 0 for a run-name change, 1 for a port change or RX off. `%{vn_log_run}` substitutes to the full path.

**Not tested yet:** a real External-mode run (COM port opened by the real-time code, To Host File writing the logs), and the bench test. A first normal-mode run A had pacing off: 600 s of simulation in 2.2 s of wall time. The CAN blocks do not run in normal simulation, which led to the QUARC change.

Earlier, in claude.ai: the C files compile cleanly (`gcc -Wall -Wextra -Wconversion`) against stub Windows/Simulink headers; the CRC table, XOR checksums and the parse → envelope → gait-event algorithms were checked on a synthetic stream with corruptions, bursts, split deliveries and pacing hiccups (Python mirror of the MATLAB code: all packets recovered, 3/3 CRC failures counted, drift 39.96 ppm for 40 ppm true, HS error < 1 ms). **Not tested:** the MATLAB files themselves (no MATLAB here), the Win32 calls on real hardware, and Simulink block insertion. Run `vn_selftest` first (it exercises M2/M3 in MATLAB without hardware), then `vn_build`, `vn_configure`, `vn_add_imu_raw` and the bench test.
