# Implementation prompt 2: shoe-mounted VN-100, raw acquisition in Simulink R2018a, timestamps and gait events

## 1. Task

Implement and test:

1. A configuration script for one VectorNav VN-100 on the right shoe.
2. Raw-byte acquisition inside the existing exoskeleton Simulink model, writing to the same set of log files.

The IMU is for offline validation only. Nothing in the run-time controller or phase estimator may read it.

General rules:
- Deliver code, not design reports.
- Where a detail is not specified, choose it, put it in the parameter file and list it in the README under "Implementer choices".
- Deliver in the milestone order of Sec. 14. Each milestone must run on its own.

## 2. System and constraints

- **PC and model.** Windows desktop PC running MATLAB/Simulink R2018a, normal mode with real-time pacing, fixed-step discrete solver, base rate 1 kHz. The motors (CubeMars AK10-9) will run over CAN through PCAN-USB in the same model. A separate implementation (the phase estimator) will also run in this model.
- **IMU.** One VN-100 on the right shoe, rigidly strapped on the lateral heel counter or the instep, with its x-axis roughly anterior. It connects through the USB development-kit cable on its own COM port.
- **The 1 kHz step must never block or be delayed by the IMU path.**
  - All reads are non-blocking.
  - Work per call is bounded.
  - All buffers have fixed size.
- **USB adapter.** If the dev-kit adapter is FTDI-based, set its latency timer to 1 ms. The path is Device Manager → Port Settings → Advanced; the Windows default is 16 ms. Record the setting in the README checklist.

## 3. Configuration script: `vn_configure.m` (run once before a session, with Simulink closed)

`vn_configure(port, opts)`, where `opts.fields` is `'base'` (default), `'base+ypr'` or `'base+ypr+mag'`.

1. **Find the current baud.** Open the port and try 115200, 921600, 460800, 230400, 128000 and 57600 in turn. At each, send `$VNRRG,01*XX` and wait up to 200 ms for a `$VNRRG,01,` reply.
2. **Identify the unit.** Read and print registers 1 (model), 2 (hardware revision), 3 (serial number) and 4 (firmware). Save them to `vn_unit_info.mat`.
3. **Check the manual.** Verify every register number, field bit, field size, byte order and the CRC definition used below against the VN-100 user manual for that firmware. Record the manual version in the README.
4. **Write the configuration.**
   - Register 6 = 0: asynchronous ASCII output off.
   - Register 75 (binary output 1):
     - AsyncMode: the VN serial port that the USB link uses; verify whether this is port 1, port 2 or both.
     - RateDivisor = IMU rate / 400. The VN-100 IMU rate is nominally 800 Hz, so the divisor is 2; verify.
     - OutputGroup = 0x01 (common group).
     - Field mask:
       - `base`: TimeStartup (bit 0), Quaternion (bit 4), AngularRate (bit 5, compensated), Accel (bit 8, compensated) → 0x0131;
       - `+ypr`: adds YawPitchRoll (bit 3);
       - `+mag`: adds MagPres (bit 10).
   - Registers 76 and 77: binary outputs 2 and 3 off (AsyncMode 0).
   - Register 5 (baud) = 921600, written last. Then reopen the port at 921600.
5. **Read back and save.**
   - Read back registers 5, 6, 75, 76 and 77, compare with what was written, and stop with an error on any mismatch.
   - Save to non-volatile memory with `$VNWNV`.
   - Close the port.
6. **Compute and save the packet layout** to `vn_layout.mat`: field mask, field order, byte offsets, types, packet length and nominal rate.
   - Packet = sync 0xFA, group byte, field mask (uint16 little-endian), payload in ascending field-bit order, CRC-16 (2 bytes).
   - For `base`: payload 8 + 16 + 12 + 12 = 48 bytes, packet 54 bytes.
   - At 400 Hz that is 21.6 kB/s, about 23% of a 921600-baud line.
7. **Checksums.** ASCII commands use the 8-bit XOR checksum of the characters between `$` and `*`, as two hex digits.

## 4. Run-time acquisition (primary path): C-MEX S-functions

### 4.1 `vn_raw_rx.c` (Level-2 C S-function, Windows API)

**Parameters** (non-tunable):
- COM port name, e.g. `'COM7'`;
- baud rate (921600);
- sample time Ts_rx (default 0.002 s);
- N_buf, the maximum bytes per call (default 128).

**mdlStart:**
- `CreateFileA("\\\\.\\COMx", GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL)`.
- `SetupComm(h, 65536, 4096)`.
- DCB: 8N1, binary, no hardware or software flow control, DTR and RTS enabled.
- `COMMTIMEOUTS {ReadIntervalTimeout = MAXDWORD, ReadTotalTimeoutMultiplier = 0, ReadTotalTimeoutConstant = 0}`, so ReadFile returns immediately with whatever is queued.
- `PurgeComm(h, PURGE_RXCLEAR | PURGE_RXABORT)`.
- Any failure calls `ssSetErrorStatus` with a clear message. Errors are raised at start only, never at run time.

**mdlOutputs** (at Ts_rx), in order. Never loop or wait.
1. t0 = QueryPerformanceCounter.
2. `ClearCommError(h, &errs, &stat)`.
3. n = min(stat.cbInQue, N_buf); if n > 0, call `ReadFile(h, buf, n, &nread, NULL)`.
4. t1 = QPC.
5. Zero-pad the unused part of the output buffer.

**Outputs:**

| Port | Type | Content |
|---|---|---|
| 1 | uint8 N_buf×1 | bytes read |
| 2 | uint16 | number of bytes read |
| 3 | double | t1 in seconds (QPC, after the read) |
| 4 | uint32 | bytes still queued after the read |
| 5 | uint32 | cumulative bytes read |
| 6 | uint32 | cumulative count of calls with a nonzero error mask or a failed read |
| 7 | uint32 | last error mask from ClearCommError (CE_RXOVER, CE_OVERRUN, CE_FRAME …) |
| 8 | double | call duration (t1 − t0) in µs |

**mdlTerminate:** CloseHandle.

**Sizing.** Nominal arrival is about 43 bytes per 2 ms. N_buf = 128 reads three times the nominal rate per call, so bursts (up to about 350 bytes with a 16 ms latency timer) drain within a few calls. Unread bytes stay in the 64 kB driver buffer, so data are lost only on driver overflow, which ports 6–7 count.

### 4.2 `qpc_clock.c` (Level-2 C S-function)

- Outputs QueryPerformanceCounter in seconds (double) at the 1 kHz base rate.
- Logged to `<subj>_<run>_clk.mat`.
- It gives the mapping between wall-clock time and t_sim, and the pacing jitter of the 1 kHz loop.

### 4.3 Build and wiring

- `vn_build.m` compiles both S-functions with `mex`, using a compiler supported by R2018a (MinGW-w64 4.9.2 or MSVC 2015/2017).
- Subsystem `IMU_RAW`:
  - `vn_raw_rx`, with two To File blocks in Timeseries format: `<subj>_<run>_imu_bytes.mat` (port 1, uint8) and `<subj>_<run>_imu_diag.mat` (ports 2–8 converted to double and muxed);
  - `qpc_clock`, with its own To File block.
- Document the wiring and the index maps in the README.

## 5. Fallback path (only if Sec. 5 fails its acceptance test in Sec. 7)

- **Forwarder.** `vn_udp_forwarder.m` runs in a second MATLAB instance.
  - It opens the VN-100 with `serial` at 921600 and polls `BytesAvailable` every 2 ms with a timer.
  - It stamps each read with QPC through a small MEX, `qpc_now.c`.
  - It sends one UDP datagram per read to 127.0.0.1:25000 with payload [double qpc; uint16 n; uint8 bytes(1:n)], n ≤ 1024.
- **Simulink.** A non-blocking UDP Receive block (maximum length 1034) at Ts_rx logs the payload and the received length to `<subj>_<run>_imu_bytes.mat`. `qpc_clock` stays in the model.
- **Offline.** The parser takes qpc from the payload as the receipt time.

## 6. Bench acceptance test: `vn_bench_test.m` (Milestone 1)

**Run A:** 10 min the IMU static for 1 min, then moved by hand.
**Run B:** the same model for 10 min without `IMU_RAW`.

Pass if, for Run A:
- after parsing (Sec. 8), CRC failures = 0, sensor-time gaps = 0 and duplicates = 0;
- the packet rate is within 0.1% of 400 Hz after drift correction;
- the error count is 0;
- the maximum call duration is < 100 µs and the mean < 20 µs;
- the backlog returns below N_buf within 50 ms of any burst.

And, comparing Run A with Run B: the 99th and 99.9th percentiles of the 1 kHz step interval (from `qpc_clock` differences) and the maximum pacing lag (wall − t_sim, after removing the initial offset) are unchanged within 5%.

Also verify that the configuration survives a power cycle (repeat the readback in `vn_configure` with a `'verify_only'` option).

## 7. Offline parser: `vn_parse.m` (Milestone 2)

`pk = vn_parse(imu_bytes_file, imu_diag_file, layout)`

1. **Byte stream.** Concatenate bytes(1:n) of each call in order. Assign each byte the QPC receipt time of its call.
2. **Packet scan.** Scan for 0xFA followed by the expected group byte and field mask, and check that enough bytes remain for the packet length.
3. **CRC.** VectorNav CRC-16-CCITT, computed over the bytes from the group byte through the two CRC bytes; a valid packet yields 0. On failure, advance one byte and rescan.
4. **Decode** little-endian fields:
   - TimeStartup (uint64 ns, converted to s);
   - quaternion (verify component order);
   - angular rate (rad/s);
   - acceleration (m/s²);
   - optionally yaw-pitch-roll (deg) and magnetic field, temperature and pressure.
5. **Receipt time** of a packet = receipt time of the call that delivered its last byte.
6. **Diagnostics** to report and return:
   - CRC failures;
   - bytes skipped during resync;
   - gaps (Δt_sensor > 1.5/400 s);
   - duplicates;
   - sensor-time resets (Δt_sensor < 0);
   - the number of calls that hit N_buf.

## 8. Timestamp reconstruction: `vn_timestamps.m` (Milestone 2)

`ts = vn_timestamps(pk, clk_file)`

1. **Segments.** Split at sensor-time resets and at gaps > 1 s.
2. **Lower-envelope fit**, per segment, of receipt QPC against sensor time (receipt delays are ≥ 0):
   - start with an ordinary least-squares fit;
   - then iterate a least-squares fit on points whose residual is at or below the 5th percentile within 2-s bins, until the parameters change by less than 1 µs over the segment.
   - The result is t_imu,qpc = a + b·t_sensor.
3. **Map to the model's time base**: t_imu,sim = interp1(qpc_clock, t_sim, t_imu,qpc). Both are monotone.
4. **Report** per segment:
   - drift (b − 1)·10⁶ in ppm;
   - residual percentiles (P5, P50, P95, P99) as latency jitter;
   - the number of points used.

Apply no constant latency correction.

## 9. Logging and file naming

- **IMU logs (this implementation):**
  - `<subj>_<run>_imu_bytes.mat` (uint8 N_buf × samples);
  - `<subj>_<run>_imu_diag.mat` (7 channels);
  - `<subj>_<run>_clk.mat` (QPC at 1 kHz).
- **Expected size:** about 100 kB/s with the defaults, about 150 MB for a 25-min run. Measure it in the bench test.
- **Runs:** one run for the baseline and one per controller sequence, the same runs as the phase-estimator logs.

## 10. Parameters: `vn_params_default.m`

| Name | Default | Notes |
|---|---|---|
| port | 'COM7' | set per PC |
| baud | 921600 | |
| rate | 400 Hz | RateDivisor = IMU rate/400 |
| fields | 'base' (mask 0x0131) | 'base+ypr', 'base+ypr+mag' |
| Ts_rx | 0.002 s | 0.001–0.01 |
| N_buf | 128 bytes | must exceed 3 × bytes per Ts_rx at nominal rate |
| driver buffer | 65536 bytes | SetupComm |
| f_lp events | 15 Hz | zero-phase |
| ms_thr | 0.5 × median peak | |
| to_win, hs_win | 0.4·T, 0.3·T | search windows |
| acc_ref_win, acc_ref_thr | ±40 ms, 1.5 g | optional HS refinement |
| hs_bias, to_bias | 0, 0 | from Sec. 10.1 |
| gap_thr | 1.5/400 s | |
| seg_gap | 1 s | |
| env_pct, env_bin | 5th percentile, 2 s | lower-envelope fit |

## 11. Deliverables and work order

Deliver separate .m and .c files plus `README.md`. The README covers:
- setup checklist (FTDI latency, compiler, COM port);
- wiring and index maps;
- the field layout verified against the manual;
- the parameter table;
- implementer choices;
- how to run each script.

1. **M1:** `vn_configure`, `vn_raw_rx.c`, `qpc_clock.c`, `vn_build`, the `IMU_RAW` wiring and `vn_bench_test`. Passing Sec. 6 is required before the IMU is used with subjects. M1 must not delay the phase-estimator work; if Sec. 4 fails, switch to Sec. 5.
2. **M2:** `vn_parse` and `vn_timestamps`.
