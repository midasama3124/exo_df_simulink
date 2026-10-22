function p = vn_params_default()
%VN_PARAMS_DEFAULT  Defaults for the shoe-mounted VN-100 acquisition.
%   p = vn_params_default() returns every parameter used by vn_configure,
%   the IMU_RAW subsystem, vn_parse, vn_timestamps, vn_gait_events and
%   vn_bench_test.  Override fields on a copy, e.g.
%       p = vn_params_default();  p.port = 'COM9';
%   Entries marked [choice] were not fixed by the specification; they are
%   listed in README.md under "Implementer choices".

% ---------------- Device and serial link (vn_configure) -------------------
p.port            = 'COM7';     % set per PC (Device Manager > Ports); vn_raw_rx gets the number (vn_port_number)
p.baud            = 921600;     % run-time baud, written to register 5
p.rate            = 400;        % Hz, binary output rate
p.imu_rate        = 800;        % Hz, nominal VN-100 IMU rate; RateDivisor = imu_rate/rate (measured by vn_configure)
p.fields          = 'base';     % 'base' (0x0131) | 'base+ypr' (0x0139) | 'base+ypr+mag' (0x0539)
p.async_mode      = 'auto';     % [choice] VN serial port of the USB link: 'auto' (detect) | 1 | 2 | 3
p.scan_bauds      = [115200 921600 460800 230400 128000 57600];
p.scan_timeout    = 0.200;      % s, wait for the $VNRRG,01 reply at each baud
p.scan_tries      = 2;          % [choice] attempts per baud (first command after a host baud switch can be lost)
p.cmd_timeout     = 0.500;      % s, ordinary register read/write
p.wnv_timeout     = 2.0;        % s, $VNWNV (flash write takes about 0.5 s)
p.detect_divisor  = 80;         % [choice] RateDivisor while detecting AsyncMode (800/80 = 10 Hz)
p.detect_time     = 0.6;        % [choice] s, listening time per candidate port
p.slow_link_util  = 0.8;        % [choice] if the final stream needs more than this fraction of the found baud, write reg 5 before the final reg 75
p.rate_check_time = 3.0;        % [choice] s, packet-rate measurement after configuring
p.rate_check_tol  = 0.01;       % [choice] relative tolerance on that measurement
p.ftdi_latency_ms = 1;          % required FTDI latency timer (Windows default 16 ms)
p.manual_version  = '';         % fill in, e.g. 'VN-100 ICD/User Manual rev X (firmware 2.x)'
p.cfg_dir         = fileparts(mfilename('fullpath'));  % [choice] where vn_unit_info.mat and vn_layout.mat are saved

% ---------------- Run-time acquisition (IMU_RAW) --------------------------
p.Ts_rx           = 0.002;      % s, vn_raw_rx sample time (0.001-0.01)
p.N_buf           = 128;        % bytes per call; must exceed 3 x bytes per Ts_rx at nominal rate
p.driver_buf      = 65536;      % bytes, SetupComm RX request (compiled into vn_raw_rx.c as VNRX_DRIVER_RXBUF)
p.Ts_clk          = 0.001;      % s, qpc_clock sample time (model base rate)
p.blk             = 'IMU_RAW';  % subsystem name at the model root

% ---------------- Logging --------------------------------------------------
p.subj            = 'S00';
p.run             = 'R00';
p.log_dir         = 'C:\exo_logs';  % [choice] local, non-synced folder (not OneDrive); must be absolute for To Host File
p.logger          = 'quarc';    % [choice] 'quarc' = To Host File (QUARC External mode) | 'tofile' = To File (normal simulation)

% ---------------- Parser and timestamps (vn_parse, vn_timestamps) --------
p.source          = 'auto';     % [choice] 'auto' | 'sfun' (primary path) | 'udp' (fallback path)
p.gap_thr         = 1.5/p.rate; % s, sensor-time gap
p.dup_thr         = 0.5/p.rate; % [choice] 0 <= dt_sensor < dup_thr counts as a duplicate
p.seg_gap         = 1.0;        % s, split segments at gaps longer than this (and at resets)
p.env_pct         = 5;          % lower-envelope percentile
p.env_bin         = 2.0;        % s, lower-envelope bin width
p.env_tol         = 1e-6;       % s, stop when the fitted line moves less than this over the segment
p.env_maxit       = 50;         % [choice] iteration cap
p.seg_min_pts     = 100;        % [choice] segments with fewer packets are not fitted (NaN times)
p.map_extrap_max  = 0.050;      % [choice] s, allowed extrapolation outside the qpc_clock range
p.imu_lp_hz       = 20;         % [choice] Hz, zero-phase low-pass for D.imu.gyro_filt / accel_filt (vn_load_run); 0 = off

% ---------------- Gait events (vn_gait_events, optional milestone 3) ------
p.f_lp_events     = 15;         % Hz, zero-phase low-pass of the sagittal angular rate
p.ms_thr_frac     = 0.5;        % mid-swing threshold = ms_thr_frac x median mid-swing peak
p.to_win          = 0.4;        % TO search window before mid-swing, fraction of stride time T
p.hs_win          = 0.3;        % HS search window after mid-swing, fraction of T
p.acc_ref         = false;      % [choice] optional HS refinement from |acc| off by default
p.acc_ref_win     = 0.040;      % s, +/- window around the gyro HS
p.acc_ref_thr     = 1.5;        % g, |acc| threshold for the refinement
p.hs_bias         = 0;          % s, added to every HS (calibrated against a reference, not in this spec)
p.to_bias         = 0;          % s, added to every TO
p.ev_axis         = 0;          % [choice] 0 = principal axis of the gyro (PCA), or 1/2/3 = IMU x/y/z
p.ev_sign         = 0;          % [choice] 0 = auto (mid-swing peak positive), or +1/-1
p.ev_min_ms       = 1.0;        % [choice] rad/s, absolute floor on mid-swing peaks (rejects standing)
p.ev_init_min_dist= 0.6;        % [choice] s, minimum spacing of candidate peaks before T is known
p.ev_min_peak_dist= 0.5;        % [choice] minimum mid-swing spacing, fraction of the median T
p.ev_T_range      = [0.5 1.5];  % [choice] valid stride time, fraction of the median T

% ---------------- Bench acceptance test (vn_bench_test) -------------------
p.bench_subj      = 'bench';
p.bench_T         = 360;        % s, length of runs A and B: bench_static_T still + 300 s moved by hand
p.bench_static_T  = 60;         % s, IMU static at the start of run A
p.bench_skip      = 2.0;        % [choice] s, start-up transient excluded from the timing statistics
p.bench_rate_tol  = 1e-3;       % packet rate within 0.1 % of p.rate
p.bench_dur_max_us  = 100;      % us
p.bench_dur_mean_us = 20;       % us
p.bench_backlog_ms  = 50;       % ms, backlog must drop below N_buf within this time
p.bench_timing_tol  = 0.05;     % A vs B, relative
p.bench_timing_abs  = 0;        % [choice] s, optional absolute floor for the A-vs-B comparison (0 = strict 5 %)
p.bench_start_timeout = 120;    % [choice] s, QUARC: wait for the model to connect after qc_start_model
p.bench_stop_margin   = 60;     % [choice] s, QUARC: stop the model with qc_stop_model if still running bench_T + this
p.bench_file_timeout  = 60;     % [choice] s, QUARC: wait for the To Host File logs after the model stops

% ---------------- Fallback path (vn_udp_forwarder) ------------------------
p.udp_port        = 25000;
p.udp_maxlen      = 1034;       % 8 (double qpc) + 2 (uint16 n) + 1024 bytes
p.fwd_period      = 0.002;      % s, forwarder timer period
p.fwd_max_n       = 1024;       % bytes per datagram
end
