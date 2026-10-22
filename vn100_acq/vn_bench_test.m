function R = vn_bench_test(mdl, step, p, run)
%VN_BENCH_TEST  Milestone-1 bench acceptance test of the IMU_RAW path.
%   R = vn_bench_test(mdl)               % 'all': run A, run B, analysis, power-cycle check
%   R = vn_bench_test(mdl, 'A')          % run A: IMU_RAW active, static p.bench_static_T s, then moved by hand
%   R = vn_bench_test(mdl, 'B')          % run B: same model, IMU_RAW/RX commented out
%   R = vn_bench_test(mdl, 'analyze')    % analyse existing <bench>_A_* and <bench>_B_* logs
%   R = vn_bench_test(mdl, 'powercycle') % prompt for a power cycle, then vn_configure verify_only
%   R = vn_bench_test(mdl, 'timing', p, 'B')  % 1 kHz loop timing of one existing run only
%                                        (no IMU criteria; for tuning the model's real-time setup,
%                                        e.g. with a shorter p.bench_T)
%
%   Pass criteria, run A:  CRC failures = 0, sensor-time gaps = 0,
%   duplicates = 0; packet rate within 0.1 % of p.rate after drift
%   correction; S-function error count = 0; call duration max < 100 us and
%   mean < 20 us; backlog back below N_buf within 50 ms of any burst.
%   Run A vs run B: P99 and P99.9 of the 1 kHz step interval and the
%   maximum pacing lag (wall - t_sim, initial offset removed) within 5 %.
%   Power cycle: configuration read back unchanged.  No vn_raw_rx call or
%   qpc_clock sample missing from the logs.
%
%   QUARC model (target quarc_*.tlc): each run sets IMU_RAW and StopTime,
%   builds with qc_build_model, starts with qc_start_model (External mode),
%   waits for the model to stop at StopTime (qc_stop_model after
%   p.bench_stop_margin, or on Ctrl+C), then waits for the To Host File
%   logs.  The real-time code drives the CAN bus: make the motors safe.
%   After the test the model's StopTime and RX are restored, so rebuild
%   before using the model again.
%   Normal simulation: SimulationCommand 'start'; refuses to run without
%   simulation pacing (EnablePacing), which would make the p.bench_T run
%   finish in seconds.  Default run length: p.bench_static_T = 60 s still,
%   then 300 s moved by hand (p.bench_T = 360 s).  The model must already contain IMU_RAW (vn_add_imu_raw) and
%   the VN-100 must have been configured.

if nargin < 2 || isempty(step), step = 'all'; end
if nargin < 3 || isempty(p), p = vn_params_default(); end
R = struct();
switch lower(step)
    case 'a'
        run_once(mdl, 'A', true, p);
        return;
    case 'b'
        run_once(mdl, 'B', false, p);
        return;
    case 'analyze'
        R = analyze(p);
    case 'powercycle'
        R.verify = powercycle(p);
        R.pass = R.verify.pass;
    case 'timing'
        if nargin < 4 || isempty(run), run = 'B'; end
        f = vn_run_files(p.bench_subj, run, p);
        R.timing = loop_timing(f.clk, p);
        return;
    case 'all'
        run_once(mdl, 'A', true, p);
        run_once(mdl, 'B', false, p);
        R = analyze(p);
        R.verify = powercycle(p);
        R.pass_runs = R.pass;
        R.pass = R.pass && R.verify.pass;
    otherwise
        error('vn_bench_test:step', 'Unknown step ''%s''.', step);
end
fout = fullfile(p.log_dir, sprintf('%s_result_%s.mat', p.bench_subj, datestr(now, 'yyyymmdd_HHMMSS')));
save(fout, 'R');
if R.pass, verdict = 'PASS'; else, verdict = 'FAIL'; end
fprintf('\nBench test %s. Results saved to %s\n', verdict, fout);
end

% =========================================================================
function run_once(mdl, run, rx_on, p)
mdl = vn_model_name(mdl);
open_system(mdl);                         % SimulationCommand / External mode need the model open
quarc = vn_is_quarc(mdl);
if ~quarc && ~strcmpi(get_param(mdl, 'EnablePacing'), 'on')
    error('vn_bench_test:pacing', ['%s runs in normal simulation without pacing, so %g s would pass in a few ' ...
        'seconds. Use QUARC External mode (QUARC > Set default options), or turn on simulation pacing.'], mdl, p.bench_T);
end
f = vn_run_files(p.bench_subj, run, p);
vn_set_run(mdl, p.bench_subj, run, p, rx_on);
old_stop = get_param(mdl, 'StopTime');
set_param(mdl, 'StopTime', sprintf('%g', p.bench_T));
cleanup = onCleanup(@() restore(mdl, old_stop, p, quarc));
setappdata(0, 'vn_log_run_lock', true);   % init.m (InitFcn) must not rename the bench logs
if quarc
    fprintf('\nBuilding %s for run %s (qc_build_model) ...\n', mdl, run);
    qc_build_model(mdl);
end
if rx_on
    fprintf('\nRun %s (%g s): keep the IMU STATIC for the first %g s, then move it by hand until the end.\n', ...
        run, p.bench_T, p.bench_static_T);
else
    fprintf('\nRun %s (%g s): IMU_RAW/RX commented out, qpc_clock still logging.\n', run, p.bench_T);
end
if quarc
    fprintf('The real-time code drives the CAN bus: make sure the motors are safe.\n');
end
input('Press Enter to start... ', 's');
t_start = now;
if quarc
    qc_start_model(mdl);
    is_on = @() strcmp(qc_get_simulation_status(mdl), 'external');
else
    set_param(mdl, 'SimulationCommand', 'start');
    is_on = @() strcmp(get_param(mdl, 'SimulationStatus'), 'running');
end
t0 = tic;
while ~is_on()
    if toc(t0) > p.bench_start_timeout
        error('vn_bench_test:start', 'The model did not start within %g s.', p.bench_start_timeout);
    end
    pause(0.1);
end
t1 = tic;
beeped = false;
while is_on()
    pause(0.5);
    if rx_on && ~beeped && toc(t1) >= p.bench_static_T
        beep;
        fprintf('  %.0f s: now move the IMU by hand.\n', toc(t1));
        beeped = true;
    end
    if quarc && toc(t1) > p.bench_T + p.bench_stop_margin
        warning('vn_bench_test:stop', 'Model still running after %.0f s; stopping it.', toc(t1));
        qc_stop_model(mdl);
    end
end
fprintf('Run %s finished after %.1f s wall time.\n', run, toc(t1));
if toc(t1) < 0.9 * p.bench_T
    warning('vn_bench_test:short', 'Run %s lasted %.1f s of wall time for StopTime %g s.', run, toc(t1), p.bench_T);
end
if quarc
    logs = {f.clk};
    if rx_on, logs = [logs, {f.bytes, f.diag}]; end
    wait_files(logs, t_start, p.bench_file_timeout);
end
end

function restore(mdl, old_stop, p, quarc)
if isappdata(0, 'vn_log_run_lock'), rmappdata(0, 'vn_log_run_lock'); end
if quarc && strcmp(qc_get_simulation_status(mdl), 'external')
    fprintf('Stopping %s ...\n', mdl);       % e.g. after Ctrl+C
    qc_stop_model(mdl);
end
set_param(mdl, 'StopTime', old_stop);
set_param([mdl '/' p.blk '/RX'], 'Commented', 'off');
if quarc
    fprintf('StopTime and IMU_RAW/RX restored: rebuild %s before using it again.\n', mdl);
end
end

function wait_files(files, t_start, timeout)
% To Host File writes on the host when the model stops: wait for new, stable files
t0 = tic;
last = -ones(1, numel(files));
while true
    d  = cellfun(@dir, files, 'UniformOutput', false);
    ok = cellfun(@(x) ~isempty(x) && x.datenum >= t_start - 1 / 86400, d);
    sz = cellfun(@(x) sum([x.bytes]), d);
    if all(ok) && isequal(sz, last), return; end
    last = sz;
    if toc(t0) > timeout
        miss = files(~ok);
        error('vn_bench_test:files', 'Logs not written within %g s: %s', timeout, strjoin(miss, ', '));
    end
    pause(1);
end
end

% =========================================================================
function R = analyze(p)
fa = vn_run_files(p.bench_subj, 'A', p);
fb = vn_run_files(p.bench_subj, 'B', p);
lf = fullfile(p.cfg_dir, 'vn_layout.mat');
if exist(lf, 'file')
    S = load(lf); layout = S.layout;
else
    warning('vn_bench_test:layout', 'vn_layout.mat not found; using vn_make_layout(''%s'').', p.fields);
    layout = vn_make_layout(p.fields, p.rate, p.baud);
end

fprintf('\n--- Run A ---\n');
pk = vn_parse(fa.bytes, fa.diag, layout, p);
ts = vn_timestamps(pk, fa.clk, p);
[~, D] = vn_load_tofile(fa.diag);
dg = pk.diag;

rows = {};
rows(end + 1, :) = {'CRC failures',               num(dg.crc_fail),   '= 0',  dg.crc_fail == 0};
rows(end + 1, :) = {'sensor-time gaps',           num(dg.gaps),       '= 0',  dg.gaps == 0};
rows(end + 1, :) = {'duplicates',                 num(dg.duplicates), '= 0',  dg.duplicates == 0};
rows(end + 1, :) = {'sensor-time resets (info)',  num(dg.resets),     '-',    true};
rows(end + 1, :) = {'RX calls missing from log',  num(dg.log_dropped_calls), '= 0', dg.log_dropped_calls == 0};

fitted = find([ts.seg.fitted]);
if isempty(fitted)
    rate_q = NaN; rate_s = NaN;
else
    [~, j] = max([ts.seg(fitted).n]);
    g = ts.seg(fitted(j));
    i1 = g.idx(1); i2 = g.idx(2);
    % sample slots, not packets: a lost packet must not look like a slow sensor
    slots  = sum(max(1, round(diff(pk.t_sensor(i1:i2)) * p.rate)));
    rate_s = slots / (pk.t_sensor(i2) - pk.t_sensor(i1));         % sensor clock
    rate_q = slots / (ts.t_imu_qpc(i2) - ts.t_imu_qpc(i1));       % PC clock, drift-corrected
end
rows(end + 1, :) = {'packet rate, PC clock [Hz]', sprintf('%.4f', rate_q), sprintf('%g +/- %.1f%%', p.rate, 100 * p.bench_rate_tol), ...
    abs(rate_q - p.rate) / p.rate <= p.bench_rate_tol};
rows(end + 1, :) = {'packet rate, sensor clock (info)', sprintf('%.4f', rate_s), '-', true};
if ~isempty(fitted)
    rows(end + 1, :) = {'clock drift (info) [ppm]', sprintf('%+.2f', g.drift_ppm), '-', true};
end
rows(end + 1, :) = {'S-function error count',    num(D(end, 5)), '= 0', D(end, 5) == 0};
rows(end + 1, :) = {'call duration max [us]',    sprintf('%.1f', max(D(:, 7))),  sprintf('< %g', p.bench_dur_max_us),  max(D(:, 7)) < p.bench_dur_max_us};
rows(end + 1, :) = {'call duration mean [us]',   sprintf('%.2f', mean(D(:, 7))), sprintf('< %g', p.bench_dur_mean_us), mean(D(:, 7)) < p.bench_dur_mean_us};
[rec, nb] = backlog_recovery(D(:, 3), D(:, 2), p.N_buf);
rows(end + 1, :) = {sprintf('backlog recovery, %d bursts [ms]', nb), sprintf('%.1f', 1e3 * rec), ...
    sprintf('<= %g', p.bench_backlog_ms), rec <= p.bench_backlog_ms / 1e3};
rows(end + 1, :) = {'calls that hit N_buf (info)', num(dg.calls_hit_nbuf), '-', true};

fprintf('\n--- Run A vs run B (1 kHz loop) ---\n');
TA = timing_stats(fa.clk, p);
TB = timing_stats(fb.clk, p);
cmp = {'step interval P99',   'p99';
       'step interval P99.9', 'p999';
       'max pacing lag',      'max_lag'};
for k = 1:size(cmp, 1)
    a = TA.(cmp{k, 2}); b = TB.(cmp{k, 2});
    okk = abs(a - b) <= max(p.bench_timing_tol * abs(b), p.bench_timing_abs);
    rows(end + 1, :) = {[cmp{k, 1} ' A / B [ms]'], sprintf('%.4f / %.4f (%+.1f%%)', 1e3 * a, 1e3 * b, 100 * (a - b) / b), ...
        sprintf('within %g%%', 100 * p.bench_timing_tol), okk}; %#ok<AGROW>
end
rows(end + 1, :) = {'clk samples missing A / B', sprintf('%d / %d', TA.missing, TB.missing), '= 0', ...
    TA.missing == 0 && TB.missing == 0};
rows(end + 1, :) = {'run length A / B (wall) [s]', sprintf('%.1f / %.1f', TA.wall, TB.wall), ...
    sprintf('>= %g', 0.9 * p.bench_T), min(TA.wall, TB.wall) >= 0.9 * p.bench_T};

% log size
fs = [file_bytes(fa.bytes), file_bytes(fa.diag), file_bytes(fa.clk)];
rows(end + 1, :) = {'log size run A (info) [kB/s]', sprintf('%.1f (bytes %.1f, diag %.1f, clk %.1f)', ...
    sum(fs) / TA.duration / 1e3, fs / TA.duration / 1e3), '-', true};

print_rows(rows);
R.rows   = rows;
R.pass   = all([rows{:, 4}]);
R.pk_diag = dg;
R.seg    = ts.seg;
R.timing = struct('A', TA, 'B', TB);
R.rate_pc = rate_q;
R.files  = struct('A', fa, 'B', fb);
end

function v = powercycle(p)
fprintf(['\nPower-cycle check: unplug the VN-100 USB cable, wait 5 s, plug it back in ' ...
    '(check the COM port number is unchanged).\n']);
input('Press Enter when ready... ', 's');
o = p;
o.verify_only = true;
v = vn_configure(p.port, o);
end

function T = timing_stats(file, p)
[t, Q] = vn_load_tofile(file);
q = double(Q(:, 1));
dt_sim  = diff(t);
missing = sum(max(0, round(dt_sim / median(dt_sim)) - 1));
wall    = q(end) - q(1);
k = t >= t(1) + p.bench_skip;
t = t(k); q = q(k);
dq  = diff(q);
lag = (q - q(1)) - (t - t(1));
T = struct('p99', vn_prctile(dq, 99), 'p999', vn_prctile(dq, 99.9), 'max_dt', max(dq), ...
    'mean_dt', mean(dq), 'max_lag', max(lag), 'end_lag', lag(end), 'duration', t(end) - t(1), ...
    'missing', missing, 'wall', wall);
fprintf('  %s: step mean %.4f ms, P99 %.4f ms, P99.9 %.4f ms, max %.3f ms, max lag %.3f ms\n', ...
    file, 1e3 * T.mean_dt, 1e3 * T.p99, 1e3 * T.p999, 1e3 * T.max_dt, 1e3 * T.max_lag);
end

function T = loop_timing(file, p)
% timing_stats plus how the lag builds up: slowdown, long-step bursts, catch-up
T = timing_stats(file, p);
[t, Q] = vn_load_tofile(file);
q  = double(Q(:, 1));
k  = t >= t(1) + p.bench_skip;
t  = t(k); q = q(k);
dq = diff(q);
T.slowdown_pct = 100 * ((q(end) - q(1)) / (t(end) - t(1)) - 1);
T.lag_rate_ms_per_s = 1e3 * T.end_lag / (t(end) - t(1));
T.n_over_1p5ms = sum(dq > 1.5e-3);
T.n_over_5ms   = sum(dq > 5e-3);
T.excess_ms    = 1e3 * sum(dq(dq > 1.1e-3) - 1e-3);
T.catchup_ms   = 1e3 * sum(dq(dq < 0.9e-3) - 1e-3);
fprintf(['  model %.3f %% slower than real time, lag grows %.2f ms/s (end %.1f ms) | steps > 1.5 ms: %d, > 5 ms: %d | ' ...
    'time lost in long steps %.0f ms, recovered in short steps %.0f ms\n'], T.slowdown_pct, T.lag_rate_ms_per_s, ...
    1e3 * T.end_lag, T.n_over_1p5ms, T.n_over_5ms, T.excess_ms, -T.catchup_ms);
end

function [worst, nb] = backlog_recovery(queued, tq, nbuf)
% episodes with queued >= N_buf; time until queued < N_buf again
over = queued(:) >= nbuf;
d  = diff([false; over; false]);
st = find(d == 1);
en = find(d == -1);              % first index back below N_buf (may be numel + 1)
nb = numel(st);
worst = 0;
for k = 1:nb
    if en(k) > numel(tq)
        worst = inf;
    else
        worst = max(worst, tq(en(k)) - tq(st(k)));
    end
end
end

function b = file_bytes(f)
d = dir(f);
if isempty(d), b = NaN; else, b = d.bytes; end
end

function s = num(x)
s = sprintf('%d', round(x));
end

function print_rows(rows)
fprintf('\n%-38s %-40s %-16s %s\n', 'criterion', 'value', 'limit', 'result');
fprintf('%s\n', repmat('-', 1, 104));
for k = 1:size(rows, 1)
    if rows{k, 4}, r = 'ok'; else, r = 'FAIL'; end
    if strcmp(rows{k, 3}, '-'), r = ''; end
    fprintf('%-38s %-40s %-16s %s\n', rows{k, 1}, rows{k, 2}, rows{k, 3}, r);
end
end
