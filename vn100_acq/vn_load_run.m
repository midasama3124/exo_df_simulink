function D = vn_load_run(subj, run, p, do_plot)
%VN_LOAD_RUN  Load one run offline: shoe IMU in physical units on t_sim, plus the motor log.
%   D = vn_load_run('S03', 'EF_blk2')            % uses vn_params_default (p.log_dir)
%   D = vn_load_run('S03', 'EF_blk2', p, true)   % also plots
%
%   Raw bytes -> packets (vn_parse, CRC checked) -> sensor time mapped to the
%   PC clock and to the model's simulation time (vn_timestamps).  Outputs:
%     D.imu.t       [s]      simulation time of each IMU sample (t_imu_sim), same axis as D.exo.t
%     D.imu.gyro    [rad/s]  angular rate, IMU frame [x y z]: the VN-100 compensated
%                            AngularRate (calibration and bias applied), 400 Hz, unfiltered
%     D.imu.accel   [m/s^2]  acceleration, IMU frame [x y z]
%     D.imu.gyro_filt, D.imu.accel_filt   zero-phase low-pass at p.imu_lp_hz (default
%                            20 Hz; 2nd-order Butterworth forward-backward, no delay),
%                            filtered per gap-free stretch; [] if p.imu_lp_hz = 0
%     D.imu.quat    [x y z w] attitude (scalar last); D.imu.quat_wxyz
%     D.imu.t_sensor [s]     VN-100 TimeStartup;  D.imu.t_qpc [s] PC clock
%     D.imu.diag, D.imu.seg  parser and timestamp diagnostics
%     D.exo.t       [s]      simulation time (1 kHz), if <subj>_<run>_motor.mat exists
%     D.exo.q, D.exo.q_dot   [joint 1, joint 2] as in the model (D.exo.q_dot_raw if logged)
%   Runs whose logs already existed get a suffix (S01_T01_2, ...; see vn_log_run):
%   load them with vn_load_run('S01', 'T01_2').  If the run is not found, the
%   runs that exist for the subject are listed.
%   The plot shows the filtered gyro axes (raw if filtering is off) and the
%   gyro magnitude over the joint position and velocity, on a shared time
%   axis; the duration of each log is printed.

if nargin < 3 || isempty(p), p = vn_params_default(); end
if nargin < 4 || isempty(do_plot), do_plot = false; end
f = vn_run_files(subj, run, p);
if ~exist(f.bytes, 'file')
    d = dir(fullfile(p.log_dir, sprintf('%s_*_clk.mat', subj)));
    runs = regexprep({d.name}, sprintf('^%s_(.*)_clk\\.mat$', subj), '$1');
    error('vn_load_run:norun', 'No logs for %s_%s in %s. Runs found for %s: %s', subj, run, ...
        p.log_dir, subj, strjoin(runs, ', '));
end
pk = vn_parse(f.bytes, f.diag, [], p);
ts = vn_timestamps(pk, f.clk, p);

D = struct('subj', subj, 'run', run, 'files', f);
D.imu = struct('t', ts.t_imu_sim, 'gyro', pk.gyro, 'accel', pk.accel, 'gyro_filt', [], 'accel_filt', [], ...
    'quat', pk.quat, 'quat_wxyz', pk.quat_wxyz, 't_sensor', pk.t_sensor, 't_qpc', ts.t_imu_qpc, ...
    'diag', pk.diag, 'seg', ts.seg, 'lp_hz', p.imu_lp_hz);
if p.imu_lp_hz > 0
    D.imu.gyro_filt  = lp_segments(pk.gyro,  pk.t_sensor, p);
    D.imu.accel_filt = lp_segments(pk.accel, pk.t_sensor, p);
end
D.exo = struct('t', [], 'q', [], 'q_dot', [], 'q_dot_raw', []);
if exist(f.motor, 'file')
    [t, X] = vn_load_tofile(f.motor);
    if ~any(size(X, 2) == [4 6])
        warning('vn_load_run:motor', '%s has %d channels, expected 4 (q, q_dot) or 6 (+ q_dot_raw) for 2 joints.', ...
            f.motor, size(X, 2));
    end
    D.exo.t = t;
    D.exo.q = X(:, 1:min(2, end));
    if size(X, 2) >= 4, D.exo.q_dot     = X(:, 3:4); end
    if size(X, 2) >= 6, D.exo.q_dot_raw = X(:, 5:6); end
    if size(X, 2) >= 2 && numel(unique(X(:, 1))) <= 2
        warning('vn_load_run:nomotor', ['%s: q takes at most 2 values over the run, so no motor data was ' ...
            'received (Peak CAN "Use separate thread for receive operation" must be off).'], f.motor);
    end
else
    fprintf('vn_load_run: no motor log %s.\n', f.motor);
end
ok = isfinite(D.imu.t);
fprintf('vn_load_run %s_%s: IMU %d samples, t_sim %.2f-%.2f s', subj, run, numel(D.imu.t), ...
    min(D.imu.t(ok)), max(D.imu.t(ok)));
if ~isempty(D.exo.t)
    fprintf(' | motor %d samples, t_sim %.2f-%.2f s', numel(D.exo.t), D.exo.t(1), D.exo.t(end));
end
fprintf('\n');

if do_plot
    plot_run(D);
end
end

function Y = lp_segments(X, tsen, p)
% vn_lp_zerophase per column, separately on each stretch without sensor-time gaps
Y = X;
brk = find(diff(tsen(:)) > p.gap_thr | diff(tsen(:)) < 0);
e = [0; brk; numel(tsen)];
fs = p.rate;
for s = 1:numel(e) - 1
    k = (e(s) + 1:e(s + 1)).';
    for c = 1:size(X, 2)
        Y(k, c) = vn_lp_zerophase(X(k, c), p.imu_lp_hz, fs);
    end
end
end

function plot_run(D)
name = strrep(sprintf('%s %s', D.subj, D.run), '_', '\_');
figure('Name', sprintf('%s %s', D.subj, D.run), 'Color', 'w');
if isempty(D.imu.gyro_filt)
    g = D.imu.gyro; lbl = 'raw';
else
    g = D.imu.gyro_filt; lbl = sprintf('%g Hz zero-phase', D.imu.lp_hz);
end
n = 2 + 2 * ~isempty(D.exo.t);
ax = gobjects(n, 1);
ax(1) = subplot(n, 1, 1);
plot(D.imu.t, g);
ylabel('gyro [rad/s]'); legend('x', 'y', 'z', 'Location', 'northeast'); grid on;
title(sprintf('%s: shoe VN-100 angular rate (IMU frame, %s)', name, lbl));
ax(2) = subplot(n, 1, 2);
plot(D.imu.t, sqrt(sum(g .^ 2, 2)), 'k');
ylabel('|gyro| [rad/s]'); grid on;
if ~isempty(D.exo.t)
    ax(3) = subplot(n, 1, 3);
    plot(D.exo.t, D.exo.q);
    ylabel('q'); legend('joint 1', 'joint 2', 'Location', 'northeast'); grid on;
    ax(4) = subplot(n, 1, 4);
    plot(D.exo.t, D.exo.q_dot);
    ylabel('q\_dot'); legend('joint 1', 'joint 2', 'Location', 'northeast'); grid on;
end
xlabel('simulation time t\_sim [s]  (zoom in: about 300 strides fit in a 5 min trial)');
linkaxes(ax, 'x');
end
