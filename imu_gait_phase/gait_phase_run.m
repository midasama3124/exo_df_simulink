%GAIT_PHASE_RUN  Continuous gait phase of one trial from the shoe VN-100.
%   Set subject, speed and condition below and run this script.  It
%     1. loads experimental_data/<subject>/<speed>ms_<condition>/<run>_*.mat
%        with vn_load_run (IMU on the model's t_sim, plus the motor log),
%     2. detects heel strikes on the chosen shoe angular velocity
%        (trough after each mid-swing peak; derived from fsr_imu_gait_detection.m,
%        without the Signal Processing Toolbox),
%     3. opens gait_phase_editor: choose the reference signal, invert it,
%        add/delete heel strikes, then "Save and close",
%     4. saves <run>_gait_phase.mat next to the logs (e.g. S01_T01_gait_phase.mat):
%        gaitPhase.time / .data       IMU time t_sim [s] / phase [%] (400 Hz)
%        gaitPhase.time_exo / .data_exo   same on the motor-log time (1 kHz)
%        gaitPhase.stride(_exo)      stride number k (between heel strikes k and k+1)
%        gaitEvents                  heel strikes (final and automatic), strides,
%                                    reference signal, options, source files
%   Phase: 0 % at a heel strike, rising linearly to 100 % at the next one; NaN
%   before the first and after the last heel strike and in strides whose
%   duration is outside opt.strideRange x median (standing, missed or extra
%   heel strikes).
%   If the output file exists and opt.resume is true, its heel strikes and
%   reference settings are loaded into the editor instead of a fresh detection.
%   Runs in MATLAB R2018a and later, no toolboxes.

%% ---------------- Trial ----------------
subject   = 'pun';     % folder under experimental_data/
speed     = '11';         % as in the folder name <speed>ms_<condition>: '08', '1', '09', '11'
condition = 'passive';    % experimental condition (only 'passive' so far)

%% ---------------- Options ----------------
opt.useGui      = true;    % false: automatic detection only, no window
opt.resume      = true;    % start from an existing <run>_gait_phase.mat
opt.axis        = 0;       % reference: 1/2/3 = gyro x/y/z, 4 = principal axis, 0 = IMU axis with the largest variance
opt.invert      = 'auto';  % true / false, or 'auto' (toe-off and heel-strike troughs negative)
opt.lpHz        = 15;      % Hz, zero-phase low-pass of the reference signal
opt.msMin       = 1.0;     % rad/s, minimum mid-swing peak
opt.msThrFrac   = 0.5;     % mid-swing threshold, fraction of the median peak
opt.msInitDist  = 0.6;     % s, minimum peak spacing before the stride time is known
opt.msMinDist   = 0.5;     % minimum mid-swing spacing, fraction of the median stride
opt.hsWin       = 0.4;     % heel-strike search window after mid-swing, fraction of the stride
opt.closeFrac   = 0.5;     % heel strikes closer than this x median interval: keep the deeper one
opt.strideRange = [0.6 1.5];  % valid stride time, fraction of the median
opt.snapWin     = 0.08;    % s, an added heel strike snaps to the lowest point within +/- this
opt.minHsSep    = 0.2;     % s, no heel strike is added this close to an existing one
opt.delTol      = 0.3;     % s, delete the nearest heel strike only if it is this close
opt.winWidth    = 10;      % s, initial window width in the editor (5, 10, 20, 60 or Inf)

%% ---------------- Files ----------------
codeDir = fileparts(mfilename('fullpath'));
root    = fileparts(codeDir);
addpath(codeDir, fullfile(root, 'vn100_acq'));
trialDir = fullfile(root, 'experimental_data', subject, sprintf('%sms_%s', speed, condition));
if ~exist(trialDir, 'dir')
    d = dir(fullfile(root, 'experimental_data', subject, '*ms_*'));
    error('gait_phase_run:nodir', 'No folder %s. Folders for %s: %s', trialDir, subject, ...
        strjoin({d([d.isdir]).name}, ', '));
end
d = dir(fullfile(trialDir, '*_imu_bytes.mat'));
if numel(d) ~= 1
    error('gait_phase_run:runs', 'Expected one *_imu_bytes.mat in %s, found %d: %s', trialDir, ...
        numel(d), strjoin({d.name}, ', '));
end
runName = regexprep(d(1).name, '_imu_bytes\.mat$', '');     % e.g. S01_T01 (or S01_T01_2)
us      = find(runName == '_', 1, 'first');
vp = vn_params_default();
vp.log_dir   = trialDir;
vp.imu_lp_hz = 0;                                            % filtered here (opt.lpHz)
D = vn_load_run(runName(1:us - 1), runName(us + 1:end), vp, false);
outFile = fullfile(trialDir, [runName '_gait_phase.mat']);
label   = sprintf('%s %s ms %s (%s)', subject, speed, condition, runName);

%% ---------------- Start state ----------------
t    = D.imu.t(:);
gyro = double(D.imu.gyro);
fs   = 1 / median(diff(t));
S0   = struct('axisChoice', opt.axis, 'invert', false, 'hsIdx', [], 'hsAutoIdx', []);
if opt.resume && exist(outFile, 'file')
    L = load(outFile, 'gaitEvents');
    S0.axisChoice = L.gaitEvents.reference.axisChoice;
    S0.invert     = L.gaitEvents.reference.inverted;
    S0.hsIdx      = L.gaitEvents.heelStrikeIdx;
    S0.hsAutoIdx  = L.gaitEvents.auto.heelStrikeIdx;
    fprintf('gait_phase_run: resuming %s (%d heel strikes).\n', outFile, numel(S0.hsIdx));
else
    if S0.axisChoice == 0
        [~, S0.axisChoice] = max(var(gyro(all(isfinite(gyro), 2), :)));
    end
    if ischar(opt.invert)
        w0 = gait_reference_signal(gyro, S0.axisChoice, false, opt.lpHz, fs);
        S0.invert = gait_auto_sign(w0, fs) < 0;
    else
        S0.invert = logical(opt.invert);
    end
end

%% ---------------- Detection and review ----------------
R = gait_phase_editor(t, gyro, S0, opt, label);
if R.cancelled
    fprintf('gait_phase_run: cancelled, nothing saved.\n');
    return;
end

%% ---------------- Phase and export ----------------
[phase, stride, okStride, Tstride] = gait_phase_compute(R.hsTime, t, opt.strideRange);
gaitPhase = struct('time', t, 'data', phase, 'stride', stride, 'units', '%', ...
    'definition', '0 % at heel strike, linear to 100 % at the next heel strike; NaN outside valid strides', ...
    'time_exo', [], 'data_exo', [], 'stride_exo', []);
if ~isempty(D.exo.t)
    [gaitPhase.data_exo, gaitPhase.stride_exo] = gait_phase_compute(R.hsTime, D.exo.t(:), opt.strideRange);
    gaitPhase.time_exo = D.exo.t(:);
end

hsT = unique(R.hsTime(:));
gaitEvents = struct();
gaitEvents.heelStrikeTimes  = hsT;
gaitEvents.heelStrikeIdx    = R.hsIdx(:);
gaitEvents.strideStartTimes = hsT(1:end - 1);
gaitEvents.strideTimes      = Tstride;
gaitEvents.strideValid      = okStride;
gaitEvents.auto.heelStrikeTimes = R.hsAutoTime(:);
gaitEvents.auto.heelStrikeIdx   = R.hsAutoIdx(:);
gaitEvents.auto.midSwingTimes   = t(R.msIdx);
gaitEvents.nAdded   = R.nAdded;
gaitEvents.nDeleted = R.nDeleted;
gaitEvents.reference = struct('axisChoice', R.axisChoice, 'axisName', R.axisName, ...
    'signalName', R.signalName, 'axis', R.axisVec, 'inverted', R.invert, 'lpHz', opt.lpHz);
gaitEvents.parameters = opt;
gaitEvents.trial  = struct('subject', subject, 'speed', speed, 'condition', condition, ...
    'run', runName, 'folder', trialDir, 'files', D.files);
gaitEvents.created       = datestr(now, 'yyyy-mm-dd HH:MM:SS');
gaitEvents.matlabVersion = version;
save(outFile, 'gaitPhase', 'gaitEvents');
fprintf(['gait_phase_run: saved %s\n  %d heel strikes (%d automatic, %d added, %d deleted), ' ...
    '%d valid strides of %d, median stride %.3f s, reference %s\n'], outFile, numel(hsT), ...
    numel(R.hsAutoIdx), R.nAdded, R.nDeleted, sum(okStride), numel(okStride), median(Tstride), R.signalName);

%% ---------------- Summary plot ----------------
figure('Name', ['Gait phase summary: ' runName]);
subplot(2, 2, [1 3]);
pg = 0:100;
W  = nan(numel(okStride), numel(pg));
for k = find(okStride(:)).'
    ii = find(stride == k);
    if numel(ii) > 2, W(k, :) = interp1(phase(ii), R.w(ii), pg, 'linear', 'extrap'); end
end
plot(pg, W.', 'Color', [0.6 0.6 0.6]); hold on;
mu = mean(W, 1, 'omitnan'); sd = std(W, 0, 1, 'omitnan');
plot(pg, mu, 'Color', [0 0.45 0.74], 'LineWidth', 2);
plot(pg, mu + sd, '--', pg, mu - sd, '--', 'Color', [0 0.45 0.74]);
grid on; xlabel('gait phase [%]'); ylabel(sprintf('%s [rad/s]', R.signalName));
title(sprintf('%s: %d valid strides (mean \\pm SD)', strrep(runName, '_', '\_'), sum(okStride)));
subplot(2, 2, 2);
plot(hsT(1:end - 1), Tstride, '.-', 'Color', [0.5 0.5 0.5]); hold on;
plot(hsT(find(~okStride)), Tstride(~okStride), 'ro'); %#ok<FNDSB>
grid on; xlabel('t\_sim [s]'); ylabel('stride time [s]');
ylim([0 min(max(Tstride) * 1.05, 3 * median(Tstride))]);
title('stride time (red: outside valid range, phase NaN)');
subplot(2, 2, 4);
k  = find(isfinite(phase), 1, 'first');
tw = t(k) + [0 8];
m  = t >= tw(1) & t <= tw(2);
yyaxis left;  plot(t(m), R.w(m)); ylabel('reference [rad/s]');
yyaxis right; plot(t(m), phase(m)); ylabel('phase [%]'); ylim([0 100]);
grid on; xlabel('t\_sim [s]'); title('first 8 s of valid phase');
