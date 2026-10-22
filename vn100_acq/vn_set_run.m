function files = vn_set_run(mdl, subj, run, p, rx_on)
%VN_SET_RUN  Point the IMU_RAW logs at a run and set block parameters.
%   files = vn_set_run(mdl, subj, run)            % same <subj>_<run> as the phase-estimator logs
%   files = vn_set_run(mdl, subj, run, p, rx_on)  % rx_on = false comments out IMU_RAW/RX (bench run B)
%
%   To Host File blocks (QUARC): sets vn_log_run('<subj>_<run>') and the
%   file names <p.log_dir>\%{vn_log_run}_*.mat.  QUARC substitutes the run
%   name when the model starts, so changing the run needs no rebuild.
%   To File blocks (normal simulation): writes the file names directly.
%   Also writes p.port (as a number), p.baud, p.Ts_rx, p.N_buf into
%   vn_raw_rx and p.Ts_clk into qpc_clock.
%
%   files.rebuild is true when a QUARC model's generated code no longer
%   matches (S-function parameters, RX on/off or a To Host File name
%   changed): run QUARC > Build (qc_build_model) before starting.  Save the
%   model afterwards if you want the settings kept.

if nargin < 4 || isempty(p), p = vn_params_default(); end
if nargin < 5 || isempty(rx_on), rx_on = true; end
mdl = vn_model_name(mdl);
files = vn_run_files(subj, run, p);
top = [mdl '/' p.blk];
if getSimulinkBlockHandle(top) == -1
    error('vn_set_run:noblk', '%s not found. Run vn_add_imu_raw(''%s'') first.', top, mdl);
end
if ~isempty(p.log_dir) && ~exist(p.log_dir, 'dir')
    mkdir(p.log_dir);
end
vn_log_run(sprintf('%s_%s', subj, run));

chg = false;
chg = set_log([top '/RX/ToFile_bytes'], files.bytes, '_imu_bytes.mat', p) || chg;
chg = set_log([top '/RX/ToFile_diag'],  files.diag,  '_imu_diag.mat',  p) || chg;
chg = set_log([top '/CLK/ToFile_clk'],  files.clk,   '_clk.mat',       p) || chg;
chg = set_log([mdl '/ToFile_motor'],    files.motor, '_motor.mat',     p) || chg;   % vn_add_motor_log
chg = set_if([top '/RX/vn_raw_rx'],  'Parameters', vn_rx_param_str(p)) || chg;
chg = set_if([top '/CLK/qpc_clock'], 'Parameters', sprintf('%.17g', p.Ts_clk)) || chg;
if rx_on, want = 'off'; state = 'RX active'; else, want = 'on'; state = 'RX commented out, clock only'; end
chg = set_if([top '/RX'], 'Commented', want) || chg;

files.rebuild = chg && vn_is_quarc(mdl);
fprintf('IMU_RAW -> %s | %s | %s (%s)\n', files.bytes, files.diag, files.clk, state);
if files.rebuild
    fprintf('  IMU_RAW settings changed: rebuild before starting (QUARC > Build, or qc_build_model(''%s'')).\n', mdl);
end
end

function changed = set_log(blk, file, suffix, p)
changed = false;
if getSimulinkBlockHandle(blk) == -1, return; end
if strcmp(get_param(blk, 'BlockType'), 'ToFile')
    changed = set_if(blk, 'Filename', file);
else                                          % QUARC To Host File
    if isempty(p.log_dir)
        error('vn_set_run:logdir', 'Set p.log_dir (e.g. ''C:\\exo_logs''): To Host File needs an absolute folder.');
    end
    changed = set_if(blk, 'file_name', fullfile(p.log_dir, ['%{vn_log_run}' suffix]));
end
end

function changed = set_if(blk, name, val)
changed = false;
if getSimulinkBlockHandle(blk) ~= -1 && ~strcmp(get_param(blk, name), val)
    set_param(blk, name, val);
    changed = true;
end
end
