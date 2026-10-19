function vn_add_imu_raw(mdl, p, mode)
%VN_ADD_IMU_RAW  Insert the IMU_RAW subsystem at the root of an existing model.
%   vn_add_imu_raw(mdl)               % mdl = model name or path to the .slx; uses vn_params_default
%   vn_add_imu_raw(mdl, p)
%   vn_add_imu_raw(mdl, p, 'replace') % delete an existing IMU_RAW first
%
%   Builds (no inports, no outports: nothing else in the model can read it)
%
%   p.logger = 'quarc' (default; QUARC External mode, files written on the host)
%   <mdl>/IMU_RAW
%     RX   vn_raw_rx (Ts_rx) --port 1 (uint8 N_buf)--> Terminator
%                            --ports 2..8 -> double -> Mux(7) -> ToFile_diag   To Host File  <log_dir>\%{vn_log_run}_imu_diag.mat   (imu_diag)
%                            --port 9 (bytes packed 4 per double) ---------> ToFile_bytes  To Host File  <log_dir>\%{vn_log_run}_imu_bytes.mat  (imu_bytes_p4)
%     CLK  qpc_clock (Ts_clk) ------------------------------------------> ToFile_clk    To Host File  <log_dir>\%{vn_log_run}_clk.mat        (clk)
%   To Host File blocks use MAT-file v.5: one variable [t_sim; data], one
%   column per call.  vn_log_run supplies <subj>_<run> each time the model
%   starts, so a new run needs no rebuild.
%
%   p.logger = 'tofile' (normal simulation only, the original layout)
%     port 1 -> To File imu_bytes (Timeseries, uint8), ports 2..8 -> To File
%     imu_diag, qpc_clock -> To File clk; file names are written directly.
%
%   imu_diag columns: 1 n_read, 2 t_rx_qpc [s], 3 queued, 4 cum_bytes,
%                     5 err_calls, 6 err_mask, 7 dur_us
%   RX sits in its own subsystem so that bench run B can comment it out
%   while the clock keeps logging (vn_set_run(..., false)).  Save the model
%   afterwards.

if nargin < 2 || isempty(p), p = vn_params_default(); end
if nargin < 3, mode = ''; end
if exist('vn_raw_rx', 'file') ~= 3 || exist('qpc_clock', 'file') ~= 3
    error('vn_add_imu_raw:mex', 'vn_raw_rx / qpc_clock MEX files not found. Run vn_build first.');
end
logger = lower(p.logger);
if ~any(strcmp(logger, {'quarc', 'tofile'}))
    error('vn_add_imu_raw:logger', 'p.logger must be ''quarc'' or ''tofile''.');
end
if strcmp(logger, 'quarc') && exist('quarc_library', 'file') ~= 4
    error('vn_add_imu_raw:quarc', 'QUARC library (quarc_library) not found on the MATLAB path.');
end
mdl = vn_model_name(mdl);                 % accepts a name or a file path
load_system('simulink');
if strcmp(logger, 'quarc'), load_system('quarc_library'); end
top = [mdl '/' p.blk];
if getSimulinkBlockHandle(top) ~= -1
    if strcmpi(mode, 'replace')
        delete_block(top);
        fprintf('Deleted the existing %s.\n', top);
    else
        error('vn_add_imu_raw:exists', ['%s already exists. Use vn_add_imu_raw(mdl, p, ''replace''), ' ...
            'or vn_set_run to change its settings.'], top);
    end
end

% solver and sample-time sanity
fs = str2double(get_param(mdl, 'FixedStep'));
if ~strcmpi(get_param(mdl, 'SolverType'), 'Fixed-step')
    warning('vn_add_imu_raw:solver', 'Model solver is not fixed-step (set Fixed-step, discrete, %g s).', p.Ts_clk);
elseif ~isfinite(fs)
    warning('vn_add_imu_raw:step', 'Fixed step size is ''%s''; set it to %g.', get_param(mdl, 'FixedStep'), p.Ts_clk);
elseif abs(p.Ts_rx/fs - round(p.Ts_rx/fs)) > 1e-9 || abs(p.Ts_clk/fs - round(p.Ts_clk/fs)) > 1e-9
    warning('vn_add_imu_raw:ts', 'Ts_rx = %g and Ts_clk = %g must be multiples of the fixed step %g.', p.Ts_rx, p.Ts_clk, fs);
end
if strcmp(logger, 'quarc') && ~vn_is_quarc(mdl)
    warning('vn_add_imu_raw:target', ['The model does not use a QUARC target yet (QUARC > Set default options). ' ...
        'The To Host File blocks are meant for QUARC External mode.']);
end
if strcmp(logger, 'quarc') && isempty(p.log_dir)
    error('vn_add_imu_raw:logdir', 'Set p.log_dir (e.g. ''C:\\exo_logs''): To Host File needs an absolute folder.');
end

% place below the existing root blocks
b  = find_system(mdl, 'SearchDepth', 1, 'Type', 'block');
y0 = 40;
for k = 1:numel(b)
    pos = get_param(b{k}, 'Position');
    y0  = max(y0, pos(4) + 40);
end
add_block('built-in/Subsystem', top, 'Position', [40 y0 200 y0 + 60]);
set_param(top, 'Description', ['Shoe VN-100 raw acquisition for OFFLINE validation only. ' ...
    'No outports: nothing in the controller or the phase estimator may read it. See README.md.']);
set_param(top, 'BackgroundColor', 'lightBlue');

rx = [top '/RX'];
ck = [top '/CLK'];
add_block('built-in/Subsystem', rx, 'Position', [60 40 200 110]);
add_block('built-in/Subsystem', ck, 'Position', [60 160 200 220]);

% ---- RX ----
sf = [rx '/vn_raw_rx'];
add_block('simulink/User-Defined Functions/S-Function', sf, 'Position', [60 40 180 360]);
set_param(sf, 'FunctionName', 'vn_raw_rx', 'Parameters', vn_rx_param_str(p));
add_sink(rx, 'ToFile_diag', 'imu_diag', [480 195 600 225], logger);
add_block('simulink/Signal Routing/Mux', [rx '/Mux'], 'Inputs', '7', 'Position', [420 100 425 320]);
names = {'imu_bytes', 'n_read', 't_rx_qpc', 'queued', 'cum_bytes', 'err_calls', 'err_mask', 'dur_us', 'imu_bytes_p4'};
if strcmp(logger, 'quarc')
    add_block('simulink/Sinks/Terminator', [rx '/Term_bytes'], 'Position', [260 47 280 67]);
    h = connect(rx, 'vn_raw_rx/1', 'Term_bytes/1');
    set_param(h, 'Name', names{1});
    add_sink(rx, 'ToFile_bytes', 'imu_bytes_p4', [440 335 560 365], logger);
    h = connect(rx, 'vn_raw_rx/9', 'ToFile_bytes/1');
    set_param(h, 'Name', names{9});
else
    add_sink(rx, 'ToFile_bytes', 'imu_bytes', [440 42 560 72], logger);
    h = connect(rx, 'vn_raw_rx/1', 'ToFile_bytes/1');
    set_param(h, 'Name', names{1});
    add_block('simulink/Sinks/Terminator', [rx '/Term_p4'], 'Position', [260 335 280 355]);
    connect(rx, 'vn_raw_rx/9', 'Term_p4/1');
end
for k = 2:8
    dtc = sprintf('DTC%d', k);
    yk  = 40 + (k - 0.5) * 320 / 9;
    add_block('simulink/Signal Attributes/Data Type Conversion', [rx '/' dtc], ...
        'OutDataTypeStr', 'double', 'Position', [270 yk - 10 330 yk + 10]);
    h = connect(rx, sprintf('vn_raw_rx/%d', k), [dtc '/1']);
    set_param(h, 'Name', names{k});
    connect(rx, [dtc '/1'], sprintf('Mux/%d', k - 1));
end
connect(rx, 'Mux/1', 'ToFile_diag/1');

% ---- CLK ----
qc = [ck '/qpc_clock'];
add_block('simulink/User-Defined Functions/S-Function', qc, 'Position', [60 40 160 80]);
set_param(qc, 'FunctionName', 'qpc_clock', 'Parameters', sprintf('%.17g', p.Ts_clk));
add_sink(ck, 'ToFile_clk', 'clk', [260 45 380 75], logger);
h = connect(ck, 'qpc_clock/1', 'ToFile_clk/1');
set_param(h, 'Name', 'qpc_s');

vn_set_run(mdl, p.subj, p.run, p, true);
fprintf('Added %s (%s logging). Save the model to keep it.\n', top, logger);
end

function add_sink(sys, name, var, pos, logger)
blk = [sys '/' name];
if strcmp(logger, 'quarc')
    add_block('quarc_library/Sinks/To Host/To Host File', blk, 'Position', pos);
    set_param(blk, 'variable_name', var, 'file_format', 'MAT-file v.5', 'decimation', '1', 'sample_time', '-1');
else
    add_block('simulink/Sinks/To File', blk, 'Position', pos);
    set_param(blk, 'MatrixName', var, 'SaveFormat', 'Timeseries');
end
end

function h = connect(sys, a, b)
try
    h = add_line(sys, a, b, 'autorouting', 'on');
catch
    h = add_line(sys, a, b);
end
end
