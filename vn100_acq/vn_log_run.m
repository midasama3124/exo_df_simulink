function s = vn_log_run(new, log_dir)
%VN_LOG_RUN  Current '<subj>_<run>' used in the IMU_RAW / motor To Host File names.
%   vn_log_run('S03_EF_blk2')               % set (vn_set_run does this)
%   s = vn_log_run('S01_T01', 'C:\exo_logs') % set from init.m; never overwrite:
%                                           % if S01_T01_*.mat exist there, use S01_T01_2, _3, ...
%   s = vn_log_run()                        % get
%   While vn_bench_test runs (appdata 'vn_log_run_lock' set), calls with a
%   log_dir (from init.m) keep the bench name.
%
%   The To Host File blocks have file names such as
%   C:\exo_logs\%{vn_log_run}_imu_bytes.mat.  QUARC evaluates %{...} in the
%   host MATLAB each time the model starts, so the run name changes without
%   rebuilding the code.  Setting it from the model's init.m (InitFcn) works
%   because Simulink compiles the model, and so runs InitFcn, on every
%   External-mode connect (the checksum check), before the model starts.
%   The value is kept in memory and in vn_current_run.mat next to this
%   file, so it survives a MATLAB restart.  If nothing was ever set, a
%   time-stamped 'unset_...' name is returned so no earlier log is
%   overwritten.

f = fullfile(fileparts(mfilename('fullpath')), 'vn_current_run.mat');
if nargin >= 1
    s = char(new);
    if isempty(regexp(s, '^[A-Za-z0-9_\-]+$', 'once'))
        error('vn_log_run:name', 'Run name ''%s'' may contain only letters, digits, _ and -.', s);
    end
    if nargin >= 2 && ~isempty(getappdata(0, 'vn_log_run_lock'))
        s = getappdata(0, 'vn_log_run');       % vn_bench_test run in progress: keep its name
        fprintf('vn_log_run: bench run in progress, keeping %s (init.m name ignored).\n', s);
        return;
    end
    if nargin >= 2 && ~isempty(log_dir)
        base = s;
        k = 1;
        while ~isempty(dir(fullfile(log_dir, [s '_*.mat'])))
            k = k + 1;
            s = sprintf('%s_%d', base, k);
        end
        if k > 1
            fprintf('vn_log_run: logs for %s already exist; this run is %s.\n', base, s);
        end
        fprintf('vn_log_run: this run logs to %s_*.mat\n', fullfile(log_dir, s));
    end
    setappdata(0, 'vn_log_run', s);
    run_name = s; %#ok<NASGU>
    save(f, 'run_name');
    return;
end
s = getappdata(0, 'vn_log_run');
if isempty(s) && exist(f, 'file')
    S = load(f);
    s = S.run_name;
    setappdata(0, 'vn_log_run', s);
end
if isempty(s)
    s = ['unset_' datestr(now, 'yyyymmdd_HHMMSS')];
    warning('vn_log_run:unset', 'No run name set (use vn_set_run); logging to %s_*.mat.', s);
end
end
