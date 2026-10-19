function s = vn_log_run(new)
%VN_LOG_RUN  Current '<subj>_<run>' used in the IMU_RAW To Host File names.
%   vn_log_run('S03_EF_blk2')   % set (vn_set_run does this)
%   s = vn_log_run()            % get
%
%   The To Host File blocks of IMU_RAW have file names such as
%   C:\exo_logs\%{vn_log_run}_imu_bytes.mat.  QUARC evaluates %{...} in the
%   host MATLAB each time the model starts, so the run name changes without
%   rebuilding the code.  The value is kept in memory and in
%   vn_current_run.mat next to this file, so it survives a MATLAB restart.
%   If nothing was ever set, a time-stamped 'unset_...' name is returned so
%   no earlier log is overwritten.

f = fullfile(fileparts(mfilename('fullpath')), 'vn_current_run.mat');
if nargin >= 1
    s = char(new);
    if isempty(regexp(s, '^[A-Za-z0-9_\-]+$', 'once'))
        error('vn_log_run:name', 'Run name ''%s'' may contain only letters, digits, _ and -.', s);
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
