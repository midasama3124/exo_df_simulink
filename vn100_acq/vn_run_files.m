function f = vn_run_files(subj, run, p)
%VN_RUN_FILES  Log file names of one run: <subj>_<run>_imu_bytes.mat,
%   <subj>_<run>_imu_diag.mat and <subj>_<run>_clk.mat in p.log_dir.
if nargin < 3 || isempty(p), p = vn_params_default(); end
base   = sprintf('%s_%s', subj, run);
f.bytes = fullfile(p.log_dir, [base '_imu_bytes.mat']);
f.diag  = fullfile(p.log_dir, [base '_imu_diag.mat']);
f.clk   = fullfile(p.log_dir, [base '_clk.mat']);
end
