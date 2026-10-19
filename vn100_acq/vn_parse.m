function pk = vn_parse(imu_bytes_file, imu_diag_file, layout, p)
%VN_PARSE  Parse the IMU_RAW logs into VN-100 packets (Milestone 2).
%   pk = vn_parse(imu_bytes_file, imu_diag_file)
%   pk = vn_parse(imu_bytes_file, imu_diag_file, layout, p)
%     layout : struct from vn_make_layout, or a path to vn_layout.mat
%              (default: p.cfg_dir/vn_layout.mat)
%     p      : vn_params_default() with any overrides
%
%   Byte logs: uint8 (To File, variable imu_bytes) or packed 4 bytes per
%   double (QUARC To Host File, variable imu_bytes_p4; vn_unpack_bytes).
%   Calls missing from the log (holes in its time row) are counted in
%   pk.diag.log_dropped_calls.
%
%   Primary path (p.source 'sfun' or auto with 7 diag channels): bytes(1:n)
%   of each call are concatenated in order and every byte carries the QPC
%   receipt time of its call (imu_diag column 2).
%   Fallback path (p.source 'udp' or auto with 1 diag channel): each logged
%   datagram is [double qpc; uint16 n; uint8 bytes(1:n)] and qpc is the
%   receipt time.
%
%   Packet receipt time = receipt time of the call that delivered its last
%   byte.  Outputs (one row per packet):
%     pk.t_sensor   TimeStartup [s]
%     pk.t_rx_qpc   receipt time, QPC [s]
%     pk.t_rx_sim   simulation time of that call [s]
%     pk.call       index of that call in the logs
%     pk.quat       [x y z w] (scalar last, as sent);  pk.quat_wxyz
%     pk.gyro       [rad/s]   pk.accel [m/s^2]
%     pk.ypr [deg], pk.mag [G], pk.temp [degC], pk.pres [kPa]   (if logged)
%     pk.diag       CRC failures, skipped bytes, gaps, duplicates, resets,
%                   calls that hit N_buf, S-function error counters, ...

if nargin < 4 || isempty(p), p = vn_params_default(); end
if nargin < 3 || isempty(layout), layout = fullfile(p.cfg_dir, 'vn_layout.mat'); end
if ischar(layout)
    S = load(layout);
    layout = S.layout;
end

[tB, B, nameB] = vn_load_tofile(imu_bytes_file);
[tD, D] = vn_load_tofile(imu_diag_file);
packed = ~isempty(regexp(nameB, '_p4$', 'once'));   % To Host File log of vn_raw_rx port 9
if packed
    nb = p.N_buf;
    if nb > 4 * size(B, 2) || nb <= 4 * (size(B, 2) - 1)
        nb = 4 * size(B, 2);
        warning('vn_parse:nbuf', 'p.N_buf = %d does not match the packed log width %d; using %d bytes per call.', ...
            p.N_buf, size(B, 2), nb);
    end
    B = vn_unpack_bytes(B, nb);
else
    B = uint8(B);
end
D = double(D);
log_dropped = dropped_calls(tD);
if log_dropped > 0
    warning('vn_parse:dropped', ['%d vn_raw_rx calls are missing from the log (time row has holes); ' ...
        'their bytes are lost and show up as skipped bytes / gaps.'], log_dropped);
end
K = min(size(B, 1), size(D, 1));
if size(B, 1) ~= size(D, 1)
    warning('vn_parse:len', 'imu_bytes has %d samples, imu_diag %d; using the first %d.', size(B, 1), size(D, 1), K);
end
B = B(1:K, :);  D = D(1:K, :);  tB = tB(1:K);
if max(abs(tB - tD(1:K))) > 1e-9
    warning('vn_parse:time', 'imu_bytes and imu_diag time vectors differ.');
end

src = lower(p.source);
if strcmp(src, 'auto')
    if size(D, 2) >= 7, src = 'sfun'; elseif size(D, 2) == 1, src = 'udp';
    else, error('vn_parse:src', 'imu_diag has %d channels; set p.source.', size(D, 2)); end
end

switch src
    case 'sfun'
        n      = D(:, 1);
        t_call = D(:, 2);
        nbuf   = size(B, 2);
        tcall_sim = tB;
    case 'udp'
        len  = D(:, 1);
        rows = find(len >= 10);
        Q    = B(rows, 1:8).';
        t_call = typecast(Q(:), 'double');
        N2   = B(rows, 9:10).';
        n    = double(typecast(N2(:), 'uint16'));
        if any(n > len(rows) - 10)
            error('vn_parse:udp', 'Datagram length field exceeds the received length.');
        end
        B    = B(rows, 11:end);
        nbuf = p.fwd_max_n;
        tcall_sim = tB(rows);
        D    = D(rows, :);
    otherwise
        error('vn_parse:src', 'Unknown source ''%s''.', src);
end
if any(n < 0 | n > size(B, 2) | n ~= round(n))
    error('vn_parse:n', 'Byte counts outside 0..%d in the logs.', size(B, 2));
end

% ---- 1. byte stream ----
Bt     = B.';
mask   = bsxfun(@le, (1:size(Bt, 1)).', n(:).');
stream = Bt(mask);
clear Bt mask
kc   = find(n > 0);                      % calls that delivered bytes
cumN = cumsum(n(kc));

% ---- 2-3. packet scan with CRC ----
[st, sd] = vn_scan_packets(stream, layout);

% ---- 5. receipt time = call that delivered the last byte ----
e      = st + layout.packet_len - 1;
pos    = discretize(e, [0.5; cumN + 0.5]);
k_call = kc(pos);

% ---- 4. decode ----
F = vn_decode_packets(stream, st, layout);
if ~isfield(F, 'time_startup')
    error('vn_parse:time', 'TimeStartup (bit 0) must be in the field mask.');
end
pk = struct();
pk.t_sensor = F.time_startup * 1e-9;
pk.t_rx_qpc = t_call(k_call);
pk.t_rx_sim = tcall_sim(k_call);
pk.call     = k_call;
if isfield(F, 'quat'),  pk.quat = F.quat;  pk.quat_wxyz = F.quat(:, [4 1 2 3]); end
if isfield(F, 'gyro'),  pk.gyro  = F.gyro;  end
if isfield(F, 'accel'), pk.accel = F.accel; end
if isfield(F, 'ypr'),   pk.ypr   = F.ypr;   end
if isfield(F, 'magpres')
    pk.mag  = F.magpres(:, 1:3);
    pk.temp = F.magpres(:, 4);
    pk.pres = F.magpres(:, 5);
end

% ---- 6. diagnostics ----
dt = diff(pk.t_sensor);
dg = sd;
dg.source         = src;
dg.packed_log     = packed;
dg.log_dropped_calls = log_dropped;
dg.calls          = numel(n);
dg.calls_hit_nbuf = sum(n == nbuf);
dg.gaps           = sum(dt > p.gap_thr);
dg.gap_idx        = find(dt > p.gap_thr);
dg.gap_missing    = sum(max(0, round(dt(dt > p.gap_thr) * p.rate) - 1));
dg.duplicates     = sum(dt >= 0 & dt < p.dup_thr);
dg.resets         = sum(dt < 0);
dg.dt_median      = median(dt);
if strcmp(src, 'sfun')
    dg.err_calls      = D(end, 5);
    dg.err_masks      = unique(D(D(:, 6) ~= 0, 6)).';
    dg.max_queued     = max(D(:, 3));
    dg.cum_bytes_ok   = D(end, 4) == mod(sum(n), 2^32);
    dg.dur_us_max     = max(D(:, 7));
    dg.dur_us_mean    = mean(D(:, 7));
    dg.dur_us_p99     = vn_prctile(D(:, 7), 99);
end
pk.diag   = dg;
pk.layout = layout;
pk.files  = {imu_bytes_file, imu_diag_file};

fprintf('vn_parse: %d packets from %d bytes in %d calls (%s) | CRC fail %d | skipped %d B | trailing %d B\n', ...
    dg.n_packets, dg.n_bytes, dg.calls, src, dg.crc_fail, dg.bytes_skipped, dg.bytes_trailing);
fprintf('          gaps %d (%d packets missing) | duplicates %d | resets %d | calls at N_buf %d\n', ...
    dg.gaps, dg.gap_missing, dg.duplicates, dg.resets, dg.calls_hit_nbuf);
if strcmp(src, 'sfun')
    fprintf('          S-fcn error calls %d | max queued %d B | call time max %.1f us, mean %.1f us | calls missing from log %d\n', ...
        dg.err_calls, dg.max_queued, dg.dur_us_max, dg.dur_us_mean, dg.log_dropped_calls);
end
end

function m = dropped_calls(t)
% calls missing from a log whose time row should advance by one sample time
if numel(t) < 3, m = 0; return; end
dt = diff(t(:));
T  = median(dt);
if ~(T > 0), m = 0; return; end
m = sum(max(0, round(dt / T) - 1));
end
