function ok = vn_selftest(outdir)
%VN_SELFTEST  Hardware-free test of vn_parse, vn_timestamps and vn_gait_events.
%   ok = vn_selftest           % writes logs to tempdir/vn_selftest
%   ok = vn_selftest(outdir)
%
%   Builds a synthetic 180 s VN-100 stream ('base', 400 Hz, sensor clock
%   +40 ppm, 20 s standing then walking with T ~ 1.1 s), passes it through
%   a model of the USB link (1 ms FTDI latency, 5 bursts held for 16 ms, 5 %
%   of packets delivered in two pieces) and of vn_raw_rx (one call per
%   2 ms, at most 128 bytes, pacing jitter and 3 ms hiccups), injects 3
%   corrupted packets and 37 garbage bytes, writes To-File-style
%   timeseries logs, then runs the offline pipeline and checks the result.
%   The same calls are also written as QUARC To Host File logs (MAT v5,
%   [t; data] per column, bytes packed 4 per double as on vn_raw_rx port 9)
%   and must give identical packets and timestamps; a copy with 3 calls
%   removed checks the missing-call count.

if nargin < 1 || isempty(outdir), outdir = fullfile(tempdir, 'vn_selftest'); end
if ~exist(outdir, 'dir'), mkdir(outdir); end
rng(1);
p = vn_params_default();
p.log_dir = outdir;
L  = vn_make_layout('base', p.rate, p.baud);
P  = L.packet_len;
fs = p.rate;
Tdur = 180;
N  = Tdur * fs;
tsen  = 5 + (0:N - 1).' / fs;              % sensor TimeStartup [s]
drift = 40e-6;
q0    = 1000;                               % QPC [s] at the first sample

% ---- gait-like sagittal angular rate ----
hs = 25; Ts = zeros(0, 1);
while hs(end) < tsen(end) - 2
    Ti = 1.1 * (1 + 0.03 * randn);
    Ts(end + 1, 1) = Ti; %#ok<AGROW>
    hs(end + 1, 1) = hs(end) + Ti; %#ok<AGROW>
end
G = @(c, s) exp(-0.5 * ((tsen - c) / s).^2);
w = 0.05 * randn(N, 1);
hs_true = zeros(numel(Ts), 1);
to_true = zeros(numel(Ts), 1);
for i = 1:numel(Ts)
    h = hs(i); Ti = Ts(i);
    w = w + 7.0 * G(h + 0.80 * Ti, 0.06 * Ti) - 3.5 * G(h + 0.62 * Ti, 0.035 * Ti) ...
          - 2.5 * G(hs(i + 1) + 0.02 * Ti, 0.025 * Ti);
    to_true(i) = h + 0.62 * Ti;
    hs_true(i) = hs(i + 1) + 0.02 * Ti;
end
gyro = [0.15 * w + 0.02 * randn(N, 1), w, -0.1 * w];
acc  = [zeros(N, 2), -9.81 * ones(N, 1)] + 0.1 * randn(N, 3);
quat = repmat([0 0 0 1], N, 1);

% ---- packets ----
pkb = zeros(N, P, 'uint8');
pkb(:, 1:4)   = repmat(L.header, N, 1);
pkb(:, 5:12)  = le_bytes(uint64(round(tsen * 1e9)), 8);
pkb(:, 13:28) = le_bytes(single(quat), 16);
pkb(:, 29:40) = le_bytes(single(gyro), 12);
pkb(:, 41:52) = le_bytes(single(acc), 12);
crc = crc_rows(pkb(:, 2:52));
pkb(:, 53) = uint8(bitshift(crc, -8));
pkb(:, 54) = uint8(bitand(crc, 255));
bad = [10001; 30002; 55556];
pkb(bad, 31) = bitxor(pkb(bad, 31), uint8(90));
ig = 40001;                                 % 37 garbage bytes after this packet
garbage = uint8(randi([0 255], 37, 1));
pk_rows = pkb.';
stream = [reshape(pk_rows(:, 1:ig), [], 1); garbage; reshape(pk_rows(:, ig + 1:end), [], 1)];

% ---- USB link ----
qtx = q0 + (1 + drift) * (tsen - tsen(1)) + 0.8e-3;     % last byte leaves the sensor
arrival = ceil(qtx / 1e-3) * 1e-3 + 0.25e-3 * rand(N, 1);
tb = q0 + 30 + 140 * rand(5, 1);
for k = 1:5
    m = arrival >= tb(k) & arrival < tb(k) + 0.016;
    arrival(m) = tb(k) + 0.016;
end
pend = (1:N).' * P;
pend(ig + 1:end) = pend(ig + 1:end) + 37;
split = rand(N, 1) < 0.05;
ce = [pend; pend(split) - P + 27; pend(ig) + 37];
ct = [arrival; arrival(split) - 0.5e-3; arrival(ig) + 1e-4];
[ce, o] = sort(ce);
ct = cummax(ct(o));

% ---- Simulink side: qpc_clock at 1 kHz, vn_raw_rx every Ts_rx ----
Kc   = round((Tdur + 1) / p.Ts_clk);
tclk = (0:Kc - 1).' * p.Ts_clk;
jit  = abs(5e-5 * randn(Kc, 1));
hk   = randi(Kc, 30, 1);
jit(hk) = jit(hk) + 3e-3;
qclk = cummax(q0 - 0.3 + tclk + jit) + (0:Kc - 1).' * 1e-12;
r     = round(p.Ts_rx / p.Ts_clk);
tcall = tclk(1:r:end);
qcall = qclk(1:r:end) + 3e-6;
K     = numel(tcall);
Nb    = p.N_buf;
B      = zeros(K, Nb, 'uint8');
nread  = zeros(K, 1);
queued = zeros(K, 1);
ic = 0; nc = numel(ct); avail = 0; rd = 0;
for k = 1:K
    while ic < nc && ct(ic + 1) <= qcall(k)
        ic = ic + 1;
        avail = ce(ic);
    end
    n = min(avail - rd, Nb);
    if n > 0
        B(k, 1:n) = stream(rd + 1:rd + n);
        rd = rd + n;
    end
    nread(k)  = n;
    queued(k) = avail - rd;
end
D = [nread, qcall + 1e-6, queued, cumsum(nread), zeros(K, 2), 5 + 3 * rand(K, 1)];

f = vn_run_files('selftest', 'R01', p);
imu_bytes = timeseries(B, tcall); %#ok<NASGU>
imu_diag  = timeseries(D, tcall); %#ok<NASGU>
clk       = timeseries(qclk, tclk); %#ok<NASGU>
save(f.bytes, 'imu_bytes', '-v7.3');
save(f.diag,  'imu_diag',  '-v7.3');
save(f.clk,   'clk',       '-v7.3');
fprintf('vn_selftest: synthetic logs in %s (%d calls, max queued %d B)\n\n', outdir, K, max(queued));

% ---- run the pipeline ----
pk = vn_parse(f.bytes, f.diag, L, p);
ts = vn_timestamps(pk, f.clk, p);
ev = vn_gait_events(pk, ts, p);

keep = setdiff((1:N).', bad);
dg = pk.diag;
C = {};
C(end + 1, :) = {'packets decoded',        dg.n_packets == N - 3,        sprintf('%d (expect %d)', dg.n_packets, N - 3)};
C(end + 1, :) = {'CRC failures',           dg.crc_fail == 3,             sprintf('%d (expect 3)', dg.crc_fail)};
C(end + 1, :) = {'bytes skipped',          dg.bytes_skipped == 3 * P + 37, sprintf('%d (expect %d)', dg.bytes_skipped, 3 * P + 37)};
C(end + 1, :) = {'gaps / dup / resets',    dg.gaps == 3 && dg.duplicates == 0 && dg.resets == 0, ...
    sprintf('%d / %d / %d (expect 3 / 0 / 0)', dg.gaps, dg.duplicates, dg.resets)};
C(end + 1, :) = {'decoded values exact',   isequal(pk.gyro, double(single(gyro(keep, :)))) && ...
    isequal(pk.accel, double(single(acc(keep, :)))) && max(abs(pk.t_sensor - tsen(keep))) < 1e-9, ''};
C(end + 1, :) = {'receipt after transmit', all(pk.t_rx_qpc >= qtx(keep)), ''};
dppm = ts.seg(1).drift_ppm;
C(end + 1, :) = {'drift estimate',         numel(ts.seg) == 1 && abs(dppm - 40) < 1, sprintf('%+.2f ppm (true +40)', dppm)};
e_env = ts.t_imu_qpc - qtx(keep);
C(end + 1, :) = {'envelope vs transmit',   std(e_env) < 50e-6, sprintf('offset %.3f ms, std %.1f us', 1e3 * mean(e_env), 1e6 * std(e_env))};
C(end + 1, :) = {'t_imu_sim finite',       all(isfinite(ts.t_imu_sim)), ''};
ht = hs_true(hs_true < tsen(end) - 0.5);
dh = arrayfun(@(t) min(abs(ev.t_hs(ev.valid) - t)), ht);
dt = arrayfun(@(t) min(abs(ev.t_to(ev.valid) - t)), to_true);
C(end + 1, :) = {'HS detected',            sum(ev.valid) >= 0.95 * numel(Ts) && max(dh) < 10e-3, ...
    sprintf('%d valid of %d strides, error median %.2f ms, max %.2f ms', sum(ev.valid), numel(Ts), 1e3 * median(dh), 1e3 * max(dh))};
C(end + 1, :) = {'TO detected',            max(dt) < 10e-3, sprintf('error median %.2f ms, max %.2f ms', 1e3 * median(dt), 1e3 * max(dt))};
C(end + 1, :) = {'no events while standing', ~any(ev.t_ms < 24), sprintf('%d', sum(ev.t_ms < 24))};
phi = vn_phase_reference(ev, tclk, p);
walk = tclk > 30 & tclk < Tdur - 5;
C(end + 1, :) = {'reference phase',        all(phi(isfinite(phi)) >= 0 & phi(isfinite(phi)) < 1) && mean(isfinite(phi(walk))) > 0.9, ...
    sprintf('defined on %.1f %% of the walking time', 100 * mean(isfinite(phi(walk))))};
C(end + 1, :) = {'no calls missing (R01)', dg.log_dropped_calls == 0, sprintf('%d', dg.log_dropped_calls)};

% ---- QUARC To Host File format (p.logger 'quarc') ----
R  = uint8(randi([0 255], 50, 130));
C(end + 1, :) = {'pack/unpack round trip', isequal(vn_unpack_bytes(pack4(R), 130), R), '130 B per call'};
f2 = vn_run_files('selftest', 'R02', p);
write_host(f2, tcall, pack4(B), D, tclk, qclk, []);
fprintf('\n--- same calls, QUARC To Host File format ---\n');
pk2 = vn_parse(f2.bytes, f2.diag, L, p);
ts2 = vn_timestamps(pk2, f2.clk, p);
same = pk2.diag.packed_log && isequal(pk2.t_sensor, pk.t_sensor) && isequal(pk2.gyro, pk.gyro) && ...
    isequal(pk2.accel, pk.accel) && isequal(pk2.t_rx_qpc, pk.t_rx_qpc) && isequal(pk2.t_rx_sim, pk.t_rx_sim) && ...
    isequal(pk2.diag.crc_fail, dg.crc_fail) && max(abs(ts2.t_imu_sim - ts.t_imu_sim)) < 1e-12;
C(end + 1, :) = {'QUARC format identical', same, sprintf('%d packets', pk2.diag.n_packets)};
f3 = vn_run_files('selftest', 'R03', p);
write_host(f3, tcall, pack4(B), D, tclk, qclk, 50001:50003);
ws = warning('off', 'vn_parse:dropped');
pk3 = vn_parse(f3.bytes, f3.diag, L, p);
warning(ws);
C(end + 1, :) = {'missing calls counted',  pk3.diag.log_dropped_calls == 3, sprintf('%d (expect 3)', pk3.diag.log_dropped_calls)};

fprintf('\n%-26s %-6s %s\n', 'check', 'result', 'detail');
for k = 1:size(C, 1)
    if C{k, 2}, rs = 'ok'; else, rs = 'FAIL'; end
    fprintf('%-26s %-6s %s\n', C{k, 1}, rs, C{k, 3});
end
ok = all([C{:, 2}]);
if ok, fprintf('\nvn_selftest PASS\n'); else, fprintf('\nvn_selftest FAIL\n'); end
end

function X = pack4(B)
% same packing as vn_raw_rx port 9: 4 bytes per double, little endian, zero tail
W  = ceil(size(B, 2) / 4);
Bp = zeros(size(B, 1), 4 * W);
Bp(:, 1:size(B, 2)) = double(B);
X  = Bp(:, 1:4:end) + 256 * Bp(:, 2:4:end) + 65536 * Bp(:, 3:4:end) + 16777216 * Bp(:, 4:4:end);
end

function write_host(f, tcall, X, D, tclk, qclk, drop)
% To Host File, MAT-file v.5: one variable [t; data], one column per sample
keep = true(numel(tcall), 1);
keep(drop) = false;
imu_bytes_p4 = [tcall(keep).'; X(keep, :).']; %#ok<NASGU>
imu_diag     = [tcall(keep).'; D(keep, :).']; %#ok<NASGU>
clk          = [tclk.'; qclk.'];               %#ok<NASGU>
save(f.bytes, 'imu_bytes_p4', '-v6');
save(f.diag,  'imu_diag',     '-v6');
save(f.clk,   'clk',          '-v6');
end

function M = le_bytes(X, nb)
v = typecast(reshape(X.', [], 1), 'uint8');
M = reshape(v, nb, []).';
end

function crc = crc_rows(M)
[~, T] = vn_crc16([]);
crc = zeros(size(M, 1), 1, 'uint16');
for j = 1:size(M, 2)
    crc = bitxor(bitshift(crc, 8), T(double(bitxor(bitshift(crc, -8), uint16(M(:, j)))) + 1));
end
end
