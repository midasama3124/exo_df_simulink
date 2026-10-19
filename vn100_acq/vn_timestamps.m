function ts = vn_timestamps(pk, clk_file, p)
%VN_TIMESTAMPS  Map VN-100 sensor time to QPC and to the model's t_sim (Milestone 2).
%   ts = vn_timestamps(pk, clk_file)
%   ts = vn_timestamps(pk, clk_file, p)
%
%   1. Segments: split at sensor-time resets (dt < 0) and gaps > p.seg_gap.
%   2. Per segment, lower-envelope fit of receipt QPC against sensor time
%      (receipt delays are >= 0): ordinary least squares, then repeated
%      least squares on the points whose residual is at or below the
%      p.env_pct percentile within p.env_bin-second bins, until the fitted
%      line moves less than p.env_tol over the segment.
%      Result: t_imu_qpc = a + b * t_sensor.
%   3. t_imu_sim = interp1(qpc_clock, t_sim, t_imu_qpc).
%   4. Report per segment: drift (b-1)*1e6 ppm, residual percentiles
%      P5/P50/P95/P99 (latency jitter above the envelope), points used.
%   No constant latency correction is applied: the envelope is the
%   minimum observed receipt delay, which still includes the fixed part of
%   the transport latency.
%
%   Outputs (rows match pk):
%     ts.t_imu_qpc, ts.t_imu_sim, ts.seg_id (0 = not fitted)
%     ts.seg(k): a, b, drift_ppm, res_pct [P5 P50 P95 P99] (s), n, n_used,
%                iterations, converged, idx (first, last row), t_qpc range
%     ts.clk:    step-interval and pacing-lag summary of the 1 kHz loop

if nargin < 3 || isempty(p), p = vn_params_default(); end
tsen = pk.t_sensor(:);
trx  = pk.t_rx_qpc(:);
N    = numel(tsen);

ts = struct();
ts.t_imu_qpc = nan(N, 1);
ts.t_imu_sim = nan(N, 1);
ts.seg_id    = zeros(N, 1);

dt  = diff(tsen);
brk = find(dt < 0 | dt > p.seg_gap);
e   = [0; brk(:); N];
seg = struct('a', {}, 'b', {}, 'drift_ppm', {}, 'res_pct', {}, 'n', {}, 'n_used', {}, ...
    'iterations', {}, 'converged', {}, 'idx', {}, 't_qpc', {}, 'fitted', {});
for s = 1:numel(e) - 1
    idx = (e(s) + 1:e(s + 1)).';
    r = fit_envelope(tsen(idx), trx(idx), p);
    r.idx = [idx(1) idx(end)];
    if r.fitted
        ts.t_imu_qpc(idx) = r.t_fit;
        ts.seg_id(idx)    = s;
        r.t_qpc = [r.t_fit(1) r.t_fit(end)];
    else
        r.t_qpc = [NaN NaN];
    end
    seg(s) = rmfield(r, 't_fit'); %#ok<AGROW>
end
ts.seg = seg;

% ---- map to t_sim ----
[tC, Q] = vn_load_tofile(clk_file);
q    = double(Q(:, 1));
keep = q > cummax([-inf; q(1:end - 1)]);           % strictly increasing
q    = q(keep);
tC   = tC(keep);
ok   = isfinite(ts.t_imu_qpc);
ts.t_imu_sim(ok) = interp1(q, tC, ts.t_imu_qpc(ok), 'linear', 'extrap');
out  = ok & (ts.t_imu_qpc < q(1) - p.map_extrap_max | ts.t_imu_qpc > q(end) + p.map_extrap_max);
ts.t_imu_sim(out) = NaN;
ts.n_outside_clk  = sum(out);

dq  = diff(q);
lag = (q - q(1)) - (tC - tC(1));
ts.clk = struct('n', numel(q), 'dt_mean', mean(dq), 'dt_p99', vn_prctile(dq, 99), ...
    'dt_p999', vn_prctile(dq, 99.9), 'dt_max', max(dq), 'lag_max', max(lag), 'lag_end', lag(end));

% ---- report ----
fprintf('vn_timestamps: %d segment(s)\n', numel(seg));
for s = 1:numel(seg)
    g = seg(s);
    if g.fitted
        if g.converged, cv = ''; else, cv = ' (NOT converged)'; end
        fprintf(['  seg %d: rows %d-%d, n %d, used %d, drift %+.2f ppm, residual P5/P50/P95/P99 = ' ...
            '%.3f/%.3f/%.3f/%.3f ms, %d iterations%s\n'], s, g.idx(1), g.idx(2), g.n, g.n_used, ...
            g.drift_ppm, 1e3 * g.res_pct, g.iterations, cv);
    else
        fprintf('  seg %d: rows %d-%d, n %d, not fitted (fewer than %d points)\n', s, g.idx(1), g.idx(2), g.n, p.seg_min_pts);
    end
end
if ts.n_outside_clk > 0
    fprintf('  %d packets fall outside the qpc_clock range (t_imu_sim = NaN)\n', ts.n_outside_clk);
end
fprintf('  1 kHz loop: step P99 %.3f ms, P99.9 %.3f ms, max %.3f ms; max pacing lag %.2f ms\n', ...
    1e3 * ts.clk.dt_p99, 1e3 * ts.clk.dt_p999, 1e3 * ts.clk.dt_max, 1e3 * ts.clk.lag_max);
end

% =========================================================================
function r = fit_envelope(x_abs, y_abs, p)
n = numel(x_abs);
r = struct('a', NaN, 'b', NaN, 'drift_ppm', NaN, 'res_pct', nan(1, 4), 'n', n, 'n_used', 0, ...
    'iterations', 0, 'converged', false, 'idx', [], 't_qpc', [], 'fitted', false, 't_fit', []);
if n < max(p.seg_min_pts, 3) || x_abs(end) <= x_abs(1)
    return;
end
x0 = x_abs(1);  y0 = y_abs(1);
x  = x_abs - x0;
y  = y_abs - y0;                       % centred for conditioning
A  = [ones(n, 1), x];
c  = A \ y;                            % ordinary least squares
[~, ~, g] = unique(floor(x / p.env_bin));
xr = [0; max(x)];
used = true(n, 1);
for it = 1:p.env_maxit
    res  = y - A * c;
    thr  = accumarray(g, res, [], @(v) vn_prctile(v, p.env_pct));
    used = res <= thr(g);
    cn   = A(used, :) \ y(used);
    chg  = max(abs((cn(1) - c(1)) + (cn(2) - c(2)) * xr));
    c    = cn;
    r.iterations = it;
    if chg < p.env_tol
        r.converged = true;
        break;
    end
end
res = y - A * c;
r.b         = c(2);
r.a         = y0 + c(1) - c(2) * x0;   % t_imu_qpc = a + b * t_sensor
r.drift_ppm = (c(2) - 1) * 1e6;
r.res_pct   = vn_prctile(res, [5 50 95 99]);
r.n_used    = sum(used);
r.fitted    = true;
r.t_fit     = y0 + A * c;
end
