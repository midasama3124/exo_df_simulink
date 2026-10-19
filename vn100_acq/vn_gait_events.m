function ev = vn_gait_events(pk, ts, p)
%VN_GAIT_EVENTS  Heel strike (HS) and toe-off (TO) from the shoe VN-100 (offline).
%   ev = vn_gait_events(pk, ts)
%   ev = vn_gait_events(pk, ts, p)
%     pk : vn_parse output     ts : vn_timestamps output
%
%   Optional milestone 3 (the prompt names gait events and lists their
%   parameters, but its method section is missing; see README).
%   Per fitted timestamp segment:
%     1. Sagittal angular rate w = gyro * axis, axis = principal axis of
%        the gyro (p.ev_axis = 0) or a chosen IMU axis; sign chosen so the
%        mid-swing (MS) peak is positive (p.ev_sign = 0).
%     2. Resample to a uniform p.rate grid in sensor time, zero-phase
%        low-pass at p.f_lp_events (15 Hz).
%     3. MS = local maxima above ms_thr = p.ms_thr_frac x median candidate
%        peak (and above p.ev_min_ms), at least p.ev_min_peak_dist x T apart.
%     4. TO = minimum of w in [MS - p.to_win*T, MS); HS = minimum in
%        (MS, MS + p.hs_win*T]; T = neighbouring MS-to-MS time (median T
%        if out of range).  Parabolic sub-sample refinement.
%     5. Optional HS refinement (p.acc_ref): first sample within
%        +/- p.acc_ref_win of the gyro HS where |acc| > p.acc_ref_thr g.
%     6. Add p.hs_bias / p.to_bias; map sensor time to QPC and t_sim with
%        the segment fits of vn_timestamps.
%   A minimum on the edge of its window (no trough inside) or a stride time
%   outside p.ev_T_range x median marks the event invalid.
%
%   ev fields (one entry per MS): t_ms, t_to, t_hs (sensor s), *_qpc, *_sim,
%   w_ms (rad/s), T_prev, T_next, seg, valid_to, valid_hs, valid, hs_acc.

if nargin < 3 || isempty(p), p = vn_params_default(); end
fs   = p.rate;
g0   = 9.80665;
tsen = pk.t_sensor(:);
gyro = double(pk.gyro);
acc  = double(pk.accel);
sid  = ts.seg_id(:);

% ---- 1. sagittal axis and sign ----
fin = all(isfinite(gyro), 2) & sid > 0;
if p.ev_axis == 0
    [V, Dg] = eig(cov(gyro(fin, :)));
    [~, im] = max(diag(Dg));
    ax = V(:, im);
else
    ax = zeros(3, 1);
    ax(p.ev_axis) = 1;
end
w_all = gyro * ax;
if p.ev_sign == 0
    sg = 1;
    if -vn_prctile(w_all(fin), 0.5) > vn_prctile(w_all(fin), 99.5), sg = -1; end
else
    sg = sign(p.ev_sign);
end
ax    = sg * ax;
w_all = sg * w_all;

E = init_events();
segs = unique(sid(sid > 0)).';
seginfo = struct('seg', {}, 'ms_thr', {}, 'T_med', {}, 'n_ms', {});
for s = segs
    ii = find(sid == s);
    if numel(ii) < 4 * fs, continue; end
    [t0, iu] = unique(tsen(ii));
    tu = (t0(1):1/fs:t0(end)).';
    wu = interp1(t0, w_all(ii(iu)), tu, 'linear');
    au = interp1(t0, sqrt(sum(acc(ii(iu), :).^2, 2)), tu, 'linear');
    wf = vn_lp_zerophase(wu, p.f_lp_events, fs);
    n  = numel(tu);

    % ---- 3. mid-swing peaks ----
    lm = local_max(wf);
    if isempty(lm), continue; end
    thr0 = max(p.ev_min_ms, 0.5 * vn_prctile(wf, 99));
    c = lm(wf(lm) > thr0);
    if numel(c) < 3, continue; end
    c = c(nms_keep(tu(c), wf(c), p.ev_init_min_dist));
    if numel(c) < 3, continue; end
    ms_thr = max(p.ev_min_ms, p.ms_thr_frac * median(wf(c)));
    T_med  = median(diff(tu(c)));
    ms = lm(wf(lm) > ms_thr);
    ms = ms(nms_keep(tu(ms), wf(ms), p.ev_min_peak_dist * T_med));
    if numel(ms) < 3, continue; end
    T_med = median(diff(tu(ms)));
    seginfo(end + 1) = struct('seg', s, 'ms_thr', ms_thr, 'T_med', T_med, 'n_ms', numel(ms)); %#ok<AGROW>
    inr = @(T) T >= p.ev_T_range(1) * T_med & T <= p.ev_T_range(2) * T_med;

    % ---- 4-5. TO and HS around each MS ----
    nm = numel(ms);
    for k = 1:nm
        tk = tu(ms(k));
        Tp = NaN; if k > 1,  Tp = tk - tu(ms(k - 1)); end
        Tn = NaN; if k < nm, Tn = tu(ms(k + 1)) - tk; end
        Tpu = Tp; if ~(inr(Tp)), Tpu = T_med; end
        Tnu = Tn; if ~(inr(Tn)), Tnu = T_med; end

        r = max(1, ms(k) - round(p.to_win * Tpu * fs)):ms(k) - 1;
        [t_to, edge_to] = window_min(tu, wf, r);
        r = ms(k) + 1:min(n, ms(k) + round(p.hs_win * Tnu * fs));
        [t_hs, edge_hs] = window_min(tu, wf, r);

        hs_acc = false;
        if p.acc_ref && isfinite(t_hs)
            ra = find(tu >= t_hs - p.acc_ref_win & tu <= t_hs + p.acc_ref_win);
            ja = find(au(ra) > p.acc_ref_thr * g0, 1, 'first');
            if ~isempty(ja), t_hs = tu(ra(ja)); hs_acc = true; end
        end

        E.seg(end + 1, 1)      = s;
        E.t_ms(end + 1, 1)     = tk;
        E.w_ms(end + 1, 1)     = wf(ms(k));
        E.t_to(end + 1, 1)     = t_to + p.to_bias;
        E.t_hs(end + 1, 1)     = t_hs + p.hs_bias;
        E.T_prev(end + 1, 1)   = Tp;
        E.T_next(end + 1, 1)   = Tn;
        E.valid_to(end + 1, 1) = ~edge_to & (isnan(Tp) | inr(Tp));
        E.valid_hs(end + 1, 1) = ~edge_hs & (isnan(Tn) | inr(Tn));
        E.hs_acc(end + 1, 1)   = hs_acc;
    end
end
E.valid = E.valid_to & E.valid_hs;

% ---- 6. map to QPC and t_sim ----
E.t_ms_qpc = map_time(E.t_ms, E.seg, tsen, sid, ts.t_imu_qpc);
E.t_to_qpc = map_time(E.t_to, E.seg, tsen, sid, ts.t_imu_qpc);
E.t_hs_qpc = map_time(E.t_hs, E.seg, tsen, sid, ts.t_imu_qpc);
E.t_ms_sim = map_time(E.t_ms, E.seg, tsen, sid, ts.t_imu_sim);
E.t_to_sim = map_time(E.t_to, E.seg, tsen, sid, ts.t_imu_sim);
E.t_hs_sim = map_time(E.t_hs, E.seg, tsen, sid, ts.t_imu_sim);

ev = E;
ev.axis    = ax;
ev.seginfo = seginfo;
ev.params  = p;
nv = sum(ev.valid);
if isempty(seginfo), Tm = NaN; else, Tm = median([seginfo.T_med]); end
fprintf('vn_gait_events: %d mid-swings, %d with valid TO and HS; median stride %.3f s; axis [%.2f %.2f %.2f]\n', ...
    numel(ev.t_ms), nv, Tm, ax);
end

% =========================================================================
function E = init_events()
z = zeros(0, 1);
E = struct('seg', z, 't_ms', z, 'w_ms', z, 't_to', z, 't_hs', z, 'T_prev', z, 'T_next', z, ...
    'valid_to', false(0, 1), 'valid_hs', false(0, 1), 'hs_acc', false(0, 1));
end

function i = local_max(x)
x = x(:);
i = find([false; x(2:end) > x(1:end - 1)] & [x(1:end - 1) >= x(2:end); false]);
end

function keep = nms_keep(t, v, dmin)
% keep the highest peaks first; drop any peak closer than dmin to a kept one
n = numel(t);
keep = false(n, 1);
[~, ord] = sort(v, 'descend');
kt = zeros(n, 1);
nk = 0;
for jj = 1:n
    j = ord(jj);
    if nk == 0 || all(abs(kt(1:nk) - t(j)) >= dmin)
        nk = nk + 1;
        kt(nk) = t(j);
        keep(j) = true;
    end
end
end

function [tq, edge] = window_min(t, v, r)
if isempty(r)
    tq = NaN; edge = true;
    return;
end
[~, j] = min(v(r));
i    = r(j);
edge = (j == 1) || (j == numel(r));
tq   = t(i);
if i > 1 && i < numel(v)                     % parabolic vertex
    y0 = v(i - 1); y1 = v(i); y2 = v(i + 1);
    den = y0 - 2 * y1 + y2;
    if den > 0
        d  = max(-0.5, min(0.5, 0.5 * (y0 - y2) / den));
        tq = t(i) + d * (t(2) - t(1));
    end
end
end

function out = map_time(tev, sev, tsen, sid, tmap)
out = nan(size(tev));
for s = unique(sev(:)).'
    ii = find(sid == s & isfinite(tmap));
    if numel(ii) < 2, continue; end
    [tu, iu] = unique(tsen(ii));
    m = sev == s & isfinite(tev);
    out(m) = interp1(tu, tmap(ii(iu)), tev(m), 'linear', 'extrap');
end
end
