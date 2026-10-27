function [hs, ms, info] = gait_hs_detect(t, w, opt)
%GAIT_HS_DETECT  Automatic heel strikes from a foot angular velocity (no toolboxes).
%   [hs, ms, info] = gait_hs_detect(t, w, opt)
%     t    [n x 1] time [s]      w  [n x 1] reference signal (gait_reference_signal:
%                                   mid-swing positive, troughs negative)
%     opt  .msMin       rad/s, absolute floor on mid-swing peaks (rejects standing)
%          .msThrFrac   mid-swing threshold = msThrFrac x median candidate peak
%          .msInitDist  s, minimum spacing of candidate peaks before the stride time T is known
%          .msMinDist   minimum mid-swing spacing, fraction of the median T
%          .hsWin       heel-strike search window after mid-swing, fraction of T
%          .closeFrac   heel strikes closer than closeFrac x median interval: keep the deeper one
%   hs   sample indices of the heel strikes (minimum of w after each mid-swing)
%   ms   sample indices of the mid-swing peaks
%   info .T_med (s), .ms_thr (rad/s), .n_edge (mid-swings without a trough in the window)
%   Same steps as vn_gait_events, on sample indices instead of sensor time, plus
%   the close-pair removal of the earlier FSR/IMU script.

t  = t(:);
w  = w(:);
n  = numel(w);
fs = 1 / median(diff(t));
hs = zeros(0, 1);
ms = zeros(0, 1);
info = struct('T_med', NaN, 'ms_thr', NaN, 'n_edge', 0);

% ---- mid-swing peaks ----
lm = local_max(w);
if isempty(lm), return; end
thr0 = max(opt.msMin, 0.5 * vn_prctile(w, 99));
c = lm(w(lm) > thr0);
c = c(nms_keep(t(c), w(c), opt.msInitDist));
if numel(c) < 3, return; end
ms_thr = max(opt.msMin, opt.msThrFrac * median(w(c)));
T_med  = median(diff(t(c)));
ms = lm(w(lm) > ms_thr);
ms = ms(nms_keep(t(ms), w(ms), opt.msMinDist * T_med));
if numel(ms) < 3, ms = zeros(0, 1); return; end
T_med = median(diff(t(ms)));
info.T_med  = T_med;
info.ms_thr = ms_thr;

% ---- heel strike = deepest trough after each mid-swing ----
nw   = round(opt.hsWin * T_med * fs);
keep = false(numel(ms), 1);
hs   = zeros(numel(ms), 1);
for k = 1:numel(ms)
    last = min(n, ms(k) + nw);
    if k < numel(ms), last = min(last, ms(k + 1) - 1); end
    r = ms(k) + 1:last;
    if numel(r) < 3, continue; end
    [~, j] = min(w(r));
    if j == 1 || j == numel(r)
        info.n_edge = info.n_edge + 1;        % no trough inside the window
        continue;
    end
    hs(k)   = r(j);
    keep(k) = true;
end
hs = hs(keep);

% ---- close pairs: keep the deeper trough ----
while numel(hs) >= 3
    d = diff(hs);
    j = find(d < opt.closeFrac * median(d), 1, 'first');
    if isempty(j), break; end
    if w(hs(j)) <= w(hs(j + 1)), hs(j + 1) = []; else, hs(j) = []; end
end
end

% =========================================================================
function i = local_max(x)
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
