function tq = gait_refine_min(t, w, idx)
%GAIT_REFINE_MIN  Sub-sample time of troughs (parabolic vertex through 3 samples).
%   tq = gait_refine_min(t, w, idx): t(idx) moved by at most half a sample
%   towards the vertex of the parabola through w(idx-1:idx+1).
t   = t(:);
w   = w(:);
idx = idx(:);
tq  = t(idx);
dt  = median(diff(t));
for k = 1:numel(idx)
    i = idx(k);
    if i <= 1 || i >= numel(w), continue; end
    y0 = w(i - 1); y1 = w(i); y2 = w(i + 1);
    den = y0 - 2 * y1 + y2;
    if den > 0
        tq(k) = t(i) + max(-0.5, min(0.5, 0.5 * (y0 - y2) / den)) * dt;
    end
end
end
