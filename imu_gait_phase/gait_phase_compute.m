function [phase, stride, okStride, T] = gait_phase_compute(hsT, tq, strideRange)
%GAIT_PHASE_COMPUTE  Continuous gait phase from heel-strike times.
%   [phase, stride, okStride, T] = gait_phase_compute(hsT, tq, strideRange)
%     hsT          heel-strike times [s]
%     tq           query times [s] (e.g. the IMU or the motor time axis)
%     strideRange  [lo hi]: a stride is valid if lo*median(T) <= T <= hi*median(T)
%   phase(t) = 100 * (t - HS_k) / (HS_k+1 - HS_k)  [%], 0 at a heel strike and
%   rising linearly to 100 at the next one; NaN before the first and after the
%   last heel strike and inside invalid strides.  stride = k (NaN where phase is).
%   okStride and T (stride times [s]) have one entry per HS interval.

hsT = unique(hsT(isfinite(hsT)));
hsT = hsT(:);
T   = diff(hsT);
okStride = false(size(T));
if ~isempty(T)
    Tmed = median(T);
    okStride = T >= strideRange(1) * Tmed & T <= strideRange(2) * Tmed;
end
sz     = size(tq);
tq     = tq(:);
phase  = nan(numel(tq), 1);
stride = nan(numel(tq), 1);
if numel(hsT) >= 2 && ~isempty(tq)
    k  = discretize(tq, hsT);                % hsT(k) <= t < hsT(k+1)
    in = ~isnan(k);
    in(in) = okStride(k(in));
    phase(in)  = 100 * (tq(in) - hsT(k(in))) ./ T(k(in));
    stride(in) = k(in);
    out = phase >= 100;                      % t equal to the last heel strike
    phase(out)  = NaN;
    stride(out) = NaN;
end
phase  = reshape(phase, sz);
stride = reshape(stride, sz);
end
