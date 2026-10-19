function y = vn_lp_zerophase(x, fc, fs)
%VN_LP_ZEROPHASE  Zero-phase low-pass without the Signal Processing Toolbox.
%   y = vn_lp_zerophase(x, fc, fs): 2nd-order Butterworth (bilinear
%   transform) run forward and backward, with odd-reflection padding and
%   steady-state initial conditions as in filtfilt.  Net response: 4th
%   order, zero phase, -6 dB at fc.  x must be uniformly sampled.
x = double(x(:));
n = numel(x);
if n < 4 || fc <= 0 || fc >= fs/2
    y = x;
    return;
end
K   = tan(pi * fc / fs);
nrm = 1 / (1 + sqrt(2) * K + K^2);
b   = K^2 * nrm * [1 2 1];
a   = [1, 2 * (K^2 - 1) * nrm, (1 - sqrt(2) * K + K^2) * nrm];
zi  = (eye(2) - [-a(2) 1; -a(3) 0]) \ (b(2:3).' - a(2:3).' * b(1));
np  = min(n - 1, 3 * max(1, round(fs / fc)));
xp  = [2 * x(1) - x(np + 1:-1:2); x; 2 * x(n) - x(n - 1:-1:n - np)];
y   = filter(b, a, xp, zi * xp(1));
y   = flipud(y);
y   = filter(b, a, y, zi * y(1));
y   = flipud(y);
y   = y(np + 1:np + n);
end
