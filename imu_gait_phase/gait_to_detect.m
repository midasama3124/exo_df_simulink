function to = gait_to_detect(t, w, hs, opt)
%GAIT_TO_DETECT  Automatic toe-offs between given heel strikes (no toolboxes).
%   to = gait_to_detect(t, w, hs, opt)
%     t    [n x 1] time [s]      w  [n x 1] reference signal (gait_reference_signal:
%                                   mid-swing positive, troughs negative)
%     hs   heel-strike sample indices (automatic or edited)
%     opt  .toStart  toe-off search starts this fraction of the stride after the heel strike
%          .toSpan   s, length of the partial strides searched before the first and
%                    after the last heel strike, as a multiple of the median stride
%          .msMin    rad/s, the stride's mid-swing peak must exceed this
%   to   sample indices of the toe-offs, sorted
%   Each stride HS_k -> HS_k+1 has one mid-swing peak (the maximum of w
%   inside it); the toe-off is the deepest trough between toStart x T after
%   HS_k and that peak (the secondary trough between two heel-strike troughs).
%   A minimum on the edge of the window (no trough inside) or above 0 gives no
%   toe-off.  The partial strides before the first and after the last heel
%   strike are searched the same way, so the trial's first and last toe-offs
%   are not lost.  Toe-offs only label events: the gait phase uses heel strikes only.

t  = t(:);
w  = w(:);
n  = numel(w);
hs = unique(hs(:));
to = zeros(0, 1);
if numel(hs) < 2, return; end
fs   = 1 / median(diff(t));
Tmed = median(diff(t(hs)));
% stride limits [a b] and the start of the search window
a  = hs(1:end - 1);
b  = hs(2:end) - 1;
s0 = a + round(opt.toStart * min(b - a, Tmed * fs));
a0 = max(1, hs(1) - round(opt.toSpan * Tmed * fs));             % before the first heel strike
a  = [a0; a];   b  = [hs(1) - 1; b];   s0 = [a0; s0];
aN = hs(end);                                                   % after the last heel strike
a  = [a; aN];   b  = [b; min(n, aN + round(opt.toSpan * Tmed * fs))];
s0 = [s0; aN + round(opt.toStart * Tmed * fs)];
keep = false(numel(a), 1);
to   = zeros(numel(a), 1);
for k = 1:numel(a)
    if b(k) - a(k) < 4, continue; end
    [wm, jm] = max(w(a(k):b(k)));
    im = a(k) + jm - 1;                       % mid-swing of this stride
    if wm < opt.msMin || im >= b(k), continue; end
    r = max(a(k), s0(k)):im - 1;
    if numel(r) < 3, continue; end
    [wv, j] = min(w(r));
    if j == 1 || j == numel(r) || wv >= 0, continue; end
    to(k)   = r(j);
    keep(k) = true;
end
to = to(keep);
end
