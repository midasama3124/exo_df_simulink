function sg = gait_auto_sign(w, fs)
%GAIT_AUTO_SIGN  Sign that puts a foot angular velocity in the detection convention.
%   sg = gait_auto_sign(w, fs) returns +1 or -1 so that sg*w has the
%   toe-off and heel-strike troughs negative and the mid-swing peak positive.
%   The extreme values cannot decide this (the toe-off trough is often
%   larger than the mid-swing peak), and the signal integrates to about zero
%   over a stride.  Instead: every quiet stance stretch is left through the
%   toe-off trough and entered through the heel-strike trough, so the first
%   large excursion after it and the last one before it vote for "negative".

a     = abs(w(:));
a99   = vn_prctile(a, 99);
quiet = a < max(0.3, 0.1 * a99);
big   = a > 0.3 * a99;
d     = diff([false; quiet; false]);
qs    = find(d == 1);                 % first quiet sample
qe    = find(d == -1) - 1;            % last quiet sample
keep  = (qe - qs + 1) >= round(0.15 * fs);
qs    = qs(keep);
qe    = qe(keep);
bigI  = find(big);
votes = 0;
for k = 1:numel(qs)
    j = bigI(find(bigI > qe(k), 1, 'first'));
    if ~isempty(j), votes = votes + sign(w(j)); end
    j = bigI(find(bigI < qs(k), 1, 'last'));
    if ~isempty(j), votes = votes + sign(w(j)); end
end
sg = 1;
if votes > 0, sg = -1; end
end
