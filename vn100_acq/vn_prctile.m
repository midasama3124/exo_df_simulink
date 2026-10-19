function q = vn_prctile(x, pct)
%VN_PRCTILE  Percentiles without the Statistics Toolbox (same definition as prctile).
%   q = vn_prctile(x, pct): sorted x(i) is taken as the 100*(i-0.5)/n
%   percentile, linear interpolation in between, clamped at the ends.
%   NaNs are ignored.
x = sort(x(~isnan(x(:))));
n = numel(x);
q = nan(size(pct));
if n == 0, return; end
if n == 1, q(:) = x; return; end
pos = pct(:) / 100 * n + 0.5;
pos = min(max(pos, 1), n);
lo  = floor(pos);
hi  = min(lo + 1, n);
w   = pos - lo;
q(:) = (1 - w) .* x(lo) + w .* x(hi);
end
