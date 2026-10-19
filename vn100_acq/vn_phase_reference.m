function [phi, stride] = vn_phase_reference(ev, t_sim, p)
%VN_PHASE_REFERENCE  Reference gait phase from consecutive valid heel strikes.
%   [phi, stride] = vn_phase_reference(ev, t_sim)
%   phi(t) = (t - HS_i) / (HS_{i+1} - HS_i) in [0, 1) for t in a stride
%   whose two heel strikes are valid, in the same segment and with a stride
%   time inside p.ev_T_range x median; NaN elsewhere.  stride = index i.
%   Use t_sim = the time vector of the phase-estimator logs to compare the
%   run-time estimate against this offline reference.
if nargin < 3 || isempty(p), p = vn_params_default(); end
m  = ev.valid_hs & isfinite(ev.t_hs_sim);
hs = ev.t_hs_sim(m);
sg = ev.seg(m);
[hs, o] = sort(hs);
sg = sg(o);
[hs, iu] = unique(hs);
sg = sg(iu);
sz     = size(t_sim);
t_sim  = t_sim(:);
phi    = nan(sz);
stride = nan(sz);
if numel(hs) < 2, return; end
phi    = phi(:);
stride = stride(:);
T    = diff(hs);
okS  = sg(1:end - 1) == sg(2:end) & T > 0;
Tmed = median(T(okS));
okS  = okS & T >= p.ev_T_range(1) * Tmed & T <= p.ev_T_range(2) * Tmed;
k    = discretize(t_sim, hs);                 % hs(k) <= t < hs(k+1)
in   = ~isnan(k);
in(in) = k(in) <= numel(T);
in(in) = okS(k(in));
phi(in)    = (t_sim(in) - hs(k(in))) ./ T(k(in));
stride(in) = k(in);
phi(phi >= 1) = NaN;                          % right edge of the last bin
phi    = reshape(phi, sz);
stride = reshape(stride, sz);
end
