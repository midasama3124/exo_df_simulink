function [t, X, name] = vn_load_tofile(file)
%VN_LOAD_TOFILE  Load a To File log as a time column and a samples x width matrix.
%   [t, X, name] = vn_load_tofile(file)
%   Accepts the Timeseries format (Simulink To File, p.logger 'tofile'), a
%   struct with fields Time and Data, or the Array format [t; data] with
%   one column per sample (Simulink To File Array format, and the QUARC To
%   Host File MAT-files of p.logger 'quarc').  The data type is preserved
%   (uint8 for imu_bytes; To Host File stores double).  name is the
%   variable name ('imu_bytes_p4' marks packed bytes, see vn_unpack_bytes).

S  = load(file);
fn = fieldnames(S);
v  = [];
name = '';
for k = 1:numel(fn)                                   % 1) timeseries
    if isa(S.(fn{k}), 'timeseries'), v = S.(fn{k}); name = fn{k}; break; end
end
if isempty(v)
    for k = 1:numel(fn)                               % 2) struct with Time/Data
        x = S.(fn{k});
        if isstruct(x) && isfield(x, 'Time') && isfield(x, 'Data'), v = x; name = fn{k}; break; end
    end
end
if isempty(v)
    for k = 1:numel(fn)                               % 3) Array format [t; data]
        x = S.(fn{k});
        if isnumeric(x) && size(x, 1) >= 2
            t = double(x(1, :)).';
            X = x(2:end, :).';
            name = fn{k};
            return;
        end
    end
    error('vn_load_tofile:format', 'No timeseries / Time-Data struct / array found in %s.', file);
end
t  = double(v.Time(:));
X  = v.Data;
n  = numel(t);
sz = size(X);
if numel(sz) == 3                                     % W1 x W2 x N
    X = reshape(X, sz(1) * sz(2), sz(3)).';
elseif sz(1) ~= n && sz(2) == n
    X = X.';
end
if size(X, 1) ~= n
    error('vn_load_tofile:size', '%s: %d time samples but data is %s.', file, n, mat2str(size(X)));
end
end
