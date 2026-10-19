function B = vn_unpack_bytes(X, nbuf)
%VN_UNPACK_BYTES  Undo the 4-bytes-per-double packing of vn_raw_rx port 9.
%   B = vn_unpack_bytes(X)        % X: calls x W doubles -> calls x 4W uint8
%   B = vn_unpack_bytes(X, nbuf)  % keep the first nbuf bytes of each call
%   Each double is b0 + 256 b1 + 65536 b2 + 16777216 b3 (little endian).
%   Errors if a value is not an integer in 0..2^32-1 (the log is not packed
%   bytes, or was altered).

X = double(X);
if any(X(:) < 0 | X(:) > 4294967295 | X(:) ~= round(X(:)))
    error('vn_unpack_bytes:value', 'Packed byte log holds values that are not integers in 0..2^32-1.');
end
[K, W] = size(X);
B = zeros(K, 4 * W, 'uint8');
v = X;
for j = 1:4
    B(:, j:4:end) = uint8(mod(v, 256));
    v = floor(v / 256);
end
if nargin >= 2 && ~isempty(nbuf)
    if nbuf > 4 * W
        error('vn_unpack_bytes:nbuf', 'nbuf = %d but the log holds only %d bytes per call.', nbuf, 4 * W);
    end
    B = B(:, 1:nbuf);
end
end
