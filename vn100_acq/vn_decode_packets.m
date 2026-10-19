function F = vn_decode_packets(stream, starts, L)
%VN_DECODE_PACKETS  Decode the little-endian fields of accepted packets.
%   F = vn_decode_packets(stream, starts, L) returns a struct with one
%   double matrix per field of the layout (rows = packets, columns =
%   components), named as in L.field_order:
%     time_startup  [ns]          (convert with *1e-9)
%     quat          [x y z w]     (scalar last, as sent)
%     gyro          [rad/s]
%     accel         [m/s^2]
%     ypr           [deg]         (if in the mask)
%     magpres       [mag(3) G, temp degC, pressure kPa]   (if in the mask)
%   Assumes a little-endian host (Windows x86/x64).

stream = uint8(stream(:));
starts = starts(:);
np     = numel(starts);
chunk  = 200000;                         % bounds the size of the index matrix
F = struct();
for k = 1:numel(L.fields)
    f   = L.fields(k);
    out = zeros(np, f.ncomp);
    for i0 = 1:chunk:np
        ii  = (i0:min(np, i0 + chunk - 1)).';
        idx = bsxfun(@plus, starts(ii), f.offset + (0:f.nbytes - 1));
        M   = reshape(stream(idx), size(idx)).';      % nbytes x n, packet-major
        v   = typecast(M(:), f.type);
        out(ii, :) = double(reshape(v, f.ncomp, []).');
    end
    F.(f.name) = out;
end
end
