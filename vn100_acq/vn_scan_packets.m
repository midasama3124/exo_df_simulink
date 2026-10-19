function [starts, d] = vn_scan_packets(stream, L)
%VN_SCAN_PACKETS  Locate CRC-valid VN binary packets in a byte stream.
%   [starts, d] = vn_scan_packets(stream, L)
%     stream : byte vector (uint8)
%     L      : layout from vn_make_layout
%     starts : 1-based index of the 0xFA sync byte of each accepted packet
%     d      : counters (candidates, packets, crc_fail, bytes_skipped,
%              bytes_trailing)
%
%   Equivalent to a sequential byte scan: look for 0xFA followed by the
%   expected group byte and field mask with a full packet length still
%   available; check the CRC (group byte through CRC bytes must give 0);
%   on success jump past the packet, on failure advance one byte and
%   rescan.  The CRC is evaluated for all header matches at once
%   (vectorized); the walk then visits them in order.

stream = uint8(stream(:));
Ns  = numel(stream);
P   = L.packet_len;
hdr = L.header;

d = struct('n_bytes', Ns, 'n_candidates', 0, 'n_packets', 0, 'crc_fail', 0, ...
           'bytes_skipped', 0, 'bytes_trailing', Ns);
starts = zeros(0, 1);
if Ns < P
    return;
end

% header matches with a full packet available
lim = Ns - P + 1;
c = find(stream(1:lim) == hdr(1));
if ~isempty(c)
    c = c(stream(c + 1) == hdr(2) & stream(c + 2) == hdr(3) & stream(c + 3) == hdr(4));
end
d.n_candidates = numel(c);
if isempty(c)
    return;
end

% CRC over bytes offset 1 .. P-1 (group byte through CRC LSB), all candidates at once
[~, T] = vn_crc16([]);
crc = zeros(numel(c), 1, 'uint16');
for j = 1:P-1
    b = uint16(stream(c + j));
    crc = bitxor(bitshift(crc, 8), T(double(bitxor(bitshift(crc, -8), b)) + 1));
end
ok = (crc == 0);

% sequential walk (identical to scanning byte by byte)
starts  = zeros(numel(c), 1);
np      = 0;
pos     = 1;     % next byte the scanner looks at
acc     = 1;     % first byte not yet accounted for (after the last accepted packet)
nfail   = 0;
skipped = 0;
for i = 1:numel(c)
    ci = c(i);
    if ci < pos
        continue;                 % inside an accepted packet, or already passed
    end
    if ok(i)
        np = np + 1;
        starts(np) = ci;
        skipped = skipped + (ci - acc);
        pos = ci + P;
        acc = pos;
    else
        nfail = nfail + 1;
        pos = ci + 1;             % advance one byte and rescan
    end
end
starts = starts(1:np);

d.n_packets      = np;
d.crc_fail       = nfail;
d.bytes_skipped  = skipped;
d.bytes_trailing = Ns - acc + 1;
end
