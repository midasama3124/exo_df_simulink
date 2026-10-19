function [crc, T] = vn_crc16(bytes)
%VN_CRC16  VectorNav CRC-16-CCITT (polynomial 0x1021, initial value 0,
%   no reflection, no final XOR; identical to CRC-16/XMODEM).
%   crc = vn_crc16(bytes) returns the CRC of a byte vector (uint8 or char).
%   [~, T] = vn_crc16([]) returns the 256-entry uint16 lookup table (column).
%
%   This is the table form of the bitwise routine in the VN-100 manual
%   (Sec. "Checksum / CRC").  For a binary packet, the CRC is sent MSB
%   first, so running the CRC over the group byte through the two CRC
%   bytes gives 0 for a valid packet.

persistent TT
if isempty(TT)
    TT = zeros(256, 1, 'uint16');
    for i = 0:255
        c = uint16(i * 256);
        for k = 1:8
            if bitand(c, uint16(32768))
                c = bitxor(bitshift(c, 1), uint16(4129));   % 0x1021
            else
                c = bitshift(c, 1);
            end
        end
        TT(i + 1) = c;
    end
end
T = TT;
crc = uint16(0);
b = uint16(bytes(:));
for k = 1:numel(b)
    crc = bitxor(bitshift(crc, 8), TT(double(bitxor(bitshift(crc, -8), b(k))) + 1));
end
end
