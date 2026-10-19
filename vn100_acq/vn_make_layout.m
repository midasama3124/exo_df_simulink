function L = vn_make_layout(fields, rate, baud)
%VN_MAKE_LAYOUT  Packet layout of VN-100 binary output 1, common group (group 1).
%   L = vn_make_layout(fields, rate, baud)
%     fields : 'base' | 'base+ypr' | 'base+ypr+mag', or a numeric field mask
%     rate   : output rate in Hz (default 400)
%     baud   : line rate used for the utilization figure (default 921600)
%
%   Packet: sync 0xFA | group byte 0x01 | field mask (uint16, little-endian)
%           | payload, fields in ascending bit order, little-endian
%           | CRC-16 (2 bytes, MSB first).
%   The CRC run over the group byte through the two CRC bytes is 0 for a
%   valid packet.  Offsets below are 0-based from the sync byte.

if nargin < 1 || isempty(fields), fields = 'base'; end
if nargin < 2 || isempty(rate),   rate = 400; end
if nargin < 3 || isempty(baud),   baud = 921600; end

if ischar(fields)
    switch lower(strrep(fields, ' ', ''))
        case 'base',          mask = hex2dec('0131');  % TimeStartup, Quaternion, AngularRate, Accel
        case 'base+ypr',      mask = hex2dec('0139');  % + YawPitchRoll
        case 'base+ypr+mag',  mask = hex2dec('0539');  % + MagPres
        otherwise
            error('vn_make_layout:fields', 'Unknown field set ''%s''.', fields);
    end
else
    mask = double(fields);
end

% Common group (group 1) field table, VN-100/200/300 binary protocol:
%   bit  name            type     ncomp  bytes/comp  units / notes
T = {
     0  'time_startup'  'uint64'  1  8  'ns since sensor start-up'
     1  'time_gps'      'uint64'  1  8  'ns (GNSS units only)'
     2  'time_syncin'   'uint64'  1  8  'ns since last SyncIn'
     3  'ypr'           'single'  3  4  'deg (yaw, pitch, roll)'
     4  'quat'          'single'  4  4  '[x y z w], scalar LAST'
     5  'gyro'          'single'  3  4  'rad/s, bias-compensated'
     6  'position'      'double'  3  8  '(GNSS units only)'
     7  'velocity'      'single'  3  4  '(GNSS units only)'
     8  'accel'         'single'  3  4  'm/s^2, compensated'
     9  'imu'           'single'  6  4  'uncompensated accel + gyro'
    10  'magpres'       'single'  5  4  'mag x,y,z [Gauss], temp [degC], pressure [kPa]'
    11  'deltatheta'    'single'  7  4  'dt, dtheta(3), dvel(3)'
    12  'ins_status'    'uint16'  1  2  '(INS units only)'
    13  'syncin_cnt'    'uint32'  1  4  'count'
    14  'time_gps_pps'  'uint64'  1  8  'ns (GNSS units only)'
    };

if mask <= 0 || mask >= 2^15 || mask ~= round(mask)
    error('vn_make_layout:mask', 'Field mask 0x%X not supported (bits 0-14 of the common group only).', mask);
end

L = struct();
L.group      = 1;
L.mask       = mask;
L.mask_hex   = sprintf('0x%04X', mask);
L.header     = uint8([250, 1, bitand(mask, 255), bitshift(mask, -8)]);
L.rate       = rate;
fields_out   = struct('bit', {}, 'name', {}, 'type', {}, 'ncomp', {}, 'nbytes', {}, 'offset', {}, 'units', {});
off = 4;                                    % payload starts after 0xFA, group, mask(2)
for k = 1:size(T, 1)
    b = T{k, 1};
    if bitand(mask, 2^b)
        f.bit    = b;
        f.name   = T{k, 2};
        f.type   = T{k, 3};
        f.ncomp  = T{k, 4};
        f.nbytes = T{k, 4} * T{k, 5};
        f.offset = off;
        f.units  = T{k, 6};
        fields_out(end+1) = f; %#ok<AGROW>
        off = off + f.nbytes;
    end
end
L.fields      = fields_out;
L.field_order = {fields_out.name};
L.payload_len = off - 4;
L.packet_len  = off + 2;
L.crc_offset  = off;                         % 0-based offset of the CRC MSB
L.bytes_per_s = L.packet_len * rate;
L.line_util   = L.bytes_per_s * 10 / baud;   % 8N1 = 10 bits per byte
L.baud        = baud;
L.created     = datestr(now, 'yyyy-mm-dd HH:MM:SS');
end
