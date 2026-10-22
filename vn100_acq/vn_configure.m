function info = vn_configure(port, opts)
%VN_CONFIGURE  One-time configuration of the shoe-mounted VN-100 (Milestone 1).
%
%   info = vn_configure(port)
%   info = vn_configure(port, opts)
%   info = vn_configure(port, struct('verify_only', true))   % after a power cycle
%
%   Run before a session with no simulation running (vn_raw_rx holds the
%   COM port while the model runs).
%
%   opts: any field of vn_params_default, plus
%     .fields       'base' (default) | 'base+ypr' | 'base+ypr+mag'
%     .verify_only  true = find the baud, identify the unit, read back
%                   registers 5, 6, 75, 76, 77 and measure the packet rate,
%                   without writing anything.
%     .async_mode   'auto' (default, detect the VN serial port the USB link
%                   uses) | 1 | 2 | 3
%
%   Steps
%     1. Scan 115200, 921600, 460800, 230400, 128000, 57600 baud with
%        $VNRRG,01 (200 ms per try).
%     2. Read registers 1-4 (model, hardware revision, serial, firmware).
%     3. Write 6 = 0, 76 = off, 77 = off; detect AsyncMode by enabling
%        binary output 1 at 10 Hz on port 1, then port 2, and listening;
%        write the final register 75 (AsyncMode, RateDivisor =
%        imu_rate/rate, group 0x01, field mask); write 5 = 921600 LAST and
%        reopen at 921600.  If the baud found cannot carry the final stream
%        (the VN-100 rejects such writes with error 12), 5 is written before
%        the final 75.
%     4. Read back 5, 6, 75, 76, 77 (error on any mismatch), measure the
%        packet rate from TimeStartup, $VNWNV, close.
%     5. Save vn_unit_info.mat and vn_layout.mat in p.cfg_dir.
%
%   verify_only saves vn_unit_info_verify_<date>.mat and returns info.pass.

if nargin < 2 || isempty(opts), opts = struct(); end
p = vn_params_default();
fn = fieldnames(opts);
for k = 1:numel(fn), p.(fn{k}) = opts.(fn{k}); end
if nargin >= 1 && ~isempty(port), p.port = port; end
p.port = sprintf('COM%d', vn_port_number(p.port));   % accepts 7 or 'COM7'
verify_only = isfield(p, 'verify_only') && logical(p.verify_only);

layout = vn_make_layout(p.fields, p.rate, p.baud);
div = p.imu_rate / p.rate;
if div < 1 || div ~= round(div)
    error('vn_configure:divisor', 'imu_rate/rate = %g is not a positive integer.', div);
end

info = struct();
info.date           = datestr(now, 'yyyy-mm-dd HH:MM:SS');
info.port           = p.port;
info.fields         = p.fields;
info.manual_version = p.manual_version;
info.verify_only    = verify_only;
if verify_only, mode_str = 'verify only'; else, mode_str = 'configure'; end
fprintf('\n=== vn_configure (%s), %s ===\n', mode_str, p.port);
if isempty(p.manual_version)
    fprintf('Note: p.manual_version is empty; record the manual/ICD revision you checked the layout against.\n');
end

% ---- FTDI latency timer (read-only registry check) ----
info.ftdi = vn_check_ftdi_latency(p.port);
if info.ftdi.found
    fprintf('FTDI latency timer on %s: %g ms\n', p.port, info.ftdi.latency_ms);
    if info.ftdi.latency_ms ~= p.ftdi_latency_ms
        warning('vn_configure:ftdi', ['FTDI latency timer is %g ms. Set it to %g ms: Device Manager > Ports > ' ...
            '%s > Properties > Port Settings > Advanced > Latency Timer, then unplug/replug.'], ...
            info.ftdi.latency_ms, p.ftdi_latency_ms, p.port);
    end
else
    fprintf('FTDI latency timer: %s. Check the adapter manually.\n', info.ftdi.note);
end

close_stale(p.port);
s = [];
try
    % ---- 1. find the current baud ----
    [s, info.baud_found] = find_baud(p);
    fprintf('Unit answers at %d baud.\n', info.baud_found);

    % ---- 2. identify ----
    info.model    = strjoin(vn_read(s, 1, p), ',');
    info.hw_rev   = strjoin(vn_read(s, 2, p), ',');
    info.serial   = strjoin(vn_read(s, 3, p), ',');
    info.firmware = strjoin(vn_read(s, 4, p), ',');
    fprintf('Model %s | hardware rev %s | serial %s | firmware %s\n', ...
        info.model, info.hw_rev, info.serial, info.firmware);
    if isempty(strfind(upper(info.model), 'VN-100')) %#ok<STREMP>
        warning('vn_configure:model', 'Model "%s" is not a VN-100; check the layout against its manual.', info.model);
    end
    [info.accel_comp, info.ref_frame_rot] = check_comp(s, p);   % read only

    if verify_only
        exp_mode = expected_async_mode(p, info);
        [info.readback, ok, msgs] = readback(s, p, layout, div, exp_mode);
        info.rate_meas = measure_rate(s, layout, div, p);
        close_port(s); s = [];
        info.async_mode = info.readback.async_mode;
        info.pass = ok && info.baud_found == p.baud && info.rate_meas.ok;
        print_readback(info.readback, msgs);
        print_rate(info.rate_meas, p);
        if info.baud_found ~= p.baud
            fprintf('FAIL: unit came up at %d baud, expected %d.\n', info.baud_found, p.baud);
        end
        if info.pass
            fprintf('VERIFY PASS: configuration survived the power cycle.\n');
        else
            warning('vn_configure:verify', 'VERIFY FAIL (see messages above).');
        end
        save(fullfile(p.cfg_dir, sprintf('vn_unit_info_verify_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'))), 'info');
        return;
    end

    save(fullfile(p.cfg_dir, 'vn_unit_info.mat'), 'info');   % identity first; completed below

    % ---- 3. write ----
    fprintf('Writing configuration (fields %s, mask %s, %d Hz, divisor %d)...\n', ...
        p.fields, layout.mask_hex, p.rate, div);
    vn_write(s, 6, '0', p);                       % ASCII async output off (active port)
    off = '0,1,01,0001';                          % AsyncMode 0 = off; other fields only need to be valid
    vn_write(s, 76, off, p);
    vn_write(s, 77, off, p);
    mode = detect_async(s, layout, p);
    info.async_mode   = mode;
    info.rate_divisor = div;
    final75 = sprintf('%d,%d,01,%04X', mode, div, layout.mask);
    slow = layout.bytes_per_s * 10 / info.baud_found > p.slow_link_util;
    if ~slow
        vn_write(s, 75, final75, p);
        s = switch_baud(s, p);                    % register 5 written last
        info.write_order = '6, 76, 77, 75 (detect), 75, 5';
    else
        fprintf(['  %d baud cannot carry %d B/s; writing register 5 before the final register 75 ' ...
            '(VN error 12 otherwise).\n'], info.baud_found, layout.bytes_per_s);
        s = switch_baud(s, p);
        vn_write(s, 75, final75, p);
        info.write_order = '6, 76, 77, 75 (detect), 5, 75';
    end

    % ---- 4. read back, measure, save to flash ----
    [info.readback, ok, msgs] = readback(s, p, layout, div, mode);
    print_readback(info.readback, msgs);
    if ~ok
        error('vn_configure:readback', 'Readback mismatch:\n  %s', strjoin(msgs, '\n  '));
    end
    info.rate_meas = measure_rate(s, layout, div, p);
    print_rate(info.rate_meas, p);
    if ~info.rate_meas.ok
        error('vn_configure:rate', ['Measured %.2f Hz (%d CRC failures); expected %g Hz. The IMU rate looks like ' ...
            '%.1f Hz: set p.imu_rate accordingly and rerun.'], info.rate_meas.rate_sensor, ...
            info.rate_meas.crc_fail, p.rate, info.rate_meas.imu_rate_est);
    end
    r = vn_txrx(s, 'VNWNV', 'WNV', 0, p.wnv_timeout);
    if ~r.found || r.is_err
        error('vn_configure:wnv', 'Write Settings ($VNWNV) failed (%s).', reply_str(r));
    end
    fprintf('Settings saved to non-volatile memory.\n');
    close_port(s); s = [];
catch ME
    close_port(s);
    rethrow(ME);
end

% ---- 5. save ----
info.field_mask = layout.mask;
info.mask_hex   = layout.mask_hex;
info.baud       = p.baud;
info.rate       = p.rate;
save(fullfile(p.cfg_dir, 'vn_unit_info.mat'), 'info');
save(fullfile(p.cfg_dir, 'vn_layout.mat'), 'layout');
fprintf('Saved vn_unit_info.mat and vn_layout.mat in %s\n', p.cfg_dir);
fprintf('Packet %d B (payload %d B) at %d Hz = %.1f kB/s, %.0f %% of %d baud.\n', ...
    layout.packet_len, layout.payload_len, p.rate, layout.bytes_per_s/1e3, 100*layout.line_util, p.baud);
end

% =========================================================================
function [s, baud] = find_baud(p)
for b = p.scan_bauds
    s = open_port(p.port, b);
    for k = 1:p.scan_tries
        r = vn_txrx(s, 'VNRRG,01', 'RRG', 1, p.scan_timeout);
        if r.found
            baud = b;
            return;
        end
    end
    close_port(s);
end
s = [];
error('vn_configure:nounit', 'No reply to $VNRRG,01 on %s at %s baud. Check power, cable and port name.', ...
    p.port, mat2str(p.scan_bauds));
end

function s = switch_baud(s, p)
% The VN-100 replies at the old baud, then switches.
r = vn_txrx(s, sprintf('VNWRG,05,%d', p.baud), 'WRG', 5, p.cmd_timeout);
if r.found && r.is_err
    error('vn_configure:baud', 'Writing register 5 = %d failed (%s).', p.baud, reply_str(r));
end
port = s.Port;
pause(0.1);
close_port(s);
s = open_port(port, p.baud);
for k = 1:3
    r = vn_txrx(s, 'VNRRG,01', 'RRG', 1, p.scan_timeout);
    if r.found, return; end
end
error('vn_configure:baud', 'No reply at %d baud after writing register 5.', p.baud);
end

function mode = detect_async(s, layout, p)
if isnumeric(p.async_mode) && ~isempty(p.async_mode)
    mode = p.async_mode;
    fprintf('  AsyncMode set by the user: %d\n', mode);
    return;
end
for m = [1 2]
    vn_write(s, 75, sprintf('%d,%d,01,%04X', m, p.detect_divisor, layout.mask), p);
    buf = read_for(s, p.detect_time);
    st = vn_scan_packets(buf, layout);
    fprintf('  AsyncMode %d: %d valid packets in %.1f s\n', m, numel(st), p.detect_time);
    if numel(st) >= 2
        mode = m;
        return;
    end
end
error('vn_configure:async', ['No binary packets on %s with AsyncMode 1 or 2. ' ...
    'Set opts.async_mode explicitly or check the cable.'], s.Port);
end

function buf = read_for(s, T)
flushinput(s);
buf = zeros(0, 1, 'uint8');
t0 = tic;
while toc(t0) < T
    nb = s.BytesAvailable;
    if nb > 0
        buf = [buf; uint8(fread(s, nb, 'uint8'))]; %#ok<AGROW>
    else
        pause(0.005);
    end
end
end

function meas = measure_rate(s, layout, div, p)
t0  = tic;
buf = read_for(s, p.rate_check_time);
el  = toc(t0);
[st, d] = vn_scan_packets(buf, layout);
meas = struct('n_packets', numel(st), 'crc_fail', d.crc_fail, 'elapsed', el, ...
    'rate_sensor', NaN, 'rate_wall', numel(st)/el, 'imu_rate_est', NaN, 'ok', false);
if numel(st) >= 10
    F  = vn_decode_packets(buf, st, layout);
    dt = diff(F.time_startup * 1e-9);
    meas.rate_sensor  = 1 / median(dt);
    meas.imu_rate_est = meas.rate_sensor * div;
    meas.ok = abs(meas.rate_sensor - p.rate) / p.rate <= p.rate_check_tol && d.crc_fail == 0;
    if isfield(F, 'accel')                % static check: keep the unit still during the measurement
        meas.accel_mean = mean(F.accel, 1);
        meas.accel_norm = mean(sqrt(sum(F.accel.^2, 2)));
        meas.accel_std  = std(F.accel, 0, 1);
    end
    if isfield(F, 'gyro'), meas.gyro_mean = mean(F.gyro, 1); end
end
end

function [rb, ok, msgs] = readback(s, p, layout, div, exp_mode)
msgs = {};
f5  = vn_read(s, 5, p);   rb.baud = str2double(f5{1});
f6  = vn_read(s, 6, p);   rb.ador = str2double(f6{1});
f75 = vn_read(s, 75, p);  rb.reg75 = strjoin(f75, ',');
rb.async_mode = NaN; rb.rate_divisor = NaN; rb.group = NaN; rb.mask = NaN;
if numel(f75) >= 4
    rb.async_mode   = str2double(f75{1});
    rb.rate_divisor = str2double(f75{2});
    rb.group        = hex2dec(f75{3});
    rb.mask         = hex2dec(f75{4});
end
f76 = vn_read(s, 76, p);  rb.reg76 = strjoin(f76, ',');  rb.async76 = str2double(f76{1});
f77 = vn_read(s, 77, p);  rb.reg77 = strjoin(f77, ',');  rb.async77 = str2double(f77{1});

if rb.baud ~= p.baud, msgs{end+1} = sprintf('reg 5 = %g, expected %d', rb.baud, p.baud); end
if rb.ador ~= 0,      msgs{end+1} = sprintf('reg 6 = %g, expected 0', rb.ador); end
if isempty(exp_mode)
    if ~any(rb.async_mode == [1 2 3])
        msgs{end+1} = sprintf('reg 75 AsyncMode = %g, expected 1, 2 or 3', rb.async_mode);
    end
elseif rb.async_mode ~= exp_mode
    msgs{end+1} = sprintf('reg 75 AsyncMode = %g, expected %d', rb.async_mode, exp_mode);
end
if rb.rate_divisor ~= div,     msgs{end+1} = sprintf('reg 75 RateDivisor = %g, expected %d', rb.rate_divisor, div); end
if rb.group ~= 1,              msgs{end+1} = sprintf('reg 75 OutputGroup = %g, expected 0x01', rb.group); end
if rb.mask ~= layout.mask,     msgs{end+1} = sprintf('reg 75 field mask = 0x%04X, expected %s', rb.mask, layout.mask_hex); end
if rb.async76 ~= 0,            msgs{end+1} = sprintf('reg 76 AsyncMode = %g, expected 0', rb.async76); end
if rb.async77 ~= 0,            msgs{end+1} = sprintf('reg 77 AsyncMode = %g, expected 0', rb.async77); end
ok = isempty(msgs);
end

function m = expected_async_mode(p, info)
m = [];
if isnumeric(p.async_mode) && ~isempty(p.async_mode)
    m = p.async_mode;
    return;
end
fn = fullfile(p.cfg_dir, 'vn_unit_info.mat');
if exist(fn, 'file')
    S = load(fn);
    if isfield(S, 'info') && isfield(S.info, 'serial') && strcmp(S.info.serial, info.serial) ...
            && isfield(S.info, 'async_mode')
        m = S.info.async_mode;
    end
end
end

function print_readback(rb, msgs)
fprintf('Readback: reg5 %g | reg6 %g | reg75 %s | reg76 %s | reg77 %s\n', ...
    rb.baud, rb.ador, rb.reg75, rb.reg76, rb.reg77);
for k = 1:numel(msgs), fprintf('  MISMATCH: %s\n', msgs{k}); end
end

function print_rate(m, p)
fprintf('Packet rate: %.3f Hz from TimeStartup (target %g), %.1f Hz on the wall clock, %d packets, %d CRC failures.\n', ...
    m.rate_sensor, p.rate, m.rate_wall, m.n_packets, m.crc_fail);
if isfield(m, 'accel_mean')
    fprintf('Mean over those packets (unit still): accel %s m/s^2 (|a| %.4f, std %s), gyro %s rad/s\n', ...
        mat2str(m.accel_mean, 5), m.accel_norm, mat2str(m.accel_std, 2), mat2str(m.gyro_mean, 3));
end
end

% ---- register access ----------------------------------------------------
function [ac, rf] = check_comp(s, p)
% Read (never write) register 25, Acceleration Compensation (C 3x3 row-major,
% then bias B 3: a_out = C*(a - B)), and register 26, Reference Frame Rotation
% (3x3).  Factory values are identity and zero; anything else scales or
% rotates the accel output (run A read |a| = 9.51 m/s^2 flat on a table).
ac = struct('C', nan(3), 'B', nan(3, 1), 'raw', '');
rf = struct('R', nan(3), 'raw', '');
try
    v = str2double(vn_read(s, 25, p));
    ac.raw = mat2str(v);
    ac.C = reshape(v(1:9), 3, 3).';
    ac.B = v(10:12).';
    v = str2double(vn_read(s, 26, p));
    rf.raw = mat2str(v);
    rf.R = reshape(v(1:9), 3, 3).';
catch ME
    fprintf('Could not read registers 25/26 (%s).\n', ME.message);
    return;
end
fprintf('Reg 25 accel compensation: C = %s, B = %s\n', mat2str(ac.C, 6), mat2str(ac.B.', 6));
fprintf('Reg 26 reference frame rotation: R = %s\n', mat2str(rf.R, 6));
if max(max(abs(ac.C - eye(3)))) > 1e-4 || any(abs(ac.B) > 1e-4)
    warning('vn_configure:accelcomp', ['Register 25 is not the factory identity/zero: the accel output is ' ...
        'rescaled/offset by the unit. Check whether this was intended.']);
end
if max(max(abs(rf.R - eye(3)))) > 1e-4
    sv = svd(rf.R);
    warning('vn_configure:refframe', ['Register 26 is not identity (singular values %s; values other than 1 ' ...
        'scale the outputs).'], mat2str(sv.', 4));
end
end

function fields = vn_read(s, reg, p)
r = vn_txrx(s, sprintf('VNRRG,%02d', reg), 'RRG', reg, p.cmd_timeout);
if ~r.found
    error('vn_configure:noreply', 'No reply to $VNRRG,%02d.', reg);
end
if r.is_err
    error('vn_configure:vnerr', '$VNRRG,%02d -> %s', reg, reply_str(r));
end
fields = r.fields;
if isempty(fields), fields = {''}; end
end

function vn_write(s, reg, val, p)
body = sprintf('VNWRG,%02d,%s', reg, val);
r = vn_txrx(s, body, 'WRG', reg, p.cmd_timeout);
if ~r.found
    error('vn_configure:noreply', 'No reply to $%s.', body);
end
if r.is_err
    hint = '';
    if r.err == 8 && reg >= 75, hint = ' (invalid register: binary outputs need firmware 2.x or later)'; end
    if r.err == 12, hint = ' (baud too low for the requested output)'; end
    error('vn_configure:vnerr', '$%s -> %s%s', body, reply_str(r), hint);
end
end

function r = vn_txrx(s, body, kind, reg, timeout)
% Send $<body>*<xor8> and wait for the matching reply (or $VNERR).
cmd = sprintf('$%s*%02X\r\n', body, vn_xor8(body));
flushinput(s);
fwrite(s, uint8(cmd));
buf = '';
r = empty_reply();
t0 = tic;
while toc(t0) < timeout
    nb = s.BytesAvailable;
    if nb > 0
        buf = [buf, char(fread(s, nb, 'uint8').')]; %#ok<AGROW>
        if numel(buf) > 16384, buf = buf(end-16383:end); end
        r = parse_reply(buf, kind, reg);
        if r.found, return; end
    else
        pause(0.002);
    end
end
end

function r = empty_reply()
r = struct('found', false, 'is_err', false, 'err', 0, 'fields', {{}}, 'line', '');
end

function r = parse_reply(txt, kind, reg)
% Valid ASCII lines are $VNxxx,...*HH (8-bit XOR) or *HHHH (CRC16), ending
% in CR/LF.  Binary packets interleaved with the replies are ignored
% because their bytes fail the checksum.
r = empty_reply();
tok = regexp(txt, '\$(VN[A-Z]{3}[^\$\*\r\n]*)\*([0-9A-Fa-f]{4}|[0-9A-Fa-f]{2})(?=[\r\n])', 'tokens');
for k = 1:numel(tok)
    body = tok{k}{1};
    chk  = tok{k}{2};
    if numel(chk) == 2
        good = vn_xor8(body) == hex2dec(chk);
    else
        good = double(vn_crc16(uint8(body))) == hex2dec(chk);
    end
    if ~good, continue; end
    parts = strsplit(body, ',', 'CollapseDelimiters', false);
    head  = parts{1};
    if strcmp(head, 'VNERR')
        r.found = true; r.is_err = true; r.line = body;
        if numel(parts) >= 2, r.err = str2double(parts{2}); end
        return;
    end
    if strcmp(kind, 'WNV')
        if strcmp(head, 'VNWNV'), r.found = true; r.line = body; return; end
    elseif strcmp(head, ['VN' kind]) && numel(parts) >= 2 && str2double(parts{2}) == reg
        r.found = true; r.fields = parts(3:end); r.line = body;
        return;
    end
end
end

function c = vn_xor8(str)
c = 0;
b = double(str);
for k = 1:numel(b), c = bitxor(c, b(k)); end
end

function s = reply_str(r)
names = {'hard fault', 'serial buffer overflow', 'invalid checksum', 'invalid command', ...
    'not enough parameters', 'too many parameters', 'invalid parameter', 'invalid register', ...
    'unauthorized access', 'watchdog reset', 'output buffer overflow', 'insufficient baud rate'};
if ~r.found
    s = 'no reply';
elseif r.is_err
    if r.err >= 1 && r.err <= numel(names), nm = names{r.err}; else, nm = 'unknown'; end
    s = sprintf('VN error %d, %s', r.err, nm);
else
    s = r.line;
end
end

% ---- port handling --------------------------------------------------------
function s = open_port(port, baud)
s = serial(port, 'BaudRate', baud, 'DataBits', 8, 'Parity', 'none', 'StopBits', 1, ...
    'FlowControl', 'none', 'InputBufferSize', 2^18, 'OutputBufferSize', 4096, 'Timeout', 1);
fopen(s);
s.DataTerminalReady = 'on';
s.RequestToSend = 'on';
flushinput(s);
end

function close_port(s)
if ~isempty(s) && isvalid(s)
    if strcmp(s.Status, 'open'), fclose(s); end
    delete(s);
end
end

function close_stale(port)
o = instrfind('Port', port);
if ~isempty(o)
    fclose(o);
    delete(o);
end
end
