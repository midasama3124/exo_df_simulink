function r = vn_check_ftdi_latency(port)
%VN_CHECK_FTDI_LATENCY  Read (never write) the FTDI latency timer of a COM port.
%   r = vn_check_ftdi_latency('COM7')
%     r.found       true if an FTDI device with that PortName was found
%     r.latency_ms  LatencyTimer value (Windows default 16; required 1)
%     r.key         registry key of the device
%     r.note        reason when not found
%   Uses "reg query HKLM\SYSTEM\CurrentControlSet\Enum\FTDIBUS /s".  To
%   change the value use Device Manager > Ports > COMx > Properties > Port
%   Settings > Advanced > Latency Timer, then unplug and replug the cable.

r = struct('found', false, 'latency_ms', NaN, 'key', '', 'note', '');
if ~ispc
    r.note = 'not a Windows PC';
    return;
end
[st, out] = system('reg query "HKLM\SYSTEM\CurrentControlSet\Enum\FTDIBUS" /s');
if st ~= 0
    r.note = 'no FTDIBUS registry key (adapter is not FTDI-based, or the FTDI driver is not installed)';
    return;
end
lines = regexp(out, '\r?\n', 'split');
key   = '';
ports = containers.Map();
lats  = containers.Map();
for k = 1:numel(lines)
    L = lines{k};
    if strncmpi(L, 'HKEY_', 5)
        key = strtrim(L);
        continue;
    end
    t = regexp(L, '^\s+(\S+)\s+(REG_\w+)\s+(.*)$', 'tokens', 'once');
    if isempty(t) || isempty(key), continue; end
    switch t{1}
        case 'PortName',     ports(key) = strtrim(t{3});
        case 'LatencyTimer', lats(key)  = strtrim(t{3});
    end
end
ks = keys(ports);
for k = 1:numel(ks)
    if strcmpi(ports(ks{k}), port)
        r.found = true;
        r.key = ks{k};
        if isKey(lats, ks{k})
            v = lats(ks{k});
            if strncmpi(v, '0x', 2), r.latency_ms = hex2dec(v(3:end)); else, r.latency_ms = str2double(v); end
        else
            r.note = 'LatencyTimer value missing';
        end
        return;
    end
end
r.note = sprintf('no FTDI device with PortName %s in the registry', port);
end
