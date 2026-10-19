function n = vn_port_number(port)
%VN_PORT_NUMBER  COM port number from 'COM7', 'com7', '\\.\COM12' or 7.
if isnumeric(port) && isscalar(port)
    n = double(port);
else
    tok = regexp(char(port), '(?i)COM(\d+)\s*$', 'tokens', 'once');
    if isempty(tok)
        error('vn_port_number:name', 'Cannot read a COM port number from ''%s''.', char(port));
    end
    n = str2double(tok{1});
end
if n ~= round(n) || n < 1 || n > 255
    error('vn_port_number:range', 'COM port number must be an integer in 1..255 (got %g).', n);
end
end
