function s = vn_rx_param_str(p)
%VN_RX_PARAM_STR  S-function parameter string for vn_raw_rx: port number, baud, Ts_rx, N_buf.
%   p.port may be 'COM7' (as used by vn_configure) or 7; vn_raw_rx gets the
%   number, because generated code (QUARC) does not support char parameters.
s = sprintf('%d, %d, %.17g, %d', vn_port_number(p.port), p.baud, p.Ts_rx, p.N_buf);
end
