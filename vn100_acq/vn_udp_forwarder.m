function vn_udp_forwarder(port, opts)
%VN_UDP_FORWARDER  Fallback acquisition path: serial -> UDP on localhost.
%   Use only if the S-function path fails the bench test.  Run it in a
%   SECOND MATLAB instance, started before the Simulink model:
%       vn_udp_forwarder('COM7')                        % until Ctrl+C
%       vn_udp_forwarder('COM7', struct('duration', 1800))
%   It opens the VN-100 with serial at p.baud, polls BytesAvailable every
%   p.fwd_period with a timer, stamps each read with qpc_now (QPC after the
%   read) and sends one datagram per read (at most p.fwd_max_n bytes) to
%   127.0.0.1:p.udp_port with payload
%       [double qpc (8 B); uint16 n (2 B); uint8 bytes(1:n)]   (little-endian)
%   In Simulink, a non-blocking UDP Receive block (max length 1034, uint8)
%   at Ts_rx logs the payload to <subj>_<run>_imu_bytes.mat and the
%   received length to <subj>_<run>_imu_diag.mat; vn_parse detects this
%   format (1 diag channel) and uses qpc from the payload as receipt time.
%   Uses java.net.DatagramSocket (no toolbox).

p = vn_params_default();
if nargin >= 2 && ~isempty(opts)
    f = fieldnames(opts);
    for k = 1:numel(f), p.(f{k}) = opts.(f{k}); end
end
if nargin >= 1 && ~isempty(port), p.port = port; end
if ~isfield(p, 'duration'), p.duration = inf; end
if exist('qpc_now', 'file') ~= 3
    error('vn_udp_forwarder:mex', 'qpc_now MEX not found. Run vn_build first.');
end

o = instrfind('Port', p.port);
if ~isempty(o), fclose(o); delete(o); end
s = serial(p.port, 'BaudRate', p.baud, 'DataBits', 8, 'Parity', 'none', 'StopBits', 1, ...
    'FlowControl', 'none', 'InputBufferSize', 2^18, 'Timeout', 1);
fopen(s);
flushinput(s);
sock = java.net.DatagramSocket();
addr = java.net.InetAddress.getByName('127.0.0.1');
n_dgram = 0;
n_bytes = 0;
n_full  = 0;
tmr = timer('ExecutionMode', 'fixedRate', 'Period', p.fwd_period, 'BusyMode', 'drop', ...
    'TimerFcn', @tick, 'Name', 'vn_udp_forwarder');
cleanup = onCleanup(@shutdown); %#ok<NASGU>
start(tmr);
fprintf('Forwarding %s @ %d -> udp://127.0.0.1:%d. Ctrl+C to stop.\n', p.port, p.baud, p.udp_port);
t0 = tic;
tp = 0;
while toc(t0) < p.duration
    pause(0.2);
    if toc(t0) - tp >= 10
        tp = toc(t0);
        fprintf('%7.0f s  datagrams %d  bytes %d  full datagrams %d\n', tp, n_dgram, n_bytes, n_full);
    end
end

    function tick(~, ~)
        nb = s.BytesAvailable;
        while nb > 0
            m = min(nb, p.fwd_max_n);
            d = fread(s, m, 'uint8');
            q = qpc_now();
            payload = [typecast(q, 'uint8'), typecast(uint16(m), 'uint8'), uint8(d(:).')];
            sock.send(java.net.DatagramPacket(typecast(payload, 'int8'), numel(payload), addr, p.udp_port));
            n_dgram = n_dgram + 1;
            n_bytes = n_bytes + m;
            n_full  = n_full + (m == p.fwd_max_n);
            nb = nb - m;
        end
    end

    function shutdown()
        try, stop(tmr); delete(tmr); catch, end
        try, fclose(s); delete(s); catch, end
        try, sock.close(); catch, end
        fprintf('\nForwarder stopped: %d datagrams, %d bytes.\n', n_dgram, n_bytes);
    end
end
