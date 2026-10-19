function vn_build()
%VN_BUILD  Compile vn_raw_rx.c, qpc_clock.c (S-functions) and qpc_now.c (MEX).
%   Needs a C compiler supported by R2018a: MinGW-w64 4.9.2 (Add-On
%   "MATLAB Support for MinGW-w64 C/C++ Compiler") or MSVC 2015/2017.
%   Select it once with "mex -setup C".  The Win32 functions used
%   (CreateFileA, ReadFile, ClearCommError, QueryPerformanceCounter, ...)
%   live in kernel32, which both compilers link by default.
%   QUARC > Build compiles vn_raw_rx.c and qpc_clock.c again into the
%   real-time code (it finds them next to the MEX files); the MEX files are
%   still needed for Simulink to compile the diagram.  Close the model (or
%   "clear mex") first if the old MEX file is locked.

if ~ispc
    error('vn_build:platform', 'These files use the Win32 API; build them on the Windows PC.');
end
here = fileparts(mfilename('fullpath'));
old = cd(here);
restore = onCleanup(@() cd(old));
cc = mex.getCompilerConfigurations('C', 'Selected');
if isempty(cc)
    error('vn_build:compiler', 'No C compiler selected. Run "mex -setup C" first.');
end
fprintf('C compiler: %s %s\n', cc(1).Name, cc(1).Version);
srcs = {'vn_raw_rx.c', 'qpc_clock.c', 'qpc_now.c'};
for k = 1:numel(srcs)
    fprintf('mex %-12s ... ', srcs{k});
    mex('-O', srcs{k});
    fprintf('ok\n');
end
fprintf('Built in %s. Keep this folder on the MATLAB path.\n', here);
end
