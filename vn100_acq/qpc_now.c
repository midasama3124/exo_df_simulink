/*
 * qpc_now.c - MEX function: t = qpc_now() returns QueryPerformanceCounter
 * in seconds (double), the same clock as qpc_clock and vn_raw_rx.  Used by
 * the fallback forwarder (vn_udp_forwarder.m).  Build with vn_build.m.
 */
#ifndef _WIN32
#error "qpc_now uses QueryPerformanceCounter and builds on Windows only."
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "mex.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    static double invFreq = 0.0;
    LARGE_INTEGER c, f;
    (void)nrhs; (void)prhs;
    if (nlhs > 1) mexErrMsgIdAndTxt("qpc_now:nargout", "qpc_now returns one output.");
    if (invFreq == 0.0) {
        if (!QueryPerformanceFrequency(&f) || f.QuadPart <= 0)
            mexErrMsgIdAndTxt("qpc_now:qpf", "QueryPerformanceFrequency failed.");
        invFreq = 1.0 / (double)f.QuadPart;
    }
    QueryPerformanceCounter(&c);
    plhs[0] = mxCreateDoubleScalar((double)c.QuadPart * invFreq);
}
