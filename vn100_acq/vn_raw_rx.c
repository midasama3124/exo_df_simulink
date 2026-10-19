/*
 * vn_raw_rx.c - Level-2 C S-function: non-blocking raw-byte reader for a
 * VectorNav VN-100 on a Windows COM port.  MATLAB/Simulink R2018a.  Runs
 * as a MEX file in normal simulation (vn_build.m) and is compiled into the
 * real-time code by the QUARC Windows 64-bit target (QUARC > Build), which
 * finds this source next to the MEX file.  All parameters are numeric
 * because noninlined S-functions in generated code do not support char
 * parameters.  They are not tunable: changing one needs a rebuild.
 *
 * Parameters (non-tunable)
 *   1  COM port number          e.g. 7 for COM7 (1..255)
 *   2  baud rate                921600
 *   3  sample time Ts_rx [s]    0.002
 *   4  N_buf                    maximum bytes read per call, 1..65535 (128)
 *
 * Outputs (one sample per call)
 *   1  uint8[N_buf]  bytes read this call, unused tail zero-padded
 *   2  uint16        number of bytes read
 *   3  double        QueryPerformanceCounter after the read [s]
 *   4  uint32        bytes still queued after the read (cbInQue snapshot - n)
 *   5  uint32        cumulative bytes read
 *   6  uint32        cumulative count of calls with a nonzero error mask
 *   7  uint32        error mask of this call: ClearCommError CE_* bits
 *                    (CE_RXOVER 0x1, CE_OVERRUN 0x2, CE_RXPARITY 0x4,
 *                    CE_FRAME 0x8, CE_BREAK 0x10), plus
 *                    0x40000000 if ClearCommError failed and
 *                    0x80000000 if ReadFile failed
 *   8  double        call duration t1 - t0 [us]
 *   9  double[ceil(N_buf/4)]  the port-1 bytes packed 4 per double, little
 *                    endian: b0 + 256 b1 + 65536 b2 + 16777216 b3 (exact
 *                    integers < 2^32).  For loggers that store everything
 *                    as double (QUARC To Host File); vn_unpack_bytes.m
 *                    restores the bytes.
 *
 * mdlOutputs never waits; its only loop is the fixed-size packing of
 * port 9 (ceil(N_buf/4) iterations).  COMMTIMEOUTS {MAXDWORD, 0, 0} makes
 * ReadFile return at once with bytes already queued, and at most N_buf
 * bytes are requested.  Unread bytes stay in the driver buffer for the
 * next call.  Errors are raised in mdlStart only; at run time problems
 * are reported on ports 6-7.
 */

#define S_FUNCTION_NAME  vn_raw_rx
#define S_FUNCTION_LEVEL 2

#ifndef _WIN32
#error "vn_raw_rx uses the Win32 serial API and builds on Windows only."
#endif

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "simstruc.h"

#define P_PORT   0
#define P_BAUD   1
#define P_TS     2
#define P_NBUF   3
#define NPARAMS  4

#define VNRX_PATH_MAX       32
#define VNRX_PORT_MAX       255
#define VNRX_NOUT           9
#define VNRX_PACK           4      /* bytes per double on port 9 */
#define VNRX_DRIVER_RXBUF   65536
#define VNRX_DRIVER_TXBUF   4096
#define VNRX_ERR_CLEARCOMM  0x40000000UL
#define VNRX_ERR_READFILE   0x80000000UL

typedef struct {
    HANDLE   h;
    double   invFreq;     /* 1 / QPC frequency [s per count] */
    uint32_T cumBytes;
    uint32_T errCalls;
    int_T    nbuf;
} vnrx_T;

static char vnrx_msg[512];

static int isPosScalar(const mxArray *a)
{
    return a != NULL && mxIsNumeric(a) && !mxIsComplex(a) &&
           mxGetNumberOfElements(a) == 1 && mxGetScalar(a) > 0.0;
}

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
    const mxArray *port = ssGetSFcnParam(S, P_PORT);
    const mxArray *baud = ssGetSFcnParam(S, P_BAUD);
    const mxArray *ts   = ssGetSFcnParam(S, P_TS);
    const mxArray *nbuf = ssGetSFcnParam(S, P_NBUF);
    double nb, pn;

    if (!isPosScalar(port)) {
        ssSetErrorStatus(S, "vn_raw_rx: parameter 1 must be the COM port number, e.g. 7 for COM7 "
                            "(char names are not supported in generated code).");
        return;
    }
    pn = mxGetScalar(port);
    if (pn != (double)(int_T)pn || pn > (double)VNRX_PORT_MAX) {
        ssSetErrorStatus(S, "vn_raw_rx: parameter 1 (COM port number) must be an integer in 1..255.");
        return;
    }
    if (!isPosScalar(baud)) {
        ssSetErrorStatus(S, "vn_raw_rx: parameter 2 (baud) must be a positive scalar.");
        return;
    }
    if (!isPosScalar(ts)) {
        ssSetErrorStatus(S, "vn_raw_rx: parameter 3 (Ts_rx) must be a positive scalar.");
        return;
    }
    if (!isPosScalar(nbuf)) {
        ssSetErrorStatus(S, "vn_raw_rx: parameter 4 (N_buf) must be a positive scalar.");
        return;
    }
    nb = mxGetScalar(nbuf);
    if (nb != (double)(int_T)nb || nb > 65535.0) {
        ssSetErrorStatus(S, "vn_raw_rx: parameter 4 (N_buf) must be an integer in 1..65535.");
        return;
    }
}
#endif

static void mdlInitializeSizes(SimStruct *S)
{
    int_T i, nbuf;

    ssSetNumSFcnParams(S, NPARAMS);
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) return;
    } else {
        return; /* Simulink reports the parameter-count mismatch */
    }
#endif
    for (i = 0; i < NPARAMS; i++) {
        ssSetSFcnParamTunable(S, i, SS_PRM_NOT_TUNABLE);
    }
    nbuf = (int_T)mxGetScalar(ssGetSFcnParam(S, P_NBUF));

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    if (!ssSetNumInputPorts(S, 0)) return;
    if (!ssSetNumOutputPorts(S, VNRX_NOUT)) return;

    ssSetOutputPortWidth(S, 0, nbuf); ssSetOutputPortDataType(S, 0, SS_UINT8);
    ssSetOutputPortWidth(S, 1, 1);    ssSetOutputPortDataType(S, 1, SS_UINT16);
    ssSetOutputPortWidth(S, 2, 1);    ssSetOutputPortDataType(S, 2, SS_DOUBLE);
    ssSetOutputPortWidth(S, 3, 1);    ssSetOutputPortDataType(S, 3, SS_UINT32);
    ssSetOutputPortWidth(S, 4, 1);    ssSetOutputPortDataType(S, 4, SS_UINT32);
    ssSetOutputPortWidth(S, 5, 1);    ssSetOutputPortDataType(S, 5, SS_UINT32);
    ssSetOutputPortWidth(S, 6, 1);    ssSetOutputPortDataType(S, 6, SS_UINT32);
    ssSetOutputPortWidth(S, 7, 1);    ssSetOutputPortDataType(S, 7, SS_DOUBLE);
    ssSetOutputPortWidth(S, 8, (nbuf + VNRX_PACK - 1) / VNRX_PACK);
    ssSetOutputPortDataType(S, 8, SS_DOUBLE);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 1);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    ssSetSimStateCompliance(S, DISALLOW_SIM_STATE);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE | SS_OPTION_CALL_TERMINATE_ON_EXIT);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, mxGetScalar(ssGetSFcnParam(S, P_TS)));
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
#if defined(MDL_START)
static void mdlStart(SimStruct *S)
{
    char port[VNRX_PATH_MAX];
    char path[VNRX_PATH_MAX + 8];
    vnrx_T *w;
    LARGE_INTEGER f;
    DCB dcb;
    COMMTIMEOUTS to;
    DWORD baud = (DWORD)mxGetScalar(ssGetSFcnParam(S, P_BAUD));
    int   pn   = (int)mxGetScalar(ssGetSFcnParam(S, P_PORT));

    ssGetPWork(S)[0] = NULL;

#if defined(MATLAB_MEX_FILE)
    /* The MEX file also runs while Simulink generates code (QUARC > Build,
       SS_SIMMODE_RTWGEN) and on the host side of External mode.  Only a
       normal simulation may open the port; the real-time code opens it
       itself (this block is not compiled there). */
    if (ssGetSimMode(S) != SS_SIMMODE_NORMAL) return;
#endif
    if (pn < 1 || pn > VNRX_PORT_MAX) {   /* mdlCheckParameters runs in MEX only */
        ssSetErrorStatus(S, "vn_raw_rx: COM port number must be in 1..255.");
        return;
    }
    snprintf(port, sizeof(port), "COM%d", pn);
    w = (vnrx_T *)calloc(1, sizeof(vnrx_T));
    if (w == NULL) {
        ssSetErrorStatus(S, "vn_raw_rx: out of memory.");
        return;
    }
    w->h    = INVALID_HANDLE_VALUE;
    w->nbuf = (int_T)mxGetScalar(ssGetSFcnParam(S, P_NBUF));
    ssGetPWork(S)[0] = w;

    if (!QueryPerformanceFrequency(&f) || f.QuadPart <= 0) {
        ssSetErrorStatus(S, "vn_raw_rx: QueryPerformanceFrequency failed.");
        return;
    }
    w->invFreq = 1.0 / (double)f.QuadPart;

    snprintf(path, sizeof(path), "\\\\.\\%s", port);   /* works for COM10 and above */
    w->h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (w->h == INVALID_HANDLE_VALUE) {
        snprintf(vnrx_msg, sizeof(vnrx_msg),
                 "vn_raw_rx: cannot open %s (Win32 error %lu). Wrong number, or the port is in use "
                 "(vn_configure, a serial object in MATLAB, VectorNav Control Center?).",
                 port, (unsigned long)GetLastError());
        ssSetErrorStatus(S, vnrx_msg);
        return;
    }
    if (!SetupComm(w->h, VNRX_DRIVER_RXBUF, VNRX_DRIVER_TXBUF)) {
        snprintf(vnrx_msg, sizeof(vnrx_msg), "vn_raw_rx: SetupComm failed on %s (Win32 error %lu).",
                 port, (unsigned long)GetLastError());
        ssSetErrorStatus(S, vnrx_msg);
        return;
    }

    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(w->h, &dcb)) {
        snprintf(vnrx_msg, sizeof(vnrx_msg), "vn_raw_rx: GetCommState failed on %s (Win32 error %lu).",
                 port, (unsigned long)GetLastError());
        ssSetErrorStatus(S, vnrx_msg);
        return;
    }
    dcb.BaudRate          = baud;
    dcb.ByteSize          = 8;
    dcb.Parity            = NOPARITY;
    dcb.StopBits          = ONESTOPBIT;
    dcb.fBinary           = TRUE;
    dcb.fParity           = FALSE;
    dcb.fOutxCtsFlow      = FALSE;
    dcb.fOutxDsrFlow      = FALSE;
    dcb.fDtrControl       = DTR_CONTROL_ENABLE;
    dcb.fDsrSensitivity   = FALSE;
    dcb.fTXContinueOnXoff = TRUE;
    dcb.fOutX             = FALSE;
    dcb.fInX              = FALSE;
    dcb.fErrorChar        = FALSE;
    dcb.fNull             = FALSE;
    dcb.fRtsControl       = RTS_CONTROL_ENABLE;
    dcb.fAbortOnError     = FALSE;   /* errors must not block later reads */
    if (!SetCommState(w->h, &dcb)) {
        snprintf(vnrx_msg, sizeof(vnrx_msg),
                 "vn_raw_rx: SetCommState failed on %s at %lu baud (Win32 error %lu).",
                 port, (unsigned long)baud, (unsigned long)GetLastError());
        ssSetErrorStatus(S, vnrx_msg);
        return;
    }

    memset(&to, 0, sizeof(to));
    to.ReadIntervalTimeout         = MAXDWORD;   /* return immediately with what is queued */
    to.ReadTotalTimeoutMultiplier  = 0;
    to.ReadTotalTimeoutConstant    = 0;
    to.WriteTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant   = 0;
    if (!SetCommTimeouts(w->h, &to)) {
        snprintf(vnrx_msg, sizeof(vnrx_msg), "vn_raw_rx: SetCommTimeouts failed on %s (Win32 error %lu).",
                 port, (unsigned long)GetLastError());
        ssSetErrorStatus(S, vnrx_msg);
        return;
    }
    if (!PurgeComm(w->h, PURGE_RXCLEAR | PURGE_RXABORT)) {
        snprintf(vnrx_msg, sizeof(vnrx_msg), "vn_raw_rx: PurgeComm failed on %s (Win32 error %lu).",
                 port, (unsigned long)GetLastError());
        ssSetErrorStatus(S, vnrx_msg);
        return;
    }
}
#endif

static void mdlOutputs(SimStruct *S, int_T tid)
{
    vnrx_T   *w      = (vnrx_T *)ssGetPWork(S)[0];
    uint8_T  *yBytes = (uint8_T  *)ssGetOutputPortSignal(S, 0);
    uint16_T *yN     = (uint16_T *)ssGetOutputPortSignal(S, 1);
    real_T   *yT     = (real_T   *)ssGetOutputPortSignal(S, 2);
    uint32_T *yQ     = (uint32_T *)ssGetOutputPortSignal(S, 3);
    uint32_T *yCum   = (uint32_T *)ssGetOutputPortSignal(S, 4);
    uint32_T *yErrN  = (uint32_T *)ssGetOutputPortSignal(S, 5);
    uint32_T *yMask  = (uint32_T *)ssGetOutputPortSignal(S, 6);
    real_T   *yDur   = (real_T   *)ssGetOutputPortSignal(S, 7);
    real_T   *yPack  = (real_T   *)ssGetOutputPortSignal(S, 8);
    int_T    npack   = ssGetOutputPortWidth(S, 8);
    int_T    i, j;
    LARGE_INTEGER t0, t1;
    COMSTAT  stat;
    DWORD    errs  = 0;
    DWORD    n     = 0;
    DWORD    nread = 0;
    uint32_T mask  = 0;
    int_T    nbuf  = ssGetOutputPortWidth(S, 0);

    UNUSED_ARG(tid);

    if (w == NULL || w->h == INVALID_HANDLE_VALUE) {   /* not reachable after a good mdlStart */
        memset(yBytes, 0, (size_t)nbuf);
        *yN = 0; *yT = 0.0; *yQ = 0; *yCum = 0; *yErrN = 0; *yMask = VNRX_ERR_READFILE; *yDur = 0.0;
        for (i = 0; i < npack; i++) yPack[i] = 0.0;
        return;
    }

    QueryPerformanceCounter(&t0);                                    /* 1 */
    memset(&stat, 0, sizeof(stat));
    if (!ClearCommError(w->h, &errs, &stat)) {                       /* 2 */
        mask |= VNRX_ERR_CLEARCOMM;
        stat.cbInQue = 0;
    }
    mask |= (uint32_T)errs;
    n = stat.cbInQue;                                                /* 3 */
    if (n > (DWORD)w->nbuf) n = (DWORD)w->nbuf;
    if (n > 0) {
        if (!ReadFile(w->h, yBytes, n, &nread, NULL)) {
            mask |= VNRX_ERR_READFILE;
            nread = 0;
        }
    }
    QueryPerformanceCounter(&t1);                                    /* 4 */
    if (nread < (DWORD)nbuf) {                                       /* 5 */
        memset(yBytes + nread, 0, (size_t)(nbuf - (int_T)nread));
    }
    for (i = 0; i < npack; i++) {                                    /* 6: port 9 */
        uint32_T v = 0;
        for (j = VNRX_PACK - 1; j >= 0; j--) {
            int_T k = i * VNRX_PACK + j;
            v = (v << 8) | (k < nbuf ? (uint32_T)yBytes[k] : 0U);
        }
        yPack[i] = (real_T)v;
    }

    w->cumBytes += (uint32_T)nread;
    if (mask != 0) w->errCalls++;

    *yN    = (uint16_T)nread;
    *yT    = (double)t1.QuadPart * w->invFreq;
    *yQ    = (stat.cbInQue > nread) ? (uint32_T)(stat.cbInQue - nread) : 0U;
    *yCum  = w->cumBytes;
    *yErrN = w->errCalls;
    *yMask = mask;
    *yDur  = (double)(t1.QuadPart - t0.QuadPart) * w->invFreq * 1.0e6;
}

static void mdlTerminate(SimStruct *S)
{
    vnrx_T *w;
    if (ssGetPWork(S) == NULL) return;
    w = (vnrx_T *)ssGetPWork(S)[0];
    if (w != NULL) {
        if (w->h != INVALID_HANDLE_VALUE) {
            CloseHandle(w->h);
            w->h = INVALID_HANDLE_VALUE;
        }
        free(w);
        ssGetPWork(S)[0] = NULL;
    }
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
