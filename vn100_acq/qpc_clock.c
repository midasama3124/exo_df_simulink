/*
 * qpc_clock.c - Level-2 C S-function: outputs QueryPerformanceCounter in
 * seconds (double) at its sample time (the 1 kHz base rate).  Logged next
 * to t_sim, it maps wall-clock time to simulation time and gives the
 * pacing jitter of the 1 kHz loop.  Build with vn_build.m.
 *
 * Parameter (non-tunable)
 *   1  sample time [s]   0.001
 * Output
 *   1  double            QPC time [s]
 */

#define S_FUNCTION_NAME  qpc_clock
#define S_FUNCTION_LEVEL 2

#ifndef _WIN32
#error "qpc_clock uses QueryPerformanceCounter and builds on Windows only."
#endif

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "simstruc.h"

#define P_TS    0
#define NPARAMS 1

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
    const mxArray *ts = ssGetSFcnParam(S, P_TS);
    if (ts == NULL || !mxIsNumeric(ts) || mxIsComplex(ts) || mxGetNumberOfElements(ts) != 1 ||
        mxGetScalar(ts) <= 0.0) {
        ssSetErrorStatus(S, "qpc_clock: parameter 1 (sample time) must be a positive scalar.");
    }
}
#endif

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NPARAMS);
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) return;
    } else {
        return;
    }
#endif
    ssSetSFcnParamTunable(S, P_TS, SS_PRM_NOT_TUNABLE);
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    if (!ssSetNumInputPorts(S, 0)) return;
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 1);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    ssSetSimStateCompliance(S, DISALLOW_SIM_STATE);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
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
    LARGE_INTEGER f;
    if (!QueryPerformanceFrequency(&f) || f.QuadPart <= 0) {
        ssSetErrorStatus(S, "qpc_clock: QueryPerformanceFrequency failed.");
        return;
    }
    ssGetRWork(S)[0] = 1.0 / (double)f.QuadPart;
}
#endif

static void mdlOutputs(SimStruct *S, int_T tid)
{
    LARGE_INTEGER c;
    real_T *y = (real_T *)ssGetOutputPortSignal(S, 0);
    UNUSED_ARG(tid);
    QueryPerformanceCounter(&c);
    y[0] = (double)c.QuadPart * ssGetRWork(S)[0];
}

static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S);
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
