// **********************************************************
// **** To build this mex function use: mex main_ekf.cpp ****
// **********************************************************

#include "src/EKF.h"
#include "src/SensorMeasurements.h"
#include "src/AircraftState.h"
#include "src/RailLocation.h"

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  main_ekf

// Need to include simstruc.h for the definition of the SimStruct and
// its associated macro definitions.
#include "simstruc.h"

// Function: mdlInitializeSizes ===============================================
// Abstract:
//    The sizes information is used by Simulink to determine the S-function
//    block's characteristics (number of inputs, outputs, states, etc.).
static void mdlInitializeSizes(SimStruct *S) {
    DECL_AND_INIT_DIMSINFO(inputDimsInfo);
    DECL_AND_INIT_DIMSINFO(outputDimsInfo);

    // No expected parameters
    ssSetNumSFcnParams(S, 0);

    // Parameter mismatch will be reported by Simulink
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    // Specify I/O
    if (!ssSetNumInputPorts(S, 3)) return;
    ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED);
    ssSetInputPortWidth(S, 1, DYNAMICALLY_SIZED);
    ssSetInputPortWidth(S, 2, DYNAMICALLY_SIZED);
    ssSetInputPortDirectFeedThrough(S, 0, 0);
    ssSetInputPortDirectFeedThrough(S, 0, 0);
    ssSetInputPortDirectFeedThrough(S, 0, 0);
    if (!ssSetNumOutputPorts(S, 2)) return;
    ssSetOutputPortWidth(S, 0, DYNAMICALLY_SIZED);
    ssSetOutputPortWidth(S, 1, DYNAMICALLY_SIZED);

    ssSetNumSampleTimes(S, 1);

    // Reserve place for C++ object
    ssSetNumPWork(S, 1);

    ssSetOperatingPointCompliance(S, USE_CUSTOM_OPERATING_POINT);

    /* Set this S-function as runtime thread-safe for multicore execution */
    ssSetRuntimeThreadSafetyCompliance(S, RUNTIME_THREAD_SAFETY_COMPLIANCE_TRUE);

    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_DISALLOW_CONSTANT_SAMPLE_TIME);
}


// Function: mdlInitializeSampleTimes =========================================
// Abstract:
//   This function is used to specify the sample time(s) for your
//   S-function. You must register the same number of sample times as
//   specified in ssSetNumSampleTimes.
static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

// Function: mdlStart =======================================================
// Abstract:
//   This function is called once at start of model execution. If you
//   have states that should be initialized once, this is the place
//   to do it.
#define MDL_START

static void mdlStart(SimStruct *S) {
    // Store new C++ object in the pointers vector
    EKF *ekf = new EKF{};
    ssGetPWork(S)[0] = ekf;
}

// Function: mdlOutputs =======================================================
// Abstract:
//   In this function, you compute the outputs of your S-function
//   block.
static void mdlOutputs(SimStruct *S, int_T tid) {
    // Retrieve C++ object from the pointers vector
    EKF *ekf = static_cast<EKF *>(ssGetPWork(S)[0]);

    // Get data addresses of I/O
    const SensorMeasurements *sensorMeasurements = (SensorMeasurements *) ssGetInputPortRealSignalPtrs(S, 0);
    const Vector3 *forces = (Vector3 *) ssGetInputPortRealSignalPtrs(S, 1);
    const Vector3 *torques = (Vector3 *) ssGetInputPortRealSignalPtrs(S, 2);
    AircraftState *aircraftState = (AircraftState *) ssGetOutputPortRealSignal(S, 0);
    RailLocation *railLocation = (RailLocation *) ssGetOutputPortRealSignal(S, 1);

    ekf->update(*sensorMeasurements, *forces, *torques, 0.01);
    ekf->getOutput(aircraftState, railLocation);
}

// Function: mdlTerminate =====================================================
// Abstract:
//   In this function, you should perform any actions that are necessary
//   at the termination of a simulation.  For example, if memory was
//   allocated in mdlStart, this is the place to free it.
static void mdlTerminate(SimStruct *S) {
    // Retrieve and destroy C++ object
    EKF *ekf = static_cast<EKF *>(ssGetPWork(S)[0]);
    delete ekf;
}


// Required S-function trailer
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else

#include "cg_sfun.h"       /* Code generation registration function */

#endif
