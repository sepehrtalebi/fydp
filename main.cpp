/**
 * This is the main code that runs in Simulink inside the S Function Builder block.
 * The code is duplicated here for reference, but must be edited within the block itself.
 **/
/* Includes_BEGIN */
#include "EKF.h"
/* Includes_END */

/* Externs_BEGIN */

/* Externs_END */

void main_kf_Start_wrapper(void **pW) {
    /* Start_BEGIN */
    KF *kf = new EKF{};
    pW[0] = kf;
    /* Start_END */
}

void main_kf_Outputs_wrapper(const real_T *forces,
                             const real_T *torques,
                             const real_T *doubleSensorMeasurements,
                             const uint8_T *uint8SensorMeasurements,
                             const boolean_T *boolSensorMeasurements,
                             real_T *aircraftState,
                             void **pW) {
    /* Output_BEGIN */
    KF *kf = static_cast<KF *>(pW[0]);
    kf->updateWrapper(doubleSensorMeasurements, uint8SensorMeasurements, boolSensorMeasurements,
                      forces, torques, 0.01);
    kf->getOutputWrapper(aircraftState);
    /* Output_END */
}

void main_kf_Terminate_wrapper(void **pW) {
    /* Terminate_BEGIN */
    KF *kf = static_cast<KF *>(pW[0]);
    delete kf;
    /* Terminate_END */
}