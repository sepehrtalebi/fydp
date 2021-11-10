#include "SensorFilter.h"

#include "Vector.h"

void SensorFilter::updateWrapper(const double *doubleSensorMeasurements, const uint8_t *uint8SensorMeasurements,
                                 const unsigned char *boolSensorMeasurements, const double *control_inputs, double dt) {
    Vector<double, 16> doubleZ{doubleSensorMeasurements};
    Vector<uint8_t, 1> uint8Z{uint8SensorMeasurements};
    Vector<bool, 1> boolZ{(bool) boolSensorMeasurements[0]};
    Vector<double, 4> control_inputsVec{control_inputs};
    SensorMeasurements sensorMeasurements = SensorMeasurements::parseZ(doubleZ, uint8Z, boolZ);
    update(sensorMeasurements, ControlInputs::parseU(control_inputsVec), dt);
}

void SensorFilter::getOutputWrapper(double *doubleAircraftState) const {
    Vector<double, 19> aircraftState = getOutput().getX();
    for (size_t i = 0; i < 19; i++) doubleAircraftState[i] = aircraftState[i];
}
