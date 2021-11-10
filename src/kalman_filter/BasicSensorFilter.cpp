#include "BasicSensorFilter.h"

void BasicSensorFilter::update(const SensorMeasurements &sensorMeasurements,
                               const ControlInputs &control_inputs, const double &dt) {
    // TODO: set state based on data
}

AircraftState BasicSensorFilter::getOutput() const {
    return state;
}