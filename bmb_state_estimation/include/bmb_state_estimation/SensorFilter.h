#pragma once

#include "../../../bmb_msgs/include/msgs/SensorMeasurements.h"
#include "../../../bmb_msgs/include/msgs/ControlInputs.h"
#include "../../../bmb_msgs/include/msgs/AircraftState.h"

class SensorFilter {
public:
    void updateWrapper(const double *doubleSensorMeasurements, const uint8_t *uint8SensorMeasurements,
                       const unsigned char *boolSensorMeasurements, const double *control_inputs, double dt);

    void getOutputWrapper(double *doubleAircraftState) const;

    virtual void update(const SensorMeasurements &sensorMeasurements,
                        const ControlInputs &control_inputs, const double &dt) = 0;

    [[nodiscard]] virtual AircraftState getOutput() const = 0;
};
