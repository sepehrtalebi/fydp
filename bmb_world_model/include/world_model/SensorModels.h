#pragma once

#include "../../../bmb_math/include/bmb_math/Vector.h"
#include "../../../bmb_math/include/bmb_math/Accel.h"
#include "../../../bmb_math/include/bmb_math/Matrix.h"
#include "../../../bmb_msgs/include/msgs/SensorMeasurements.h"
#include "Constants.h"
#include <utility>

SensorMeasurements getSensorMeasurements(const Vector<double, n> &state, const Accel<double> &accel);

void getSensorMeasurementsWrapper(const double *aircraft_state, double *double_sensor_measurements,
                                  uint8_t *uint8_sensor_measurements, unsigned char *bool_sensor_measurements);

std::pair<Matrix<double, p, n>, Matrix<double, p, 6>> getSensorMeasurementsJacobian(const Vector<double, n> &state, const Accel<double> &current_loads);
