#pragma once

#include "Vector.h"
#include "Wrench.h"
#include "Accel.h"
#include "Matrix.h"
#include "SensorMeasurements.h"
#include "Constants.h"

#define SENSOR_NOISE 1

SensorMeasurements getSensorMeasurements(const Vector<double, n> &state, const Accel<double> &accel);

void getSensorMeasurementsWrapper(const double *aircraft_state, double *double_sensor_measurements,
                                  uint8_t *uint8_sensor_measurements, unsigned char *bool_sensor_measurements);

Matrix<double, p, n> getSensorMeasurementsJacobian(const Vector<double, n> &state, const Wrench<double> &current_loads);
