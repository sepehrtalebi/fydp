#pragma once

#include "Vector.h"
#include "Wrench.h"
#include "Matrix.h"
#include "SensorMeasurements.h"
#include "Constants.h"

#define SENSOR_NOISE 1

SensorMeasurements getSensorMeasurements(const Vector<double, n> &state);

void getSensorMeasurementsWrapper(const double *aircraft_state, double *sensor_measurements);

Matrix<double, p, n> getSensorMeasurementsJacobian(const Vector<double, n> &state, const Wrench<double> &current_loads);
