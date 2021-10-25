#pragma once

#include "Vector.h"
#include "SensorMeasurements.h"
#include "Constants.h"

#define SENSOR_NOISE 1

SensorMeasurements getSensorMeasurements(const Vector<double, n> &state);
