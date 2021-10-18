#pragma once

#include "Vector.h"
#include "SensorMeasurements.h"
#include "Constants.h"

SensorMeasurements getSensorMeasurements(const Vector<double, n> &state);
