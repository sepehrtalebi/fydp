#pragma once

#include <bmb_math/Vector.h>
#include <bmb_math/Accel.h>
#include <bmb_math/Matrix.h>
#include <bmb_msgs/SensorMeasurements.h>
#include <bmb_world_model/Constants.h>

#include <utility>

bmb_msgs::SensorMeasurements getSensorMeasurements(const Vector<double, n> &state, const Accel<double> &accel);

std::pair<Matrix<double, p, n>, Matrix<double, p, 6>> getSensorMeasurementsJacobian(const Vector<double, n> &state, const Accel<double> &current_loads);
