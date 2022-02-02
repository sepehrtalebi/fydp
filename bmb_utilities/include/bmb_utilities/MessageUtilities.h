#pragma once

#include <bmb_msgs/SensorMeasurements.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_math/Vector.h>

namespace bmb_utilities {

[[nodiscard]] Vector<double, 21> as_vector(const bmb_msgs::SensorMeasurements& msg);

[[nodiscard]] bmb_msgs::SensorMeasurements as_msg(const Vector<double, 21>& sensor_measurements);

[[nodiscard]] Vector<double, 13> as_vector(const bmb_msgs::AircraftState& msg);

[[nodiscard]] bmb_msgs::AircraftState as_msg(const Vector<double, 13>& aircraft_state);

}  // namespace bmb_utilities
