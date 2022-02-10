#pragma once

#include <bmb_math/Vector.h>
#include <bmb_math/Vector3.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_msgs/SensorMeasurements.h>

namespace bmb_math {

[[nodiscard]] Vector<double, bmb_msgs::SensorMeasurements::SIZE> as_vector(
    const bmb_msgs::SensorMeasurements& msg);

[[nodiscard]] bmb_msgs::SensorMeasurements as_msg(
    const Vector<double, bmb_msgs::SensorMeasurements::SIZE>&
        sensor_measurements);

[[nodiscard]] Vector<double, bmb_msgs::AircraftState::SIZE> as_vector(
    const bmb_msgs::AircraftState& msg);

[[nodiscard]] bmb_msgs::AircraftState as_msg(
    const Vector<double, bmb_msgs::AircraftState::SIZE>& aircraft_state);

[[nodiscard]] Vector3<double> as_vector(const bmb_msgs::ControlInputs& msg);

[[nodiscard]] bmb_msgs::ControlInputs as_msg(
    const Vector3<double>& control_inputs);

}  // namespace bmb_math
