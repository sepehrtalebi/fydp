#pragma once

#include <bmb_math/Matrix.h>
#include <bmb_math/Wrench.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>

Wrench<double> getAppliedLoads(
    const bmb_msgs::AircraftState& state,
    const bmb_msgs::ControlInputs& control_inputs);

Matrix<double, 6, bmb_msgs::AircraftState::SIZE> getAppliedLoadsJacobian(
    const bmb_msgs::AircraftState& state,
    const bmb_msgs::ControlInputs& control_inputs);
