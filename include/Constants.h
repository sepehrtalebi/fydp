#pragma once

#include "Vector3.h"

static constexpr const double PI = 3.14159265358979323846;
static constexpr const double GRAVITATIONAL_ACCELERATION = 9.81;
static constexpr const double MASS = 1; // aircraft mass, kg
static const Vector3<double> WEIGHT{0, 0, MASS * GRAVITATIONAL_ACCELERATION};

// aileron aerodynamic constant
static constexpr const double LIFT_GAIN_AILERON = 1;
// displacement of aerodynamic center of ailerons from the center of mass
static const Vector3<double> L_RIGHT_AILERON{0, 1, 0};
static const Vector3<double> L_LEFT_AILERON{L_RIGHT_AILERON[0], -L_RIGHT_AILERON[1], L_RIGHT_AILERON[2]};

// aircraft body aerodynamic constants
static constexpr const double DRAG_GAIN_BODY = 1;
static constexpr const double LIFT_GAIN_BODY = 1;
// displacement of aerodynamic center of the body from the center of mass
static const Vector3<double> L_BODY{0, 0, 0};

// rudder aerodynamic constant
static constexpr const double LIFT_GAIN_RUDDER = 1;
// displacement of aerodynamic center of rudder from the center of mass
static const Vector3<double> L_RUDDER{-1, 0, 0};
