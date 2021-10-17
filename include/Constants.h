#pragma once

#include "Vector3.h"
#include "Matrix.h"

// mathematical constants
static constexpr const double PI = 3.14159265358979323846;

// environmental constants
static constexpr const double GRAVITATIONAL_ACCELERATION = 9.81; // m/s^2
static constexpr const double ATMOSPHERIC_PRESSURE = 101325; // Pa
static constexpr const double AIR_DENSITY = 1.225; //kg/m^3

// aircraft inertial constants
static constexpr const double MASS = 1; // aircraft mass, kg
static const Vector3<double> WEIGHT{0, 0, MASS * GRAVITATIONAL_ACCELERATION};
static const Matrix<double, 3, 3> INERTIA_TENSOR{1, 0, 0, 0, 1, 0, 0, 0, 1};
static const Matrix<double, 3, 3> INERTIA_TENSOR_INV = INERTIA_TENSOR.inv();

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
