#pragma once

#include "Vector.h"
#include "Matrix.h"
#include "Vector3.h"
#include "SensorMeasurements.h"
#include "AircraftState.h"
#include "RailLocation.h"

class EKF {
public:
    constexpr static const int n = 34; // number of states
    constexpr static const int p = 18; // number of sensor measurements
    constexpr static const double g = 9.81; // gravitational acceleration
    constexpr static const double P_atm = 101325; // Pa
    constexpr static const double rho_air = 1.225; // kg/m^3
    enum {
        px, py, pz,  // position in earth  frame
        q0, q1, q2, q3,  // rotation from earth frame to body frame
        vx, vy, vz,  // velocity in body frame
        wx, wy, wz,  // angular velocity in body frame
        ax, ay, az,  // acceleration in body frame
        ang_ax, ang_ay, ang_az,  //angular acceleration in body frame
        magx, magy, magz,  // magnetic field vector in body frame
        accel_bx, accel_by, accel_bz,  // accelerometer bias in body frame
        gyro_bx, gyro_by, gyro_bz,  // gyroscope bias in body frame
        mag_bx, mag_by, mag_bz,  // magnetometer bias in body frame
        rail_x, rail_y,  // rail offset position in earth frame
        rail_h  // rail heading in earth frame
    };

private:
    Vector<n> x{};
    Matrix<n, n> P = Matrix<n, n>::identity();
    const double m = 1; // kg
    const Matrix<3, 3> inertia_inv = Matrix<3, 3>::identity(); // 1/(kg * m^2)
    const Matrix<n, n> Q = Matrix<n, n>::identity();
    const Matrix<p, p> R = Matrix<p, p>::identity();

public:
    EKF();

    void update(const SensorMeasurements &sensorMeasurements, const Vector3& forces, const Vector3& torques, double dt);

    void getOutput(AircraftState *aircraftState, RailLocation *railLocation) const;

private:
    Vector<n> f(const Vector3& f, const Vector3& T, double dt) const;

    Matrix<n, n> f_jacobian(const Vector3& f, const Vector3& T, double dt) const;

    Vector<p> h(const Vector3& f, const Vector3& T, double dt) const;

    Matrix<p, n> h_jacobian(const Vector3& f, const Vector3& T, double dt) const;
};
