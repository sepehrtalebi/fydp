#pragma once

#include "Vector.h"
#include "Matrix.h"
#include "Vector3.h"
#include "SensorMeasurements.h"
#include "AircraftState.h"
#include "ControlInputs.h"

class KF {
public:
    constexpr static const int n = 31; // number of states
    constexpr static const int p = 18; // number of sensor measurements
    constexpr static const double g = 9.81; // gravitational acceleration
    constexpr static const double P_atm = 101325; // Pa
    constexpr static const double rho_air = 1.225; // kg/m^3
    static const Matrix<double, 3, 3> inertia_inv; // 1/(kg * m^2)
    enum {
        px, py, pz,  // position in earth frame
        q0, q1, q2, q3,  // rotation from earth frame to body frame
        vx, vy, vz,  // velocity in body frame
        wx, wy, wz,  // angular velocity in body frame
        ax, ay, az,  // acceleration in body frame
        ang_ax, ang_ay, ang_az,  //angular acceleration in body frame
        magx, magy, magz,  // magnetic field vector in body frame
        accel_bx, accel_by, accel_bz,  // accelerometer bias in body frame
        gyro_bx, gyro_by, gyro_bz,  // gyroscope bias in body frame
        mag_bx, mag_by, mag_bz  // magnetometer bias in body frame
    };

protected:
    Vector<double, n> x{};
    Matrix<double, n, n> P = Matrix<double, n, n>::identity();
    const Matrix<double, n, n> Q = Matrix<double, n, n>::identity();
    const Matrix<double, p, p> R = Matrix<double, p, p>::identity();

public:
    KF();

    void updateWrapper(const double *doubleSensorMeasurements, const uint8_t *uint8SensorMeasurements,
                       const unsigned char *boolSensorMeasurements, const double *control_inputs, double dt);

    void getOutputWrapper(double *doubleAircraftState) const;

    virtual void update(const SensorMeasurements &sensorMeasurements,
                        const ControlInputs &control_inputs, double dt) = 0;

    AircraftState getOutput() const;

protected:
    static Vector<double, n> f(const Vector<double, n> &x, const ControlInputs &control_inputs, double dt);

    static Vector<double, p> h(const Vector<double, n> &x, const ControlInputs &control_inputs, double dt);
};


