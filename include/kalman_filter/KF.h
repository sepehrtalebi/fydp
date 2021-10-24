#pragma once

#include "Vector.h"
#include "Matrix.h"
#include "Vector3.h"
#include "SensorMeasurements.h"
#include "AircraftState.h"
#include "ControlInputs.h"
#include "AppliedLoads.h"
#include "Constants.h"

class KF {
public:
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
    AppliedLoads applied_loads{};

public:
    KF();

    void updateWrapper(const double *doubleSensorMeasurements, const uint8_t *uint8SensorMeasurements,
                       const unsigned char *boolSensorMeasurements, const double *control_inputs, double dt);

    void getOutputWrapper(double *doubleAircraftState) const;

    virtual void update(const SensorMeasurements &sensorMeasurements,
                        const ControlInputs &control_inputs, double dt);

    [[nodiscard]] AircraftState getOutput() const;

protected:
    Vector<double, n> f(const Vector<double, n> &state, double dt);

    static Vector<double, p> h(const Vector<double, n> &state, double dt);
};


