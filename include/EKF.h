#pragma once

#include "KF.h"

class EKF : public KF {
public:
    void update(const SensorMeasurements &sensorMeasurements,
                const Vector3<double>& forces, const Vector3<double>& torques, double dt) override;
private:
    Matrix<double, n, n> f_jacobian(const Vector3<double>& f, const Vector3<double>& T, double dt) const;

    Matrix<double, p, n> h_jacobian(const Vector3<double>& f, const Vector3<double>& T, double dt) const;
};
