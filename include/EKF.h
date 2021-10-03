#pragma once

#include "KF.h"
#include "Expression.h"
#include "Matrix3D.h"

class EKF : public KF {
public:
    EKF();

    void update(const SensorMeasurements &sensorMeasurements,
                const Vector3<double>& forces, const Vector3<double>& torques, double dt) override;
private:
    const Matrix3D<ExprPtr, 4, 4, 3> quat_quat_jac;

    Matrix<double, n, n> f_jacobian(const Vector3<double>& f, const Vector3<double>& T, double dt) const;

    Matrix<double, p, n> h_jacobian(const Vector3<double>& f, const Vector3<double>& T, double dt) const;
};
