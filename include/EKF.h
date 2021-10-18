#pragma once

#include "KF.h"
#include "Expression.h"
#include "Matrix3D.h"

class EKF : public KF {
public:
    void update(const SensorMeasurements &sensorMeasurements,
                const ControlInputs &control_inputs, double dt) override;

private:
    static const Matrix3D<ExprPtr, 4, 4, 3> quat_to_quat_jac_expr;

    Matrix<double, n, n> f_jacobian(const Vector<double, n> &x, double dt);

    Matrix<double, p, n> h_jacobian(const Vector<double, n> &x, double dt);
};
