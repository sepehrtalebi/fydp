#pragma once

#include "KF.h"
#include "Expression.h"
#include "Matrix.h"
#include "Matrix3D.h"
#include <utility>

class EKF : public KF {
public:
    void updateKF(const SensorMeasurements &sensorMeasurements,
                  const double &dt) override;

private:
    static const Matrix3D<ExprPtr, 4, 4, 3> QUAT_TO_QUAT_JAC_EXPR;
    static const Matrix<double, 6, 6> WRENCH_TO_ACCEL_JAC;

    static std::pair<Matrix<double, n, n>, Matrix<double, n, 6>> fJacobian(const Vector<double, n> &x, const double &dt);
};
