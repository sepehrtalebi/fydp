#pragma once

#include "KF.h"
#include "Expression.h"
#include "Matrix3D.h"

class EKF : public KF {
public:
    void updateKF(const SensorMeasurements &sensorMeasurements,
                  const double &dt) override;

private:
    static const Matrix3D<ExprPtr, 4, 4, 3> QUAT_TO_QUAT_JAC_EXPR;

    [[nodiscard]] Matrix<double, n, n> fJacobian(const Vector<double, n> &x, const double &dt) const;

    [[nodiscard]] Matrix<double, p, n> hJacobian(const Vector<double, n> &x, const double &dt) const;
};
