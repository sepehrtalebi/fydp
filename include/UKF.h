#ifndef EKF_UKF_H
#define EKF_UKF_H

#pragma once

#include "KF.h"

class UKF: public KF {
private:
    Matrix<double, 2*n + 1, n> sigma = Matrix<double, 2*n + 1, n>::zeros();
    Matrix<double, n, n> P_cholesky = Matrix<double, n, n>::identity();
    constexpr static const float alpha = 0.1; //scaling factor for the spread of
                                                // the sigma points around the mean 0<=alpha<=1 usually small
    constexpr static const float kappa = 0; //secondary scaling factor
    constexpr static const float lambda = alpha*alpha * (n + kappa) - n; //scaling factor for sigma points
    double gamma = std::sqrt(n + lambda); //multiply by covariance for sigma points
                                            // TODO: should be const and static but cant figure it out
    void update(const SensorMeasurements &sensorMeasurements,
               const Vector3<double>& forces, const Vector3<double>& torques, double dt) override;
public:
    UKF();
};

#endif //EKF_UKF_H
