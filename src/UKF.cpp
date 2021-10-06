#include "UKF.h"

UKF::UKF() {
    this->P_cholesky = P.cholesky();
    this->x[q0] = 1;
    this->P[px][px] = this->P[py][py] = 0;
}

void UKF::update(const SensorMeasurements &sensorMeasurements,
            const Vector3<double>& forces, const Vector3<double>& torques, double dt) {
    Vector<double, UKF::n> x_new = f(forces, torques, dt);
    auto new_sigma = sigma;

    for (int i = 0; i < n; i++) { //i is column number, j is row number
        new_sigma[0][i] = this->x[i];
        for (int j = 1; j < n; j++) new_sigma[j][i] = this->x[i] + P_cholesky[j][i] * gamma;
        for (int j = n+1; j < 2*n; j++) new_sigma[j][i] = this->x[i] - P_cholesky[j][i] * gamma;
    }

    //TODO: transform sigma through state update function. this is state sigma
    //TODO: state weights i = 0: w = lambda / (N + lambda) i = 1 to 2N: w = 1 / (2 * (N + lambda))
    //TODO: covariance weights i = 0: w = lambda / (N + lambda) + (1 - alpha*alpha + beta) i = 1 to 2N: w is same
    //TODO: state estimate is state weights times state sigma
    //TODO: state covariance is covariance weight times new sigma - state estimate dot itself
    //TODO: transform sigma points through measurement function
    //TODO: mean measurement vector is state weights times transformed sigma points
    //TODO: measurement covariance is covariance weights times transformed sigma points minus mean measurement vector dot itself
    //TODO: cross covariance is state covariance and measurement covariance
    //TODO: Kalman gain is cross covariance times inverse of measurement covariance
    //TODO: standard kalman gain update


}