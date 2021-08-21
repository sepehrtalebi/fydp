#include "EKF.h"
#include "Quaternion.h"

EKF::EKF() {
    this->x[q0] = 1;
    this->P[px][px] = this->P[py][py] = 0;
}

void EKF::update(const SensorMeasurements &sensorMeasurements, const Vector3& forces, const Vector3& torques, double dt) {
    // prediction step
    Matrix<n, n> f_jac = f_jacobian(forces, torques, dt);

    Vector<n> new_x = this->f(forces, torques, dt);
    for (int i = 0; i < n; i++) this->x[i] = new_x[i];

    Matrix<n, n> new_P = f_jac * this->P * f_jac.transpose() + this->Q;
    for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) this->P[i] = new_P[i];

    // update step
    Vector<p> z = sensorMeasurements.getZ();
    Vector<p> h = this->h(forces, torques, dt);
    Matrix<p, n> h_jac = this->h_jacobian(forces, torques, dt);
    Matrix<n, p> h_jac_transpose = h_jac.transpose();

    Vector<p> y = z - h;
    // multiplication order doesn't matter, both require n*p*(n+p) multiplications regardless of order
    Matrix<p, p> S = h_jac * this->P * h_jac_transpose + this->R;
    Matrix<n, p> K = this->P * h_jac_transpose * S.inv();

    this->x += K * y;
    this->P -= K * h_jac * this->P;
}

void EKF::getOutput(AircraftState *aircraftState, RailLocation *railLocation) const {
    Quaternion quat{this->x[q0], this->x[q1], this->x[q2], this->x[q3]};
    Vector3 eulerAngles = quat.toEulerAngles();

    for (int i = 0; i < 3; i++) {
        aircraftState->Position[i] = this->x[px + 1];
        aircraftState->EulerAngles[i] = eulerAngles[i];
        aircraftState->BodyVelocity[i] = this->x[vx + i];
        aircraftState->BodyAngularVelocity[i] = this->x[wx + i];
        aircraftState->BodyAngularAcceleration[i] = this->x[ang_ax + i];
        aircraftState->BodyAcceleration[i] = this->x[ang_ax + i];
    }
    aircraftState->DCM = quat.toDCM();
    railLocation->RailOffset = Vector<2>{this->x[rail_x], this->x[rail_y]};
    railLocation->RailHeading = this->x[rail_h];
}

Vector<EKF::n> EKF::f(const Vector3 &f, const Vector3 &T, double dt) const {
    Quaternion quat = Quaternion{x[q0], x[q1], x[q2], x[q3]};

    Vector<n> x_new = x;

    Vector3 v = Vector3{x[vx], x[vy], x[vz]};
    Vector3 v_abs = quat.unrotate(v);

    Vector3 w = Vector3{x[wx], x[wy], x[wz]};
    Vector3 w_abs = quat.unrotate(w);

    Quaternion quat_new = Quaternion{quat + quat.E().transpose() * w_abs * (dt / 2)};
    quat_new.normalize();

    Vector3 ang_a_new = Vector3{inertia_inv * T};

    Vector3 mag = Vector3{x[magx], x[magy], x[magz]};
    Vector3 mag_b = Vector3{x[mag_bx], x[mag_by], x[mag_bz]};
    Vector3 mag_new = Vector3{quat_new.rotate(quat.unrotate(Vector3{mag - mag_b})) + mag_b};

    for (int i = 0; i < 3; i++) {
        x_new[px + i] += v_abs[i] * dt;
        x_new[q0 + i] = quat_new[i];
        x_new[vx + i] += x[ax + i] * dt;
        x_new[wx + i] += x[ang_ax + i] * dt;
        x_new[ax + i] = f[i] / this->m;
        x_new[ang_ax + i] = ang_a_new[i];
        x_new[magx + i] = mag_new[i];
    }
    x_new[q3] = quat_new.q3; // not included in for loop

    return x_new;
}

Matrix<EKF::n, EKF::n> EKF::f_jacobian(const Vector3 &f, const Vector3 &T, double dt) const {
    // the ith row and jth column represents the derivative of
    // the ith output state with respect to the jth input state
    Matrix<n, n> f_jac = Matrix<n, n>::zeros();

    Quaternion quat = Quaternion{x[q0], x[q1], x[q2], x[q3]};
    Matrix<3, 3> DCM_inv_dt = quat.cong().toDCM() * dt;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) f_jac[px + i][vx + j] = DCM_inv_dt[i][j];
        f_jac[vx + i][ax + i] = dt;
        f_jac[wx + i][ang_ax + i] = dt;
        // TODO: quaternion derivatives and magnetic field derivatives
    }
    return f_jac;
}

Vector<EKF::p> EKF::h(const Vector3 &f, const Vector3 &T, double dt) const {
    // TODO
    return SensorMeasurements{P_atm - rho_air * g * (-this->x[pz]),
                              0, 0, 0, 0, 0,
                              Vector<2>{0, 0},
                              Vector3{f / m},
                              Vector3{this->x[wx], this->x[wy], this->x[wz]},
                              0, 0, -this->x[pz], 8}
                              .getZ();
}


Matrix<EKF::p, EKF::n> EKF::h_jacobian(const Vector3 &f, const Vector3 &T, double dt) const {
    // the ith row and jth column represents the derivative of
    // the ith output measurement with respect to the jth input state
    Matrix<p, n> h_jac = Matrix<p, n>::zeros();
    // TODO
    h_jac[SensorMeasurements::P][px] = rho_air * g;

    h_jac[SensorMeasurements::IMU_wx][wx] = 1;
    h_jac[SensorMeasurements::IMU_wy][wy] = 1;
    h_jac[SensorMeasurements::IMU_wz][wz] = 1;

    h_jac[SensorMeasurements::alt][pz] = -1;
    return h_jac;
}
