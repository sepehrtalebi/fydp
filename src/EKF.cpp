#include "EKF.h"
#include "Quaternion.h"

EKF::EKF() {
    x[q0] = 1;
    P[px][px] = P[py][py] = 0;
}

void EKF::updateWrapper(const double *doubleSensorMeasurements, const uint8_t *uint8SensorMeasurements,
                        const unsigned char *boolSensorMeasurements, const double *forces, const double *torques, double dt) {
    Vector<double, 16> doubleZ;
    for (int i = 0; i < 16; i++) doubleZ[i] = doubleSensorMeasurements[i];
    Vector<uint8_t, 1> uint8Z{uint8SensorMeasurements[0]};
    Vector<bool, 1> boolZ{(bool) boolSensorMeasurements[0]};
    Vector3<double> forcesVec{forces[0], forces[1], forces[2]};
    Vector3<double> torquesVec{torques[0], torques[1], torques[2]};
    SensorMeasurements sensorMeasurements = SensorMeasurements::parseZ(doubleZ, uint8Z, boolZ);
    update(sensorMeasurements, forcesVec, torquesVec, dt);
}

void EKF::getOutputWrapper(double *doubleAircraftState) const {
    Vector<double, 19> aircraftState = getOutput().getX();
    for (int i = 0; i < 19; i++) doubleAircraftState[i] = aircraftState[i];
}

void EKF::update(const SensorMeasurements &sensorMeasurements, const Vector3<double>& forces, const Vector3<double>& torques, double dt) {
    // prediction step
    Matrix<double, n, n> f_jac = f_jacobian(forces, torques, dt);

    Vector<double, n> new_x = f(forces, torques, dt);
    for (int i = 0; i < n; i++) x[i] = new_x[i];

    Matrix<double, n, n> new_P = f_jac * P * f_jac.transpose() + Q;
    for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) P[i][j] = new_P[i][j];

    // update step
    Vector<double, p> z = sensorMeasurements.getZ();
    Vector<double, p> h_mat = h(forces, torques, dt);
    Matrix<double, p, n> h_jac = h_jacobian(forces, torques, dt);
    Matrix<double, n, p> h_jac_transpose = h_jac.transpose();

    Vector<double, p> y = z - h_mat;
    // multiplication order doesn't matter, both require n*p*(n+p) multiplications regardless of order
    Matrix<double, p, p> S = h_jac * P * h_jac_transpose + R;
    Matrix<double, n, p> K = P * h_jac_transpose * S.inv();

    x += K * y;
    P -= K * h_jac * P;
}

AircraftState EKF::getOutput() const {
    return AircraftState{Vector3<double>{x[px], x[py], x[pz]},
                         Quaternion<double>{x[q0], x[q1], x[q2], x[q3]},
                         Vector3<double>{x[vx], x[vy], x[vz]},
                         Vector3<double>{x[wx], x[wy], x[wz]},
                         Vector3<double>{x[ang_ax], x[ang_ay], x[ang_az]},
                         Vector3<double>{x[ax], x[ay], x[az]}};
}

Vector<double, EKF::n> EKF::f(const Vector3<double> &f, const Vector3<double> &T, double dt) const {
    Quaternion<double> quat = Quaternion<double>{x[q0], x[q1], x[q2], x[q3]};

    Vector<double, n> x_new = x;

    Vector3<double> v = Vector3<double>{x[vx], x[vy], x[vz]};
    Vector3<double> v_abs = quat.unrotate(v);

    Vector3<double> w = Vector3<double>{x[wx], x[wy], x[wz]};
    Vector3<double> w_abs = quat.unrotate(w);

    Quaternion<double> quat_new = Quaternion<double>{quat + quat.E().transpose() * w_abs * (dt / 2)};
    quat_new.normalize();

    Vector3<double> ang_a_new = Vector3<double>{inertia_inv * T};

    Vector3<double> mag = Vector3<double>{x[magx], x[magy], x[magz]};
    Vector3<double> mag_b = Vector3<double>{x[mag_bx], x[mag_by], x[mag_bz]};
    Vector3<double> mag_new = Vector3<double>{quat_new.rotate(quat.unrotate(
            Vector3<double>{mag - mag_b})) + mag_b};

    for (int i = 0; i < 3; i++) {
        x_new[px + i] += v_abs[i] * dt;
        x_new[q0 + i] = quat_new[i];
        x_new[vx + i] += x[ax + i] * dt;
        x_new[wx + i] += x[ang_ax + i] * dt;
        x_new[ax + i] = f[i] / m;
        x_new[ang_ax + i] = ang_a_new[i];
        x_new[magx + i] = mag_new[i];
    }
    x_new[q3] = quat_new.q3; // not included in for loop

    return x_new;
}

Matrix<double, EKF::n, EKF::n> EKF::f_jacobian(const Vector3<double> &f, const Vector3<double> &T, double dt) const {
    // the ith row and jth column represents the derivative of
    // the ith output state with respect to the jth input state
    Matrix<double, n, n> f_jac = Matrix<double, n, n>::zeros();

    Quaternion<double> quat = Quaternion<double>{x[q0], x[q1], x[q2], x[q3]};
    Matrix<double, 3, 3> DCM_inv_dt = quat.cong().toDCM() * dt;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) f_jac[px + i][vx + j] = DCM_inv_dt[i][j];
        f_jac[vx + i][ax + i] = dt;
        f_jac[wx + i][ang_ax + i] = dt;
        // TODO: quaternion derivatives and magnetic field derivatives
    }
    return f_jac;
}

Vector<double, EKF::p> EKF::h(const Vector3<double> &f, const Vector3<double> &T, double dt) const {
    // TODO
    return SensorMeasurements{P_atm - rho_air * g * (-x[pz]),
                              0, 0, 0, 0, false,
                              Vector<double, 2>{0, 0},
                              Vector3<double>{f / m},
                              Vector3<double>{x[wx], x[wy], x[wz]},
                              0, 0, -x[pz], 8}
                              .getZ();
}


Matrix<double, EKF::p, EKF::n> EKF::h_jacobian(const Vector3<double> &f, const Vector3<double> &T, double dt) const {
    // the ith row and jth column represents the derivative of
    // the ith output measurement with respect to the jth input state
    Matrix<double, p, n> h_jac = Matrix<double, p, n>::zeros();
    // TODO
    h_jac[SensorMeasurements::P][px] = rho_air * g;

    h_jac[SensorMeasurements::IMU_wx][wx] = 1;
    h_jac[SensorMeasurements::IMU_wy][wy] = 1;
    h_jac[SensorMeasurements::IMU_wz][wz] = 1;

    h_jac[SensorMeasurements::alt][pz] = -1;
    return h_jac;
}
