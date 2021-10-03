#include "EKF.h"
#include "Quaternion.h"
#include "Variable.h"
#include <map>
#include <string>

EKF::EKF() {
    Quaternion<ExprPtr> quat{std::make_shared<Variable>("q0"),
                             std::make_shared<Variable>("q1"),
                             std::make_shared<Variable>("q2"),
                             std::make_shared<Variable>("q3")};
    Matrix<ExprPtr, 4, 3> mat = quat.E().transpose() * quat.cong().toDCM();
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) for (int k = 0; k < 3; k++)
        quat_quat_jac[i][j][k] = mat[j][k]->diff(std::static_pointer_cast<Variable>(quat[i])->getIdentifier())->simplify();
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
    }

    const std::map<std::string, double> subs = {{"q0", x[q0]},
                                                {"q1", x[q1]},
                                                {"q2", x[q2]},
                                                {"q3", x[q3]}};
    // TODO: get lambda working
    Matrix3D<double, 4, 4, 3> mat; // = quat_quat_jac.applyFunc<double>([&subs] (const ExprPtr &expr) { return expr->evaluate(subs); });
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) for (int k = 0; k < 3; k++)
        mat[i][j][k] = quat_quat_jac[i][j][k]->evaluate(subs);
    Matrix<double, 4, 4> quat_quat_jac_val = mat * Vector3<double>{x[wx], x[wy], x[wz]} * (dt / 2);
    for (int i = 0; i < 4; i++) for(int j = 0; j < 4; j++) f_jac[q0 + i][q0 + j] = quat_quat_jac_val[i][j];

    // Matrix that is multiplied by w to given the rate of change of quat
    Matrix<double, 4, 3> w_to_quat_jac = quat.E().transpose() * DCM_inv_dt / 2;
    for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++)
        f_jac[q0 + i][wx + j] = w_to_quat_jac[i][j];

    // TODO: magnetic field derivatives

    return f_jac;
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
