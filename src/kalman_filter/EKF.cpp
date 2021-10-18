#include "EKF.h"
#include "Quaternion.h"
#include "Variable.h"
#include "AppliedLoads.h"
#include "Constants.h"
#include <map>
#include <string>

Matrix3D<ExprPtr, 4, 4, 3> get_quat_quat_jac_expr() {
    Matrix3D<ExprPtr, 4, 4, 3> expr;

    Quaternion<ExprPtr> quat{std::make_shared<Variable>("q0"),
                             std::make_shared<Variable>("q1"),
                             std::make_shared<Variable>("q2"),
                             std::make_shared<Variable>("q3")};
    Matrix<ExprPtr, 4, 3> mat = quat.E().transpose() * quat.cong().toDCM();
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) for (int k = 0; k < 3; k++)
        expr[i][j][k] = mat[j][k]->diff(std::static_pointer_cast<Variable>(quat[i])->getIdentifier())->simplify();

    return expr;
}

const Matrix3D<ExprPtr, 4, 4, 3> EKF::quat_to_quat_jac_expr = get_quat_quat_jac_expr();

void EKF::update(const SensorMeasurements &sensorMeasurements, const ControlInputs& control_inputs, double dt) {
    // call superclass update function first
    KF::update(sensorMeasurements, control_inputs, dt);

    // prediction step
    Matrix<double, n, n> f_jac = f_jacobian(x, dt);

    Vector<double, n> new_x = f(x, dt);
    for (int i = 0; i < n; i++) x[i] = new_x[i];

    Matrix<double, n, n> new_P = f_jac * P * f_jac.transpose() + Q;
    for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) P[i][j] = new_P[i][j];

    // update step
    Vector<double, p> z = sensorMeasurements.getZ();
    Vector<double, p> h_mat = h(x, dt);
    Matrix<double, p, n> h_jac = h_jacobian(x, dt);
    Matrix<double, n, p> h_jac_transpose = h_jac.transpose();

    Vector<double, p> y = z - h_mat;
    // multiplication order doesn't matter, both require n*p*(n+p) multiplications regardless of order
    Matrix<double, p, p> S = h_jac * P * h_jac_transpose + R;
    Matrix<double, n, p> K = P * h_jac_transpose * S.inv();

    x += K * y;
    P -= K * h_jac * P;
}

Matrix<double, n, n> EKF::f_jacobian(const Vector<double, n> &x, double dt) {
    // TODO: take into account the effect of getAppliedLoads on the jacobian

    // the ith row and jth column represents the derivative of
    // the ith output state with respect to the jth input state
    Matrix<double, n, n> f_jac = Matrix<double, n, n>::zeros();

    // setup some basic variables for use later
    Quaternion<double> quat{x[q0], x[q1], x[q2], x[q3]};
    Matrix<double, 3, 3> DCM_inv = quat.cong().toDCM();
    Vector3<double> w_abs = quat.unrotate(Vector3<double>{x[wx], x[wy], x[wz]});
    Quaternion<double> quat_new = quat + quat.E().transpose() * w_abs * (dt / 2);
    const std::map<std::string, double> subs = {{"q0", x[q0]},
                                                {"q1", x[q1]},
                                                {"q2", x[q2]},
                                                {"q3", x[q3]}};

    // TODO: get lambda working
    Matrix3D<double, 4, 4, 3> mat; // = quat_to_quat_jac_expr.applyFunc<double>([&subs] (const ExprPtr &expr) { return expr->evaluate(subs); });
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) for (int k = 0; k < 3; k++)
        mat[i][j][k] = quat_to_quat_jac_expr[i][j][k]->evaluate(subs);
    Matrix<double, 4, 4> quat_to_quat_jac = mat * Vector3<double>{x[wx], x[wy], x[wz]} * (dt / 2);

    Matrix<double, 4, 3> quat_to_w_jac = quat.E().transpose() * DCM_inv * dt / 2;

    // TODO: magnetic field derivatives with respect to quaternion and angular velocity
    /**
     * Derivation:
     * mag_new = quat_new.toDCM() * quat.cong().toDCM() * (mag - mag_b) + mag_b
     * Define M = quat_new.toDCM() * quat.cong().toDCM()
     * mag_new = M * (mag - mag_b) + mag_b
     * (mag_new - mag) / dt = (M * (mag - mag_b) + mag_b - mag) / dt
     * (mag_new - mag) / dt = (M - I) / dt * (mag - mag_b)
     * Thus, mag_to_mag_jac = (M - I) / dt and mag_mag_b_jac = -mag_to_mag_jac
     */
    Matrix<double, 3, 3> mag_to_mag_jac = (quat_new.toDCM() * DCM_inv - Matrix<double, 3, 3>::identity()) / dt;

    // fill in the separate elements into f_jac
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            f_jac[px + i][vx + j] = DCM_inv[i][j] * dt; // derivative of position with respect to velocity
            f_jac[magx + i][magx + j] = mag_to_mag_jac[i][j]; // derivative of magnetic field with respect to itself
            f_jac[magx + i][mag_bx + j] = -mag_to_mag_jac[i][j]; // derivative of magnetic field with respect to magnetic field bias
        }
        f_jac[vx + i][ax + i] = dt; // derivative of velocity with respect to acceleration
        f_jac[wx + i][ang_ax + i] = dt; // derivative of angular velocity with respect to angular acceleration
    }
    for (int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) f_jac[q0 + i][q0 + j] = quat_to_quat_jac[i][j]; // derivative of quaternion with respect to itself
        for (int j = 0; j < 3; j++) f_jac[q0 + i][wx + j] = quat_to_w_jac[i][j]; // derivative of quaternion with respect to angular velocity
    }

    return f_jac;
}

Matrix<double, p, n> EKF::h_jacobian(const Vector<double, n> &x, double dt) {
    // the ith row and jth column represents the derivative of
    // the ith output measurement with respect to the jth input state
    Matrix<double, p, n> h_jac = Matrix<double, p, n>::zeros();
    // TODO
    h_jac[SensorMeasurements::PRESSURE][px] = AIR_DENSITY * GRAVITATIONAL_ACCELERATION;

    h_jac[SensorMeasurements::IMU_wx][wx] = 1;
    h_jac[SensorMeasurements::IMU_wy][wy] = 1;
    h_jac[SensorMeasurements::IMU_wz][wz] = 1;

    h_jac[SensorMeasurements::ALT][pz] = -1;
    return h_jac;
}
