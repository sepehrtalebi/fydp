#pragma once

#include "../../../bmb_math/include/bmb_math/Wrench.h"
#include "../../../bmb_math/include/bmb_math/Vector.h"
#include "../../../bmb_math/include/bmb_math/Matrix.h"
#include "../../../bmb_msgs/include/msgs/ControlInputs.h"
#include "Constants.h"
#include "../../../bmb_differentiation/include/bmb_differentiation/runtime/Expression.h"

class AppliedLoads {
public:
    void update(const ControlInputs &control_inputs);

    void updateWrapper(const double *control_inputs, const double *aircraft_state, double *forces, double *torques);

    [[nodiscard]] Wrench<double> getAppliedLoads(const Vector<double, n> &state) const;

    [[nodiscard]] Matrix<double, 6, n> getAppliedLoadsJacobian(const Vector<double, n> &state) const;

private:
    static const Matrix<ExprPtr, 3, 4> QUAT_TO_WEIGHT_JAC_EXPR;

    ControlInputs current_control_inputs{0, 0, 0, 0};
    ControlInputs last_control_inputs{0, 0, 0, 0};
    double last_propeller_ang_vel = 0;

    [[nodiscard]] double getPropellerAngVelocity() const;

    [[nodiscard]] Wrench<double> getPropellerLoads() const;

    [[nodiscard]] Wrench<double> getRightAileronLoads(const double &velocity) const;

    [[nodiscard]] Wrench<double> getLeftAileronLoads(const double &velocity) const;

    [[nodiscard]] Wrench<double> getElevatorLoads(const double &velocity) const;

    static Wrench<double> getEnvironmentalLoads(const Vector<double, n> &state);
};
