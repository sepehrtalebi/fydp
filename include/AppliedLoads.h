#pragma once

#include "Wrench.h"
#include "Vector.h"
#include "ControlInputs.h"
#include "Constants.h"

class AppliedLoads {
public:
    void update(const ControlInputs &control_inputs);

    void updateWrapper(const double *control_inputs, const double *aircraft_state, double *forces, double *torques);

    [[nodiscard]] Wrench<double> getAppliedLoads(const Vector<double, n> &state) const;

private:
    ControlInputs current_control_inputs{0, 0, 0, 0};
    ControlInputs last_control_inputs{0, 0, 0, 0};
    double last_propeller_ang_vel = 0;

    static double saturation(const double &value, const double &limit);

    static double saturation(const double &value, const double &min, const double &max);

    [[nodiscard]] double getPropellerAngVelocity() const;

    [[nodiscard]] Wrench<double> getPropellerLoads() const;

    [[nodiscard]] Wrench<double> getRightAileronLoads(const double &velocity) const;

    [[nodiscard]] Wrench<double> getLeftAileronLoads(const double &velocity) const;

    [[nodiscard]] Wrench<double> getElevatorLoads(const double &velocity) const;

    static Wrench<double> getEnvironmentalLoads(const Vector<double, n> &state);
};
