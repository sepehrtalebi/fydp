#pragma once

#include "Wrench.h"
#include "Vector.h"
#include "ControlInputs.h"
#include "KF.h"

class AppliedLoads {
public:
    AppliedLoads();

    void update(const ControlInputs &control_inputs);

    void updateWrapper(const double *control_inputs, const double *aircraft_state, double *forces, double *torques);

    [[nodiscard]] Wrench<double> getAppliedLoads(const Vector<double, KF::n> &state) const;

private:
    ControlInputs current_control_inputs;
    ControlInputs last_control_inputs;
    double last_propeller_ang_vel;

    static double saturation(const double &value, const double &limit);

    static double saturation(const double &value, const double &min, const double &max);

    [[nodiscard]] double getPropellerAngVelocity() const;

    [[nodiscard]] Wrench<double> getPropellerLoads() const;

    [[nodiscard]] Wrench<double> getRightAileronLoads(const double &velocity) const;

    [[nodiscard]] Wrench<double> getLeftAileronLoads(const double &velocity) const;

    [[nodiscard]] Wrench<double> getElevatorLoads(const double &velocity) const;

    static Wrench<double> getEnvironmentalLoads(const Vector<double, KF::n> &state);
};
