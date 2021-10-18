#include "KF.h"
#include "Quaternion.h"
#include "Constants.h"

KF::KF() {
    x[q0] = 1;
    P[px][px] = P[py][py] = 0;
}

void KF::updateWrapper(const double *doubleSensorMeasurements, const uint8_t *uint8SensorMeasurements,
                        const unsigned char *boolSensorMeasurements, const double *control_inputs, double dt) {
    Vector<double, 16> doubleZ;
    for (int i = 0; i < 16; i++) doubleZ[i] = doubleSensorMeasurements[i];
    Vector<uint8_t, 1> uint8Z{uint8SensorMeasurements[0]};
    Vector<bool, 1> boolZ{(bool) boolSensorMeasurements[0]};
    Vector<double, 4> control_inputsVec{control_inputs[0], control_inputs[1], control_inputs[2], control_inputs[3]};
    SensorMeasurements sensorMeasurements = SensorMeasurements::parseZ(doubleZ, uint8Z, boolZ);
    update(sensorMeasurements, ControlInputs::parseU(control_inputsVec), dt);
}

void KF::getOutputWrapper(double *doubleAircraftState) const {
    Vector<double, 19> aircraftState = getOutput().getX();
    for (int i = 0; i < 19; i++) doubleAircraftState[i] = aircraftState[i];
}

void KF::update(const SensorMeasurements & /** sensorMeasurements **/, const ControlInputs &control_inputs, double /** dt **/) {
    applied_loads.update(control_inputs);
}

AircraftState KF::getOutput() const {
    return AircraftState{Vector3<double>{x[px], x[py], x[pz]},
                         Quaternion<double>{x[q0], x[q1], x[q2], x[q3]},
                         Vector3<double>{x[vx], x[vy], x[vz]},
                         Vector3<double>{x[wx], x[wy], x[wz]},
                         Vector3<double>{x[ang_ax], x[ang_ay], x[ang_az]},
                         Vector3<double>{x[ax], x[ay], x[az]}};
}

Vector<double, n> KF::f(const Vector<double, n> &x, double dt) {
    Wrench<double> wrench = applied_loads.getAppliedLoads(x);

    Quaternion<double> quat = Quaternion<double>{x[q0], x[q1], x[q2], x[q3]};

    Vector<double, n> x_new = x;

    Vector3<double> v = Vector3<double>{x[vx], x[vy], x[vz]};
    Vector3<double> v_abs = quat.unrotate(v);

    Vector3<double> w = Vector3<double>{x[wx], x[wy], x[wz]};
    Vector3<double> w_abs = quat.unrotate(w);

    Quaternion<double> quat_new = Quaternion<double>{quat + quat.E().transpose() * w_abs * (dt / 2)};
    quat_new.normalize();

    Vector3<double> ang_a_new = Vector3<double>{INERTIA_TENSOR_INV * wrench.torque};

    Vector3<double> mag = Vector3<double>{x[magx], x[magy], x[magz]};
    Vector3<double> mag_b = Vector3<double>{x[mag_bx], x[mag_by], x[mag_bz]};
    Vector3<double> mag_new = Vector3<double>{quat_new.rotate(quat.unrotate(
            Vector3<double>{mag - mag_b})) + mag_b};

    for (int i = 0; i < 3; i++) {
        x_new[px + i] += v_abs[i] * dt;
        x_new[q0 + i] = quat_new[i];
        x_new[vx + i] += x[ax + i] * dt;
        x_new[wx + i] += x[ang_ax + i] * dt;
        x_new[ax + i] = wrench.force[i] / MASS;
        x_new[ang_ax + i] = ang_a_new[i];
        x_new[magx + i] = mag_new[i];
    }
    x_new[q3] = quat_new.q3; // not included in for loop

    return x_new;
}

Vector<double, p> KF::h(const Vector<double, n> &x, double dt) {
    Wrench<double> wrench = applied_loads.getAppliedLoads(x);
    // TODO
    return SensorMeasurements{ATMOSPHERIC_PRESSURE - AIR_DENSITY * GRAVITATIONAL_ACCELERATION * (-x[pz]),
                              0, 0, 0, 0, false,
                              Vector<double, 2>{0, 0},
                              Vector3<double>{wrench.force / MASS},
                              Vector3<double>{x[wx], x[wy], x[wz]},
                              0, 0, -x[pz], 8}
                              .getZ();
}
