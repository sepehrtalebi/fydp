#include "SensorModels.h"

static void railDetection(const Vector<double, n> &state, SensorMeasurements &sensor_measurements) {
    sensor_measurements.found_rail = false;
}

SensorMeasurements getSensorMeasurements(const Vector<double, n> &state) {
    SensorMeasurements sensor_measurements;
    railDetection(state, sensor_measurements);

//    Wrench<double> wrench = applied_loads.getAppliedLoads(x);
//    // TODO
//    return SensorMeasurements{ATMOSPHERIC_PRESSURE - AIR_DENSITY * GRAVITATIONAL_ACCELERATION * (-x[pz]),
//                              0, 0, 0, 0, false,
//                              Vector<double, 2>{0, 0},
//                              Vector3<double>{wrench.force / MASS},
//                              Vector3<double>{x[wx], x[wy], x[wz]},
//                              0, 0, -x[pz], 8}
//                              .getZ();
    return sensor_measurements;
}
