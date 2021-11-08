#include "SensorModels.h"
#include "Vector3.h"
#include "Quaternion.h"
#include "KF.h"
#include <cmath>

#if SENSOR_NOISE == 1
#include <chrono>
#include <random>
static unsigned seed = std::chrono::system_clock::now().time_since_epoch().count(); // NOLINT(cert-err58-cpp)
static auto random_engine = std::default_random_engine(seed); // NOLINT(cert-err58-cpp)
static std::normal_distribution<double> optical_flow_noise(0, 1); // NOLINT(cert-err58-cpp)
#endif

static void railDetection(const Quaternion<double> &quat, const double &altitude, SensorMeasurements &sensor_measurements) {
    if (altitude < 0) {
        // cannot see rail if we are underground
        // leave found_rail as false and leave rail_pixel_x, rail_pixel_y, rail_pixel_width, and rail_angle as 0
        return;
    }

    Vector3<double> down_earth = quat.unrotate({0, 0, 1}); // down (i.e. the direction the camera is pointing) in earth coordinates
    if (down_earth.z < 0) {
        // cannot see rail if the plane is upside down
        return;
    }

    // rail offset in body coordinates
    Vector3<double> rail_offset_body = quat.rotate(down_earth * altitude / down_earth.z);

    Vector3<double> rail_pixel_location = rail_offset_body * CAMERA_GAIN / rail_offset_body.z;
    if (HALF_CAMERA_HORIZONTAL_PIXELS < std::abs(rail_pixel_location.x) ||
            HALF_CAMERA_VERTICAL_PIXELS < std::abs(rail_pixel_location.y)) {
        // rail tracks are out of the field of view of the camera
        return;
    }

    // rail tracks are visible
    sensor_measurements.found_rail = true;
    sensor_measurements.rail_pixel_x = rail_pixel_location.x;
    sensor_measurements.rail_pixel_y = rail_pixel_location.y;
    sensor_measurements.rail_pixel_width = RAIL_WIDTH * CAMERA_GAIN / rail_offset_body.z;

    Vector3<double> rail_direction_body = quat.rotate(NORTH); // rail direction in body coordinates
    Vector3<double> rail_direction_pixels = rail_direction_body * CAMERA_GAIN / rail_offset_body.z;
    sensor_measurements.rail_angle = std::atan2(rail_direction_pixels.y, rail_direction_pixels.x);
}

static void opticalFlow(const Vector3<double> &velocity, const double &altitude, SensorMeasurements &sensor_measurements) {
    Vector3<double> pixel_velocity = velocity* OPTICAL_FLOW_VELOCITY_GAIN / altitude;
    sensor_measurements.pixel_velocity[0] = pixel_velocity.x;
    sensor_measurements.pixel_velocity[1] = pixel_velocity.y;
#if SENSOR_NOISE == 1
    sensor_measurements.pixel_velocity[0] += optical_flow_noise(random_engine);
    sensor_measurements.pixel_velocity[1] += optical_flow_noise(random_engine);
#endif
}

SensorMeasurements getSensorMeasurements(const Vector<double, n> &state) {
    Quaternion<double> quat{state[KF::q0], state[KF::q1], state[KF::q2], state[KF::q3]};
    Vector3<double> velocity{state[KF::vx], state[KF::vy], state[KF::vz]};
    double altitude = -state[KF::pz];

    SensorMeasurements sensor_measurements;
    railDetection(quat, altitude, sensor_measurements);
    opticalFlow(velocity, altitude, sensor_measurements);

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

void getSensorMeasurementsWrapper(const double *aircraft_state, double *double_sensor_measurements,
                                  uint8_t *uint8_sensor_measurements, unsigned char *bool_sensor_measurements) {
    Vector<double, n> aircraft_state_vec;

    // only copy over the position, orientation, velocity, and angular velocity
    // TODO: clean up
    for (size_t i = 0; i < 13; i++) aircraft_state_vec[i] = aircraft_state[i];

    getSensorMeasurements(aircraft_state_vec).assignZ(double_sensor_measurements, uint8_sensor_measurements, bool_sensor_measurements);
}

Matrix<double, p, n> getSensorMeasurementsJacobian(const Vector<double, n> &state, const Wrench<double> &current_loads) {
    // the ith row and jth column represents the derivative of
    // the ith output measurement with respect to the jth input state
    Matrix<double, p, n> h_jac = Matrix<double, p, n>::zeros();
    // TODO
    h_jac[SensorMeasurements::PRESSURE][KF::px] = AIR_DENSITY * GRAVITATIONAL_ACCELERATION;

    h_jac[SensorMeasurements::IMU_wx][KF::wx] = 1;
    h_jac[SensorMeasurements::IMU_wy][KF::wy] = 1;
    h_jac[SensorMeasurements::IMU_wz][KF::wz] = 1;

    h_jac[SensorMeasurements::ALT][KF::pz] = -1;

    return h_jac;
}
