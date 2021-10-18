#pragma once

#include "Vector.h"
#include "Vector3.h"

struct SensorMeasurements {
    enum {
        P,
        rail_a, rail_w, rail_x, rail_y, rail_found,
        pixel_vx, pixel_vy,
        IMU_ax, IMU_ay, IMU_az,
        IMU_wx, IMU_wy, IMU_wz,
        lat, longitude, alt, sat_count
    };
    double pressure;
    double rail_angle;
    double rail_pixel_width;
    double rail_pixel_x;
    double rail_pixel_y;
    bool found_rail;
    Vector<double, 2> pixel_velocity;
    Vector3<double> imu_acceleration;
    Vector3<double> imu_angular_velocity;
    double latitude;
    double Longitude;
    double gps_altitude;
    uint8_t satellite_count;

    static SensorMeasurements parseZ(const Vector<double, 16> &doubleZ, const Vector<uint8_t, 1> &uint8Z,
                                     const Vector<bool, 1> &boolZ);

    // may be unnecessary and replaced with more specific functions in the future
    [[nodiscard]] Vector<double, 18> getZ() const;
};
