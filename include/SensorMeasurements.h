#pragma once

#include "Vector.h"
#include "Vector3.h"

struct SensorMeasurements {
    enum {
        PRESSURE,
        RAIL_ANGLE, RAIL_WIDTH, RAIL_x, RAIL_y, RAIL_FOUND,
        PIXEL_vx, PIXEL_vy,
        IMU_ax, IMU_ay, IMU_az,
        IMU_wx, IMU_wy, IMU_wz,
        LAT, LONG, ALT, SAT_COUNT
    };
    double pressure;
    double rail_angle;
    double rail_pixel_width;
    double rail_pixel_x;
    double rail_pixel_y;
    bool found_rail;
    Vector<2> pixel_velocity;
    Vector3<> imu_acceleration;
    Vector3<> imu_angular_velocity;
    double latitude;
    double longitude;
    double gps_altitude;
    uint8_t satellite_count;

    static SensorMeasurements parseZ(const Vector<16> &doubleZ, const Vector<1, uint8_t> &uint8Z,
                                     const Vector<1, bool> &boolZ);

    void assignZ(double *doubleZ, uint8_t *uint8tZ, unsigned char *boolZ) const;

    // may be unnecessary and replaced with more specific functions in the future
    [[nodiscard]] Vector<18> getZ() const;
};
