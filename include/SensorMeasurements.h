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
    double Pressure;
    double RailAngle;
    double RailPixelWidth;
    double RailPixelX;
    double RailPixelY;
    bool FoundRail;
    Vector<double, 2> PixelVelocity;
    Vector3<double> IMUAcceleration;
    Vector3<double> IMUAngularVelocity;
    double Latitude;
    double Longitude;
    double GPSAltitude;
    uint8_t SatelliteCount;

    static SensorMeasurements parseZ(const Vector<double, 16> &doubleZ, const Vector<uint8_t, 1> &uint8Z,
                                     const Vector<bool, 1> &boolZ);

    // may be unnecessary and replaced with more specific functions in the future
    [[nodiscard]] Vector<double, 18> getZ() const;
};
