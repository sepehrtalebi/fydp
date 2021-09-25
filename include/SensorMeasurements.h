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
    double FoundRail;
    Vector<2> PixelVelocity;
    Vector3 IMUAcceleration;
    Vector3 IMUAngularVelocity;
    double Latitude;
    double Longitude;
    double GPSAltitude;
    double SatelliteCount;

    Vector<18> getZ() const;
};
