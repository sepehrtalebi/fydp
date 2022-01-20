#include "../include/msgs/SensorMeasurements.h"

SensorMeasurements SensorMeasurements::parseZ(const Vector<double, 16> &doubleZ, const Vector<uint8_t, 1> &uint8Z,
                                              const Vector<bool, 1> &boolZ) {
    return SensorMeasurements{
            doubleZ[0],
            doubleZ[1], doubleZ[2], doubleZ[3], doubleZ[4], boolZ[0],
            Vector<double, 2>{doubleZ[5], doubleZ[6]},
            Vector3<double>{doubleZ[7], doubleZ[8], doubleZ[9]},
            Vector3<double>{doubleZ[10], doubleZ[11], doubleZ[12]},
            doubleZ[13], doubleZ[14], doubleZ[15], uint8Z[0]};
}

void SensorMeasurements::assignZ(double *doubleZ, uint8_t *uint8tZ, unsigned char *boolZ) const {
    doubleZ[0] = pressure;
    doubleZ[1] = rail_angle;
    doubleZ[2] = rail_pixel_width;
    doubleZ[3] = rail_pixel_x;
    doubleZ[4] = rail_pixel_y;
    doubleZ[5] = pixel_velocity[0];
    doubleZ[6] = pixel_velocity[1];
    doubleZ[7] = imu_acceleration[0];
    doubleZ[8] = imu_acceleration[1];
    doubleZ[9] = imu_acceleration[2];
    doubleZ[10] = imu_angular_velocity[0];
    doubleZ[11] = imu_angular_velocity[1];
    doubleZ[12] = imu_angular_velocity[2];
    doubleZ[13] = latitude;
    doubleZ[14] = longitude;
    doubleZ[15] = gps_altitude;
    uint8tZ[0] = satellite_count;
    boolZ[0] = found_rail;
}

Vector<double, 18> SensorMeasurements::getZ() const {
    return Vector<double, 18>{
            pressure, rail_angle, rail_pixel_width, rail_pixel_x, rail_pixel_y, (double) found_rail,
            pixel_velocity[0], pixel_velocity[1], imu_acceleration.x, imu_acceleration.y,
            imu_acceleration.z,
            imu_angular_velocity.x, imu_angular_velocity.y, imu_angular_velocity.z, latitude, longitude,
            gps_altitude, (double) satellite_count};
}
