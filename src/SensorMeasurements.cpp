#include "SensorMeasurements.h"

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

Vector<double, 18> SensorMeasurements::getZ() const {
    return Vector<double, 18>{
            Pressure, RailAngle, RailPixelWidth, RailPixelX, RailPixelY, (double) FoundRail,
            PixelVelocity[0], PixelVelocity[1], IMUAcceleration.x, IMUAcceleration.y,
            IMUAcceleration.z,
            IMUAngularVelocity.x, IMUAngularVelocity.y, IMUAngularVelocity.z, Latitude, Longitude,
            GPSAltitude, (double) SatelliteCount};
}
