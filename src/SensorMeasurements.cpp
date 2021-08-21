#include "SensorMeasurements.h"

Vector<18> SensorMeasurements::getZ() const {
    return Vector<18>{Pressure, RailAngle, RailPixelWidth, RailPixelX, RailPixelY, FoundRail,
                      PixelVelocity[0], PixelVelocity[1], IMUAcceleration.x, IMUAcceleration.y, IMUAcceleration.z,
                      IMUAngularVelocity.x, IMUAngularVelocity.y, IMUAngularVelocity.z, Latitude, Longitude,
                      GPSAltitude, SatelliteCount};
}
