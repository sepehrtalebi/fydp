#include "AircraftState.h"

Vector<double, 19> AircraftState::getX() const {
    return Vector<double, 19>{
            Position.x, Position.y, Position.z,
            Orientation.q0, Orientation.q1, Orientation.q2, Orientation.q3,
            BodyVelocity.x, BodyVelocity.y, BodyVelocity.z,
            BodyAngularVelocity.x, BodyAngularVelocity.y, BodyAngularVelocity.z,
            BodyAngularAcceleration.x, BodyAngularAcceleration.y, BodyAngularAcceleration.z,
            BodyAcceleration.x, BodyAcceleration.y, BodyAcceleration.z};
}
