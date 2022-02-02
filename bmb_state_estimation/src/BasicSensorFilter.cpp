#include "bmb_state_estimation/BasicSensorFilter.h"
#include <bmb_msgs/SensorMeasurements.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_math/Quaternion.h>
#include <bmb_math/Vector3.h>

BasicSensorFilter::BasicSensorFilter() {
    state.pose.orientation.w = 1;
}

void BasicSensorFilter::update(const bmb_msgs::SensorMeasurements& sensor_measurements,
                               const bmb_msgs::ControlInputs& control_inputs,
                               const double &dt) {
    Vector3<double> last_position{state.pose.position};
    Quaternion<double> last_orientation{state.pose.orientation};
    Vector3<double> w_abs = orientation.unrotate(Vector3{state.twist.angular});

    Vector3<double> position;
    // TODO: x and y based on GPS
    position.z = -sensorMeasurements.gps_altitude;
    position.copy_to(state.pose.position);

    Quaternion<double> orientation = last_orientation + last_orientation.E().transpose() * w_abs * (dt / 2);
    orientation.normalize();
    orientation.copy_to(state.pose.orientation);

    Vector3<double> velocity = (position - last_position) / dt;
    velocity.copy_to(state.twist.linear);
    state.twist.angular = sensor_measurements.imu_reading.angular_velocity;
}

bmb_msgs::AircraftState BasicSensorFilter::getOutput() const {
    return state;
}
