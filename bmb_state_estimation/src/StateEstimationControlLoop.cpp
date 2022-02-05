#include "bmb_state_estimation/StateEstimationControlLoop.h"

#include <ros/ros.h>
#include <bmb_state_estimation/UKF.h>
#include <bmb_msgs/AircraftState.h>

StateEstimationControlLoop::StateEstimationControlLoop(
    ros::NodeHandle& nh, const double& update_frequency) : update_frequency(update_frequency) {
  // initialize subscribers
  pressure_sensor_sub_ = nh.subscribe(
      "pressure_sensor_reading", 1, &StateEstimationControlLoop::pressureSensorCallback, this);
  imu_sub_ = nh.subscribe(
      "imu_reading", 1, &StateEstimationControlLoop::imuCallback, this);
  gps_sub_ = nh.subscribe(
      "gps_reading", 1, &StateEstimationControlLoop::gpsCallback, this);
  rail_detection_sub_ = nh.subscribe(
      "rail_detection", 1, &StateEstimationControlLoop::railDetectionCallback, this);
  optical_flow_sub_ = nh.subscribe(
      "optical_flow_reading", 1, &StateEstimationControlLoop::opticalFlowCallback, this);

  // initialize publishers
  aircraft_state_pub_ = nh.advertise<bmb_msgs::AircraftState>("aircraft_state", 1);
}


void StateEstimationControlLoop::pressureSensorCallback(const sensor_msgs::FluidPressure& msg) {
  latest_measurements.pressure_reading = msg;
}

void StateEstimationControlLoop::imuCallback(const sensor_msgs::Imu& msg) {
  latest_measurements.imu_reading = msg;
}

void StateEstimationControlLoop::gpsCallback(const sensor_msgs::NavSatFix& msg) {
  latest_measurements.gps_reading = msg;
}

void StateEstimationControlLoop::railDetectionCallback(const bmb_msgs::RailDetection& msg) {
  latest_measurements.rail_detection = msg;
}

void StateEstimationControlLoop::opticalFlowCallback(const bmb_msgs::OpticalFlowReading& msg) {
  latest_measurements.optical_flow_reading = msg;
}

void StateEstimationControlLoop::controlInputsCallback(const bmb_msgs::ControlInputs& msg) {
  latest_control_inputs = msg;
}

void StateEstimationControlLoop::spin() {
  ros::Rate rate{update_frequency};
  const double period = 1 / update_frequency;
  while (ros::ok()) {
    ros::spinOnce();
    kf.update(latest_measurements, latest_control_inputs, period);
    aircraft_state_pub_.publish(kf.getOutput());
    rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "bmb_state_estimation");
  ros::NodeHandle nh;
  StateEstimationControlLoop control_loop{nh, 60};
  control_loop.spin();
}
