#include "bmb_state_estimation/StateEstimationControlLoop.h"

#include <ros/ros.h>
#include <bmb_state_estimation/UKF.h>
#include <bmb_msgs/AircraftState.h>

StateEstimationControlLoop::StateEstimationControlLoop() {
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

}

void StateEstimationControlLoop::imuCallback(const sensor_msgs::Imu& msg) {

}

void StateEstimationControlLoop::gpsCallback(const sensor_msgs::NavSatFix& msg) {

}

void StateEstimationControlLoop::railDetectionCallback(const bmb_msgs::RailDetection& msg) {

}

void StateEstimationControlLoop::opticalFlowCallback(const bmb_msgs::OpticalFlow& msg) {

}

StateEstimationControlLoop::spin() {

}


int main(int argc, char** argv) {
  ros::init(argc, argv, "bmb_state_estimation");
  ros::NodeHandle nh;
  UKF kf{};
  StateEstimationControlLoop control_loop{nh, kf};
  control_loop.spin();
}
