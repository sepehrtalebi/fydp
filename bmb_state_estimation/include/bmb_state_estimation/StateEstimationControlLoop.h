#pragma once

#include <bmb_msgs/OpticalFlowReading.h>
#include <bmb_msgs/RailDetection.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_state_estimation/UKF.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

class StateEstimationControlLoop {
 public:
  StateEstimationControlLoop(ros::NodeHandle& nh, const double& update_frequency);

  void spin();

 private:
  // Message Callbacks

  void pressureSensorCallback(const sensor_msgs::FluidPressure& msg);

  void imuCallback(const sensor_msgs::Imu& msg);

  void gpsCallback(const sensor_msgs::NavSatFix& msg);

  void railDetectionCallback(const bmb_msgs::RailDetection& msg);

  void opticalFlowCallback(const bmb_msgs::OpticalFlow& msg);

  void controlInputsCallback(const bmb_msgs::ControlInputs& msg);

  // Subscriber Objects (need to keep in scope)
  ros::Subscriber pressure_sensor_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber rail_detection_sub_;
  ros::Subscriber optical_flow_sub_;
  ros::Subscriber control_inputs_sub_;

  // Publisher Objects
  ros::Publisher aircraft_state_pub_;

  // member variables
  double update_frequency;
  UKF kf{};
  bmb_msgs::SensorMeasurements latest_measurements;
  bmb_msgs::ControlInputs latest_control_inputs;
};
