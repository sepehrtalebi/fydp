#pragma once

#include <bmb_msgs/OpticalFlowReading.h>
#include <bmb_msgs/RailDetection.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

class StateEstimationControlLoop {
 public:
  StateEstimationControlLoop();

  void spin();

 private:
  // Message Callbacks

  void pressureSensorCallback(const sensor_msgs::FluidPressure::ConstPtr& msg);

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

  void railDetectionCallback(const bmb_msgs::RailDetection::ConstPtr& msg);

  void opticalFlowCallback(const bmb_msgs::OpticalFlow::ConstPtr& msg);

  // Subscriber Objects (need to keep in scope)
  ros::Subscriber pressure_sensor_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber rail_detection_sub_;
  ros::Subscriber optical_flow_sub_;

  // Publisher Objects
  ros::Publisher aircraft_state_pub_;


};
