#include "bmb_state_estimation/StateEstimationControlLoop.h"

#include <ros/ros.h>
#include <bmb_state_estimation/UKF.h>


StateEstimationControlLoop::StateEstimationControlLoop() {
  ros::Subscriber sub = nh.subscribe("sensor_measurements", 1, sensor);
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
