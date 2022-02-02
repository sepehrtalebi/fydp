#include <ros/ros.h>
#include <bmb_state_estimation/UKF.h>
#include <bmb_state_estimation/StateEstimationControlLoop.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "bmb_state_estimation");
  ros::NodeHandle nh;
  UKF kf{};
  StateEstimationControlLoop control_loop{nh, kf};
  control_loop.spin();
}
