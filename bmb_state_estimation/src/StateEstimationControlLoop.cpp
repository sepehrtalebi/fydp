#include "bmb_state_estimation/StateEstimationControlLoop.h"

StateEstimationControlLoop::StateEstimationControlLoop() {
  ros::Subscriber sub = nh.subscribe("sensor_measurements", 1, sensor);
}

StateEstimationControlLoop::spin() {

}
