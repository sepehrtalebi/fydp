#include "bmb_gazebo/ARISGazeboPlugin.h"
#include <bmb_msgs/ControlInputs.h>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <cstddef>
#include <functional>
#include <mutex>
#include <string>

using namespace gazebo;

ARISGazeboPlugin::~ARISGazeboPlugin() {
  event::Events::DisconnectWorldUpdateBegin(this->update_connection);
}

void ARISGazeboPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  if (!_model) {
    ROS_FATAL("ARISGazeboPlugin _model pointer is NULL");
    return;
  }
  if (!_sdf) {
    ROS_FATAL("ARISGazeboPlugin _sdf pointer is NULL");
    return;
  }
  if (!ros::isInitialized()) {
    ROS_FATAL("ROS API plugin not loaded");
    return;
  }

  // Read the required joint name parameters.
  static constexpr std::array<std::string, 4> required_params = {
      "propeller", "right_aileron", "left_aileron", "elevator"};
  for (size_t i = 0; i < required_params.size(); i++) {
    const std::string& param = required_params[i];
    if (!_sdf->HasElement(param)) {
      ROS_FATAL_STREAM("Unable to find the <" << param << "> parameter.");
      return;
    }

    const std::string joint_name = _sdf->Get<std::string>(param);
    this->joints[i] = _model->GetJoint(joint_name);
    if (!this->joints[i]) {
      ROS_FATAL_STREAM("Failed to find joint [" << joint_name
                                                << "] aborting plugin load.");
      return;
    }
  }

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->update_connection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ARISGazeboPlugin::update, this, std::placeholders::_1));

  // Initialize ROS subscriber
  this->control_inputs_sub = this->nh.subscribe(
      "/control_inputs", 1, &ARISGazeboPlugin::controlInputsCallback, this);

  ROS_INFO("ARIS ready to fly. The force will be with you");
}

void ARISGazeboPlugin::controlInputsCallback(
    const bmb_msgs::ControlInputs& msg) {
  std::lock_guard<std::mutex> lock(this->mutex);
  this->latest_controls_input = msg;
}

void ARISGazeboPlugin::update(const common::UpdateInfo& /** _info **/) {
  bmb_msgs::ControlInputs msg;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    msg = this->latest_control_inputs;
  }

  this->joints[kPropeller]->SetForce(0, msg.propeller_voltage);
  this->joints[kRightAileron]->SetAngle(0, msg.right_aileron_angle);
  this->joints[kLeftAileron]->SetAngle(0, -msg.right_aileron_angle);
  this->joints[kElevator]->SetAngle(0, msg.elevator_angle);
}

// Register plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(ARISGazeboPlugin);
