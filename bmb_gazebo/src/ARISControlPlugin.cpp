#include "bmb_gazebo/ARISControlPlugin.h"
#include <bmb_msgs/ControlInputs.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <array>
#include <cstddef>
#include <functional>
#include <mutex>
#include <string>

using namespace gazebo;

ARISControlPlugin::~ARISControlPlugin() {
#if GAZEBO_MAJOR_VERSION >= 8
  this->update_connection.reset();
#else
  event::Events::DisconnectWorldUpdateBegin(this->update_connection);
#endif
}

void ARISControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  if (!_model) {
    ROS_FATAL("ARISControlPlugin _model pointer is NULL");
    return;
  }
  if (!_sdf) {
    ROS_FATAL("ARISControlPlugin _sdf pointer is NULL");
    return;
  }
  if (!ros::isInitialized()) {
    ROS_FATAL("ROS API plugin not loaded");
    return;
  }

  // Read the required joint name parameters.
  static const std::array<std::string, 4> required_params{
      // cannot use constexpr with std::string
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
      std::bind(&ARISControlPlugin::update, this, std::placeholders::_1));

  // Initialize ROS subscriber
  this->control_inputs_sub = this->nh.subscribe(
      "/control_inputs", 1, &ARISControlPlugin::controlInputsCallback, this);

  ROS_INFO("ARIS ready to fly. The force will be with you");
}

void ARISControlPlugin::controlInputsCallback(
    const bmb_msgs::ControlInputs& msg) {
  std::lock_guard<std::mutex> lock(this->mutex);
  this->latest_control_inputs = msg;
}

void ARISControlPlugin::update(const common::UpdateInfo& /** _info **/) {
  bmb_msgs::ControlInputs msg;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    msg = this->latest_control_inputs;
  }

  this->joints[kPropeller]->SetForce(0, msg.propeller_force);
#if GAZEBO_MAJOR_VERSION >= 8
  this->joints[kRightAileron]->SetPosition(0, msg.right_aileron_angle);
  this->joints[kLeftAileron]->SetPosition(0, -msg.right_aileron_angle);
  this->joints[kElevator]->SetPosition(0, msg.elevator_angle);
#else
  this->joints[kRightAileron]->SetAngle(0, msg.right_aileron_angle);
  this->joints[kLeftAileron]->SetAngle(0, -msg.right_aileron_angle);
  this->joints[kElevator]->SetAngle(0, msg.elevator_angle);
#endif
}

// Register plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(ARISControlPlugin);
