#pragma once

#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_msgs/SensorMeasurements.h>

class SensorFilter {
 public:
  virtual void update(const bmb_msgs::SensorMeasurements& sensor_measurements,
                      const bmb_msgs::ControlInputs& control_inputs,
                      const double& dt) = 0;

  [[nodiscard]] virtual bmb_msgs::AircraftState getOutput() const = 0;
};
