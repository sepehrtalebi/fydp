#pragma once

#include <bmb_msgs/AircraftState.h>
#include <bmb_msgs/ControlInputs.h>
#include <bmb_msgs/SensorMeasurements.h>
#include <bmb_state_estimation/SensorFilter.h>

class BasicSensorFilter : public SensorFilter {
 private:
  bmb_msgs::AircraftState state;

 public:
  BasicSensorFilter();

  void update(const bmb_msgs::SensorMeasurements& sensor_measurements,
              const bmb_msgs::ControlInputs& control_inputs,
              const double& dt) override;

  [[nodiscard]] bmb_msgs::AircraftState getOutput() const override;
};
