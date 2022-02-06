#pragma once

#include <ros/ros.h>
#include <bmb_msgs/ReferenceCommand.h>
#include <bmb_msgs/AircraftState.h>

#include <fstream>
#include <vector>
#include <optional>

class GlobalPathPlanner {
    std::vector<double> airspeed;
    std:: vector<double> altitude;
    std::vector<double> lattitude;
    std::vector<double> longitude;
    int waypoint_index;
    int number_of_waypoints;
    const double RADIUS_TOL = 10; //m
public:
    GlobalPathPlanner();

    std::optional<bmb_msgs::ReferenceCommand> update(const bmb_msgs::AircraftState& msg);

    //TODO: starting point in global coordinates has to be stored in memory (in GPS Sensor, accessible by service),
    //      then global path planner needs to convert relative coordinate to global coordinate
};