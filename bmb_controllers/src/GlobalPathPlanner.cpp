#include "bmb_controllers/GlobalPathPlanner.h"

#include <ros/ros.h>
#include <bmb_msgs/ReferenceCommand.h>
#include <bmb_msgs/AircraftState.h>
#include <bmb_utilities/MathUtils>

#include <fstream>
#include <vector>
#include <optional>

GlobalPathPlanner::GlobalPathPlanner() waypoint_number(0) {
    std::ifstream hard_coded_coordinates{"hard_coded_coordinates.csv"}; //TODO: make hard coded coordinates csv
    assert(hard_coded_coordinates);
    std::string data;
    while (std::getline(hard_coded_coordinates, data)) {
        std::stringstream sep(data);
        std::string       field;

        // First column is airspeed
        std::getline(sep, field, ',');
        airspeed.push_back(radians(std::stod(field)));

        // Second column is altitude
        std::getline(sep, field, ',');
        altitude.push_back(radians(std::stod(field)));

        // Third column is x coord lattitude, north is positive
        std::getline(sep, field, ',');
        longitude.push_back(radians(std::stod(field)));

        // Fourth column is y coord longitude, east is positive
        std::getline(sep, field, ',');
        longitude.push_back(radians(std::stod(field)));
    }
    assert(airspeed.size() == altitude.size());
    assert(airspeed.size() == lattitude.size());
    assert(airspeed.size() == longitude.size());
    number_of_waypoints = airspeed.size();
    hard_coded_coordinates.close();
}


std::optional<sensor_msgs::ReferenceCommand> update(const bmb_msgs::AircraftState& msg) {
    bmb_msgs::AircraftState ref_cmd = msg;
    if (squared(msg.pose.position.x - lattitude[waypoint_index]) +
        squared(msg.pose.position.y - longitude[waypoint_index]) +
        squared(msg.pose.position.altitude - altitude[waypoint_index]) <
        squared(RADIUS_TOL)) {
        waypoint_index++;
        return ref_cmd;
    }
    ref_cmd.pose.airspeed = airspeed[waypoint_index];
    ref_cmd.pose.altitude = altitude[waypoint_index];
    ref_cmd.pose.x = lattitude[waypoint_index];
    ref_cmd.pose.y = longitude[waypoint_index];

}