#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "dnb_gantry_simulator/gantry_controller.h"
#include "dnb_gantry_simulator/gantry_driver.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "dnb_gantry_simulator_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    ROS_INFO("Starting DNB Gantry Simulator Node");
    
    // Load parameters
    double simulation_rate = pnh.param("simulation_rate", 100.0);
    
    // Create controller and driver
    simulator::GantryController controller(simulation_rate);
    simulator::GantryDriver driver(&controller);
    
    ROS_INFO("DNB Gantry Simulator Node started successfully");
    
    ros::spin();
    
    return 0;
}