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
    double min_x = pnh.param("min_x", -0.4);
    double max_x = pnh.param("max_x", 0.4);
    double min_y = pnh.param("min_y", -0.3);
    double max_y = pnh.param("max_y", 0.3);
    double min_z = pnh.param("min_z", -0.5);
    double max_z = pnh.param("max_z", 0.5);
    double max_speed = pnh.param("max_speed", 0.2);
    
    // Create controller and driver
    simulator::GantryController controller(simulation_rate);
    
    // Initialize controller with limits
    simulator::GantryLimits limits;
    limits.min_x = min_x;
    limits.max_x = max_x;
    limits.min_y = min_y;
    limits.max_y = max_y;
    limits.min_z = min_z;
    limits.max_z = max_z;
    limits.min_speed = 0.01;
    limits.max_speed = max_speed;
    
    simulator::GantryPosition init_pos;
    init_pos.x = 0.0;
    init_pos.y = 0.0;
    init_pos.z = 0.0;
    
    controller.initialize(limits, init_pos, 0.1);
    
    simulator::GantryDriver driver(&controller);
    
    ROS_INFO("DNB Gantry Simulator Node started successfully");
    ROS_INFO("Workspace: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f] m", 
        min_x, max_x, min_y, max_y, min_z, max_z);
    
    ros::spin();
    
    return 0;
}