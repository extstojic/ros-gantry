#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "gantry_controller.h"
#include "gantry_driver.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "dnb_gantry_simulator_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    ROS_INFO("Starting DNB Gantry Simulator Node");
    
    // Load parameters
    double x_min = pnh.param("x_min", -0.5);
    double x_max = pnh.param("x_max", 0.5); 
    double y_min = pnh.param("y_min", -0.3);
    double y_max = pnh.param("y_max", 0.3);
    double z_min = pnh.param("z_min", -0.4);
    double z_max = pnh.param("z_max", 0.1);
    double max_speed = pnh.param("max_speed", 0.2);
    
    // Create controller and driver
    GantryController controller(x_min, x_max, y_min, y_max, z_min, z_max, max_speed);
    GantryDriver driver(nh, &controller);
    
    ROS_INFO("DNB Gantry Simulator Node started successfully");
    
    ros::spin();
    
    return 0;
}