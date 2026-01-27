#include <ros/ros.h>
#include <dnb_gantry_simulator/gantry_controller.h>
#include <dnb_gantry_simulator/gantry_driver.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "dnb_gantry_simulator");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Duration(1.0).sleep();

    double min_x = -0.4, max_x = 0.4;
    double min_y = -0.3, max_y = 0.3;
    double min_z = -0.5, max_z = 0.5;
    double max_speed = 0.2;
    double min_speed = 0.01;

    private_nh.getParam("min_x", min_x);
    private_nh.getParam("max_x", max_x);
    private_nh.getParam("min_y", min_y);
    private_nh.getParam("max_y", max_y);
    private_nh.getParam("min_z", min_z);
    private_nh.getParam("max_z", max_z);
    private_nh.getParam("max_speed", max_speed);

    simulator::GantryLimits limits;
    limits.min_x = min_x;
    limits.max_x = max_x;
    limits.min_y = min_y;
    limits.max_y = max_y;
    limits.min_z = min_z;
    limits.max_z = max_z;
    limits.min_speed = min_speed;
    limits.max_speed = max_speed;

    simulator::GantryPosition init_pos;
    init_pos.x = 0.0;
    init_pos.y = 0.0;
    init_pos.z = 0.0;

    ros::Rate loop_rate(20.0);

    simulator::GantryController controller(50.0);
    simulator::GantryDriver driver(&controller);


    ros::Duration(1.0).sleep();

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
