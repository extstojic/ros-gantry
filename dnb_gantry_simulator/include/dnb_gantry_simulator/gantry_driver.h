#ifndef DNB_GANTRY_DRIVER_H_
#define DNB_GANTRY_DRIVER_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>
#include <dnb_gantry_simulator/gantry_controller.h>
#include <dnb_gantry_simulator/MoveGantry.h>

namespace simulator {

class GantryDriver {
public:
    GantryDriver(GantryController* controller);
    ~GantryDriver();

private:
    void cb_update(GantryPosition position, bool moved);
    bool cb_move(dnb_gantry_simulator::MoveGantry::Request &req, dnb_gantry_simulator::MoveGantry::Response &res);
    bool cb_stop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void cb_reset(const std_msgs::String &msg);
    void publishJointStates(GantryPosition position);

    ros::NodeHandle nh;
    ros::NodeHandle stop_nh;
    ros::CallbackQueue stop_queue;

    ros::ServiceServer srv_move;
    ros::ServiceServer srv_stop;
    ros::Subscriber sub_notify_reset_simulation;
    ros::Publisher pub_notify_changed_transforms;
    ros::Publisher pub_joint_states;
    ros::Publisher pub_status;

    tf2_ros::StaticTransformBroadcaster broadcaster;
    GantryController* controller;
};

}

#endif
