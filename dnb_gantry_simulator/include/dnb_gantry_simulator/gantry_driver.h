#ifndef DNB_GANTRY_DRIVER_H_
#define DNB_GANTRY_DRIVER_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>
#include <dnb_msgs/ComponentStatus.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>
#include <industrial_msgs/RobotStatus.h>
#include <robot_movement_interface/CommandList.h>
#include <robot_movement_interface/Result.h>
#include <robot_movement_interface/GetFK.h>
#include <robot_movement_interface/EulerFrame.h>
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
    bool cb_get_fk(robot_movement_interface::GetFK::Request &req, robot_movement_interface::GetFK::Response &res);
    bool cb_get_marker_init(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void cb_reset(const std_msgs::String &msg);
    void cb_command_list(const robot_movement_interface::CommandList::ConstPtr &msg);
    void cb_position_update_timer(const ros::TimerEvent &event);
    void publishJointStates(GantryPosition position);

    ros::NodeHandle nh;
    ros::NodeHandle stop_nh;
    ros::CallbackQueue stop_queue;

    ros::ServiceServer srv_move;
    ros::ServiceServer srv_stop;
    ros::ServiceServer srv_stop_robot_right_now;
    ros::ServiceServer srv_get_fk;
    ros::ServiceServer srv_get_marker_init;
    ros::Subscriber sub_notify_reset_simulation;
    ros::Subscriber sub_command_list;
    ros::Publisher pub_notify_changed_transforms;
    ros::Publisher pub_joint_states;
    ros::Publisher pub_joint_states_global;
    ros::Publisher pub_status;
    ros::Publisher pub_robot_status;
    ros::Publisher pub_command_result;
    ros::Publisher pub_dnb_tool_frame;
    ros::Publisher pub_dnb_tool_frame_global;
    ros::Timer position_update_timer;

    tf2_ros::TransformBroadcaster broadcaster;
    GantryController* controller;
};

}

#endif
