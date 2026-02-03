#include <dnb_gantry_simulator/gantry_driver.h>
#include <dnb_msgs/ComponentStatus.h>
#include <industrial_msgs/RobotStatus.h>

using namespace simulator;

GantryDriver::GantryDriver(GantryController* controller) {
    ros::NodeHandle private_nh("~");
    stop_nh = ros::NodeHandle("~");
    this->controller = controller;

    ROS_INFO("=== GANTRY SIMULATOR v2.0 - BUILD Feb 3 2026 ===");

    // Get limits from parameters
    double min_x = private_nh.param("min_x", -0.4);
    double max_x = private_nh.param("max_x", 0.4);
    double min_y = private_nh.param("min_y", -0.3);
    double max_y = private_nh.param("max_y", 0.3);
    double min_z = private_nh.param("min_z", -0.5);
    double max_z = private_nh.param("max_z", 0.5);
    double max_speed = private_nh.param("max_speed", 0.2);

    // Initialize controller with limits and initial position at center
    GantryLimits limits = {min_x, max_x, min_y, max_y, min_z, max_z, 0.0, max_speed};
    GantryPosition init_pos = {0.0, 0.0, 0.0};
    controller->initialize(limits, init_pos, max_speed * 0.5);

    stop_nh.setCallbackQueue(&stop_queue);

    sub_notify_reset_simulation = nh.subscribe("/notify_reset_simulation", 1, &GantryDriver::cb_reset, this);
    srv_move = private_nh.advertiseService("move", &GantryDriver::cb_move, this);
    srv_stop = stop_nh.advertiseService("stop", &GantryDriver::cb_stop, this);
    srv_stop_robot_right_now = nh.advertiseService("stop_robot_right_now", &GantryDriver::cb_stop, this);  // Global stop service
    pub_notify_changed_transforms = nh.advertise<std_msgs::Empty>("/notify_changed_system_transformations", 1);
    pub_joint_states = private_nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    pub_joint_states_global = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);  // Global for robot_state_publisher
    pub_status = private_nh.advertise<dnb_msgs::ComponentStatus>("status", 1, true);
    pub_robot_status = nh.advertise<industrial_msgs::RobotStatus>("/robot_status", 10, true);

    // Publish component status
    dnb_msgs::ComponentStatus status_msg;
    status_msg.status_id = dnb_msgs::ComponentStatus::RUNNING;
    status_msg.status_msg = "Started";
    pub_status.publish(status_msg);

    // Publish robot status (industrial_msgs format)
    industrial_msgs::RobotStatus robot_status;
    robot_status.mode.val = industrial_msgs::RobotMode::AUTO;
    robot_status.e_stopped.val = industrial_msgs::TriState::FALSE;
    robot_status.drives_powered.val = industrial_msgs::TriState::TRUE;
    robot_status.motion_possible.val = industrial_msgs::TriState::TRUE;
    robot_status.in_motion.val = industrial_msgs::TriState::FALSE;
    robot_status.in_error.val = industrial_msgs::TriState::FALSE;
    robot_status.error_code = 0;
    pub_robot_status.publish(robot_status);

    controller->register_update_callback(std::bind(&GantryDriver::cb_update, this, std::placeholders::_1, std::placeholders::_2));
}

GantryDriver::~GantryDriver() {
}

void GantryDriver::publishJointStates(GantryPosition position) {
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.push_back("x_joint");
    joint_state.name.push_back("y_joint");
    joint_state.name.push_back("z_joint");
    joint_state.position.push_back(position.x);
    joint_state.position.push_back(position.y);
    joint_state.position.push_back(position.z);
    joint_state.velocity.push_back(0.0);
    joint_state.velocity.push_back(0.0);
    joint_state.velocity.push_back(0.0);
    pub_joint_states.publish(joint_state);
    pub_joint_states_global.publish(joint_state);  // Also publish to /joint_states for robot_state_publisher
}

void GantryDriver::cb_update(GantryPosition position, bool moved) {
    publishJointStates(position);

    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = "robot_base";
    tf.header.stamp = ros::Time::now();
    tf.child_frame_id = "end_effector";
    tf.transform.translation.x = position.x;
    tf.transform.translation.y = position.y;
    tf.transform.translation.z = position.z;
    tf.transform.rotation.w = 1.0;
    broadcaster.sendTransform(tf);

    stop_queue.callAvailable(ros::WallDuration());

    if (moved) {
        std_msgs::Empty msg;
        pub_notify_changed_transforms.publish(msg);
    }
}

bool GantryDriver::cb_move(dnb_gantry_simulator::MoveGantry::Request &req, dnb_gantry_simulator::MoveGantry::Response &res) {
    double speed;
    if (req.speed_unit == "M/S") {
        speed = req.speed;
    } else if (req.speed_unit == "PERCENT") {
        speed = controller->getLimits().max_speed * req.speed / 100.0;
    } else {
        res.error_code = dnb_gantry_simulator::MoveGantry::Response::ERROR_INVALID_SPEED;
        return true;
    }

    if (controller->setSpeed(speed)) {
        if (controller->setTarget(req.x, req.y, req.z)) {
            if (controller->awaitFinished()) {
                res.error_code = dnb_gantry_simulator::MoveGantry::Response::SUCCESS;
            } else {
                res.error_code = dnb_gantry_simulator::MoveGantry::Response::ERROR_EXTERNAL_ABORT;
            }
        } else {
            res.error_code = dnb_gantry_simulator::MoveGantry::Response::ERROR_POSITION_NOT_REACHABLE;
        }
    } else {
        res.error_code = dnb_gantry_simulator::MoveGantry::Response::ERROR_INVALID_SPEED;
    }

    return true;
}

bool GantryDriver::cb_stop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    controller->stop();
    res.success = true;
    return true;
}

void GantryDriver::cb_reset(const std_msgs::String &msg) {
    if (msg.data == "" || msg.data == ros::this_node::getName()) {
        controller->reset();
    }
}
