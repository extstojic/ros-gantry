#include <dnb_gantry_simulator/gantry_driver.h>
#include <dnb_msgs/ComponentStatus.h>
#include <industrial_msgs/RobotStatus.h>
#include <robot_movement_interface/CommandList.h>
#include <robot_movement_interface/Result.h>
#include <robot_movement_interface/GetFK.h>
#include <std_msgs/Float32.h>

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

    ROS_INFO("Gantry limits: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f] MaxSpeed: %.2f",
             min_x, max_x, min_y, max_y, min_z, max_z, max_speed);

    // Initialize controller with limits and initial position at center
    GantryLimits limits = {min_x, max_x, min_y, max_y, min_z, max_z, 0.0, max_speed};
    GantryPosition init_pos = {0.0, 0.0, 0.0};
    controller->initialize(limits, init_pos, max_speed * 0.5);

    ROS_INFO("Gantry initialized at home position: x=0.0, y=0.0, z=0.0");

    stop_nh.setCallbackQueue(&stop_queue);

    sub_notify_reset_simulation = nh.subscribe("/notify_reset_simulation", 1, &GantryDriver::cb_reset, this);
    srv_move = private_nh.advertiseService("move", &GantryDriver::cb_move, this);
    srv_stop = stop_nh.advertiseService("stop", &GantryDriver::cb_stop, this);
    srv_stop_robot_right_now = nh.advertiseService("/stop_robot_right_now", &GantryDriver::cb_stop, this);
    srv_get_marker_init = private_nh.advertiseService("get_marker_init", &GantryDriver::cb_get_marker_init, this);  // Marker initialization with current position
    pub_notify_changed_transforms = nh.advertise<std_msgs::Empty>("/notify_changed_system_transformations", 1);
    pub_joint_states = private_nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    pub_joint_states_global = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);  // Global for robot_state_publisher
    pub_status = private_nh.advertise<dnb_msgs::ComponentStatus>("status", 1, true);
    pub_robot_status = nh.advertise<industrial_msgs::RobotStatus>("/robot_status", 10, true);
    
    // Robot movement interface (for UI jogging/movement commands)
    sub_command_list = nh.subscribe("/command_list", 10, &GantryDriver::cb_command_list, this);
    pub_command_result = nh.advertise<robot_movement_interface::Result>("/command_result", 10);
    pub_dnb_tool_frame = private_nh.advertise<robot_movement_interface::EulerFrame>("tool_frame", 100, true);  // Latched tool frame for marker initialization
    
    // Also publish to global topic for drag&bot marker to use directly
    pub_dnb_tool_frame_global = nh.advertise<robot_movement_interface::EulerFrame>("/dnb_tool_frame", 100, true);
    
    // CRITICAL: delta_interface_v3.py subscribes to this topic for current_pose used in jog calculations
    pub_dnb_tool_frame_robotbase = nh.advertise<robot_movement_interface::EulerFrame>("/dnb_tool_frame_robotbase", 100, true);

    // CRITICAL: delta_interface multiplies jog delta by this value - must be non-zero!
    pub_current_speed_scale = nh.advertise<std_msgs::Float32>("/current_speed_scale", 10, true);
    std_msgs::Float32 speed_scale_msg;
    speed_scale_msg.data = 1.0;  // 100% speed scale
    pub_current_speed_scale.publish(speed_scale_msg);
    ROS_INFO("Published /current_speed_scale = 1.0");

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
    
    // Immediately publish initial position so marker can read it
    GantryPosition current_pos = controller->getCurrentPosition();
    robot_movement_interface::EulerFrame init_tcp;
    init_tcp.x = current_pos.x;
    init_tcp.y = current_pos.y;
    init_tcp.z = current_pos.z;
    init_tcp.alpha = 0.0;
    init_tcp.beta = 0.0;
    init_tcp.gamma = 0.0;
    pub_dnb_tool_frame.publish(init_tcp);
    pub_dnb_tool_frame_global.publish(init_tcp);
    pub_dnb_tool_frame_robotbase.publish(init_tcp);  // For delta_interface jog commands
    
    // Start a timer to publish position continuously so marker always reads fresh values
    position_update_timer = private_nh.createTimer(ros::Duration(0.1), 
        &GantryDriver::cb_position_update_timer, this);

    // Start a timer to process queued commands (behaviour similar to Nachi driver)
    process_command_timer = private_nh.createTimer(ros::Duration(0.05), &GantryDriver::cb_process_command_timer, this);
}

GantryDriver::~GantryDriver() {
}

void GantryDriver::cb_position_update_timer(const ros::TimerEvent &event) {
    // Publish current position frequently so marker always reads fresh values
    GantryPosition current_pos = controller->getCurrentPosition();
    robot_movement_interface::EulerFrame tcp_pose;
    tcp_pose.x = current_pos.x;
    tcp_pose.y = current_pos.y;
    tcp_pose.z = current_pos.z;
    tcp_pose.alpha = 0.0;
    tcp_pose.beta = 0.0;
    tcp_pose.gamma = 0.0;
    pub_dnb_tool_frame.publish(tcp_pose);
    pub_dnb_tool_frame_global.publish(tcp_pose);
    pub_dnb_tool_frame_robotbase.publish(tcp_pose);  // For delta_interface jog commands
}

void GantryDriver::publishJointStates(GantryPosition position) {
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.header.frame_id = "robot_base";  // Add frame_id
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

    // Also broadcast manufacturer_base → robot_state_tcp for drag&bot compatibility
    tf.header.frame_id = "manufacturer_base";
    tf.child_frame_id = "robot_state_tcp";
    broadcaster.sendTransform(tf);

    // Publish TCP pose to override dnb_tool_manager's cached value
    robot_movement_interface::EulerFrame tcp_pose;
    tcp_pose.x = position.x;
    tcp_pose.y = position.y;
    tcp_pose.z = position.z;
    tcp_pose.alpha = 0.0;  // No rotation for gantry
    tcp_pose.beta = 0.0;
    tcp_pose.gamma = 0.0;
    pub_dnb_tool_frame.publish(tcp_pose);
    pub_dnb_tool_frame_global.publish(tcp_pose);  // Also publish to global topic for marker
    pub_dnb_tool_frame_robotbase.publish(tcp_pose);  // For delta_interface jog commands

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

void GantryDriver::cb_command_list(const robot_movement_interface::CommandList::ConstPtr &msg) {
    ROS_INFO("Gantry received command list with %lu commands (queued)", msg->commands.size());

    // If replace flag is set, clear pending commands
    if (msg->replace_previous_commands) {
        command_queue.clear();
        ROS_INFO("Gantry: replace_previous_commands=true -> cleared command_queue");
    }

    // Append incoming commands to internal queue (Nachi-style behaviour)
    for (const auto &cmd : msg->commands) {
        command_queue.push_back(cmd);
    }
}

// Timer: process pending commands sequentially
void GantryDriver::cb_process_command_timer(const ros::TimerEvent &evt) {
    if (processing_command) return; // still executing
    if (command_queue.empty()) return;

    // Pop front and execute
    robot_movement_interface::Command cmd = command_queue.front();
    command_queue.pop_front();

    processing_command = true;

    robot_movement_interface::Result result;
    result.header.stamp = ros::Time::now();
    result.command_id = cmd.command_id;

    // Basic handling similar to original logic
    std::string cmd_type = cmd.command_type;
    if (cmd_type.empty()) cmd_type = "LIN";

    if (cmd_type == "LIN" || cmd_type == "PTP" || cmd_type == "JOINTS") {
        if (cmd.pose.size() >= 3) {
            double x = cmd.pose[0];
            double y = cmd.pose[1];
            double z = cmd.pose[2];

            // Convert from mm to m if necessary
            if (std::abs(x) > 10 || std::abs(y) > 10 || std::abs(z) > 10) {
                x /= 1000.0; y /= 1000.0; z /= 1000.0;
            }

            // If this was intended as relative (jog), remote-control uses replace=false to indicate
            // However we cannot see the original CommandList replace flag here; the remote-control
            // typically encodes the absolute target when publishing — we assume cmd.pose contains proper target

            // Determine speed
            double speed = controller->getLimits().max_speed * 0.5; // default
            if (cmd.velocity.size() > 0 && cmd.velocity[0] > 0) {
                if (cmd.velocity_type == "PERCENT" || cmd.velocity_type == "%") {
                    speed = controller->getLimits().max_speed * cmd.velocity[0] / 100.0;
                } else if (cmd.velocity_type == "M/S") {
                    speed = cmd.velocity[0];
                } else if (cmd.velocity_type == "MM/S") {
                    speed = cmd.velocity[0] / 1000.0;
                }
            }

            controller->setSpeed(speed);
            if (controller->setTarget(x, y, z)){
                if (controller->awaitFinished()){
                    result.result_code = robot_movement_interface::Result::SUCCESS;
                    result.additional_information = "Move completed";
                    GantryPosition final_pos = controller->getCurrentPosition();
                    cb_update(final_pos, true);
                } else {
                    result.result_code = robot_movement_interface::Result::FAILURE_STOP_TRIGGERED;
                    result.additional_information = "Move aborted";
                }
            } else {
                result.result_code = robot_movement_interface::Result::FAILURE_OUT_OF_REACH;
                result.additional_information = "Target position out of reach";
            }
        } else {
            result.result_code = robot_movement_interface::Result::FAILURE_EXECUTION;
            result.additional_information = "Invalid pose: need at least 3 values (x,y,z)";
        }
    } else {
        result.result_code = robot_movement_interface::Result::SUCCESS;
        result.additional_information = "Command type not implemented for gantry";
    }

    pub_command_result.publish(result);
    processing_command = false;
}

bool GantryDriver::cb_get_fk(robot_movement_interface::GetFK::Request &req, robot_movement_interface::GetFK::Response &res) {
    // Forward kinematics for gantry: direct mapping from joints to pose
    // For a prismatic gantry, joint positions directly correspond to TCP position
    // joints[0] = x position, joints[1] = y position, joints[2] = z position
    
    // Get current position from controller
    GantryPosition pos = controller->getCurrentPosition();
    
    // If joints array is empty or has fewer than 3 elements, return current position
    // This is used by drag&bot to initialize the interactive marker
    if (req.joints.size() == 0 || req.joints.size() < 3) {
        ROS_DEBUG("FK: Empty/incomplete joints array - returning current position [%.4f, %.4f, %.4f]", pos.x, pos.y, pos.z);
        res.pose.x = pos.x;
        res.pose.y = pos.y;
        res.pose.z = pos.z;
        res.pose.alpha = 0.0;
        res.pose.beta = 0.0;
        res.pose.gamma = 0.0;
        res.error = 0;  // Success
        return true;
    }
    
    // If joints provided, compute FK from those joint values
    // For gantry: joints directly map to cartesian position
    res.pose.x = req.joints[0];
    res.pose.y = req.joints[1];
    res.pose.z = req.joints[2];
    res.pose.alpha = 0.0;  // No rotation for gantry
    res.pose.beta = 0.0;
    res.pose.gamma = 0.0;
    
    res.error = 0;  // Success
    
    ROS_DEBUG("FK: Computed pose [%.4f, %.4f, %.4f, 0, 0, 0] from joints", res.pose.x, res.pose.y, res.pose.z);
    
    return true;
}

bool GantryDriver::cb_get_marker_init(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    // Return current robot position for interactive marker initialization
    // This service is called by drag&bot to set the initial position of the interactive marker
    // Without this, the marker would always initialize at [0, 0, 0]
    
    GantryPosition pos = controller->getCurrentPosition();
    
    ROS_INFO("Marker initialization requested - returning current position [%.4f, %.4f, %.4f]", pos.x, pos.y, pos.z);
    
    res.success = true;
    res.message = std::to_string(pos.x) + "," + std::to_string(pos.y) + "," + std::to_string(pos.z);
    
    return true;
}
