#include <dnb_gantry_simulator/gantry_driver.h>
#include <dnb_msgs/ComponentStatus.h>
#include <industrial_msgs/RobotStatus.h>
#include <robot_movement_interface/CommandList.h>
#include <robot_movement_interface/Result.h>
#include <robot_movement_interface/GetFK.h>

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
    srv_stop_robot_right_now = nh.advertiseService("stop_robot_right_now", &GantryDriver::cb_stop, this);  // Global stop service
    srv_get_fk = private_nh.advertiseService("get_fk", &GantryDriver::cb_get_fk, this);  // Forward kinematics for UI markers
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
    ROS_INFO("Gantry received command list with %lu commands", msg->commands.size());
    
    for (const auto& cmd : msg->commands) {
        robot_movement_interface::Result result;
        result.header.stamp = ros::Time::now();
        result.command_id = cmd.command_id;
        
        ROS_INFO("Processing command type: %s, pose_type: %s, pose_size: %lu, replace: %d", 
                 cmd.command_type.c_str(), cmd.pose_type.c_str(), cmd.pose.size(), msg->replace_previous_commands);
        
        // Debug: print all pose values
        std::string pose_str = "Pose values: ";
        for (size_t i = 0; i < cmd.pose.size(); i++) {
            pose_str += std::to_string(cmd.pose[i]) + " ";
        }
        ROS_INFO("%s", pose_str.c_str());
        
        // Debug: print velocity info
        if (cmd.velocity.size() > 0) {
            ROS_INFO("Velocity: %.4f %s", cmd.velocity[0], cmd.velocity_type.c_str());
        }
        
        // Handle different command types
        // Note: UI jog commands may have empty command_type, treat them as LIN
        std::string cmd_type = cmd.command_type;
        if (cmd_type.empty()) {
            cmd_type = "LIN";  // Default to LIN for jog commands
            ROS_INFO("Empty command_type, defaulting to LIN");
        }
        
        if (cmd_type == "LIN" || cmd_type == "PTP" || cmd_type == "JOINTS") {
            // For gantry, we interpret pose as XYZ positions
            if (cmd.pose.size() >= 3) {
                double x = cmd.pose[0];
                double y = cmd.pose[1];
                double z = cmd.pose[2];
                
                // Check if values might be in millimeters (drag&bot often uses mm)
                // If values are > 10, assume millimeters and convert to meters
                if (std::abs(x) > 10 || std::abs(y) > 10 || std::abs(z) > 10) {
                    ROS_INFO("Converting from mm to m (values seem to be in mm)");
                    x /= 1000.0;
                    y /= 1000.0;
                    z /= 1000.0;
                }
                
                // The UI sends the marker's absolute position (relative to marker init)
                // NOT a delta to add to current position
                // Just use the pose directly as the target
                ROS_INFO("Target position from marker: x=%.4f, y=%.4f, z=%.4f (replace=%d)", 
                         x, y, z, msg->replace_previous_commands);
                
                // Handle velocity
                double speed = controller->getLimits().max_speed * 0.5;  // Default 50%
                if (cmd.velocity.size() > 0 && cmd.velocity[0] > 0) {
                    if (cmd.velocity_type == "PERCENT" || cmd.velocity_type == "%") {
                        speed = controller->getLimits().max_speed * cmd.velocity[0] / 100.0;
                    } else if (cmd.velocity_type == "M/S") {
                        speed = cmd.velocity[0];
                    } else if (cmd.velocity_type == "MM/S") {
                        speed = cmd.velocity[0] / 1000.0;  // Convert mm/s to m/s
                    }
                }
                
                ROS_INFO("Moving to: x=%.4f, y=%.4f, z=%.4f at speed=%.4f m/s", x, y, z, speed);
                
                controller->setSpeed(speed);
                if (controller->setTarget(x, y, z)) {
                    if (controller->awaitFinished()) {
                        result.result_code = robot_movement_interface::Result::SUCCESS;
                        result.additional_information = "Move completed";
                        ROS_INFO("Move completed successfully");
                        
                        // Explicitly publish the new position after movement
                        GantryPosition final_pos = controller->getCurrentPosition();
                        ROS_INFO("Publishing final position: x=%.4f, y=%.4f, z=%.4f", final_pos.x, final_pos.y, final_pos.z);
                        cb_update(final_pos, true);  // true = position changed
                    } else {
                        result.result_code = robot_movement_interface::Result::FAILURE_STOP_TRIGGERED;
                        result.additional_information = "Move aborted";
                        ROS_INFO("Move aborted");
                    }
                } else {
                    result.result_code = robot_movement_interface::Result::FAILURE_OUT_OF_REACH;
                    result.additional_information = "Target position out of reach";
                    ROS_WARN("Target out of reach! Limits: x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]",
                             controller->getLimits().min_x, controller->getLimits().max_x,
                             controller->getLimits().min_y, controller->getLimits().max_y,
                             controller->getLimits().min_z, controller->getLimits().max_z);
                }
            } else {
                result.result_code = robot_movement_interface::Result::FAILURE_EXECUTION;
                result.additional_information = "Invalid pose: need at least 3 values (x, y, z)";
            }
        } else {
            // Unknown command type - just succeed for now
            ROS_WARN("Unknown command type: '%s' - treating as no-op", cmd_type.c_str());
            result.result_code = robot_movement_interface::Result::SUCCESS;
            result.additional_information = "Command type not implemented for gantry";
        }
        
        pub_command_result.publish(result);
        ROS_INFO("Command %d completed with result code: %d", cmd.command_id, result.result_code);
    }
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
