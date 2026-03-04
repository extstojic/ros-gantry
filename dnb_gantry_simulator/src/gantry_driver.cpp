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
    double min_z = private_nh.param("min_z", -0.4);
    double max_z = private_nh.param("max_z", 0.4);
    double max_speed = private_nh.param("max_speed", 0.2);

    ROS_INFO("Gantry limits: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f] MaxSpeed: %.2f",
             min_x, max_x, min_y, max_y, min_z, max_z, max_speed);

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
    pub_dnb_tool_frame = private_nh.advertise<robot_movement_interface::EulerFrame>("tool_frame", 10, true);
    
    // Fully replace dnb_tool_manager: publish to ALL topics it normally owns
    // dnb_tool_manager is killed on startup to prevent dual-publisher flickering
    pub_dnb_tool_frame_global = nh.advertise<robot_movement_interface::EulerFrame>("/dnb_tool_frame", 10, true);
    pub_dnb_tool_frame_robotbase = nh.advertise<robot_movement_interface::EulerFrame>("/dnb_tool_frame_robotbase", 10, true);
    pub_tool_frame = nh.advertise<robot_movement_interface::EulerFrame>("/tool_frame", 10, true);
    pub_tool_frame_world = nh.advertise<robot_movement_interface::EulerFrame>("/tool_frame_world", 10, true);

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
    
    ROS_INFO("[INIT] Publishing initial tool frame: x=%.4f, y=%.4f, z=%.4f", current_pos.x, current_pos.y, current_pos.z);
    
    pub_dnb_tool_frame.publish(init_tcp);
    pub_dnb_tool_frame_global.publish(init_tcp);
    pub_dnb_tool_frame_robotbase.publish(init_tcp);
    pub_tool_frame.publish(init_tcp);
    pub_tool_frame_world.publish(init_tcp);
    
    // Start a timer to publish position continuously at high rate (100Hz) to override dnb_tool_manager
    position_update_timer = private_nh.createTimer(ros::Duration(0.01), 
        &GantryDriver::cb_position_update_timer, this);

    // Start a timer to process queued commands (behaviour similar to Nachi driver)
    process_command_timer = private_nh.createTimer(ros::Duration(0.05), &GantryDriver::cb_process_command_timer, this);
}

GantryDriver::~GantryDriver() {
}

void GantryDriver::cb_position_update_timer(const ros::TimerEvent &event) {
    // Publish current position topics at 100Hz
    GantryPosition current_pos = controller->getCurrentPosition();
    
    // CRITICAL: Publish joint states at same rate as tool frame to keep TF buffer in sync
    // This prevents "extrapolation into the past" errors from other nodes
    publishJointStates(current_pos);
    
    // Create and fully initialize EulerFrame message (has no header, just x,y,z,alpha,beta,gamma)
    robot_movement_interface::EulerFrame tcp_pose;
    tcp_pose.x = current_pos.x;
    tcp_pose.y = current_pos.y;
    tcp_pose.z = current_pos.z;
    tcp_pose.alpha = 0.0;
    tcp_pose.beta = 0.0;
    tcp_pose.gamma = 0.0;
    
    // Publish to ALL topics (we fully replace dnb_tool_manager)
    pub_dnb_tool_frame.publish(tcp_pose);
    pub_dnb_tool_frame_global.publish(tcp_pose);
    pub_dnb_tool_frame_robotbase.publish(tcp_pose);
    pub_tool_frame.publish(tcp_pose);
    pub_tool_frame_world.publish(tcp_pose);
    
    // Broadcast TF: tool0 -> dnb_tool_frame (replaces dnb_tool_manager's TF broadcast)
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = "tool0";
    tf_msg.child_frame_id = "dnb_tool_frame";
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;
    broadcaster.sendTransform(tf_msg);
    
    ROS_DEBUG_THROTTLE(1.0, "[PUB 100Hz] x=%.6f y=%.6f z=%.6f",
                       tcp_pose.x, tcp_pose.y, tcp_pose.z);
    
    // Republish speed scale to ensure it stays at 1.0
    std_msgs::Float32 speed_scale_msg;
    speed_scale_msg.data = 1.0;
    pub_current_speed_scale.publish(speed_scale_msg);
}

void GantryDriver::publishJointStates(GantryPosition position) {
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.header.frame_id = "robot_base";
    joint_state.name.push_back("x_joint");
    joint_state.name.push_back("y_joint");
    joint_state.name.push_back("z_joint");
    joint_state.position.push_back(position.x);
    joint_state.position.push_back(position.y);
    joint_state.position.push_back(position.z);
    joint_state.velocity.push_back(0.0);
    joint_state.velocity.push_back(0.0);
    joint_state.velocity.push_back(0.0);
    
    ROS_DEBUG_THROTTLE(1.0, "[PUB JointState] x=%.6f y=%.6f z=%.6f @ %.9f", 
                       position.x, position.y, position.z, joint_state.header.stamp.toSec());
    
    pub_joint_states.publish(joint_state);
    pub_joint_states_global.publish(joint_state);
}

void GantryDriver::cb_update(GantryPosition position, bool moved) {
    // Single source of truth: position_update_timer publishes joint states and TCP pose at 100Hz
    // Only handle callback queue and movement notification here
    
    ROS_DEBUG("[cb_update] called with position: x=%.6f, y=%.6f, z=%.6f, moved=%d", 
              position.x, position.y, position.z, moved ? 1 : 0);
    
    stop_queue.callAvailable(ros::WallDuration());

    if (moved) {
        ROS_DEBUG("[cb_update] Publishing movement notification");
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

// Timer: process pending commands sequentially (NON-BLOCKING with state machine)
void GantryDriver::cb_process_command_timer(const ros::TimerEvent &evt) {
    // Check if a command is being executed
    if (processing_command) {
        // Check if movement finished
        GantryPosition pos = controller->getCurrentPosition();
        double dx = command_target.x - pos.x;
        double dy = command_target.y - pos.y;
        double dz = command_target.z - pos.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        bool finished = (dist < 0.000001);  // ZERO_BORDER
        
        // Timeout check - if waiting longer than 30s, something is wrong
        double elapsed = (ros::Time::now() - command_start_time).toSec();
        if (elapsed > 30.0) {
            ROS_ERROR("Command execution timeout - stopping");
            controller->stop();
            robot_movement_interface::Result result;
            result.header.stamp = ros::Time::now();
            result.command_id = current_command_id;
            result.result_code = robot_movement_interface::Result::FAILURE_STOP_TRIGGERED;
            result.additional_information = "Command timeout (>30s)";
            pub_command_result.publish(result);
            processing_command = false;
            return;
        }
        
        if (finished) {
            robot_movement_interface::Result result;
            result.header.stamp = ros::Time::now();
            result.command_id = current_command_id;
            result.result_code = robot_movement_interface::Result::SUCCESS;
            result.additional_information = "Move completed";
            pub_command_result.publish(result);
            processing_command = false;
            // Fall through to process next command
        } else {
            return;  // Still moving, check again next timer tick
        }
    }
    
    // No command executing - check for next command
    if (command_queue.empty()) return;

    // Pop front and START executing (don't wait for it to finish!)
    robot_movement_interface::Command cmd = command_queue.front();
    command_queue.pop_front();

    processing_command = true;
    current_command_id = cmd.command_id;
    command_start_time = ros::Time::now();

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
            controller->setTarget(x, y, z);  // Now clamps to limits
            command_target = {x, y, z};      // Store target for comparison in next tick
            
            ROS_INFO("Gantry: Starting command execution: move to (%.4f, %.4f, %.4f)", x, y, z);
        } else {
            robot_movement_interface::Result result;
            result.header.stamp = ros::Time::now();
            result.command_id = cmd.command_id;
            result.result_code = robot_movement_interface::Result::FAILURE_EXECUTION;
            result.additional_information = "Invalid pose: need at least 3 values (x,y,z)";
            pub_command_result.publish(result);
            processing_command = false;
        }
    } else {
        robot_movement_interface::Result result;
        result.header.stamp = ros::Time::now();
        result.command_id = cmd.command_id;
        result.result_code = robot_movement_interface::Result::SUCCESS;
        result.additional_information = "Command type not implemented for gantry";
        pub_command_result.publish(result);
        processing_command = false;
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
