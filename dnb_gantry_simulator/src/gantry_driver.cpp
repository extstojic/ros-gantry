#include <dnb_gantry_simulator/gantry_driver.h>

using namespace simulator;

GantryDriver::GantryDriver(GantryController* controller) {
    ros::NodeHandle private_nh("~");
    stop_nh = ros::NodeHandle("~");
    this->controller = controller;

    stop_nh.setCallbackQueue(&stop_queue);

    sub_notify_reset_simulation = nh.subscribe("/notify_reset_simulation", 1, &GantryDriver::cb_reset, this);
    srv_move = private_nh.advertiseService("move", &GantryDriver::cb_move, this);
    srv_stop = stop_nh.advertiseService("stop", &GantryDriver::cb_stop, this);
    pub_notify_changed_transforms = nh.advertise<std_msgs::Empty>("/notify_changed_system_transformations", 1);

    controller->register_update_callback(std::bind(&GantryDriver::cb_update, this, std::placeholders::_1, std::placeholders::_2));
}

GantryDriver::~GantryDriver() {
}

void GantryDriver::cb_update(GantryPosition position, bool moved) {
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = "gantry_base";
    tf.header.stamp = ros::Time::now();
    tf.child_frame_id = "gantry_effector";
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
