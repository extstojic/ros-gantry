#include <dnb_gantry_simulator/gantry_controller.h>

using namespace simulator;

GantryController::GantryController(double simulation_rate) {
    this->simulation_rate = simulation_rate;
    loop_thread = std::thread(&GantryController::loop, this);
    reset();
}

GantryController::~GantryController() {
    loop_thread.join();
}

void GantryController::loop() {
    ros::Rate loop_rate(simulation_rate);
    while (ros::ok()) {
        tick();
        loop_rate.sleep();
    }
}

void GantryController::tick() {
    mutex_target.lock();
    mutex_position.lock();
    mutex_speed.lock();

    double max_step = speed / simulation_rate;
    double dx = target_pos.x - current_pos.x;
    double dy = target_pos.y - current_pos.y;
    double dz = target_pos.z - current_pos.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    mutex_speed.unlock();

    bool moved = false;
    if (dist > ZERO_BORDER) {
        if (dist > max_step) {
            current_pos.x += (dx / dist) * max_step;
            current_pos.y += (dy / dist) * max_step;
            current_pos.z += (dz / dist) * max_step;
        } else {
            current_pos = target_pos;
        }
        moved = true;
    }

    mutex_position.unlock();
    mutex_target.unlock();

    if (update_callback) {
        update_callback(current_pos, moved);
    }
}

bool GantryController::initialize(GantryLimits lim, GantryPosition init, double init_spd) {
    limits = lim;
    init_pos = init;
    init_speed = init_spd;
    speed = init_spd;

    mutex_target.lock();
    current_pos = init_pos;
    target_pos = init_pos;
    mutex_target.unlock();

    initialized = true;
    return true;
}

bool GantryController::setTarget(double x, double y, double z) {
    // Clamp to limits instead of rejecting - allows smooth jogging at boundaries
    x = std::max(limits.min_x, std::min(limits.max_x, x));
    y = std::max(limits.min_y, std::min(limits.max_y, y));
    z = std::max(limits.min_z, std::min(limits.max_z, z));

    mutex_target.lock();
    target_pos.x = x;
    target_pos.y = y;
    target_pos.z = z;
    abort = false;
    mutex_target.unlock();
    return true;
}

bool GantryController::setSpeed(double spd) {
    if (spd < limits.min_speed || spd > limits.max_speed) return false;
    mutex_speed.lock();
    speed = spd;
    mutex_speed.unlock();
    return true;
}

bool GantryController::awaitFinished() {
    ros::Rate rate(50);
    while (ros::ok() && !abort) {
        mutex_position.lock();
        mutex_target.lock();
        double dx = target_pos.x - current_pos.x;
        double dy = target_pos.y - current_pos.y;
        double dz = target_pos.z - current_pos.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        mutex_target.unlock();
        mutex_position.unlock();

        if (dist < ZERO_BORDER) return true;
        rate.sleep();
    }
    return false;
}

void GantryController::stop() {
    mutex_target.lock();
    target_pos = current_pos;
    abort = true;
    mutex_target.unlock();
}

void GantryController::reset() {
    mutex_target.lock();
    current_pos = init_pos;
    target_pos = init_pos;
    mutex_target.unlock();

    if (update_callback) {
        update_callback(current_pos, true);
    }
}

GantryPosition GantryController::getCurrentPosition() {
    mutex_position.lock();
    GantryPosition pos = current_pos;
    mutex_position.unlock();
    return pos;
}

GantryLimits GantryController::getLimits() {
    return limits;
}

double GantryController::getSpeed() {
    mutex_speed.lock();
    double s = speed;
    mutex_speed.unlock();
    return s;
}

void GantryController::register_update_callback(std::function<void(GantryPosition, bool)> callback) {
    update_callback = callback;
}
