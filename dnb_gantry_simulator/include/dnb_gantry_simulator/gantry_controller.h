#ifndef DNB_GANTRY_CONTROLLER_H_
#define DNB_GANTRY_CONTROLLER_H_

#define ZERO_BORDER 0.000001

#include <cmath>
#include <thread>
#include <mutex>
#include <functional>
#include <ros/ros.h>

namespace simulator {

struct GantryLimits {
    double min_x, max_x;
    double min_y, max_y;
    double min_z, max_z;
    double min_speed, max_speed;
};

struct GantryPosition {
    double x, y, z;
};

class GantryController {
public:
    GantryController(double simulation_rate);
    ~GantryController();

    void tick();
    bool initialize(GantryLimits limits, GantryPosition init_pos, double init_speed);
    bool setTarget(double x, double y, double z);
    bool setSpeed(double speed);
    bool awaitFinished();
    void stop();
    void reset();

    GantryPosition getCurrentPosition();
    GantryLimits getLimits();
    double getSpeed();

    void register_update_callback(std::function<void(GantryPosition, bool)> callback);

private:
    void loop();

    double simulation_rate;
    bool initialized = false;
    bool abort = false;
    std::thread loop_thread;
    std::mutex mutex_target;
    std::mutex mutex_position;
    std::mutex mutex_speed;
    std::function<void(GantryPosition, bool)> update_callback;

    GantryLimits limits;
    GantryPosition init_pos;
    GantryPosition current_pos;
    GantryPosition target_pos;
    double speed;
    double init_speed;
};

}

#endif
