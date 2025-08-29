#ifndef COMMON_DEFS_H
#define COMMON_DEFS_H

#include <iostream>
#include <vector>
#include <string>
#include <memory>

// Forward declarations for types used by stubs
struct Waypoint;
struct DetectedObject;

class VehicleState {
public:
    double speed;
    double acceleration;
    double steering_angle;
    // other vehicle state variables
};

class SensorFusion {
public:
    void process() {
        // Sensor fusion logic
    }
    // other sensor fusion methods
};

class PathPlanner {
public:
    void plan() {
        // Path planning logic
    }
    // other path planning methods

    // Minimal AD integrations (stubs)
    void setGlobalPath(const std::vector<Waypoint>&) {}
    std::vector<Waypoint> planLocalPath(const Waypoint&, const std::vector<DetectedObject>&) { return {}; }
    void displayPlannerStatus() const {}
};

class MotionController {
public:
    void control() {
        // Motion control logic
    }
    // other motion control methods

    // Minimal AD integrations (stubs)
    void setTargets(double, double) {}
    void updateControl(double) {}
    std::pair<double,double> getControlOutputs() const { return {0.0, 0.0}; }
    void displayControlStatus() const {}
};

#endif // COMMON_DEFS_H
