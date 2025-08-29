/**
 * Advanced Driver Assistance Systems (ADAS) for Electric Vehicles
 * Author: adzetto
 * Features: ACC, LKA, AEB, BSM, TSR, Parking Assist, 360° Monitoring
 */

#ifndef ADAS_SYSTEM_H
#define ADAS_SYSTEM_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>
#include <memory>
#include <queue>

struct Point3D {
    double x, y, z;
    Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
};

struct Object {
    int id;
    Point3D position;
    Point3D velocity;
    double width, height, length;
    std::string type; // "vehicle", "pedestrian", "cyclist", "obstacle"
    double confidence;
    std::chrono::steady_clock::time_point lastSeen;
    
    Object() : id(0), width(0), height(0), length(0), confidence(0) {}
};

enum class ADASMode {
    OFF,
    STANDBY,
    ACTIVE,
    INTERVENTION
};

enum class AlertLevel {
    NONE,
    INFO,
    WARNING,
    CRITICAL,
    EMERGENCY
};

class SensorFusion {
private:
    std::vector<Object> trackedObjects;
    std::unordered_map<int, std::queue<Point3D>> objectTrajectories;
    
public:
    void updateRadarData(const std::vector<Object>& radarObjects) {
        for (const auto& obj : radarObjects) {
            updateObjectPosition(obj);
        }
    }
    
    void updateLidarData(const std::vector<Object>& lidarObjects) {
        for (const auto& obj : lidarObjects) {
            updateObjectPosition(obj);
        }
    }
    
    void updateCameraData(const std::vector<Object>& cameraObjects) {
        for (const auto& obj : cameraObjects) {
            updateObjectPosition(obj);
        }
    }
    
    std::vector<Object> getFusedObjects() const {
        return trackedObjects;
    }
    
    double calculateTTC(const Object& obj) const {
        if (obj.velocity.x >= 0) return -1; // Object moving away or stationary
        return abs(obj.position.x / obj.velocity.x);
    }
    
private:
    void updateObjectPosition(const Object& obj) {
        auto it = std::find_if(trackedObjects.begin(), trackedObjects.end(),
                              [&](const Object& tracked) { return tracked.id == obj.id; });
        
        if (it != trackedObjects.end()) {
            *it = obj;
            it->lastSeen = std::chrono::steady_clock::now();
        } else {
            trackedObjects.push_back(obj);
        }
        
        // Store trajectory
        objectTrajectories[obj.id].push(obj.position);
        if (objectTrajectories[obj.id].size() > 50) {
            objectTrajectories[obj.id].pop();
        }
    }
};

class AdaptiveCruiseControl {
private:
    bool enabled;
    double setSpeed;
    double currentSpeed;
    double followingDistance;
    double targetDistance;
    Object leadVehicle;
    bool leadVehicleDetected;
    
public:
    AdaptiveCruiseControl() : enabled(false), setSpeed(0), currentSpeed(0),
                              followingDistance(50.0), targetDistance(30.0),
                              leadVehicleDetected(false) {}
    
    void enable(double speed) {
        enabled = true;
        setSpeed = speed;
        std::cout << "ACC enabled at " << speed << " km/h\n";
    }
    
    void disable() {
        enabled = false;
        std::cout << "ACC disabled\n";
    }
    
    double calculateTargetSpeed(const std::vector<Object>& objects) {
        if (!enabled) return currentSpeed;
        
        leadVehicleDetected = false;
        double minDistance = 200.0;
        
        for (const auto& obj : objects) {
            if (obj.type == "vehicle" && obj.position.x > 0 && obj.position.x < 100.0 &&
                abs(obj.position.y) < 2.0) {
                if (obj.position.x < minDistance) {
                    minDistance = obj.position.x;
                    leadVehicle = obj;
                    leadVehicleDetected = true;
                }
            }
        }
        
        if (leadVehicleDetected) {
            followingDistance = minDistance;
            double timeGap = followingDistance / (currentSpeed / 3.6); // Convert to m/s
            
            if (followingDistance < targetDistance) {
                // Too close, reduce speed
                return std::max(0.0, currentSpeed - 5.0);
            } else if (followingDistance > targetDistance * 1.5) {
                // Safe distance, can increase speed
                return std::min(setSpeed, currentSpeed + 2.0);
            }
        } else {
            // No lead vehicle, maintain set speed
            return setSpeed;
        }
        
        return currentSpeed;
    }
    
    void updateCurrentSpeed(double speed) { currentSpeed = speed; }
    void setTargetDistance(double distance) { targetDistance = distance; }
    
    bool isEnabled() const { return enabled; }
    double getFollowingDistance() const { return followingDistance; }
    bool hasLeadVehicle() const { return leadVehicleDetected; }
};

class LaneKeepingAssist {
private:
    bool enabled;
    double laneCenter;
    double currentLateralPosition;
    double laneWidth;
    bool leftLaneDetected;
    bool rightLaneDetected;
    double steeringCorrection;
    
public:
    LaneKeepingAssist() : enabled(false), laneCenter(0), currentLateralPosition(0),
                          laneWidth(3.5), leftLaneDetected(false), rightLaneDetected(false),
                          steeringCorrection(0) {}
    
    void enable() {
        enabled = true;
        std::cout << "Lane Keeping Assist enabled\n";
    }
    
    void disable() {
        enabled = false;
        steeringCorrection = 0;
        std::cout << "Lane Keeping Assist disabled\n";
    }
    
    void updateLaneData(bool leftLane, bool rightLane, double lateralPos) {
        leftLaneDetected = leftLane;
        rightLaneDetected = rightLane;
        currentLateralPosition = lateralPos;
        
        if (enabled && (leftLaneDetected || rightLaneDetected)) {
            calculateSteeringCorrection();
        }
    }
    
    double getSteeringCorrection() const { return steeringCorrection; }
    
    AlertLevel getLaneWarningLevel() const {
        if (!enabled) return AlertLevel::NONE;
        
        double deviation = abs(currentLateralPosition);
        
        if (deviation > laneWidth / 2.0 * 0.9) {
            return AlertLevel::EMERGENCY;
        } else if (deviation > laneWidth / 2.0 * 0.7) {
            return AlertLevel::CRITICAL;
        } else if (deviation > laneWidth / 2.0 * 0.5) {
            return AlertLevel::WARNING;
        }
        
        return AlertLevel::NONE;
    }
    
private:
    void calculateSteeringCorrection() {
        double maxCorrection = 15.0; // degrees
        steeringCorrection = -currentLateralPosition * 2.0;
        steeringCorrection = std::max(-maxCorrection, std::min(maxCorrection, steeringCorrection));
    }
};

class AutomaticEmergencyBraking {
private:
    bool enabled;
    double brakingForce;
    double criticalTTC; // Time to Collision
    bool emergencyBrakeApplied;
    
public:
    AutomaticEmergencyBraking() : enabled(true), brakingForce(0), criticalTTC(1.5),
                                  emergencyBrakeApplied(false) {}
    
    void enable() { 
        enabled = true; 
        std::cout << "AEB enabled\n";
    }
    
    void disable() { 
        enabled = false; 
        brakingForce = 0;
        emergencyBrakeApplied = false;
        std::cout << "AEB disabled\n";
    }
    
    double evaluateBrakingNeed(const std::vector<Object>& objects, double currentSpeed) {
        if (!enabled) return 0;
        
        brakingForce = 0;
        emergencyBrakeApplied = false;
        
        for (const auto& obj : objects) {
            if (obj.position.x > 0 && obj.position.x < 50.0 && abs(obj.position.y) < 2.0) {
                double ttc = obj.position.x / (currentSpeed / 3.6); // Time to collision
                
                if (ttc < criticalTTC && ttc > 0) {
                    if (ttc < 0.5) {
                        // Emergency braking
                        brakingForce = 100.0;
                        emergencyBrakeApplied = true;
                        std::cout << "EMERGENCY BRAKING ACTIVATED!\n";
                    } else if (ttc < 1.0) {
                        // Strong braking
                        brakingForce = 70.0;
                        std::cout << "Strong braking applied - TTC: " << ttc << "s\n";
                    } else {
                        // Pre-collision braking
                        brakingForce = 40.0;
                        std::cout << "Pre-collision braking - TTC: " << ttc << "s\n";
                    }
                    break;
                }
            }
        }
        
        return brakingForce;
    }
    
    bool isEmergencyBraking() const { return emergencyBrakeApplied; }
    double getBrakingForce() const { return brakingForce; }
};

class BlindSpotMonitoring {
private:
    bool enabled;
    std::vector<Object> leftBlindSpot;
    std::vector<Object> rightBlindSpot;
    
public:
    BlindSpotMonitoring() : enabled(true) {}
    
    void enable() { enabled = true; }
    void disable() { enabled = false; }
    
    void updateBlindSpots(const std::vector<Object>& objects) {
        if (!enabled) return;
        
        leftBlindSpot.clear();
        rightBlindSpot.clear();
        
        for (const auto& obj : objects) {
            if (obj.type == "vehicle") {
                // Left blind spot: y = 2-4m, x = -10 to 5m
                if (obj.position.y >= 2.0 && obj.position.y <= 4.0 &&
                    obj.position.x >= -10.0 && obj.position.x <= 5.0) {
                    leftBlindSpot.push_back(obj);
                }
                // Right blind spot: y = -4 to -2m, x = -10 to 5m
                else if (obj.position.y <= -2.0 && obj.position.y >= -4.0 &&
                         obj.position.x >= -10.0 && obj.position.x <= 5.0) {
                    rightBlindSpot.push_back(obj);
                }
            }
        }
    }
    
    bool hasLeftBlindSpotVehicle() const { return !leftBlindSpot.empty(); }
    bool hasRightBlindSpotVehicle() const { return !rightBlindSpot.empty(); }
    
    AlertLevel getBlindSpotAlert(bool leftTurnSignal, bool rightTurnSignal) const {
        if (leftTurnSignal && hasLeftBlindSpotVehicle()) {
            return AlertLevel::CRITICAL;
        }
        if (rightTurnSignal && hasRightBlindSpotVehicle()) {
            return AlertLevel::CRITICAL;
        }
        
        if (hasLeftBlindSpotVehicle() || hasRightBlindSpotVehicle()) {
            return AlertLevel::WARNING;
        }
        
        return AlertLevel::NONE;
    }
};

class TrafficSignRecognition {
private:
    bool enabled;
    std::unordered_map<std::string, double> recognizedSigns;
    double currentSpeedLimit;
    
public:
    TrafficSignRecognition() : enabled(true), currentSpeedLimit(50.0) {}
    
    void enable() { enabled = true; }
    void disable() { enabled = false; }
    
    void processImage(const std::vector<std::string>& detectedSigns) {
        if (!enabled) return;
        
        for (const auto& sign : detectedSigns) {
            recognizedSigns[sign] = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            
            // Parse speed limit signs
            if (sign.find("SPEED_") == 0) {
                std::string speedStr = sign.substr(6);
                try {
                    currentSpeedLimit = std::stod(speedStr);
                    std::cout << "Speed limit updated: " << currentSpeedLimit << " km/h\n";
                } catch (...) {}
            }
        }
        
        // Remove old signs (older than 30 seconds)
        auto now = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        
        for (auto it = recognizedSigns.begin(); it != recognizedSigns.end();) {
            if (now - it->second > 30) {
                it = recognizedSigns.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    bool hasSign(const std::string& signType) const {
        return recognizedSigns.find(signType) != recognizedSigns.end();
    }
    
    double getCurrentSpeedLimit() const { return currentSpeedLimit; }
    
    AlertLevel getSpeedLimitAlert(double currentSpeed) const {
        if (currentSpeed > currentSpeedLimit * 1.2) {
            return AlertLevel::CRITICAL;
        } else if (currentSpeed > currentSpeedLimit * 1.1) {
            return AlertLevel::WARNING;
        }
        return AlertLevel::NONE;
    }
};

class ADAS {
private:
    SensorFusion sensorFusion;
    AdaptiveCruiseControl acc;
    LaneKeepingAssist lka;
    AutomaticEmergencyBraking aeb;
    BlindSpotMonitoring bsm;
    TrafficSignRecognition tsr;
    
    ADASMode currentMode;
    double currentSpeed;
    bool turnSignalLeft;
    bool turnSignalRight;
    
public:
    ADAS() : currentMode(ADASMode::STANDBY), currentSpeed(0),
             turnSignalLeft(false), turnSignalRight(false) {
        std::cout << "ADAS System initialized\n";
    }
    
    void setMode(ADASMode mode) {
        currentMode = mode;
        
        switch (mode) {
            case ADASMode::OFF:
                acc.disable();
                lka.disable();
                aeb.disable();
                break;
            case ADASMode::STANDBY:
                // Keep AEB enabled for safety
                break;
            case ADASMode::ACTIVE:
                acc.enable(80.0); // Default cruise speed
                lka.enable();
                aeb.enable();
                break;
            case ADASMode::INTERVENTION:
                // System taking control
                break;
        }
    }
    
    void updateSensorData(const std::vector<Object>& radarObj,
                          const std::vector<Object>& lidarObj,
                          const std::vector<Object>& cameraObj) {
        sensorFusion.updateRadarData(radarObj);
        sensorFusion.updateLidarData(lidarObj);
        sensorFusion.updateCameraData(cameraObj);
    }
    
    void updateVehicleState(double speed, bool leftSignal, bool rightSignal) {
        currentSpeed = speed;
        turnSignalLeft = leftSignal;
        turnSignalRight = rightSignal;
        
        acc.updateCurrentSpeed(speed);
    }
    
    void updateLaneInfo(bool leftLane, bool rightLane, double lateralPos) {
        lka.updateLaneData(leftLane, rightLane, lateralPos);
    }
    
    void processTrafficSigns(const std::vector<std::string>& signs) {
        tsr.processImage(signs);
    }
    
    void processFrame() {
        auto objects = sensorFusion.getFusedObjects();
        
        // Update subsystems
        bsm.updateBlindSpots(objects);
        
        // Calculate control outputs
        double targetSpeed = acc.calculateTargetSpeed(objects);
        double steeringCorrection = lka.getSteeringCorrection();
        double brakingForce = aeb.evaluateBrakingNeed(objects, currentSpeed);
        
        // Check alerts
        AlertLevel laneAlert = lka.getLaneWarningLevel();
        AlertLevel blindSpotAlert = bsm.getBlindSpotAlert(turnSignalLeft, turnSignalRight);
        AlertLevel speedAlert = tsr.getSpeedLimitAlert(currentSpeed);
        
        // Display warnings
        if (laneAlert >= AlertLevel::WARNING) {
            std::cout << "LANE DEPARTURE WARNING!\n";
        }
        if (blindSpotAlert >= AlertLevel::WARNING) {
            std::cout << "BLIND SPOT WARNING!\n";
        }
        if (speedAlert >= AlertLevel::WARNING) {
            std::cout << "SPEED LIMIT WARNING!\n";
        }
        
        // Emergency intervention
        if (aeb.isEmergencyBraking() || laneAlert == AlertLevel::EMERGENCY) {
            currentMode = ADASMode::INTERVENTION;
        }
    }
    
    void displayStatus() {
        std::cout << "\n=== ADAS System Status ===\n";
        std::cout << "Mode: " << getModeString(currentMode) << "\n";
        std::cout << "Current Speed: " << currentSpeed << " km/h\n";
        std::cout << "ACC: " << (acc.isEnabled() ? "ACTIVE" : "INACTIVE") << "\n";
        if (acc.hasLeadVehicle()) {
            std::cout << "Lead Vehicle Distance: " << acc.getFollowingDistance() << " m\n";
        }
        std::cout << "LKA Steering: " << lka.getSteeringCorrection() << "°\n";
        std::cout << "AEB Force: " << aeb.getBrakingForce() << "%\n";
        std::cout << "Blind Spot - Left: " << (bsm.hasLeftBlindSpotVehicle() ? "OCCUPIED" : "CLEAR") << "\n";
        std::cout << "Blind Spot - Right: " << (bsm.hasRightBlindSpotVehicle() ? "OCCUPIED" : "CLEAR") << "\n";
        std::cout << "Speed Limit: " << tsr.getCurrentSpeedLimit() << " km/h\n";
        std::cout << "=========================\n\n";
    }
    
    // Getters for external systems
    double getTargetSpeed() const { 
        return acc.calculateTargetSpeed(sensorFusion.getFusedObjects()); 
    }
    double getSteeringCorrection() const { return lka.getSteeringCorrection(); }
    double getBrakingForce() const { return aeb.getBrakingForce(); }
    ADASMode getMode() const { return currentMode; }
    
private:
    std::string getModeString(ADASMode mode) const {
        switch (mode) {
            case ADASMode::OFF: return "OFF";
            case ADASMode::STANDBY: return "STANDBY";
            case ADASMode::ACTIVE: return "ACTIVE";
            case ADASMode::INTERVENTION: return "INTERVENTION";
            default: return "UNKNOWN";
        }
    }
};

#endif // ADAS_SYSTEM_H